"""Unified camera pipeline for the workbench.

    source (ROS image topic / USB camera)
        → FrameBuffer (latest raw frame)
        → detector (ONNX, optional)
        → overlay drawing
        → JPEG output buffer
        → MJPEG stream / snapshot

A single :class:`CameraService` serves every browser: the worker thread
runs only while at least one MJPEG client is connected, and closes the
camera when the last one leaves. On the robot ``usb_cam`` already owns
``/dev/video0``, so the car camera is consumed through its ROS topic;
USB sources are for extra cameras (DJI / Orbbec) only.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

import cv2
import numpy as np

from . import config, overlay

_LOG = logging.getLogger("infra.camera")


# ---------------------------------------------------------------------------
# Frame buffer
# ---------------------------------------------------------------------------


class FrameBuffer:
    """Latest item plus a sequence number; readers wait for a newer one."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._item: Any = None
        self._seq = 0

    def put(self, item: Any) -> None:
        with self._cond:
            self._item = item
            self._seq += 1
            self._cond.notify_all()

    def wait(self, last_seq: int, timeout: float) -> Tuple[int, Any]:
        """Return ``(seq, item)`` once ``seq != last_seq`` or on timeout."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout)
            return self._seq, self._item

    def latest(self) -> Tuple[int, Any]:
        with self._cond:
            return self._seq, self._item


# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

USB_CAMERAS = {
    "dji": ("DJI Action 5 Pro", ("osmoaction5", "djipocket3")),
    "orbbec": ("Orbbec 深度相机（彩色）", ("orbbec", "rgb")),
}


def open_usb_camera(source: str, width: int = 1280, height: int = 720, fps: int = 30):
    """Open a USB camera by name (dji or orbbec); returns cv2.VideoCapture."""
    label, names = USB_CAMERAS[source]
    found = False
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            name = (node / "name").read_text().lower().replace(" ", "")
            index = (node / "index").read_text().strip()
        except OSError:
            continue
        matches = any(p in name for p in names) if source == "dji" else all(p in name for p in names)
        if not matches or index != "0":
            continue
        found = True
        camera = cv2.VideoCapture("/dev/" + node.name, cv2.CAP_V4L2)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            camera.set(cv2.CAP_PROP_FPS, fps)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return camera
        camera.release()
    if found:
        raise RuntimeError(f"{label} 打不开，请检查相机是否被其他程序占用")
    raise RuntimeError(f"未找到 {label}，请检查 USB 连接" + ("并在相机上选择「网络摄像头」" if source == "dji" else ""))


class RosImageSource:
    """Subscribes to a sensor_msgs/Image topic and fills a FrameBuffer.

    Uses ``rospy.AnyMsg`` and deserialises by hand: rospy shares one
    connection per topic inside a process, and ros_bridge already counts
    ``/usb_cam/image_raw`` frames with ``AnyMsg``.
    """

    def __init__(self, topic: str, buffer: FrameBuffer) -> None:
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self._image_cls = Image
        self._bridge = CvBridge()
        self._buffer = buffer
        self._sub = rospy.Subscriber(topic, rospy.AnyMsg, self._cb, queue_size=1, buff_size=2 ** 24)

    def _cb(self, raw) -> None:
        try:
            msg = self._image_cls().deserialize(raw._buff)
            self._buffer.put(self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
        except Exception as exc:
            _LOG.warning("image conversion failed: %s", exc)

    def close(self) -> None:
        self._sub.unregister()


class UsbSource:
    """Reads a USB camera in its own thread and fills a FrameBuffer."""

    def __init__(self, name: str, buffer: FrameBuffer) -> None:
        self._camera = open_usb_camera(name)
        self._buffer = buffer
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"usb-{name}", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            ok, image = self._camera.read()
            if ok:
                self._buffer.put(image)
            else:
                self._stop.wait(0.1)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._camera.release()


def list_sources() -> List[Dict[str, str]]:
    """ROS image topics currently published plus the known USB cameras."""
    sources = []
    try:
        import rospy

        for topic, msg_type in sorted(rospy.get_published_topics()):
            if msg_type == "sensor_msgs/Image":
                sources.append({"id": f"ros:{topic}", "label": f"ROS {topic}"})
    except Exception as exc:
        _LOG.debug("cannot list ROS topics: %s", exc)
    default = "ros:/usb_cam/image_raw"
    if not any(s["id"] == default for s in sources):
        sources.insert(0, {"id": default, "label": "ROS /usb_cam/image_raw（未发布）"})
    for key, (label, _names) in USB_CAMERAS.items():
        sources.append({"id": f"usb:{key}", "label": f"USB {label}"})
    return sources


def _open_source(source_id: str, buffer: FrameBuffer):
    kind, _, name = source_id.partition(":")
    if kind == "ros" and name.startswith("/"):
        from .ros_bridge import get_bridge

        if not get_bridge().ready:
            raise RuntimeError("ROS 未连接（roscore 未运行？）")
        return RosImageSource(name, buffer)
    if kind == "usb" and name in USB_CAMERAS:
        return UsbSource(name, buffer)
    raise ValueError(f"未知相机来源 {source_id!r}")


# ---------------------------------------------------------------------------
# Camera service
# ---------------------------------------------------------------------------


def _placeholder(lines: List[str]) -> np.ndarray:
    image = np.full((360, 640, 3), 40, dtype=np.uint8)
    return overlay.draw_banner(image, lines)


class CameraService:
    """Owns the active source, detector and annotated JPEG output."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.settings: Dict[str, Any] = {
            "source": "ros:/usb_cam/image_raw",
            "detector": "__none__",
            "conf": None,   # None → detectors.yaml threshold
            "iou": None,
            "fps": 10.0,
        }
        self._version = 0             # bumped on every settings change
        self._clients = 0
        self._closing = False
        self._thread: Optional[threading.Thread] = None
        self._raw = FrameBuffer()
        self.output = FrameBuffer()   # JPEG bytes
        self._detectors: Dict[Tuple, Any] = {}
        self.status: Dict[str, Any] = {"running": False, "error": None, "detections": [], "report": None}

    # -- settings -----------------------------------------------------------

    def configure(self, **changes: Any) -> Dict[str, Any]:
        with self._lock:
            for key, value in changes.items():
                if key in self.settings and value is not None:
                    self.settings[key] = value
            for key in ("conf", "iou"):  # explicit "auto" resets to yaml default
                if changes.get(key) == "auto":
                    self.settings[key] = None
            self._version += 1
            return dict(self.settings)

    def _detector(self, name: str, conf, iou):
        from .detector_core import create_detector

        key = (name, conf, iou)
        if key not in self._detectors:
            self._detectors.clear()   # keep one model in memory on the robot
            self._detectors[key] = create_detector(name, conf, iou)
        return self._detectors[key]

    # -- streaming ----------------------------------------------------------

    def stream(self, boundary: bytes = b"frame") -> Iterator[bytes]:
        """MJPEG generator; one per HTTP client (iterated in a worker thread)."""
        with self._lock:
            self._clients += 1
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="camera-pipeline", daemon=True)
                self._thread.start()
        seq = -1
        try:
            while not self._closing:
                seq, jpeg = self.output.wait(seq, timeout=1.0)
                if jpeg is None:
                    continue
                yield (b"--" + boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        finally:
            with self._lock:
                self._clients -= 1

    def _run(self) -> None:
        source = None
        source_id = None
        version = -1
        raw_seq = -1
        last_frame = time.monotonic()
        last_infer = 0.0
        last_open = 0.0
        try:
            while True:
                with self._lock:
                    if self._clients <= 0 or self._closing:
                        # Cleared under the lock so a client arriving while
                        # the source closes starts a fresh worker.
                        self._thread = None
                        break
                    settings = dict(self.settings)
                    current = self._version
                self.status["running"] = True

                if current != version:
                    version = current
                    self.status.update(error=None, detections=[], report=None)
                    if settings["source"] != source_id and source is not None:
                        source.close()
                        source = None
                    source_id = settings["source"]
                    last_open = 0.0   # user re-applied: retry immediately

                if source is None:
                    # (Re)open; a failed source is retried every 2 s.
                    if time.monotonic() - last_open < 2.0:
                        time.sleep(0.2)
                        continue
                    last_open = time.monotonic()
                    try:
                        source = _open_source(source_id, self._raw)
                        last_frame = time.monotonic()
                        self.status["error"] = None
                    except Exception as exc:
                        if self.status.get("error") != str(exc):  # log each new error once
                            _LOG.warning("camera source %s failed: %s", source_id, exc)
                        self.status["error"] = str(exc)
                        self._emit(_placeholder([f"Source error: {source_id}", str(exc)[:70]]))
                        continue

                seq, frame = self._raw.wait(raw_seq, timeout=0.5)
                now = time.monotonic()
                if seq == raw_seq or frame is None:
                    if now - last_frame > 3.0:
                        self._emit(_placeholder([f"No frames from {source_id}"]))
                    continue
                raw_seq, last_frame = seq, now
                # Throttle to the requested FPS (inference is the expensive part).
                period = 1.0 / max(1.0, float(settings["fps"]))
                if now - last_infer < period:
                    continue
                last_infer = now
                self._emit(self._annotate(frame.copy(), settings))
        except Exception as exc:  # never leave clients hanging silently
            _LOG.exception("camera pipeline crashed")
            self.status["error"] = str(exc)
            self._emit(_placeholder(["Camera pipeline error", str(exc)[:70]]))
        finally:
            with self._lock:
                if self._thread is threading.current_thread():
                    self._thread = None   # crashed: let the next client restart
            if source is not None:
                source.close()
            self.status["running"] = False

    def _annotate(self, frame: np.ndarray, settings: Dict[str, Any]) -> np.ndarray:
        from .detector_core import NO_DETECTOR, TRASH_PIPELINE

        name = settings["detector"]
        if name == NO_DETECTOR:
            return frame
        start = time.monotonic()
        try:
            detector = self._detector(name, settings["conf"], settings["iou"])
            if name == TRASH_PIPELINE:
                report = detector.analyze(frame)
                overlay.draw_trash_pipeline(frame, report)
                self.status.update(report=report, detections=[], error=None)
                summary = (f"Trash: {report.get('placement_status')} bins={report['bin_count']} "
                           f"waste={report['waste_count']}")
            else:
                objects = detector.detect(frame)
                dets = [{"label": o.label, "box": [int(v) for v in o.box], "confidence": round(float(o.confidence), 3),
                         "class_id": detector.class_names.index(o.label)} for o in objects]
                overlay.draw_boxes(frame, dets)
                self.status.update(detections=dets, report=None, error=None)
                summary = f"{name}: {len(dets)} objects"
        except Exception as exc:
            self.status["error"] = f"{name}: {exc}"
            return overlay.draw_banner(frame, [f"Detector error: {name}", str(exc)[:70]])
        elapsed = (time.monotonic() - start) * 1000
        return overlay.draw_banner(frame, [f"{summary} | {elapsed:.0f} ms"])

    def _emit(self, image: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.output.put(encoded.tobytes())

    def close(self) -> None:
        """Server shutdown: end every MJPEG response and the worker."""
        self._closing = True

    # -- snapshot -----------------------------------------------------------

    def snapshot(self) -> Path:
        """Save the latest annotated frame; requires an open preview."""
        if not self.status.get("running"):
            raise RuntimeError("请先打开相机预览再保存快照")
        _seq, jpeg = self.output.latest()
        if jpeg is None:
            raise RuntimeError("还没有可保存的画面")
        config.ensure_runtime_dirs()
        path = config.SNAPSHOT_DIR / time.strftime("snap_%Y%m%d_%H%M%S.jpg")
        n = 1
        while path.exists():
            path = config.SNAPSHOT_DIR / time.strftime(f"snap_%Y%m%d_%H%M%S_{n}.jpg")
            n += 1
        path.write_bytes(jpeg)
        return path


_service: Optional[CameraService] = None


def get_camera() -> CameraService:
    global _service
    if _service is None:
        _service = CameraService()
    return _service
