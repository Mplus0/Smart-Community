"""Unified camera pipeline for the workbench.

    source (ROS image topic / USB camera)
        → FrameBuffer (latest raw frame)
        → detector (ONNX, optional)
        → overlay drawing
        → JPEG output buffer
        → MJPEG stream / snapshot

A single :class:`CameraService` serves every browser: the worker thread
runs only while at least one MJPEG client is connected, and closes the
camera when the last one leaves. The onboard camera can be opened
directly at /dev/video0 or through its ROS topic when usb_cam owns it.
"""

from __future__ import annotations

import logging
import re
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

import cv2
import numpy as np

from . import config, overlay
from .detector_core import (
    BUILDING_FIRE_PREVIEW, METER_PREVIEW, NO_DETECTOR, PLATE_PREVIEW, THERMAL_PREVIEW,
    TRASH_PIPELINE, PreviewOCR, building_name, create_detector,
)

_LOG = logging.getLogger("infra.camera")
THERMAL_IMAGE_TOPIC = "/building_d_temperature/image"
METER_IMAGE_TOPIC = "/ugreen_4k/image_raw"
MAIN_CAMERA_SOURCE = "ros:/usb_cam/image_raw"
PREVIEW_SOURCES = {THERMAL_PREVIEW: f"ros:{THERMAL_IMAGE_TOPIC}", METER_PREVIEW: f"ros:{METER_IMAGE_TOPIC}"}
SOURCE_LABELS = {THERMAL_IMAGE_TOPIC: "楼宇 D 红外热图", METER_IMAGE_TOPIC: "UGREEN 4K 仪表相机"}


# ---------------------------------------------------------------------------
# Frame buffer
# ---------------------------------------------------------------------------


class FrameBuffer:
    """Latest item plus a sequence number; readers wait for a newer one."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._item: Any = None
        self.updated = 0.0
        self._seq = 0

    def put(self, item: Any) -> None:
        with self._cond:
            self.updated = time.monotonic()
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

def capture_devices():
    """Read udev capture capabilities without opening any camera."""
    for node in sorted(Path("/sys/class/video4linux").glob("video[0-9]*")):
        device = Path("/dev") / node.name
        try:
            if not device.is_char_device():
                continue
            name = (node / "name").read_text().strip()
            number = (node / "dev").read_text().strip()
            record = Path("/run/udev/data", "c" + number)
            caps = next((line for line in record.read_text(errors="replace").splitlines()
                         if line.startswith("E:ID_V4L_CAPABILITIES=")), "") if record.exists() else ""
            if caps:
                if ":capture:" not in caps:
                    continue
            elif (node / "index").read_text().strip() != "0":
                continue
        except OSError:
            continue
        yield device, name


def open_usb_camera(source: str, width: int = 1280, height: int = 720, fps: int = 30):
    for device, name in capture_devices():
        model = name.lower().replace(" ", "")
        # Accept older clients that select a camera by model.
        alias = ((source == "orbbec" and "orbbec" in model and "rgb" in model)
                 or (source == "dji" and any(k in model for k in ("osmoaction5", "djipocket3"))))
        if source != device.name and not alias:
            continue
        camera = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            camera.set(cv2.CAP_PROP_FPS, fps)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return camera
        camera.release()
        raise RuntimeError(f"无法打开 {device}，请检查权限或占用情况；若由 ROS 采集，请选择对应图像话题")
    raise RuntimeError(f"未找到相机 {source}，请检查连接")


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
        buff_size = 2 ** 25 if topic == METER_IMAGE_TOPIC else 2 ** 24
        self._sub = rospy.Subscriber(topic, rospy.AnyMsg, self._cb, queue_size=1, buff_size=buff_size)

    def _cb(self, raw) -> None:
        try:
            msg = self._image_cls().deserialize(raw._buff)
            self._buffer.put(self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
        except Exception as exc:
            _LOG.warning("image conversion failed: %s", exc)

    def close(self) -> None:
        self._sub.unregister()


class ThermalImageSource(RosImageSource):
    """Pair the existing building D preview with its live ROS status text."""

    def __init__(self, buffer: FrameBuffer) -> None:
        import rospy
        from std_msgs.msg import String

        super().__init__(THERMAL_IMAGE_TOPIC, buffer)
        self._result = FrameBuffer()
        self._result_sub = rospy.Subscriber("/building_d_temperature/result", String,
                                           lambda message: self._result.put((time.monotonic(), message.data)),
                                           queue_size=1)

    def report(self) -> Dict[str, Any]:
        _, sample = self._result.latest()
        message = sample[1] if sample and time.monotonic() - sample[0] < 3.0 else ""
        return {"kind": "thermal", "message": message}

    def close(self) -> None:
        self._result_sub.unregister()
        super().close()


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
    """ROS image topics currently published plus connected capture devices."""
    sources = []
    try:
        import rospy

        for topic, msg_type in sorted(rospy.get_published_topics()):
            if msg_type == "sensor_msgs/Image" and topic != "/thermal_camera/temperature":
                sources.append({"id": f"ros:{topic}", "label": SOURCE_LABELS.get(topic, f"ROS {topic}")})
    except Exception as exc:
        _LOG.debug("cannot list ROS topics: %s", exc)
    for device, name in capture_devices():
        sources.append({"id": f"usb:{device.name}", "label": f"{name}（{device}）"})
    return sources


def _open_source(source_id: str, buffer: FrameBuffer):
    kind, _, name = source_id.partition(":")
    if kind == "ros" and name.startswith("/"):
        from .ros_bridge import get_bridge

        if not get_bridge().ready:
            raise RuntimeError("ROS 未连接（roscore 未运行？）")
        if name == THERMAL_IMAGE_TOPIC:
            return ThermalImageSource(buffer)
        return RosImageSource(name, buffer)
    if kind == "usb":
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
            "source": MAIN_CAMERA_SOURCE,
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
        self._ocr = PreviewOCR()
        self.status: Dict[str, Any] = {"running": False, "error": None, "detections": [], "report": None}

        self._raw_source = None
        self._record_thread = None
        self._record_stop = threading.Event()
        self.status.update(recording=False, video_path="", record_error="")

    def _capture_frame(self):
        _, frame = self._raw.latest()
        if (not self.status["running"] or self.settings["source"] != self._raw_source
                or frame is None or time.monotonic() - self._raw.updated > 2):
            raise RuntimeError("请先选择视频源并等待实时画面")
        return frame

    @staticmethod
    def _reserve_file(directory, name, suffix):
        folder = Path(directory).expanduser() if directory else config.RUNTIME_DIR / "captures"
        if not folder.is_absolute():
            raise ValueError("请输入小车上的绝对路径或 ~/ 路径")
        stem = name.strip() or time.strftime("capture_%Y%m%d_%H%M%S")
        if stem.lower().endswith(suffix):
            stem = stem[:-len(suffix)]
        if not re.fullmatch(r"[\w .-]{1,100}", stem) or stem.startswith("."):
            raise ValueError("文件名可包含中英文、数字、空格、横线或下划线")
        folder.mkdir(parents=True, exist_ok=True)
        for n in range(10000):
            path = folder / ((stem if not n else f"{stem}_{n}") + suffix)
            try:
                path.touch(exist_ok=False)
                return path
            except FileExistsError:
                continue
        raise ValueError("同名文件过多，请更换名称")

    def collect(self, directory, name, video):
        with self._lock:
            frame = self._capture_frame()
            if video and self.status["recording"]:
                raise RuntimeError("已有录像进行中")
            path = self._reserve_file(directory, name, ".avi" if video else ".jpg")
            try:
                if not video:
                    if not cv2.imwrite(str(path), frame):
                        raise RuntimeError("图片写入失败")
                    return path
                fps = float(self.settings["fps"])
                size = frame.shape[1::-1]
                writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, size)
                if not writer.isOpened():
                    writer.release()
                    raise RuntimeError("无法创建 MJPEG 视频")
                writer.write(frame)
                self._record_stop.clear()
                self.status.update(recording=True, video_path=str(path), record_error="")
                self._record_thread = threading.Thread(target=self._record, args=(writer, path, fps, size), daemon=True)
                self._record_thread.start()
                return path
            except Exception:
                path.unlink(missing_ok=True)
                raise

    def _record(self, writer, path, fps, size):
        next_frame = time.monotonic()
        next_disk_check = next_frame
        try:
            while True:
                next_frame += 1 / fps
                if self._record_stop.wait(max(0, next_frame - time.monotonic())):
                    break
                frame = self._capture_frame()
                if frame.shape[1::-1] != size:
                    raise RuntimeError("视频源分辨率改变")
                now = time.monotonic()
                if now >= next_disk_check:
                    if shutil.disk_usage(path.parent).free < 32 * 1024 * 1024:
                        raise RuntimeError("磁盘空间不足")
                    next_disk_check = now + 1
                writer.write(frame)
                next_frame = max(next_frame, now)
        except Exception as exc:
            self.status["record_error"] = str(exc)
        finally:
            writer.release()
            with self._lock:
                self.status["recording"] = False

    def stop_recording(self):
        with self._lock:
            thread = self._record_thread
            path = self.status["video_path"]
            self._record_stop.set()
        if thread:
            thread.join()
        return path

    # -- settings -----------------------------------------------------------

    def configure(self, **changes: Any) -> Dict[str, Any]:
        with self._lock:
            required_source = PREVIEW_SOURCES.get(changes.get("detector"))
            if required_source:
                changes["source"] = required_source
            elif changes.get("detector") in (PLATE_PREVIEW, BUILDING_FIRE_PREVIEW):
                source = changes.get("source") or self.settings["source"]
                if source in PREVIEW_SOURCES.values() or source.startswith("ros:/thermal_camera/"):
                    changes["source"] = MAIN_CAMERA_SOURCE
            elif changes.get("source") and changes.get("detector") is None:
                active_detector = self.settings["detector"]
                active_source = PREVIEW_SOURCES.get(active_detector)
                if active_source and changes["source"] != active_source:
                    changes["detector"] = NO_DETECTOR
                if active_detector in (PLATE_PREVIEW, BUILDING_FIRE_PREVIEW) and (
                    changes["source"] == PREVIEW_SOURCES[THERMAL_PREVIEW]
                    or changes["source"].startswith("ros:/thermal_camera/")
                ):
                    changes["detector"] = NO_DETECTOR
            if self.status["recording"] and changes.get("source") not in (None, self.settings["source"]):
                raise RuntimeError("请先停止录像，再切换视频源")
            for key, value in changes.items():
                if key in self.settings and value is not None:
                    self.settings[key] = value
            for key in ("conf", "iou"):  # explicit "auto" resets to yaml default
                if changes.get(key) == "auto":
                    self.settings[key] = None
            self._version += 1
            return dict(self.settings)

    def _detector(self, name: str, conf, iou):
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
                    if (self._clients <= 0 and not self.status["recording"]) or self._closing:
                        # Cleared under the lock so a client arriving while
                        # the source closes starts a fresh worker.
                        self._thread = None
                        break
                    settings = dict(self.settings)
                    current = self._version
                self.status["running"] = True

                if current != version:
                    version = current
                    self._ocr.reset()
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
                        self._raw = FrameBuffer()
                        raw_seq = -1
                        source = _open_source(source_id, self._raw)
                        self._raw_source = source_id
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
                        self._ocr.reset()
                        self.status.update(error=f"未收到 {source_id} 的图像", detections=[], report=None)
                        self._emit(_placeholder([f"No frames from {source_id}"]))
                    continue
                raw_seq, last_frame = seq, now
                # Throttle to the requested FPS (inference is the expensive part).
                period = 1.0 / max(1.0, float(settings["fps"]))
                if now - last_infer < period:
                    continue
                last_infer = now
                if settings["detector"] == THERMAL_PREVIEW:
                    self.status.update(report=source.report(), detections=[], error=None)
                    self._emit(frame)
                else:
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
        name = settings["detector"]
        if name == NO_DETECTOR:
            self.status.update(detections=[], report=None, error=None)
            return frame
        start = time.monotonic()
        try:
            if name == METER_PREVIEW:
                annotated, report = self._detector(name, None, None).analyze(frame)
                self.status.update(report=report, detections=[], error=None)
                return annotated
            if name == PLATE_PREVIEW:
                reading = self._ocr.sample(frame, '/license_plate_ocr_preview')
                report = dict(kind='plate', pending=reading is None, plate='', confidence=None, ocr_error='')
                if reading is not None:
                    report.update(plate=next(iter(reading['texts']), ''),
                                  confidence=next(iter(reading['scores']), None), ocr_error=reading['error'])
                self.status.update(report=report, detections=[], error=None)
                return frame
            detector_name = 'fire_detector' if name == BUILDING_FIRE_PREVIEW else name
            detector = self._detector(detector_name, settings["conf"], settings["iou"])
            if name == TRASH_PIPELINE:
                report = detector.analyze(frame)
                overlay.draw_trash_pipeline(frame, report)
                self.status.update(report=report, detections=[], error=None)
                summary = (f"Trash: {report.get('placement_status')} bins={report['bin_count']} "
                           f"waste={report['waste_count']}")
            else:
                objects = detector.detect(frame)
                if name == BUILDING_FIRE_PREVIEW:
                    # Send a clean frame to OCR before drawing boxes on the preview.
                    fires = [obj for obj in objects if obj.label == 'fire']
                    reading = self._ocr.sample(frame, '/building_name_ocr_preview') if fires else None
                    if not fires:
                        self._ocr.reset()
                    report = dict(kind='building_fire', fire_count=len(fires),
                                  pending=bool(fires) and reading is None,
                                  building=building_name(reading['texts']) if reading else '',
                                  ocr_error=reading['error'] if reading else '')
                    objects = fires
                else:
                    report = None
                dets = [{"label": o.label, "box": [int(v) for v in o.box], "confidence": round(float(o.confidence), 3),
                         "class_id": detector.class_names.index(o.label)} for o in objects]
                overlay.draw_boxes(frame, dets)
                self.status.update(detections=dets, report=report, error=None)
                summary = f"{detector_name}: {len(dets)} objects"
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
        self.stop_recording()

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
