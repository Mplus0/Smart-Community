"""Camera core classes - extracted from camera_web_preview.py for reuse in infra.

Batch 3 scope:
- Preview class: thread-safe frame buffering and MJPEG encoding
- Camera sources: ROS topic, DJI, Orbbec, thermal
- USB camera discovery and opening
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Optional, Tuple

import cv2


# ---------------------------------------------------------------------------
# Camera discovery and opening
# ---------------------------------------------------------------------------

CAMERAS = {
    "dji": ("DJI Action 5 Pro", ("osmoaction5", "djipocket3")),
    "orbbec": ("Orbbec 深度相机（彩色）", ("orbbec", "rgb")),
}


def open_usb_camera(source: str, width: int = 1920, height: int = 1080, fps: int = 30) -> Tuple:
    """Open a USB camera by name (dji or orbbec) and return (cv2.VideoCapture, device_path)."""
    if source not in CAMERAS:
        raise ValueError(f"Unknown camera source: {source}")

    label, names = CAMERAS[source]
    found = False

    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            name = (node / "name").read_text().lower().replace(" ", "")
            index = (node / "index").read_text().strip()
        except OSError:
            continue

        # Match camera name
        matches = any(part in name for part in names) if source == "dji" else all(
            part in name for part in names
        )
        if not matches or index != "0":
            continue

        found = True
        device = "/dev/" + node.name
        camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            camera.set(cv2.CAP_PROP_FPS, fps)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return camera, device
        camera.release()

    if found:
        raise RuntimeError(
            f"{label} 打不开，请检查相机是否被其他程序占用。"
        )
    if source == "dji":
        raise RuntimeError(
            "未找到 DJI，请检查 USB 连接并在相机上选择「网络摄像头」。"
        )
    raise RuntimeError("未找到 Orbbec 彩色相机，请检查 USB 连接。")


# ---------------------------------------------------------------------------
# Preview: thread-safe frame buffer
# ---------------------------------------------------------------------------


class Preview:
    """Thread-safe MJPEG frame buffer.

    - ``publish(image)`` encodes and stores an OpenCV frame
    - ``capture(camera)`` continuously reads and publishes from a cv2.VideoCapture
    - ``stopped`` Event signals shutdown
    - Internal ``condition`` coordinates encoder and HTTP streamer
    """

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.stopped = threading.Event()
        self.frame: Optional[bytes] = None
        self.sequence: int = 0

    def publish(self, image) -> None:
        """Encode an OpenCV frame to JPEG and update buffer.

        Called by either capture() worker or detector.
        """
        try:
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return
        except cv2.error as exc:
            print(f"Camera preview failed: {exc}", flush=True)
            return

        with self.condition:
            self.frame = encoded.tobytes()
            self.sequence += 1
            self.condition.notify_all()

    def capture(self, camera) -> None:
        """Continuously read frames from cv2.VideoCapture and publish them.

        Blocks until stopped.
        """
        failures = 0
        while not self.stopped.is_set():
            ok, image = camera.read()
            if ok:
                failures = 0
                self.publish(image)
            else:
                failures += 1
                if failures >= 30:
                    print(
                        "Camera stopped providing frames; restart the preview.",
                        flush=True,
                    )
                    self.stopped.set()
                self.stopped.wait(0.1)

    def get_frame(self, last_sequence: int = -1) -> Tuple[int, Optional[bytes]]:
        """Wait for a new frame (with timeout) and return (sequence, frame_bytes).

        Used by HTTP MJPEG streamer.
        """
        with self.condition:
            # Wait for a new frame or timeout
            self.condition.wait_for(
                lambda: self.sequence != last_sequence or self.stopped.is_set(),
                timeout=1.0,
            )
            if self.sequence == last_sequence or self.frame is None:
                return last_sequence, None
            return self.sequence, self.frame
