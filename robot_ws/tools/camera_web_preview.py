#!/usr/bin/env python3
"""Preview ROS or a selected USB camera in a browser (CAMERA=ros/dji/orbbec)."""
import argparse
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

CAMERAS = {
    "dji": ("DJI Action 5 Pro", ("osmoaction5", "djipocket3")),
    "orbbec": ("Orbbec 深度相机（彩色）", ("orbbec", "rgb")),
}


def open_usb_camera(source, width=1920, height=1080, fps=30):
    label, names = CAMERAS[source]
    found = False
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            name = (node / "name").read_text().lower().replace(" ", "")
            index = (node / "index").read_text().strip()
        except OSError:
            continue
        matches = any(part in name for part in names) if source == "dji" else all(
            part in name for part in names)
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
        raise RuntimeError("{} 打不开，请检查相机是否被其他程序占用。".format(label))
    if source == "dji":
        raise RuntimeError("未找到 DJI，请检查 USB 连接并在相机上选择「网络摄像头」。")
    raise RuntimeError("未找到 Orbbec 彩色相机，请检查 USB 连接。")


class Preview:
    def __init__(self):
        self.condition = threading.Condition()
        self.stopped = threading.Event()
        self.frame = None
        self.sequence = 0

    def publish(self, image):
        try:
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return
        except cv2.error as exc:
            print("Camera preview failed: {}".format(exc), flush=True)
            return
        with self.condition:
            self.frame = encoded.tobytes()
            self.sequence += 1
            self.condition.notify_all()

    def capture(self, camera):
        failures = 0
        while not self.stopped.is_set():
            ok, image = camera.read()
            if ok:
                failures = 0
                self.publish(image)
            else:
                failures += 1
                if failures >= 30:
                    print("Camera stopped providing frames; restart the preview.", flush=True)
                    self.stopped.set()
                self.stopped.wait(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=("ros", "dji", "orbbec"),
                        default=os.environ.get("CAMERA", "ros"))
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--topic", default="/usb_cam/image_raw")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", default="0.0.0.0", help="HTTP listen address (default: all interfaces)")
    # Leave ROS remapping arguments available to rospy in ROS mode.
    args, remaining = parser.parse_known_args()
    if args.camera != "ros" and remaining:
        parser.error("unrecognized arguments: " + " ".join(remaining))
    preview = Preview()
    subscriber = camera = worker = None
    rospy = None
    if args.camera == "ros":
        import rospy
        from cv_bridge import CvBridge, CvBridgeError
        from sensor_msgs.msg import Image
        bridge = CvBridge()
        rospy.init_node("camera_web_preview", anonymous=True, disable_signals=True)

        def receive(message):
            try:
                preview.publish(bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"))
            except CvBridgeError as exc:
                rospy.logwarn_throttle(5, "Camera preview failed: %s", exc)

        subscriber = rospy.Subscriber(args.topic, Image, receive,
                                      queue_size=1, buff_size=32 * 1024 * 1024)
        rospy.on_shutdown(preview.stopped.set)
        source = args.topic
    else:
        try:
            camera, device = open_usb_camera(
                args.camera, args.width or 1920, args.height or 1080, args.fps)
        except RuntimeError as exc:
            parser.error(str(exc))
        source = "{} ({}x{}, {} fps requested)".format(
            device, int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)), args.fps)
        worker = threading.Thread(target=preview.capture, args=(camera,), daemon=True)
        worker.start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = (b'<!doctype html><meta name="viewport" content="width=device-width">'
                        b'<title>Camera preview</title><h2>Camera preview</h2>'
                        b'<img src="/stream" style="max-width:100%;height:auto">')
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                sequence = -1
                try:
                    while not preview.stopped.is_set():
                        with preview.condition:
                            preview.condition.wait_for(
                                lambda: preview.sequence != sequence or preview.stopped.is_set(), timeout=1)
                            if preview.sequence == sequence or preview.frame is None:
                                continue
                            sequence, frame = preview.sequence, preview.frame
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n" +
                                         b"Content-Length: " + str(len(frame)).encode() +
                                         b"\r\n\r\n" + frame + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, format, *args):
            pass

    server = None
    try:
        server = ThreadingHTTPServer((args.host, args.port), Handler)
        server.daemon_threads = True
        print("Camera: {}  Listening: {}:{}".format(source, args.host, args.port), flush=True)
        print("Preview: http://<robot-ip>:{}/ (use the robot's LAN IP)".format(args.port), flush=True)
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        preview.stopped.set()
        with preview.condition:
            preview.condition.notify_all()
        if server:
            server.server_close()
        if subscriber:
            subscriber.unregister()
            rospy.signal_shutdown("Preview stopped")
        if worker:
            worker.join(timeout=3)
        if camera and not worker.is_alive():
            camera.release()


if __name__ == "__main__":
    main()
