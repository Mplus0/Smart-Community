#!/usr/bin/env python3
"""从 ROS 图像话题或 USB 相机采集图像，支持在网页切换相机。"""

import argparse
import json
import os
import re
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>小车图像采集</title>
<style>
body{margin:0;background:#10151d;color:#eaf0f7;font:16px system-ui,sans-serif}
main{max-width:960px;margin:28px auto;padding:0 20px}h1{font-size:24px}
.view{background:#05080c;border-radius:12px;overflow:hidden;aspect-ratio:16/9}
img{display:block;width:100%;height:100%;object-fit:contain}
img[hidden]{display:none}
.bar{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin:16px 0}
button{background:#263443;color:#eaf0f7;border:0;border-radius:8px;padding:10px 16px;font:inherit;cursor:pointer}
button[aria-pressed="true"],#capture{background:#5ee0ad;color:#09281c}
button:disabled{opacity:.5;cursor:default}p{line-height:1.7;overflow-wrap:anywhere}
#status,#folder{color:#a9b9cd}#result{min-height:24px}
</style>
<main>
<h1>小车图像采集</h1>
<div class="bar" role="group" aria-label="选择相机">
 <button data-camera="orbbec" aria-pressed="false">Orbbec 彩色</button>
 <button data-camera="dji" aria-pressed="false">DJI Action 5 Pro</button>
 <button data-camera="ros" aria-pressed="false">ROS 图像话题</button>
</div>
<p id="status" role="status">正在连接相机…</p>
<div class="view"><img id="preview" alt="相机实时画面" hidden></div>
<div class="bar"><button id="capture" disabled>拍照保存</button><span>本次已保存 <b id="count">0</b> 张</span></div>
<p id="result" role="status"></p>
<p id="quality"></p>
<p>预览和保存均应用当前软件变焦及锐化。放大会损失细节，锐化不能恢复模糊细节。</p>
<p id="folder"></p>
</main>
<script>
const preview = document.getElementById('preview');
const capture = document.getElementById('capture');
const statusText = document.getElementById('status');
const result = document.getElementById('result');
const choices = [...document.querySelectorAll('[data-camera]')];
let current = null;
let busy = false;
let polling = false;

function clearPreview() {
 preview.removeAttribute('src');
 preview.hidden = true;
}
preview.onerror = clearPreview;

function controls() {
 capture.disabled = busy || !current || !current.camera_ready;
 choices.forEach(button => {
  button.disabled = busy;
  button.setAttribute('aria-pressed', String(current?.camera === button.dataset.camera));
 });
}

function show(data) {
 if (data.generation !== current?.generation || !data.camera_ready) clearPreview();
 current = data;
 statusText.textContent = data.label + ' · ' + (data.camera_ready ? data.resolution : data.message);
 document.getElementById('count').textContent = data.count;
 document.getElementById('folder').textContent = data.folder;
 document.getElementById('quality').textContent = '软件变焦：' + data.zoom + ' 倍；锐化：' + data.sharpness
  + '；预览 JPEG 质量：' + data.preview_quality + '；保存 JPEG 质量：' + data.jpeg_quality;
 if (data.camera_ready && !preview.hasAttribute('src')) {
  preview.src = '/stream.mjpg?generation=' + data.generation;
  preview.hidden = false;
 }
 controls();
}

async function request(path, payload) {
 const options = {cache: 'no-store'};
 if (payload) Object.assign(options, {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify(payload)
 });
 const response = await fetch(path, options);
 const data = await response.json();
 if (!response.ok) throw Error(data.error);
 return data;
}

async function poll() {
 if (busy || polling) return;
 polling = true;
 try {
  const data = await request('/status');
  if (!busy && (!current || data.generation >= current.generation)) show(data);
 } catch (error) {
  if (!busy) {
   clearPreview();
   current = null;
   statusText.textContent = '网页连接断开，正在重试…';
   controls();
  }
 } finally {
  polling = false;
 }
}

async function act(action) {
 busy = true;
 controls();
 try {
  await action();
 } catch (error) {
  result.textContent = error.message;
 } finally {
  busy = false;
  controls();
  poll();
 }
}

choices.forEach(button => button.onclick = () => act(async () => {
 clearPreview();
 result.textContent = '';
 statusText.textContent = '正在切换相机…';
 show(await request('/camera', {camera: button.dataset.camera}));
}));
capture.onclick = () => act(async () => {
 const data = await request('/capture', {generation: current.generation});
 result.textContent = '已保存 ' + data.filename;
 document.getElementById('count').textContent = data.count;
});

poll();
setInterval(poll, 700);
</script></html>""".encode("utf-8")

CAMERA_LABELS = {"ros": "ROS 图像话题", "orbbec": "Orbbec 彩色", "dji": "DJI Action 5 Pro"}


def processing_settings(args, camera):
    zoom = args.zoom if args.zoom is not None else (2.0 if camera == "dji" else 1.0)
    sharpness = args.sharpness if args.sharpness is not None else (0.5 if camera == "dji" else 0.0)
    return zoom, sharpness


class CameraSource:
    def __init__(self, args, stopped):
        self.args = args
        self.stopped = stopped
        self.condition = threading.Condition()
        self.camera = args.camera
        self.generation = 0
        self.frame = None
        self.arrival = 0.0
        self.sequence = 0
        self.message = "等待相机画面"
        self.ros_started = False
        self.ros = None
        self.subscriber = None
        self.worker = threading.Thread(target=self.run_usb, daemon=True)
        self.worker.start()
        if self.camera == "ros":
            self.start_ros()

    def select(self, camera):
        with self.condition:
            if self.camera != camera:
                self.camera = camera
                self.generation += 1
                self.frame = None
                self.message = "等待相机画面"
                self.condition.notify_all()
        if camera == "ros":
            self.start_ros()

    def start_ros(self):
        with self.condition:
            if self.ros_started:
                return
            self.ros_started = True
        threading.Thread(target=self.run_ros, daemon=True).start()

    def run_ros(self):
        try:
            import rospy
            from cv_bridge import CvBridge, CvBridgeError
            from sensor_msgs.msg import Image

            self.ros = rospy
            rospy.init_node("collect_dataset", anonymous=True, disable_signals=True)
            if self.stopped.is_set():
                return
            bridge = CvBridge()

            def receive(message):
                with self.condition:
                    if self.camera != "ros":
                        return
                    generation = self.generation
                try:
                    image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                except CvBridgeError as exc:
                    self.set_message(generation, str(exc))
                    return
                self.publish(generation, image)

            self.subscriber = rospy.Subscriber(
                self.args.topic, Image, receive, queue_size=1, buff_size=32 * 1024 * 1024)
        except Exception as exc:
            with self.condition:
                self.ros_started = False
                if self.camera == "ros":
                    self.message = "ROS 连接失败：{}".format(exc)

    def publish(self, generation, image):
        import cv2

        with self.condition:
            if generation != self.generation:
                return
            zoom, sharpness = processing_settings(self.args, self.camera)
        # Process once before caching so preview and saved photos match.
        if zoom > 1.0:
            height, width = image.shape[:2]
            crop_width = max(1, round(width / zoom))
            crop_height = max(1, round(height / zoom))
            left, top = (width - crop_width) // 2, (height - crop_height) // 2
            image = cv2.resize(
                image[top:top + crop_height, left:left + crop_width],
                (width, height), interpolation=cv2.INTER_LINEAR)
        if sharpness > 0.0:
            blurred = cv2.GaussianBlur(image, (0, 0), 1.0)
            image = cv2.addWeighted(image, 1.0 + sharpness, blurred, -sharpness, 0)
        with self.condition:
            if generation == self.generation:
                self.frame = image
                self.arrival = time.monotonic()
                self.sequence += 1
                self.message = "等待相机新帧"
                self.condition.notify_all()

    def set_message(self, generation, message):
        with self.condition:
            if generation == self.generation:
                self.message = message
                self.frame = None
                self.condition.notify_all()

    def run_usb(self):
        from camera_web_preview import open_usb_camera
        while not self.stopped.is_set():
            with self.condition:
                if self.camera == "ros":
                    self.condition.wait(timeout=0.5)
                    continue
                camera_name, generation = self.camera, self.generation
            device = None
            try:
                device, path = open_usb_camera(camera_name)
                while not self.stopped.is_set() and generation == self.generation:
                    ok, image = device.read()
                    if not ok:
                        raise RuntimeError("无法从 {} 读取画面，正在重试".format(path))
                    self.publish(generation, image)
            except Exception as exc:
                self.set_message(generation, str(exc))
            finally:
                if device is not None:
                    device.release()
            with self.condition:
                if generation == self.generation and not self.stopped.is_set():
                    self.condition.wait(timeout=1.0)

    def status(self):
        with self.condition:
            ready = self.frame is not None and time.monotonic() - self.arrival <= 2
            zoom, sharpness = processing_settings(self.args, self.camera)
            return {
                "camera": self.camera,
                "label": CAMERA_LABELS[self.camera],
                "generation": self.generation,
                "camera_ready": ready,
                "resolution": "{} × {}".format(self.frame.shape[1], self.frame.shape[0]) if ready else "",
                "message": self.message,
                "zoom": zoom, "sharpness": sharpness,
                "preview_quality": self.args.preview_quality,
                "jpeg_quality": self.args.jpeg_quality,
            }

    def image(self, generation=None, after=None, timeout=0):
        with self.condition:
            if generation is None:
                generation = self.generation
            deadline = time.monotonic() + timeout
            while not self.stopped.is_set():
                if generation != self.generation:
                    raise RuntimeError("相机已切换，请等待新画面。")
                remaining = deadline - time.monotonic()
                if (self.frame is not None and time.monotonic() - self.arrival <= 2
                        and (after is None or self.sequence != after)):
                    return self.frame, self.sequence, self.camera
                if remaining <= 0:
                    break
                self.condition.wait(timeout=remaining)
            raise RuntimeError("未收到新鲜相机图像。")

    def close(self):
        with self.condition:
            self.condition.notify_all()
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.ros.signal_shutdown("采集结束")
        self.worker.join(timeout=3)


def start_web_preview(args, source, save, get_status, change_camera, stopped):
    import cv2

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *values):
            pass

        def reply(self, status, content_type, data):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def json_reply(self, status, data):
            self.reply(status, "application/json; charset=utf-8",
                       json.dumps(data, ensure_ascii=False).encode("utf-8"))

        def jpeg(self, image):
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, args.preview_quality])
            if not ok:
                raise RuntimeError("预览图像编码失败")
            return encoded.tobytes()

        def do_GET(self):
            url = urlsplit(self.path)
            try:
                if url.path == "/":
                    self.reply(200, "text/html; charset=utf-8", PAGE)
                elif url.path == "/status":
                    self.json_reply(200, get_status())
                elif url.path == "/snapshot.jpg":
                    try:
                        data = self.jpeg(source.image()[0])
                    except (RuntimeError, cv2.error) as exc:
                        self.json_reply(503, {"error": str(exc)})
                    else:
                        self.reply(200, "image/jpeg", data)
                elif url.path == "/stream.mjpg":
                    generation = source.status()["generation"]
                    requested = parse_qs(url.query).get("generation")
                    if requested and requested[0] != str(generation):
                        self.json_reply(409, {"error": "相机已切换"})
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.connection.settimeout(5)
                    sequence = None
                    while not stopped.is_set():
                        try:
                            image, sequence, _ = source.image(generation, sequence, timeout=0.5)
                        except RuntimeError:
                            # 断开旧画面，等浏览器按新相机的状态重连。
                            break
                        data = self.jpeg(image)
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(data)).encode("ascii") + b"\r\n\r\n" + data + b"\r\n")
                        stopped.wait(0.1)
                else:
                    self.reply(404, "text/plain; charset=utf-8", b"Not found")
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def do_POST(self):
            path = urlsplit(self.path).path
            try:
                if path not in ("/camera", "/capture"):
                    self.reply(404, "text/plain", b"Not found")
                    return
                if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                    self.json_reply(415, {"error": "请使用 application/json 请求"})
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 1024:
                        raise ValueError
                    self.connection.settimeout(5)
                    data = json.loads(self.rfile.read(size))
                    if not isinstance(data, dict):
                        raise ValueError
                except (ValueError, TimeoutError):
                    self.json_reply(400, {"error": "无效请求"})
                    return
                if path == "/camera":
                    camera = data.get("camera")
                    if not isinstance(camera, str) or camera not in CAMERA_LABELS:
                        self.json_reply(400, {"error": "未知相机"})
                        return
                    self.json_reply(200, change_camera(camera))
                else:
                    generation = data.get("generation")
                    if generation is not None and type(generation) is not int:
                        self.json_reply(400, {"error": "无效相机状态"})
                        return
                    try:
                        saved = save(generation)
                    except (OSError, RuntimeError, cv2.error) as exc:
                        self.json_reply(503, {"error": str(exc)})
                    else:
                        self.json_reply(200, saved)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

    server = ThreadingHTTPServer((args.web_host, args.web_port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="数据集名称，例如 person 或 trashbin")
    parser.add_argument("--camera", choices=CAMERA_LABELS, default="ros",
                        help="启动时使用的相机，默认 ros。网页可切换。")
    parser.add_argument("--topic", default="/usb_cam/image_raw", help="ROS 相机 Image 话题")
    parser.add_argument("--datasets-dir", type=Path,
                        default=Path("/home/mowen/workspace/yolo_train/datasets/dashboard").expanduser(),
                        help="数据集根目录，默认 robot_ws/datasets")
    parser.add_argument("--no-preview", action="store_true", help="关闭桌面窗口，SSH 回车拍照")
    parser.add_argument("--web-port", type=int, default=0, help="网页端口，例如 8080。默认关闭。")
    parser.add_argument("--web-host", default="127.0.0.1", help="网页监听地址，默认仅本机")
    parser.add_argument("--web-only", action="store_true", help="仅使用网页预览和拍照")
    parser.add_argument("--zoom", type=float, default=None,
                        help="软件变焦 1～2；默认 DJI 为 2，其他相机为 1")
    parser.add_argument("--sharpness", type=float, default=None,
                        help="软件锐化 0～2，0 关闭；默认 DJI 为 0.5，其他为 0")
    parser.add_argument("--preview-quality", type=int, default=95,
                        help="网页预览 JPEG 质量 1～100，默认 95")
    parser.add_argument("--jpeg-quality", type=int, default=95,
                        help="保存 JPEG 质量 1～100，默认 95；100 仍不是无损")
    args = parser.parse_args()
    if (not args.name.strip() or args.name in (".", "..")
            or any(c in args.name for c in '/\\<>:"|?*')
            or any(ord(c) < 32 for c in args.name)):
        parser.error("数据集名称不能为空，也不能包含路径分隔符或非法文件名字符")
    if not 0 <= args.web_port <= 65535:
        parser.error("网页端口必须在 1～65535 范围内，0 表示关闭")
    if args.web_only and not args.web_port:
        parser.error("--web-only 需要同时指定 --web-port")
    if args.zoom is not None and not 1.0 <= args.zoom <= 2.0:
        parser.error("--zoom 必须在 1～2 范围内")
    if args.sharpness is not None and not 0.0 <= args.sharpness <= 2.0:
        parser.error("--sharpness 必须在 0～2 范围内")
    if not 1 <= args.preview_quality <= 100 or not 1 <= args.jpeg_quality <= 100:
        parser.error("JPEG 质量必须在 1～100 范围内")
    if args.web_only:
        args.no_preview = True
    return args


def prepare_directories(args):
    dataset = args.datasets_dir.expanduser().resolve() / args.name
    (dataset / "images").mkdir(parents=True, exist_ok=True)
    return dataset


def main():
    args = parse_args()
    import cv2

    if not args.no_preview:
        no_gui = re.search(r"^\s*GUI:\s*NONE\s*$", cv2.getBuildInformation(), re.M)
        no_display = not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if no_gui or no_display:
            print("当前环境没有可用桌面预览，改用终端采集模式。", flush=True)
            args.no_preview = True

    folder = prepare_directories(args) / "images"
    stopped = threading.Event()
    save_lock = threading.Lock()
    source = CameraSource(args, stopped)
    count = 0
    server = None
    window_created = False

    def save(generation=None, timeout=0, after=None):
        nonlocal count
        with save_lock:
            image, sequence, camera = source.image(generation, after, timeout)
            filename = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
            path = folder / filename
            if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]):
                raise OSError("图像保存失败：" + str(path))
            count += 1
            print("已保存 [{}] {} {}".format(count, CAMERA_LABELS[camera], path), flush=True)
            return {"filename": filename, "path": str(path), "count": count,
                    "camera": camera, "sequence": sequence}

    def status():
        data = source.status()
        data.update(count=count, folder=str(folder))
        return data

    def change_camera(camera):
        with save_lock:
            source.select(camera)
            return status()

    print("起始相机：{}\n数据集目录：{}".format(CAMERA_LABELS[args.camera], folder), flush=True)
    try:
        if args.web_port:
            server = start_web_preview(args, source, save, status, change_camera, stopped)
            print("网页：http://{}:{}/".format(args.web_host, args.web_port), flush=True)
        if args.web_only:
            print("网页采集模式，按 Ctrl+C 退出。", flush=True)
            stopped.wait()
        elif args.no_preview:
            print("回车采集一张最新图像。输入 q 后回车退出。", flush=True)
            while not stopped.is_set():
                command = input().strip().lower()
                if command == "q":
                    break
                if not command:
                    try:
                        save(timeout=5)
                    except (RuntimeError, OSError, cv2.error) as exc:
                        print("采集失败：{}".format(exc), flush=True)
        else:
            print("预览窗口按空格或 S 保存图像，按 Q 或 Esc 退出。", flush=True)
            window = "Dataset capture"
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            window_created = True
            last_saved = None
            while not stopped.is_set():
                try:
                    image, _, _ = source.image(timeout=0.1)
                except RuntimeError:
                    pass
                else:
                    cv2.imshow(window, image)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                    break
                if key in (ord("s"), ord("S"), 32):
                    try:
                        last_saved = save(after=last_saved)["sequence"]
                    except (RuntimeError, OSError, cv2.error) as exc:
                        print("采集失败：{}".format(exc), flush=True)
    except (KeyboardInterrupt, EOFError):
        pass
    except (OSError, cv2.error) as exc:
        print("采集失败：{}".format(exc), flush=True)
        return 1
    finally:
        stopped.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        source.close()
        if window_created:
            cv2.destroyAllWindows()
        print("采集结束，本次保存 {} 张。".format(count), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
