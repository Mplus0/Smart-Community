#!/usr/bin/env python3
"""从 ROS 相机话题采集原图，支持桌面、SSH 和浏览器预览。"""

import argparse
import json
import os
import re
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>小车图像采集</title>
<style>
body{margin:0;background:#10151d;color:#eaf0f7;font:16px system-ui,sans-serif}
main{max-width:960px;margin:28px auto;padding:0 20px}h1{font-size:24px}
img{display:block;width:100%;min-height:160px;background:#05080c;border-radius:12px}
.bar{display:flex;align-items:center;gap:20px;flex-wrap:wrap;margin:18px 0}
button{background:#5ee0ad;color:#09281c;border:0;border-radius:8px;padding:12px 24px;font:inherit;cursor:pointer}
button:disabled{opacity:.5;cursor:wait}p{line-height:1.7;overflow-wrap:anywhere}
#status{color:#a9b9cd}#result{min-height:28px}small{color:#a9b9cd}
</style><main><h1>小车图像采集</h1>
<p id="status">正在连接相机…</p><img id="preview" src="/stream.mjpg" alt="相机实时画面">
<div class="bar"><button id="capture" disabled>拍照保存</button><span>本次已保存 <b id="count">0</b> 张</span></div>
<p id="result" role="status" aria-live="polite"></p>
<small>预览约 10 帧/秒；拍照保存相机原图。保存位置：</small><p id="folder"></p></main>
<script>
const preview=document.getElementById('preview'), button=document.getElementById('capture');
let saving=false, ready=false;
preview.onerror=()=>setTimeout(()=>{preview.src='/stream.mjpg?t='+Date.now()},1000);
async function status(){
 try{
  const response=await fetch('/status',{cache:'no-store'});
  if(!response.ok)throw Error('无法读取相机状态');
  const data=await response.json(); ready=data.camera_ready;
  document.getElementById('status').textContent=ready?'实时画面已连接':'等待相机图像，请检查相机连接';
  document.getElementById('count').textContent=data.count;
  document.getElementById('folder').textContent=data.folder;
 }catch(error){ready=false;document.getElementById('status').textContent='连接已断开，正在重试…'}
 button.disabled=saving||!ready;
}
button.onclick=async()=>{
 saving=true;button.disabled=true;
 try{
  const response=await fetch('/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  const data=await response.json();if(!response.ok)throw Error(data.error);
  document.getElementById('result').textContent='已保存：'+data.filename;
  document.getElementById('count').textContent=data.count;
 }catch(error){document.getElementById('result').textContent='保存失败：'+error.message}
 finally{saving=false;await status()}
};
status();setInterval(status,1000);
</script></html>""".encode("utf-8")


def start_web_preview(args, get_image, save_image, get_status, stopped):
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

        def jpeg(self):
            ok, encoded = cv2.imencode(".jpg", get_image(),
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError("预览图像编码失败")
            return encoded.tobytes()

        def do_GET(self):
            path = urlsplit(self.path).path
            try:
                if path == "/":
                    self.reply(200, "text/html; charset=utf-8", PAGE)
                elif path == "/status":
                    self.json_reply(200, get_status())
                elif path == "/snapshot.jpg":
                    try:
                        data = self.jpeg()
                    except Exception as exc:
                        self.json_reply(503, {"error": str(exc)})
                        return
                    self.reply(200, "image/jpeg", data)
                elif path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    while not stopped.is_set():
                        try:
                            data = self.jpeg()
                        except Exception:
                            stopped.wait(0.2)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(data)).encode("ascii") + b"\r\n\r\n" + data + b"\r\n")
                        self.wfile.flush()
                        stopped.wait(0.1)
                else:
                    self.reply(404, "text/plain; charset=utf-8", b"Not found")
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def do_POST(self):
            if urlsplit(self.path).path != "/capture":
                self.reply(404, "text/plain", b"Not found")
                return
            # 浏览器使用 JSON 请求，防止其他网站通过普通表单触发拍照。
            if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                self.json_reply(415, {"error": "请使用 application/json 请求"})
                return
            try:
                self.json_reply(200, save_image(get_image()))
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                self.json_reply(503, {"error": str(exc)})

    server = ThreadingHTTPServer((args.web_host, args.web_port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="数据集名称，例如 person 或 trashbin")
    parser.add_argument("--topic", default="/usb_cam/image_raw", help="相机 Image 话题")
    parser.add_argument("--datasets-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "datasets",
                        help="数据集根目录，默认 robot_ws/datasets")
    parser.add_argument("--no-preview", action="store_true", help="关闭桌面窗口，SSH 回车拍照")
    parser.add_argument("--web-port", type=int, default=0, help="网页预览端口，例如 8080；默认关闭")
    parser.add_argument("--web-host", default="127.0.0.1", help="网页监听地址，默认仅本机；局域网用 0.0.0.0")
    parser.add_argument("--web-only", action="store_true", help="仅使用网页预览和拍照，适合后台运行")
    args = parser.parse_args()
    if (not args.name.strip() or args.name in (".", "..")
            or any(c in args.name for c in '/\\<>:"|?*')
            or any(ord(c) < 32 for c in args.name)):
        parser.error("数据集名称不能为空，也不能包含路径分隔符或非法文件名字符")
    if not 0 <= args.web_port <= 65535:
        parser.error("网页端口必须在 1～65535 范围内；0 表示关闭")
    if args.web_only and not args.web_port:
        parser.error("--web-only 需要同时指定 --web-port")
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
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image

    if not args.no_preview:
        no_gui = re.search(r"^\s*GUI:\s*NONE\s*$", cv2.getBuildInformation(), re.M)
        no_display = not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if no_gui or no_display:
            print("当前环境没有可用桌面预览，改用终端采集模式。", flush=True)
            args.no_preview = True

    dataset = prepare_directories(args)
    rospy.init_node("collect_dataset", anonymous=True)
    bridge = CvBridge()
    count = 0
    latest = [None, 0.0]
    frame_lock = threading.Lock()
    save_lock = threading.Lock()
    stopped = threading.Event()
    server = None
    subscriber = None
    window_created = False
    rospy.on_shutdown(stopped.set)

    def update_frame(message):
        with frame_lock:
            latest[:] = [message, time.monotonic()]

    def latest_message():
        with frame_lock:
            message, arrival = latest
        if message is None or time.monotonic() - arrival > 2.0:
            raise RuntimeError("未收到新鲜相机图像，请检查相机节点")
        return message

    def get_image():
        return bridge.imgmsg_to_cv2(latest_message(), desired_encoding="bgr8")

    def save(image):
        nonlocal count
        with save_lock:
            filename = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
            path = dataset / "images" / filename
            if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise OSError("图像保存失败：" + str(path))
            count += 1
            print("已保存 [{}] {}".format(count, path), flush=True)
            return {"filename": filename, "path": str(path), "count": count}

    def status():
        with frame_lock:
            message, arrival = latest
        with save_lock:
            saved_count = count
        return {"camera_ready": message is not None and time.monotonic() - arrival <= 2.0,
                "count": saved_count, "folder": str(dataset / "images")}

    print("相机话题：{}\n数据集目录：{}".format(args.topic, dataset), flush=True)
    try:
        subscriber = rospy.Subscriber(args.topic, Image, update_frame,
                                      queue_size=1, buff_size=16 * 1024 * 1024)
        if args.web_port:
            server = start_web_preview(args, get_image, save, status, stopped)
            print("网页预览：http://{}:{}/；网页可点击拍照保存。".format(
                args.web_host, args.web_port), flush=True)
        if args.web_only:
            print("网页采集模式运行中。", flush=True)
            rospy.spin()
        elif args.no_preview:
            print("回车采集一张最新图像；输入 q 后回车退出。", flush=True)
            while not rospy.is_shutdown():
                command = input().strip().lower()
                if command == "q":
                    break
                if command:
                    continue
                try:
                    message = rospy.wait_for_message(args.topic, Image, timeout=5.0)
                    save(bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"))
                except (rospy.ROSException, CvBridgeError) as exc:
                    print("未采集到有效图像：{}".format(exc), flush=True)
        else:
            print("在预览窗口按空格或 S 保存原图，按 Q 或 Esc 退出。", flush=True)
            window = "Dataset capture"
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            window_created = True
            last_saved = None
            while not rospy.is_shutdown():
                try:
                    message = latest_message()
                except RuntimeError:
                    message = rospy.wait_for_message(args.topic, Image, timeout=5.0)
                image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                cv2.imshow(window, image)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                    break
                if key in (ord("s"), ord("S"), 32):
                    if (rospy.Time.now() - message.header.stamp).to_sec() > 2.0 or message is last_saved:
                        print("当前帧已过期或已保存，请等待相机新帧。", flush=True)
                        continue
                    save(image)
                    last_saved = message
    except (KeyboardInterrupt, EOFError, rospy.ROSInterruptException):
        pass
    except (rospy.ROSException, CvBridgeError, OSError, cv2.error) as exc:
        print("采集失败：{}".format(exc), flush=True)
        return 1
    finally:
        stopped.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        if subscriber is not None:
            subscriber.unregister()
        if window_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        if not rospy.is_shutdown():
            rospy.signal_shutdown("采集结束")
        print("采集结束，本次保存 {} 张。".format(count), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
