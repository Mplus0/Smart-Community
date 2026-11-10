#!/usr/bin/env python3
"""Preview a configured ONNX detector in a browser using a ROS camera topic."""

import argparse
from collections import Counter
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = WORKSPACE / 'src/community_perception/config/detectors.yaml'


def load_detector(config_path, name):
    import yaml

    configs = yaml.safe_load(config_path.read_text(encoding='utf-8'))['detectors']
    if name not in configs:
        raise ValueError('Unknown detector {!r}; choose: {}'.format(name, ', '.join(configs)))
    config = configs[name]
    # Resolve the existing ROS package substitution without requiring ROS.
    value = config['model_path'].replace(
        '$(find community_perception)', str(WORKSPACE / 'src/community_perception'))
    model = Path(value).expanduser()
    if not model.is_absolute():
        model = config_path.resolve().parent / model
    return config, model


class ONNXDetector:
    """Match the deployed service's resize, confidence filtering and NMS."""

    def __init__(self, model, class_names, confidence, iou):
        import cv2
        import numpy as np
        import onnxruntime as ort

        self.cv2, self.np = cv2, np
        self.session = ort.InferenceSession(str(model), providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        shape = self.input.shape
        output = self.session.get_outputs()[0].shape
        if (len(shape) != 4 or shape[:2] != [1, 3]
                or any(not isinstance(size, int) or size <= 0 for size in shape[2:])
                or self.input.type != 'tensor(float)'):
            raise ValueError('Expected static FP32 input [1,3,H,W]; got {}'.format(shape))
        if (not class_names or len(output) != 3 or output[0] != 1
                or output[1] != 4 + len(class_names)):
            raise ValueError('Expected raw output [1,4+classes,N]; got {} for {} classes'.format(
                output, len(class_names)))
        self.height, self.width = shape[2:]
        self.class_names = class_names
        self.confidence, self.iou = confidence, iou

    def render(self, image):
        cv2, np = self.cv2, self.np
        start = time.monotonic()
        tensor = cv2.cvtColor(cv2.resize(image, (self.width, self.height)), cv2.COLOR_BGR2RGB)
        tensor = tensor.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        predictions = self.session.run(None, {self.input.name: tensor})[0][0].T
        h, w = image.shape[:2]
        boxes, scores, ids = [], [], []
        for row in predictions:
            class_id = int(np.argmax(row[4:]))
            score = float(row[4 + class_id])
            if score < self.confidence:
                continue
            cx, cy, bw, bh = row[:4]
            x1 = max(0, min(int((cx - bw / 2) * w / self.width), w - 1))
            y1 = max(0, min(int((cy - bh / 2) * h / self.height), h - 1))
            x2 = max(0, min(int((cx + bw / 2) * w / self.width), w - 1))
            y2 = max(0, min(int((cy + bh / 2) * h / self.height), h - 1))
            if x2 - x1 > 5 and y2 - y1 > 5:
                boxes.append([x1, y1, x2 - x1, y2 - y1])
                scores.append(score)
                ids.append(class_id)
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.iou)
        counts = Counter()
        canvas = image.copy()
        for index in np.asarray(keep, dtype=int).reshape(-1):
            x, y, bw, bh = boxes[index]
            class_id = ids[index]
            name = self.class_names[class_id]
            counts[name] += 1
            palette = ((0, 220, 0), (0, 0, 255), (255, 160, 0),
                       (0, 220, 220), (220, 0, 220), (220, 220, 0))
            color = palette[class_id % len(palette)]
            cv2.rectangle(canvas, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(canvas, '{} {:.2f}'.format(name, scores[index]),
                        (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        elapsed = (time.monotonic() - start) * 1000
        lines = ['Objects: {}  inference: {:.0f} ms'.format(sum(counts.values()), elapsed)]
        # Keep counts readable for models with many classes.
        summary = ''
        for name, count in sorted(counts.items()):
            item = '{}: {}  '.format(name, count)
            if summary and cv2.getTextSize(summary + item, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] > w - 16:
                lines.append(summary)
                summary = ''
            summary += item
        if summary:
            lines.append(summary)
        cv2.rectangle(canvas, (0, 0), (w, min(h, 25 * len(lines) + 5)), (0, 0, 0), -1)
        for index, text in enumerate(lines):
            cv2.putText(canvas, text, (8, 22 + 25 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ok, encoded = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError('Failed to encode preview image')
        return encoded.tobytes()


class PreviewServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, title='模型检测测试'):
        super().__init__(address, PreviewHandler)
        self.title = title
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.stopped = False

    def publish(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.condition.notify_all()


class PreviewHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            title = escape(self.server.title)
            page = ('<!doctype html><meta charset="utf-8"><title>' + title + '</title>'
                    '<style>body{background:#151a20;color:#eee;font:16px sans-serif;'
                    'margin:24px}img{max-width:100%;height:auto}</style>'
                    '<h2>' + title + '</h2><p>检测框显示类别与置信度，顶部显示目标数量和处理耗时。'
                    '等待画面时请确认相机话题正在发布；相机停止后保留最后一帧。</p>'
                    '<img src="/stream.mjpg" alt="等待相机画面">').encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path != '/stream.mjpg':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        sequence = -1
        try:
            while True:
                with self.server.condition:
                    self.server.condition.wait_for(
                        lambda: self.server.stopped or (
                            self.server.jpeg is not None and self.server.sequence != sequence),
                        timeout=1,
                    )
                    if self.server.stopped:
                        return
                    if self.server.jpeg is None or self.server.sequence == sequence:
                        continue
                    jpeg, sequence = self.server.jpeg, self.server.sequence
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                self.wfile.flush()
        except (ConnectionError, OSError):
            pass

    def log_message(self, format, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG, help='Detector YAML configuration')
    parser.add_argument('--detector', default='people_detector', help='Detector name in the configuration')
    parser.add_argument('--model', type=Path, help='Override the configured ONNX path')
    parser.add_argument('--topic', default='/usb_cam/image_raw')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--conf', type=float, help='Override configured confidence threshold')
    parser.add_argument('--iou', type=float, help='Override configured IoU threshold')
    parser.add_argument('--fps', type=float, default=10, help='Maximum inference rate')
    parser.add_argument('--image', type=Path, help='Preview one image without ROS')
    args = parser.parse_args()
    try:
        config, model_path = load_detector(args.config, args.detector)
    except (OSError, KeyError, ValueError) as exc:
        parser.error(str(exc))
    confidence = args.conf if args.conf is not None else config['conf_threshold']
    iou = args.iou if args.iou is not None else config['iou_threshold']
    if args.fps <= 0 or not 0 <= confidence <= 1 or not 0 <= iou <= 1:
        parser.error('fps must be positive; conf and iou must be in [0, 1]')
    model_path = args.model or model_path
    if not model_path.is_file():
        parser.error('Model not found: {}'.format(model_path))
    detector = ONNXDetector(model_path, config['class_names'], confidence, iou)
    print('Detector: {}; model: {}; confidence: {}; IoU: {}'.format(
        args.detector, model_path, confidence, iou), flush=True)
    rospy, subscriber = None, None
    latest = [None]
    if args.image:
        image = detector.cv2.imread(str(args.image))
        if image is None:
            parser.error('Cannot read image: {}'.format(args.image))
        jpeg = detector.render(image)
    else:
        import rospy
        from cv_bridge import CvBridge, CvBridgeError
        from sensor_msgs.msg import Image

        rospy.init_node('test_detector_model', anonymous=True)
        bridge = CvBridge()
    server = PreviewServer((args.host, args.port), args.detector)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    print('Preview: http://127.0.0.1:{}/ (remote: http://<robot-ip>:{}/)'.format(args.port, args.port), flush=True)
    try:
        if args.image:
            server.publish(jpeg)
            while True:
                time.sleep(0.2)
        else:
            subscriber = rospy.Subscriber(args.topic, Image, lambda msg: latest.__setitem__(0, msg),
                                          queue_size=1, buff_size=16 * 1024 * 1024)
            print('Waiting for camera topic: ' + args.topic, flush=True)
            rate, previous = rospy.Rate(args.fps), None
            while not rospy.is_shutdown():
                message = latest[0]
                if message is not None and message is not previous:
                    previous = message
                    try:
                        image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                        server.publish(detector.render(image))
                    except CvBridgeError as exc:
                        rospy.logwarn_throttle(5, 'Camera conversion failed: {}'.format(exc))
                rate.sleep()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if rospy is None or not isinstance(exc, rospy.ROSInterruptException):
            raise
    finally:
        if subscriber is not None:
            subscriber.unregister()
        if rospy is not None:
            rospy.signal_shutdown('Preview closed')
        with server.condition:
            server.stopped = True
            server.condition.notify_all()
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == '__main__':
    main()
