#!/usr/bin/env python3
"""Preview ONNX detection or full trash sorting from a ROS camera or image."""

import argparse
from collections import Counter
import json
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = WORKSPACE / 'src/community_perception/config/detectors.yaml'
sys.path.insert(0, str(WORKSPACE / 'src/community_mission/scripts'))
from rules import Detection, trash_roi, trash_result


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

    def __init__(self, model, class_names, confidence, iou, letterbox=False):
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
        self.letterbox = letterbox

    def detect(self, image):
        cv2, np = self.cv2, self.np
        h, w = image.shape[:2]
        if self.letterbox:
            gain = min(self.width / w, self.height / h)
            nw, nh = round(w * gain), round(h * gain)
        else:
            nw, nh = self.width, self.height
        left, top = (self.width - nw) // 2, (self.height - nh) // 2
        resized = cv2.resize(image, (nw, nh))
        resized = cv2.copyMakeBorder(resized, top, self.height - nh - top,
                                     left, self.width - nw - left,
                                     cv2.BORDER_CONSTANT, value=(114, 114, 114))
        tensor = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = tensor.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        predictions = self.session.run(None, {self.input.name: tensor})[0][0].T
        boxes, scores, ids = [], [], []
        for row in predictions:
            class_id = int(np.argmax(row[4:]))
            score = float(row[4 + class_id])
            if score < self.confidence:
                continue
            cx, cy, bw, bh = row[:4]
            x1 = max(0, min(int((cx - bw / 2 - left) * w / nw), w - 1))
            y1 = max(0, min(int((cy - bh / 2 - top) * h / nh), h - 1))
            x2 = max(0, min(int((cx + bw / 2 - left) * w / nw), w - 1))
            y2 = max(0, min(int((cy + bh / 2 - top) * h / nh), h - 1))
            if x2 - x1 > 5 and y2 - y1 > 5:
                boxes.append([x1, y1, x2 - x1, y2 - y1])
                scores.append(score)
                ids.append(class_id)
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.iou)
        return [Detection(self.class_names[ids[index]],
                          (boxes[index][0], boxes[index][1],
                           boxes[index][0] + boxes[index][2], boxes[index][1] + boxes[index][3]),
                          scores[index]) for index in np.asarray(keep, dtype=int).reshape(-1)]

    def render(self, image):
        cv2 = self.cv2
        start = time.monotonic()
        objects = self.detect(image)
        counts = Counter()
        canvas = image.copy()
        for obj in objects:
            x1, y1, x2, y2 = obj.box
            name = obj.label
            class_id = self.class_names.index(name)
            counts[name] += 1
            palette = ((0, 220, 0), (0, 0, 255), (255, 160, 0),
                       (0, 220, 220), (220, 0, 220), (220, 220, 0))
            color = palette[class_id % len(palette)]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(canvas, '{} {:.2f}'.format(name, obj.confidence),
                        (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        elapsed = (time.monotonic() - start) * 1000
        lines = ['Objects: {}  inference: {:.0f} ms'.format(sum(counts.values()), elapsed)]
        # Keep counts readable for models with many classes.
        summary = ''
        for name, count in sorted(counts.items()):
            item = '{}: {}  '.format(name, count)
            if summary and cv2.getTextSize(summary + item, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] > image.shape[1] - 16:
                lines.append(summary)
                summary = ''
            summary += item
        if summary:
            lines.append(summary)
        return encode_preview(canvas, lines)


def encode_preview(canvas, lines):
    import cv2
    h, w = canvas.shape[:2]
    cv2.rectangle(canvas, (0, 0), (w, min(h, 25 * len(lines) + 5)), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(canvas, text, (8, 22 + 25 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    ok, encoded = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError('Failed to encode preview image')
    return encoded.tobytes()


class TrashPreview:
    """Use the mission's ROI and matching rules with the configured local models."""

    def __init__(self, config_path, confidence=None, iou=None):
        import yaml

        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        self.detectors = {}
        for name in ('trash_bin_detector', 'waste_detector'):
            settings, model = load_detector(config_path, name)
            threshold = config['mission_confidence'][name]
            if name == 'waste_detector' and confidence is not None:
                threshold = confidence
            self.detectors[name] = ONNXDetector(
                model, settings['class_names'], threshold,
                settings['iou_threshold'] if iou is None else iou,
                letterbox=settings.get('letterbox', False))
        self.cv2, self.np = self.detectors['waste_detector'].cv2, self.detectors['waste_detector'].np
        rules_file = WORKSPACE / 'src/community_mission/config/recognition_rules.yaml'
        self.settings = yaml.safe_load(rules_file.read_text(encoding='utf-8'))['recognition_rules']
        self.last_result = None

    def render(self, image):
        cv2 = self.cv2
        start = time.monotonic()
        h, w = image.shape[:2]
        bins = self.detectors['trash_bin_detector'].detect(image)
        roi = trash_roi(bins, w, h, self.settings)
        waste = []
        if roi is not None:
            x1, y1, x2, y2 = roi
            objects = self.detectors['waste_detector'].detect(image[y1:y2, x1:x2].copy())
            waste = [Detection(d.label, (d.box[0] + x1, d.box[1] + y1,
                                         d.box[2] + x1, d.box[3] + y1), d.confidence) for d in objects]
        report = dict(trash_result(bins, waste, self.settings), roi=roi)
        canvas = image.copy()
        if roi is not None:
            cv2.rectangle(canvas, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 255), 2)
        colors = {'CORRECT': (0, 200, 0), 'INCORRECT': (0, 0, 255), 'UNKNOWN': (0, 165, 255)}
        for match in report['matches']:
            bx1, by1, bx2, by2 = match['bin_bbox']
            color = colors[match['status']]
            cv2.rectangle(canvas, (bx1, by1), (bx2, by2), color, 2)
            cv2.putText(canvas, match['bin_label'] + ' ' + match['status'],
                        (bx1, max(16, by1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            for obj in match['garbage']:
                gx1, gy1, gx2, gy2 = obj['bbox']
                cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), color, 2)
                cv2.line(canvas, ((bx1 + bx2) // 2, by2),
                         ((gx1 + gx2) // 2, (gy1 + gy2) // 2), color, 1)
                cv2.putText(canvas, '{} {:.2f}'.format(obj['label'], obj['confidence']),
                            (gx1, max(16, gy1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        for obj in report['unmatched']:
            gx1, gy1, gx2, gy2 = obj['bbox']
            cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), colors['UNKNOWN'], 2)
        report['duration_ms'] = round((time.monotonic() - start) * 1000, 1)
        self.last_result = report
        return encode_preview(canvas, [
            'Trash: {} | bins={} waste={} | {:.0f} ms'.format(
                report['placement_status'], len(bins), len(waste), report['duration_ms']),
            'ROI: yellow | Correct: green | Incorrect: red | Unknown: orange'])


class PreviewServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, title='模型检测测试'):
        super().__init__(address, PreviewHandler)
        self.title = title
        self.condition = threading.Condition()
        self.jpeg = None
        self.result = None
        self.sequence = 0
        self.stopped = False

    def publish(self, jpeg, result=None):
        with self.condition:
            self.jpeg = jpeg
            self.result = result
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
                    '<pre id="result" style="white-space:pre-wrap"></pre>'
                    '<img src="/stream.mjpg" alt="等待相机画面">'
                    '<script>setInterval(async()=>{try{const r=await fetch("/result.json",{cache:"no-store"});'
                    'const d=await r.json();document.getElementById("result").textContent='
                    'd?d.announcement:"";}catch(e){}},1000);</script>').encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path == '/result.json':
            with self.server.condition:
                payload = json.dumps(self.server.result, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
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
    parser.add_argument('--trash', action='store_true', help='Run bucket detection, ROI and trash matching; --conf sets trash confidence')
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
        config, model_path = load_detector(args.config, 'waste_detector' if args.trash else args.detector)
    except (OSError, KeyError, ValueError) as exc:
        parser.error(str(exc))
    confidence = args.conf if args.conf is not None else config['conf_threshold']
    iou = args.iou if args.iou is not None else config['iou_threshold']
    if args.fps <= 0 or not 0 <= confidence <= 1 or not 0 <= iou <= 1:
        parser.error('fps must be positive; conf and iou must be in [0, 1]')
    model_path = args.model or model_path
    if not model_path.is_file():
        parser.error('Model not found: {}'.format(model_path))
    if args.trash and args.model:
        parser.error('--model is for single-detector preview; configure both trash model paths in YAML')
    detector = (TrashPreview(args.config, args.conf, args.iou) if args.trash else
                ONNXDetector(model_path, config['class_names'], confidence, iou,
                             letterbox=config.get('letterbox', False)))
    if args.trash:
        confidence = detector.detectors['waste_detector'].confidence
        iou = detector.detectors['waste_detector'].iou
    print('Detector: {}; model: {}; confidence: {}; IoU: {}'.format(
        'trash pipeline' if args.trash else args.detector, model_path, confidence, iou), flush=True)
    rospy, subscriber = None, None
    latest = [None]
    if args.image:
        try:
            image = detector.cv2.imdecode(detector.np.fromfile(str(args.image), dtype=detector.np.uint8),
                                          detector.cv2.IMREAD_COLOR)
        except OSError as exc:
            parser.error(str(exc))
        if image is None:
            parser.error('Cannot read image: {}'.format(args.image))
        jpeg = detector.render(image)
    else:
        import rospy
        from cv_bridge import CvBridge, CvBridgeError
        from sensor_msgs.msg import Image

        rospy.init_node('detector_preview', anonymous=True)
        bridge = CvBridge()
    server = PreviewServer((args.host, args.port), '垃圾分类完整流程' if args.trash else args.detector)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    print('Preview: http://127.0.0.1:{}/ (remote: http://<robot-ip>:{}/)'.format(args.port, args.port), flush=True)
    try:
        if args.image:
            server.publish(jpeg, getattr(detector, 'last_result', None))
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
                        server.publish(detector.render(image), getattr(detector, 'last_result', None))
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
