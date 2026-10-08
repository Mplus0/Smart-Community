#!/usr/bin/env python3
"""HyperLPR3 ROS1 camera recognition in an isolated environment, without cv_bridge."""
import json
import math
import sys
import threading
import time
from copy import copy
from pathlib import Path

import cv2
import numpy as np
import rospy
from PIL import Image as PILImage, ImageDraw, ImageFont
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prepare_hyperlpr_models import verify_cache


def decode_bgr(msg):
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}
    if msg.encoding not in channels:
        raise ValueError("Unsupported encoding: " + msg.encoding)
    count = channels[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * count:
        raise ValueError("Invalid image dimensions/step")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size != msg.height * msg.step:
        raise ValueError("Image buffer length does not match height * step")
    pixels = raw.reshape(msg.height, msg.step)[:, :msg.width * count]
    pixels = pixels.reshape(msg.height, msg.width, count)
    conversions = {"rgb8": cv2.COLOR_RGB2BGR, "mono8": cv2.COLOR_GRAY2BGR,
                   "bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR}
    if msg.encoding == "mono8":
        pixels = pixels[:, :, 0]
    if msg.encoding in conversions:
        pixels = cv2.cvtColor(pixels, conversions[msg.encoding])
    return np.ascontiguousarray(pixels).copy()


def encode_bgr(pixels, header):
    pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
    msg = Image()
    # rospy serialization assigns an output seq; never mutate the input header.
    msg.header = copy(header)
    msg.height, msg.width = pixels.shape[:2]
    msg.encoding = "bgr8"
    msg.step = msg.width * 3
    msg.is_bigendian = 0
    msg.data = pixels.tobytes()
    return msg


def load_font(requested):
    candidates = [requested] if requested else [
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    ]
    for name in candidates:
        if Path(name).is_file():
            try:
                return ImageFont.truetype(name, 22)
            except OSError:
                pass
    if requested:
        raise ValueError("Cannot load requested Chinese font: " + requested)
    rospy.logwarn("No Chinese font found: image labels use plate indexes; JSON keeps full text. Set ~font_path.")
    return None


def annotate(image, plates, font):
    for plate in plates:
        x1, y1, x2, y2 = plate["bbox_xyxy"]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    if font is None:
        for index, plate in enumerate(plates):
            x, y = plate["bbox_xyxy"][:2]
            cv2.putText(image, f"plate#{index} {plate['confidence']:.3f}",
                        (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
        return image
    canvas = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(canvas)
    for plate in plates:
        text = f"{plate['text']} {plate['confidence']:.3f}"
        x, y = plate["bbox_xyxy"][:2]
        box = draw.textbbox((0, 0), text, font=font)
        tw, th = box[2] - box[0], box[3] - box[1]
        x = max(0, min(x, image.shape[1] - tw - 6))
        y = max(0, min(y - th - 8, image.shape[0] - th - 6))
        draw.rectangle((x, y, x + tw + 6, y + th + 6), fill=(0, 0, 0))
        draw.text((x + 3 - box[0], y + 3 - box[1]), text,
                  font=font, fill=(0, 255, 0))
    return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


def main():
    rospy.init_node("plate_recognition")
    input_topic = rospy.get_param("~input_topic", "/camera/color/image_raw")
    image_topic = rospy.get_param("~image_topic", "/perception/plates_image")
    result_topic = rospy.get_param("~result_topic", "/perception/plates_json")
    names = [rospy.resolve_name(topic) for topic in
             (input_topic, image_topic, result_topic)]
    if len(set(names)) != 3:
        raise ValueError("Input, image output and JSON output topics must differ")
    max_rate = float(rospy.get_param("~max_rate", 5.0))
    min_conf = float(rospy.get_param("~min_confidence", 0.5))
    stale_seconds = float(rospy.get_param("~stale_seconds", 2.0))
    if (not all(math.isfinite(v) for v in (max_rate, min_conf, stale_seconds))
            or max_rate <= 0 or stale_seconds <= 0 or not 0 <= min_conf <= 1):
        raise ValueError("Invalid rate, confidence or stale timeout")
    font = load_font(rospy.get_param("~font_path", ""))
    models_dir = rospy.get_param("~models_dir", "")
    if not models_dir:
        raise ValueError("~models_dir must point to the bundled models/hyperlpr3 directory")
    try:
        cache = verify_cache(models_dir)
    except Exception as error:
        raise RuntimeError("HyperLPR cache not ready; run prepare_hyperlpr_models.py "
                           "with the same user/HOME before launch: " + str(error)) from error
    rospy.loginfo("Verified offline HyperLPR3 cache: %s", cache)
    import hyperlpr3 as lpr3
    catcher = lpr3.LicensePlateCatcher(detect_level=lpr3.DETECT_LEVEL_HIGH)
    rospy.loginfo("Recognition providers: %s", catcher.pipeline.recognizer.session.get_providers())
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
    json_pub = rospy.Publisher(result_topic, String, queue_size=1)
    lock = threading.Lock()
    latest = [None]

    def receive(msg):
        with lock:
            latest[0] = (msg, time.monotonic())

    subscriber = rospy.Subscriber(input_topic, Image, receive,
                                  queue_size=1, buff_size=8 * 1024 * 1024)
    rospy.loginfo("Ready: %s -> %s and %s; max %.1f Hz",
                  input_topic, image_topic, result_topic, max_rate)
    next_time = 0.0
    last_received = time.monotonic()
    while not rospy.is_shutdown():
        now = time.monotonic()
        if now < next_time:
            time.sleep(min(0.02, next_time - now))
            continue
        with lock:
            item = latest[0]
            latest[0] = None
        if item is None:
            if now - last_received > stale_seconds:
                rospy.logwarn_throttle(5, "No fresh camera images; no result is republished")
            time.sleep(0.02)
            continue
        source, received_at = item
        last_received = received_at
        if now - received_at > stale_seconds:
            continue
        started = time.perf_counter()
        next_time = now + 1.0 / max_rate
        try:
            pixels = decode_bgr(source)
            inference_start = time.perf_counter()
            raw_results = catcher(pixels)
            inference_ms = (time.perf_counter() - inference_start) * 1000
            plates = []
            for text, confidence, plate_type, box in raw_results:
                confidence = float(confidence)
                if not math.isfinite(confidence) or confidence < min_conf:
                    continue
                x1, y1, x2, y2 = [int(v) for v in box]
                x1, x2 = max(0, x1), min(source.width, x2)
                y1, y2 = max(0, y1), min(source.height, y2)
                if x2 <= x1 or y2 <= y1:
                    continue
                plates.append({"text": str(text), "confidence": confidence,
                               "plate_type_id": int(plate_type),
                               "bbox_xyxy": [x1, y1, x2, y2]})
            marked = annotate(pixels, plates, font)
            payload = {
                "schema_version": 1, "engine": "hyperlpr3",
                "header": {"seq": source.header.seq,
                           "stamp": {"secs": source.header.stamp.secs,
                                     "nsecs": source.header.stamp.nsecs},
                           "frame_id": source.header.frame_id},
                "width": source.width, "height": source.height,
                "inference_ms": inference_ms,
                "processing_ms": (time.perf_counter() - started) * 1000,
                "plates": plates,
            }
            line = json.dumps(payload, ensure_ascii=False, allow_nan=False)
            image_pub.publish(encode_bgr(marked, source.header))
            json_pub.publish(String(data=line))
            rospy.loginfo_throttle(2, "plates=%s inference=%.1fms",
                                   [p["text"] for p in plates], inference_ms)
        except Exception as error:
            rospy.logerr_throttle(5, "Plate frame failed: %s", str(error))
    subscriber.unregister()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
