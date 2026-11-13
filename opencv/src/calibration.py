"""交互标定：重复框选、逐点确认，最终预览后才保存 -30～50、每格 1 的标定。"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

from .core import read_image, validate_config, write_image
from .selection import PointSelection, RectangleSelection, rectangle_to_roi


DEFAULT_VALUES = tuple(range(-30, 51, 10))
METER_SCALE = {"min": -30, "max": 50, "step": 1}
HEADER = 84


def _preview(image, max_width, max_height, max_scale=1.0):
    scale = min(max_scale, max_width / image.shape[1], max_height / image.shape[0])
    size = (max(1, round(image.shape[1] * scale)), max(1, round(image.shape[0] * scale)))
    return cv2.resize(image, size)


def _with_header(image, lines):
    canvas = np.zeros((image.shape[0] + HEADER, image.shape[1], 3), dtype=np.uint8)
    canvas[HEADER:] = image
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (8, 20 + index * 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.46, (0, 255, 255), 1, cv2.LINE_AA)
    return canvas


def _select_roi(image):
    preview = _preview(image, 1200, 700)
    height, width = preview.shape[:2]
    state = RectangleSelection(width, height)
    window = "Select dial | redraw until Enter confirms"
    message = "Drag a rectangle around the whole large dial."
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    def callback(event, x, y, flags, userdata):
        local_y = y - HEADER
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= x < width and 0 <= local_y < height:
            state.start(x, local_y)
        elif event == cv2.EVENT_MOUSEMOVE:
            state.move(x, local_y)
        elif event == cv2.EVENT_LBUTTONUP:
            state.finish(x, local_y)

    cv2.setMouseCallback(window, callback)
    try:
        while True:
            shown = preview.copy()
            if state.rectangle is not None:
                x1, y1, x2, y2 = (round(v) for v in state.rectangle)
                cv2.rectangle(shown, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imshow(window, _with_header(shown, [message,
                       "Drag again: replace | Enter: confirm ROI",
                       "R/Backspace: clear | Esc: cancel (no file saved)"]))
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                raise ValueError("已取消标定，没有保存文件")
            if key in (8, 127, ord("r"), ord("R")):
                state.reset()
            if key in (10, 13):
                rectangle = state.confirm()
                if rectangle is not None:
                    return rectangle_to_roi(rectangle, image.shape[1::-1], preview.shape[1::-1])
                message = "Draw a valid rectangle, release the mouse, then Enter."
    finally:
        cv2.destroyWindow(window)


def _click_points(image, labels, validator=None):
    preview = _preview(image, 1000, 650, 1.5)
    scale_x = preview.shape[1] / image.shape[1]
    scale_y = preview.shape[0] / image.shape[0]
    state = PointSelection(len(labels))
    window = "Calibration | click again to replace | Enter: confirm"
    message = ""
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    def callback(event, x, y, flags, userdata):
        nonlocal message
        local_y = y - HEADER
        if (event == cv2.EVENT_LBUTTONDOWN and 0 <= x < preview.shape[1]
                and 0 <= local_y < preview.shape[0]):
            state.select(x / scale_x, local_y / scale_y)
            message = ""

    cv2.setMouseCallback(window, callback)
    try:
        while True:
            shown = preview.copy()
            for index, (point, label) in enumerate(zip(state.points, labels)):
                if point is None:
                    continue
                position = (round(point[0] * scale_x), round(point[1] * scale_y))
                color = (0, 180, 255) if index == state.index else (0, 255, 0)
                cv2.circle(shown, position, 4, color, -1)
                cv2.putText(shown, label, position, cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
            if state.points[0] is not None and state.points[1] is not None:
                center = (round(state.points[0][0] * scale_x), round(state.points[0][1] * scale_y))
                radius = math.dist(state.points[0], state.points[1])
                axes = (round(radius * scale_x), round(radius * scale_y))
                cv2.ellipse(shown, center, axes, 0, 0, 360, (0, 255, 0), 1)
            if state.reviewing:
                title = "Review all points | Enter: SAVE | Backspace: edit last"
            else:
                title = "Step {}/{}: {}".format(state.index + 1, len(labels), labels[state.index])
            cv2.imshow(window, _with_header(shown, [title,
                       "Click again: replace | Enter: confirm current | Backspace: previous",
                       message or "R: clear current | Esc: cancel (no file saved)"]))
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                raise ValueError("已取消标定，没有保存文件")
            if key in (8, 127):
                state.back()
                message = ""
            if key in (ord("r"), ord("R")):
                state.reset_current()
            if key in (10, 13):
                if state.reviewing:
                    try:
                        if validator is not None:
                            validator(state.points)
                    except ValueError as exc:
                        print("标定未通过：{}；可按 Backspace 返回修改。".format(exc), flush=True)
                        message = "Invalid geometry/order. Backspace to correct points."
                        continue
                    return state.points
                point = state.points[state.index]
                if point is None:
                    message = "Click a point first; Enter confirms the current candidate."
                    continue
                if state.index == 1:
                    radius = math.dist(state.points[0], point)
                    if not 0 < radius <= min(image.shape[:2]) / 2:
                        message = "Invalid radius. Click again within the cropped dial."
                        continue
                if state.index >= 2 and math.dist(state.points[0], point) < math.dist(*state.points[:2]) * 0.5:
                    message = "Click the corresponding outer tick line, away from center."
                    continue
                state.confirm()
    finally:
        cv2.destroyWindow(window)


def calibrate(image_path, output_path, values, direction, roi=None, exclude_circles=None):
    image = read_image(image_path)
    if len(values) < 2 or not all(math.isfinite(v) for v in values):
        raise ValueError("至少选择两个有限数值刻度")
    if any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError("刻度必须按数值严格递增输入")
    if values[0] != -30 or values[-1] != 50 or any(v != round(v) for v in values):
        raise ValueError("本仪表量程 -30～50、每格 1；标定点必须为整数，并包含 -30 和 50")
    if roi is None:
        print("反复拖框可替换选区，按 Enter 才确认。", flush=True)
        roi = _select_roi(image)
    x, y, width, height = roi
    if min(x, y) < 0 or min(width, height) <= 0 or x + width > image.shape[1] or y + height > image.shape[0]:
        raise ValueError("未框选有效表盘区域")
    crop = image[y:y + height, x:x + width]
    labels = ["pointer center", "outer tick circle"] + ["tick {:g}".format(v) for v in values]

    def build_config(points):
        return {
            "version": 1,
            "frame_size": [image.shape[1], image.shape[0]],
            "roi": roi,
            "center": points[0],
            "radius": math.dist(points[0], points[1]),
            "direction": direction,
            "scale": dict(METER_SCALE),
            "ticks": [{"value": v, "point": p} for v, p in zip(values, points[2:])],
            "exclude_circles": exclude_circles or [],
            "settings": {},
        }

    print("每步可重复点击，Enter 确认并进入下一步；Backspace 返回上一步。", flush=True)
    print("刻度顺序：{}；最后进入总览，再按 Enter 才保存文件。".format(
          ", ".join("{:g}".format(v) for v in values)), flush=True)
    points = _click_points(crop, labels, lambda points: validate_config(build_config(points)))
    config = build_config(points)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_image(output_path.with_suffix(".reference.png"), crop)
    output_path.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    return config
