"""仪表读数核心；所有角度均为图像坐标：右侧 0°、向下 90°。"""

import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np


DEFAULTS = {
    "angle_step_deg": 0.25,
    "inner_ratio": 0.22,
    "outer_ratio": 0.78,
    "min_radius_px": 40,
    "min_sharpness": 8.0,
    "min_score": 0.035,
    "min_prominence": 0.012,
    "min_coverage": 0.30,
    "min_valid_fraction": 0.60,
}


def read_image(path):
    """imdecode 支持 Windows 中文路径，不对原图缩放。"""
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("无法解码图片：{}".format(path))
    return image


def write_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode(path.suffix or ".png", image)
    if not ok:
        raise ValueError("无法编码调试图片：{}".format(path))
    data.tofile(str(path))


def point_angle(center, point):
    return math.degrees(math.atan2(point[1] - center[1], point[0] - center[0])) % 360


def scale_table(config):
    ticks = config.get("ticks", [])
    if not ticks:
        return None
    values = np.asarray([tick["value"] for tick in ticks], dtype=float)
    angles = np.asarray([point_angle(config["center"], tick["point"]) for tick in ticks])
    sign = 1 if config["direction"] == "clockwise" else -1
    progress = (sign * (angles - angles[0])) % 360
    if len(ticks) < 2 or not np.all(np.diff(values) > 0) or not np.all(np.diff(progress) > 0):
        raise ValueError("至少标定两个点，数值必须递增，刻度位置须按指定方向依次排列")
    return values, angles[0], sign, progress


def validate_config(config):
    if config.get("version") != 1:
        raise ValueError("标定文件 version 必须为 1")
    fw, fh = config["frame_size"]
    x, y, width, height = config["roi"]
    if not all(isinstance(v, int) and not isinstance(v, bool) for v in (fw, fh, x, y, width, height)):
        raise ValueError("frame_size 和 roi 必须使用整数像素坐标")
    if min(fw, fh, width, height) <= 0 or min(x, y) < 0 or x + width > fw or y + height > fh:
        raise ValueError("表盘 ROI 必须位于原图内，且宽高大于 0")
    cx, cy = config["center"]
    radius = config["radius"]
    if not all(math.isfinite(float(v)) for v in (cx, cy, radius)):
        raise ValueError("圆心和半径必须为有限数值")
    if not (0 <= cx < width and 0 <= cy < height and 0 < radius <= min(width, height) / 2):
        raise ValueError("圆心须位于 ROI 内，半径须为正且不超过 ROI 短边的一半")
    if config.get("direction") not in ("clockwise", "counterclockwise"):
        raise ValueError("direction 必须为 clockwise 或 counterclockwise")
    for circle in config.get("exclude_circles", []):
        if len(circle) != 3 or not all(math.isfinite(float(v)) for v in circle) or circle[2] <= 0:
            raise ValueError("屏蔽圆须为有限的 [x,y,正半径]")
    for tick in config.get("ticks", []):
        px, py = tick["point"]
        if not all(math.isfinite(float(v)) for v in (tick["value"], px, py)):
            raise ValueError("刻度数值和坐标必须为有限数值")
        if not (0 <= px < width and 0 <= py < height) or math.hypot(px - cx, py - cy) < radius * 0.5:
            raise ValueError("刻度点须在 ROI 内，并远离旋转中心")
    scale = config.get("scale")
    if scale is not None:
        lower, upper, step = scale["min"], scale["max"], scale["step"]
        if (not all(math.isfinite(float(v)) for v in (lower, upper, step))
                or lower >= upper or step != 1 or lower != round(lower) or upper != round(upper)):
            raise ValueError("量程上下限须为递增整数，当前整数读数模块的刻度间隔须为 1")
        ticks = config.get("ticks", [])
        if ticks and (ticks[0]["value"] != lower or ticks[-1]["value"] != upper
                      or any(tick["value"] != round(tick["value"]) for tick in ticks)):
            raise ValueError("标定点须使用整数刻度，并覆盖量程下限和上限")
    settings = {**DEFAULTS, **config.get("settings", {})}
    if set(settings) != set(DEFAULTS):
        raise ValueError("settings 存在未知配置项")
    if not all(math.isfinite(float(v)) for v in settings.values()):
        raise ValueError("检测参数必须为有限数值")
    if not 0 < settings["inner_ratio"] < settings["outer_ratio"] < 1:
        raise ValueError("采样半径比例须满足 0 < inner_ratio < outer_ratio < 1")
    if not 0.05 <= settings["angle_step_deg"] <= 2:
        raise ValueError("角度步长须在 0.05～2 度之间")
    if settings["min_radius_px"] <= 0 or settings["min_sharpness"] < 0:
        raise ValueError("尺寸阈值必须为正，清晰度阈值不能为负")
    for key in ("min_score", "min_prominence", "min_coverage", "min_valid_fraction"):
        if not 0 <= settings[key] <= 1:
            raise ValueError(key + " 须在 0～1 之间")
    scale_table(config)
    return settings


def load_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    validate_config(config)
    return config


def adapt_config_to_frame_size(config, frame_size):
    """只换算整幅图近似等比缩放后的坐标，不修改标定文件或放大输入图片。"""
    reference_width, reference_height = config["frame_size"]
    width, height = frame_size
    if [width, height] == config["frame_size"]:
        return config
    # 允许宽高各有半个像素的缩放取整误差；存在共同缩放倍率才接受。
    lower = max((width - 0.5) / reference_width, (height - 0.5) / reference_height)
    upper = min((width + 0.5) / reference_width, (height + 0.5) / reference_height)
    if lower > upper + 1e-12:
        raise ValueError("图片长宽比与标定不同，可能经过裁剪、旋转或拉伸；"
                         "请使用同一构图的等比例缩放图片，或重新定位标定")
    scale_x, scale_y = width / reference_width, height / reference_height
    x, y, roi_width, roi_height = config["roi"]
    left, top = math.floor(x * scale_x), math.floor(y * scale_y)
    right = min(width, math.ceil((x + roi_width) * scale_x))
    bottom = min(height, math.ceil((y + roi_height) * scale_y))

    def transform_point(point):
        return [(x + point[0]) * scale_x - left, (y + point[1]) * scale_y - top]

    adapted = copy.deepcopy(config)
    adapted["frame_size"] = [width, height]
    adapted["roi"] = [left, top, right - left, bottom - top]
    adapted["center"] = transform_point(config["center"])
    # 不缩放最小像素尺寸和清晰度阈值，避免小图因为适配而绕过质量检查。
    radial_scale = min(scale_x, scale_y)
    adapted["radius"] = config["radius"] * radial_scale
    for original, tick in zip(config.get("ticks", []), adapted.get("ticks", [])):
        tick["point"] = transform_point(original["point"])
    adapted["exclude_circles"] = [transform_point(circle[:2]) + [circle[2] * radial_scale]
                                  for circle in config.get("exclude_circles", [])]
    validate_config(adapted)
    return adapted


def value_from_angle(config, angle):
    values, start, sign, progress = scale_table(config)
    position = (sign * (angle - start)) % 360
    # 数值误差不能把起点误判成 360 度。
    if math.isclose(position, 360, abs_tol=1e-6):
        position = 0.0
    if position > progress[-1] + 1e-6:
        raise ValueError("指针位于已标定量程之外")
    return float(np.interp(position, progress, values))


def nearest_integer(value):
    """就近取整；恰好半整数时远离零，支持温度计的负数。"""
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


def _scan(gray, mask, config, settings):
    radius = config["radius"]
    cx, cy = config["center"]
    table = scale_table(config)
    if table is None:
        angles = np.arange(0, 360, settings["angle_step_deg"])
    else:
        _, start, sign, progress = table
        count = max(2, int(math.ceil(progress[-1] / settings["angle_step_deg"])) + 1)
        angles = (start + sign * np.linspace(0, progress[-1], count)) % 360
    radians = np.deg2rad(angles)[:, None]
    radii = np.linspace(radius * settings["inner_ratio"], radius * settings["outer_ratio"],
                        max(32, int(radius * 1.2)))[None, :]
    base_x = cx + np.cos(radians) * radii
    base_y = cy + np.sin(radians) * radii

    def sample(offset, source=gray, interpolation=cv2.INTER_LINEAR):
        mx = (base_x - np.sin(radians) * offset).astype(np.float32)
        my = (base_y + np.cos(radians) * offset).astype(np.float32)
        return cv2.remap(source, mx, my, interpolation, borderMode=cv2.BORDER_CONSTANT,
                         borderValue=0)

    half_width = max(0.8, radius * 0.008)
    flank = max(4, radius * 0.04)
    core = np.mean([sample(offset).astype(float) for offset in (-half_width, 0, half_width)], axis=0)
    sides = np.mean([sample(offset).astype(float) for offset in (-flank, flank)], axis=0)
    valid = core < 250  # 白色表面不自动认作反光；仅排除候选中心线上饱和的采样。
    for offset in (-flank, -half_width, 0, half_width, flank):
        valid &= sample(offset, mask, cv2.INTER_NEAREST) > 0
    contrast = np.clip((sides - core) / 255.0, 0, 1)
    count = valid.sum(axis=1)
    scores = (contrast * valid).sum(axis=1) / np.maximum(count, 1)
    coverage = ((contrast > 0.05) & valid).mean(axis=1)
    valid_fraction = valid.mean(axis=1)
    eligible = (valid_fraction >= settings["min_valid_fraction"]) & (coverage >= settings["min_coverage"])
    scores[~eligible] = 0
    best = int(np.argmax(scores))
    separation = np.abs((angles - angles[best] + 180) % 360 - 180)
    alternatives = scores[separation > 8]
    runner_up = float(alternatives.max()) if len(alternatives) else 0.0
    metrics = {
        "score": round(float(scores[best]), 5),
        "prominence": round(float(scores[best] - runner_up), 5),
        "coverage": round(float(coverage[best]), 5),
        "valid_fraction": round(float(valid_fraction[best]), 5),
    }
    return float(angles[best]), metrics


def read_frame(frame, config):
    """返回 (结构化结果, ROI 调试图)。配置错误抛异常，识别失败返回 rejected。"""
    settings = validate_config(config)
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
        raise ValueError("frame 必须为 uint8 BGR 三通道图片")
    reference_frame_size = list(config["frame_size"])
    input_frame_size = list(frame.shape[1::-1])
    config = adapt_config_to_frame_size(config, input_frame_size)
    x, y, width, height = config["roi"]
    crop = frame[y:y + height, x:x + width].copy()
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(gray)
    center = tuple(int(round(v)) for v in config["center"])
    radius = config["radius"]
    cv2.circle(mask, center, int(radius * 0.9), 255, -1)
    for ex, ey, er in config.get("exclude_circles", []):
        cv2.circle(mask, (int(ex), int(ey)), int(er), 0, -1)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    pixels = laplacian[mask > 0]
    sharpness = float(pixels.var()) if len(pixels) else 0.0
    angle, metrics = _scan(gray, mask, config, settings)
    metrics.update({"sharpness": round(sharpness, 3), "radius_px": round(radius, 2)})
    reasons = []
    if radius < settings["min_radius_px"]:
        reasons.append("表盘像素尺寸太小，请靠近或提高有效成像尺寸")
    if sharpness < settings["min_sharpness"]:
        reasons.append("清晰度不足，请停车、对焦后重拍")
    if metrics["score"] < settings["min_score"] or metrics["coverage"] < settings["min_coverage"]:
        reasons.append("没有找到足够连续的暗色指针")
    if metrics["valid_fraction"] < settings["min_valid_fraction"]:
        reasons.append("候选方向的有效采样不足，检查反光、遮挡或屏蔽区域")
    if metrics["prominence"] < settings["min_prominence"]:
        reasons.append("存在多个接近的候选方向，请检查小表盘、文字或配重干扰")
    result = {"status": "rejected", "reading": None, "rounded": None,
              "angle_deg": round(angle, 3), "metrics": metrics, "reasons": reasons,
              "resolution": {"calibrated": reference_frame_size, "input": input_frame_size,
                             "coordinates_scaled": reference_frame_size != input_frame_size}}
    if not reasons:
        if not config.get("ticks"):
            result["status"] = "needs_calibration"
            result["reasons"] = ["仅检测到候选指针方向；缺少人工确认的刻度标定，未输出读数"]
        else:
            value = value_from_angle(config, angle)
            result.update(status="ok", reading=round(value, 4), rounded=nearest_integer(value))
    debug = crop.copy()
    cv2.circle(debug, center, int(radius), (0, 255, 0), 1, cv2.LINE_AA)
    for ratio in (settings["inner_ratio"], settings["outer_ratio"]):
        cv2.circle(debug, center, int(radius * ratio), (180, 160, 0), 1, cv2.LINE_AA)
    for ex, ey, er in config.get("exclude_circles", []):
        cv2.circle(debug, (int(ex), int(ey)), int(er), (255, 0, 255), 1)
    for tick in config.get("ticks", []):
        point = tuple(int(round(v)) for v in tick["point"])
        cv2.circle(debug, point, 3, (255, 100, 0), -1)
        cv2.putText(debug, str(tick["value"]), point, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)
    endpoint = (int(center[0] + radius * 0.83 * math.cos(math.radians(angle))),
                int(center[1] + radius * 0.83 * math.sin(math.radians(angle))))
    color = (0, 255, 0) if result["status"] == "ok" else (0, 160, 255)
    cv2.arrowedLine(debug, center, endpoint, color, 2, cv2.LINE_AA, tipLength=0.08)
    cv2.circle(debug, center, 3, (0, 0, 255), -1)
    label = "{} angle={:.2f}".format(result["status"], angle)
    if result["rounded"] is not None:
        label += " value={:.2f} -> {}".format(result["reading"], result["rounded"])
    cv2.rectangle(debug, (0, 0), (width, 27), (0, 0, 0), -1)
    cv2.putText(debug, label, (5, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return result, debug
