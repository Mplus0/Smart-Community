"""Detector core classes, extracted from tools/detector_preview.py.

- ONNXDetector: load and run ONNX models with NMS (same preprocessing as
  the deployed detector service)
- TrashPipeline: bin detection + ROI + waste detection + matching
- list_detectors / create_detector: registry used by camera_core
"""

from __future__ import annotations

import re
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# Import Detection and rules from mission scripts
# (infra should have access to robot_ws root in sys.path)
try:
    # Assume robot_ws is in sys.path or is the cwd parent
    _WORKSPACE = Path(__file__).resolve().parents[1]  # robot_ws
    if str(_WORKSPACE / "src/community_mission/scripts") not in sys.path:
        sys.path.insert(0, str(_WORKSPACE / "src/community_mission/scripts"))
    from rules import Detection, trash_roi, trash_result
except ImportError:
    # Fallback: define a minimal Detection namedtuple
    from collections import namedtuple
    Detection = namedtuple("Detection", ["label", "box", "confidence"])

    def trash_roi(*args, **kwargs):
        """Fallback stub."""
        return None

    def trash_result(*args, **kwargs):
        """Fallback stub."""
        return {"placement_status": "N/A", "matches": [], "unmatched": []}


DEFAULT_CONFIG = _WORKSPACE / "src/community_perception/config/detectors.yaml"


# ---------------------------------------------------------------------------
# Detector configuration loading
# ---------------------------------------------------------------------------


def load_detector_config(config_path: Path, name: str) -> Tuple[Dict[str, Any], Path]:
    """Load detector config from detectors.yaml and return (config_dict, model_path).

    Resolves $(find community_perception) substitution.
    """
    configs = yaml.safe_load(config_path.read_text(encoding="utf-8"))["detectors"]
    if name not in configs:
        raise ValueError(
            f"Unknown detector {name!r}; choose: {', '.join(configs)}"
        )
    config = configs[name]
    # Resolve ROS package substitution
    value = config["model_path"].replace(
        "$(find community_perception)", str(_WORKSPACE / "src/community_perception")
    )
    model = Path(value).expanduser()
    if not model.is_absolute():
        model = config_path.resolve().parent / model
    return config, model


# ---------------------------------------------------------------------------
# ONNX detector
# ---------------------------------------------------------------------------


class ONNXDetector:
    """Single-stage ONNX detector with confidence filtering and NMS.

    Matches the deployed service's preprocessing (letterbox or plain resize).
    """

    def __init__(
        self,
        model: Path,
        class_names: List[str],
        confidence: float,
        iou: float,
        letterbox: bool = False,
    ) -> None:
        import onnxruntime as ort

        self.session = ort.InferenceSession(
            str(model), providers=["CPUExecutionProvider"]
        )
        self.input = self.session.get_inputs()[0]
        shape = self.input.shape
        output = self.session.get_outputs()[0].shape

        if (
            len(shape) != 4
            or shape[:2] != [1, 3]
            or any(not isinstance(size, int) or size <= 0 for size in shape[2:])
            or self.input.type != "tensor(float)"
        ):
            raise ValueError(f"Expected static FP32 input [1,3,H,W]; got {shape}")

        if (
            not class_names
            or len(output) != 3
            or output[0] != 1
            or output[1] != 4 + len(class_names)
        ):
            raise ValueError(
                f"Expected raw output [1,4+classes,N]; got {output} for {len(class_names)} classes"
            )

        self.height, self.width = shape[2:]
        self.class_names = class_names
        self.confidence = confidence
        self.iou = iou
        self.letterbox = letterbox

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run inference and return list of Detection namedtuples."""
        h, w = image.shape[:2]

        # Resize
        if self.letterbox:
            gain = min(self.width / w, self.height / h)
            nw, nh = round(w * gain), round(h * gain)
        else:
            nw, nh = self.width, self.height

        left, top = (self.width - nw) // 2, (self.height - nh) // 2
        resized = cv2.resize(image, (nw, nh))
        resized = cv2.copyMakeBorder(
            resized,
            top,
            self.height - nh - top,
            left,
            self.width - nw - left,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        # Prepare tensor
        tensor = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = tensor.transpose(2, 0, 1)[None].astype(np.float32) / 255.0

        # Inference
        predictions = self.session.run(None, {self.input.name: tensor})[0][0].T

        # Decode boxes
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

        # NMS
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.iou)
        return [
            Detection(
                self.class_names[ids[index]],
                (
                    boxes[index][0],
                    boxes[index][1],
                    boxes[index][0] + boxes[index][2],
                    boxes[index][1] + boxes[index][3],
                ),
                scores[index],
            )
            for index in np.asarray(keep, dtype=int).reshape(-1)
        ]


# Alias for compatibility
YoloDetector = ONNXDetector


# ---------------------------------------------------------------------------
# Trash classification pipeline
# ---------------------------------------------------------------------------


class TrashPipeline:
    """Full garbage classification: bin detection + ROI + waste detection + matching."""

    def __init__(
        self,
        config_path: Path = DEFAULT_CONFIG,
        confidence: Optional[float] = None,
        iou: Optional[float] = None,
    ) -> None:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        self.detectors = {}

        for name in ("trash_bin_detector", "waste_detector"):
            settings, model = load_detector_config(config_path, name)
            threshold = config["mission_confidence"][name]
            if name == "waste_detector" and confidence is not None:
                threshold = confidence
            iou_val = settings["iou_threshold"] if iou is None else iou
            self.detectors[name] = ONNXDetector(
                model,
                settings["class_names"],
                threshold,
                iou_val,
                letterbox=settings.get("letterbox", False),
            )

        rules_file = _WORKSPACE / "src/community_mission/config/recognition_rules.yaml"
        self.settings = yaml.safe_load(rules_file.read_text(encoding="utf-8"))[
            "recognition_rules"
        ]
        self.last_result: Optional[Dict[str, Any]] = None

    def analyze(self, image: np.ndarray) -> Dict[str, Any]:
        """Bin detection → ROI → waste detection → matching report.

        Drawing is left to :func:`infra.overlay.draw_trash_pipeline`.
        """
        h, w = image.shape[:2]
        bins = self.detectors["trash_bin_detector"].detect(image)
        roi = trash_roi(bins, w, h, self.settings)
        waste = []
        if roi is not None:
            x1, y1, x2, y2 = roi
            objects = self.detectors["waste_detector"].detect(image[y1:y2, x1:x2].copy())
            waste = [
                Detection(d.label, (d.box[0] + x1, d.box[1] + y1, d.box[2] + x1, d.box[3] + y1), d.confidence)
                for d in objects
            ]
        report = dict(trash_result(bins, waste, self.settings), roi=roi)
        report["bin_count"] = len(bins)
        report["waste_count"] = len(waste)
        return report


# ---------------------------------------------------------------------------
# Registry used by the camera pipeline
# ---------------------------------------------------------------------------

def building_name(texts):
    """Accept a single plausible building sign from the fire frame."""
    candidates = set()
    signs = re.compile(r'楼宇[A-D]|(?<![A-Z0-9])[A-D](?:号)?(?:大楼|大厦|楼|栋)|[\u4e00-\u9fff]{1,12}(?:大楼|大厦)')
    for text in texts:
        candidates.update(signs.findall(''.join(text.upper().split())))
    if not candidates:
        candidates.update(signs.findall(''.join(''.join(text.upper().split()) for text in texts)))
    return next(iter(candidates)) if len(candidates) == 1 else ''


class PreviewOCR:
    """Run one preview OCR request at a time without blocking video frames."""

    def __init__(self):
        self._lock = threading.Lock()
        self._running = False
        self._generation = 0
        self._next_request = 0.0
        self._result = None

    def reset(self):
        with self._lock:
            self._generation += 1
            self._next_request = 0.0
            self._result = None

    def sample(self, image, service):
        with self._lock:
            if not self._running and time.monotonic() >= self._next_request:
                self._running = True
                threading.Thread(target=self._read, args=(image.copy(), service, self._generation),
                                 name='camera-preview-ocr', daemon=True).start()
            return self._result

    def _read(self, image, service, generation):
        try:
            import rospy
            from community_perception.srv import LicensePlateOCR
            from cv_bridge import CvBridge

            rospy.wait_for_service(service, timeout=1.0)
            message = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
            response = rospy.ServiceProxy(service, LicensePlateOCR)(message)
            if response.status_message not in ('', '未识别到文字', '未识别到车牌'):
                raise RuntimeError(response.status_message)
            result = dict(texts=list(response.plate_texts), scores=list(response.confidences), error='')
        except Exception as error:
            result = dict(texts=[], scores=[], error=str(error))
        with self._lock:
            self._running = False
            if self._generation == generation:
                self._result = result
                self._next_request = time.monotonic() + 3.0


class MeterPreview:
    """Run the same reader and 4K calibration as the wp_11 task."""

    def __init__(self, show_debug=False):
        import meter_reader

        self.show_debug = show_debug

        self.reader_module = meter_reader
        profile = Path(meter_reader.__file__).with_name("meter_profile") / "profile.json"
        self.reader = meter_reader.Reader(profile, match_max_side=3840)

    def analyze(self, image: np.ndarray):
        debug = {} if self.show_debug else None
        values, annotated, alignment = self.reader.read(image, camera_view=True, debug=debug)
        report = {
            "kind": "meter",
            "alignment": alignment,
            "temperature_c": self.reader_module.quantize_reading("temperature", values["temperature"]),
            "humidity_percent": self.reader_module.quantize_reading("humidity", values["humidity"]),
        }
        if debug is not None:
            matches = None
            if alignment != "alignment_failed":
                matches = self.reader_module.draw_inlier_matches(
                    self.reader.template, image, debug["reference_points"], debug["frame_points"], max_side=480)
            report["inlier_count"] = len(debug.get("frame_points", ()))
            annotated = meter_debug_grid(annotated, debug["gray"], matches, debug.get("aligned_gray"))
        return annotated, report


def meter_debug_grid(overlay, gray, matches, aligned):
    """Lay out intermediate images in a fixed-size MJPEG frame."""
    width, height, header = 640, 360, 28
    row_height = height + header
    grid = np.full((row_height * 2, width * 2, 3), 24, dtype=np.uint8)
    panels = (
        ("Camera + needles", overlay),
        ("Input grayscale", gray),
        ("Template / camera alignment", matches),
        ("Perspective-corrected grayscale", aligned),
    )
    for index, (title, image) in enumerate(panels):
        x, y = (index % 2) * width, (index // 2) * row_height
        cv2.putText(grid, title, (x + 10, y + 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (235, 235, 235), 1, cv2.LINE_AA)
        if image is None:
            cv2.putText(grid, "Alignment unavailable", (x + 20, y + header + height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1, cv2.LINE_AA)
            continue
        h, w = image.shape[:2]
        scale = min(width / w, height / h)
        image = cv2.resize(image, (max(1, round(w * scale)), max(1, round(h * scale))),
                           interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        h, w = image.shape[:2]
        left, top = x + (width - w) // 2, y + header + (height - h) // 2
        grid[top:top + h, left:left + w] = image
    return grid


#: Pseudo detector running the full trash classification pipeline.
TRASH_PIPELINE = "__trash_pipeline__"
METER_PREVIEW = "__meter__"
METER_DEBUG_PREVIEW = "__meter_debug__"
THERMAL_PREVIEW = "__thermal__"
PLATE_PREVIEW = "__plate__"
BUILDING_FIRE_PREVIEW = "__building_fire__"
NO_DETECTOR = "__none__"


def list_detectors(config_path: Path = DEFAULT_CONFIG) -> List[str]:
    """Detector names from detectors.yaml plus the workbench preview modes."""
    configs = yaml.safe_load(config_path.read_text(encoding="utf-8"))["detectors"]
    return list(configs) + [TRASH_PIPELINE, METER_PREVIEW, METER_DEBUG_PREVIEW, THERMAL_PREVIEW, PLATE_PREVIEW, BUILDING_FIRE_PREVIEW]


def create_detector(name: str, confidence: Optional[float] = None, iou: Optional[float] = None,
                    config_path: Path = DEFAULT_CONFIG):
    """Load a detector by name; ``None`` thresholds use detectors.yaml values."""
    if name == TRASH_PIPELINE:
        return TrashPipeline(config_path, confidence, iou)
    if name in (METER_PREVIEW, METER_DEBUG_PREVIEW):
        return MeterPreview(show_debug=name == METER_DEBUG_PREVIEW)
    settings, model = load_detector_config(config_path, name)
    if not model.is_file():
        raise FileNotFoundError(f"模型文件不存在：{model}")
    return ONNXDetector(
        model,
        settings["class_names"],
        settings["conf_threshold"] if confidence is None else confidence,
        settings["iou_threshold"] if iou is None else iou,
        letterbox=settings.get("letterbox", False),
    )
