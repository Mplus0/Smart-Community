"""Detector core classes - extracted from detector_preview.py for reuse in infra.

Batch 3 scope:
- ONNXDetector: Load and run ONNX models with NMS
- YoloDetector: Alias for ONNXDetector (for compatibility)
- TrashPipeline: Full garbage classification workflow (ROI + matching)
- load_detector_config: Parse detectors.yaml
"""

from __future__ import annotations

import sys
import time
from collections import Counter
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

    def render(self, image: np.ndarray) -> bytes:
        """Detect and draw boxes, return JPEG bytes."""
        start = time.monotonic()
        objects = self.detect(image)
        counts = Counter()
        canvas = image.copy()

        palette = (
            (0, 220, 0),
            (0, 0, 255),
            (255, 160, 0),
            (0, 220, 220),
            (220, 0, 220),
            (220, 220, 0),
        )

        for obj in objects:
            x1, y1, x2, y2 = obj.box
            name = obj.label
            class_id = self.class_names.index(name)
            counts[name] += 1
            color = palette[class_id % len(palette)]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                canvas,
                f"{name} {obj.confidence:.2f}",
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        elapsed = (time.monotonic() - start) * 1000
        lines = [f"Objects: {sum(counts.values())}  inference: {elapsed:.0f} ms"]

        # Multi-line counts
        summary = ""
        for name, count in sorted(counts.items()):
            item = f"{name}: {count}  "
            text_width = cv2.getTextSize(
                summary + item, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )[0][0]
            if summary and text_width > image.shape[1] - 16:
                lines.append(summary)
                summary = ""
            summary += item
        if summary:
            lines.append(summary)

        return encode_preview(canvas, lines)


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

    def render(self, image: np.ndarray) -> bytes:
        """Run full pipeline and return annotated JPEG."""
        start = time.monotonic()
        h, w = image.shape[:2]

        bins = self.detectors["trash_bin_detector"].detect(image)
        roi = trash_roi(bins, w, h, self.settings)
        waste = []

        if roi is not None:
            x1, y1, x2, y2 = roi
            objects = self.detectors["waste_detector"].detect(
                image[y1:y2, x1:x2].copy()
            )
            waste = [
                Detection(
                    d.label,
                    (d.box[0] + x1, d.box[1] + y1, d.box[2] + x1, d.box[3] + y1),
                    d.confidence,
                )
                for d in objects
            ]

        report = dict(trash_result(bins, waste, self.settings), roi=roi)
        canvas = image.copy()

        # Draw ROI
        if roi is not None:
            cv2.rectangle(canvas, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 255), 2)

        # Draw matches
        colors = {"CORRECT": (0, 200, 0), "INCORRECT": (0, 0, 255), "UNKNOWN": (0, 165, 255)}
        for match in report["matches"]:
            bx1, by1, bx2, by2 = match["bin_bbox"]
            color = colors[match["status"]]
            cv2.rectangle(canvas, (bx1, by1), (bx2, by2), color, 2)
            cv2.putText(
                canvas,
                match["bin_label"] + " " + match["status"],
                (bx1, max(16, by1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )
            for obj in match["garbage"]:
                gx1, gy1, gx2, gy2 = obj["bbox"]
                cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), color, 2)
                cv2.line(
                    canvas,
                    ((bx1 + bx2) // 2, by2),
                    ((gx1 + gx2) // 2, (gy1 + gy2) // 2),
                    color,
                    1,
                )
                cv2.putText(
                    canvas,
                    f"{obj['label']} {obj['confidence']:.2f}",
                    (gx1, max(16, gy1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                )

        # Draw unmatched waste
        for obj in report["unmatched"]:
            gx1, gy1, gx2, gy2 = obj["bbox"]
            cv2.rectangle(canvas, (gx1, gy1), (gx2, gy2), colors["UNKNOWN"], 2)

        report["duration_ms"] = round((time.monotonic() - start) * 1000, 1)
        self.last_result = report

        return encode_preview(
            canvas,
            [
                f"Trash: {report['placement_status']} | bins={len(bins)} waste={len(waste)} | {report['duration_ms']:.0f} ms",
                "ROI: yellow | Correct: green | Incorrect: red | Unknown: orange",
            ],
        )


# ---------------------------------------------------------------------------
# Encoding utility
# ---------------------------------------------------------------------------


def encode_preview(canvas: np.ndarray, lines: List[str]) -> bytes:
    """Draw text lines over black band, encode to JPEG."""
    h, w = canvas.shape[:2]
    cv2.rectangle(canvas, (0, 0), (w, min(h, 25 * len(lines) + 5)), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(
            canvas,
            text,
            (8, 22 + 25 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
    ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError("Failed to encode preview image")
    return encoded.tobytes()
