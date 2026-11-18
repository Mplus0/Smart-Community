"""Overlay visualization utilities for detection results.

The camera pipeline (camera_core) uses:
- draw_boxes: detection boxes with labels and confidence
- draw_trash_pipeline: trash bin + ROI + matching lines
- draw_banner: status lines on top of the frame

Kept for later special modes:
- draw_traffic_light_state: Large status display
- draw_thermal_building: Four-zone temperature visualization
- draw_fall_count: Person tracking with standing/fallen状态
- draw_license_plate: License plate detection with OCR text
- draw_obstacle_routing: Obstacle boxes with route decision

All drawing functions modify the image in-place and return it for chaining.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Basic detection visualization
# ---------------------------------------------------------------------------


_PALETTE = [
    (0, 220, 0),
    (0, 0, 255),
    (255, 160, 0),
    (0, 220, 220),
    (220, 0, 220),
    (220, 220, 0),
]


def draw_boxes(
    image: np.ndarray,
    detections: List[Dict[str, Any]],
    palette: Optional[List[Tuple[int, int, int]]] = None,
) -> np.ndarray:
    """Draw boxes with labels and confidence scores.

    detections: [{"box": (x1, y1, x2, y2), "label": str, "confidence": float,
                  "class_id": int (optional, picks a stable colour)}, ...]
    """
    palette = palette or _PALETTE
    for idx, det in enumerate(detections):
        x1, y1, x2, y2 = (int(v) for v in det["box"])
        label = det.get("label", "object")
        conf = det.get("confidence", 1.0)
        color = palette[det.get("class_id", idx) % len(palette)]
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"{label} {conf:.2f}", (x1, max(20, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return image


def draw_banner(image: np.ndarray, lines: List[str]) -> np.ndarray:
    """Draw white status lines on a black band across the top of the frame."""
    h, w = image.shape[:2]
    cv2.rectangle(image, (0, 0), (w, min(h, 25 * len(lines) + 5)), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(image, text, (8, 22 + 25 * index), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1)
    return image


# ---------------------------------------------------------------------------
# Trash pipeline visualization
# ---------------------------------------------------------------------------


def draw_trash_pipeline(
    image: np.ndarray, report: Dict[str, Any]
) -> np.ndarray:
    """Draw trash classification result: ROI + bins + waste + matching lines.

    report: output from trash_result(), with keys:
        - roi: (x1, y1, x2, y2) or None
        - matches: [{"bin_bbox": ..., "garbage": [...], "status": "CORRECT|INCORRECT|UNKNOWN", ...}]
        - unmatched: [{"bbox": ..., ...}]
        - placement_status: str
    """
    roi = report.get("roi")
    if roi is not None:
        x1, y1, x2, y2 = (int(v) for v in roi)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)

    colors = {
        "CORRECT": (0, 200, 0),
        "INCORRECT": (0, 0, 255),
        "UNKNOWN": (0, 165, 255),
    }

    # Draw matches
    for match in report.get("matches", []):
        bx1, by1, bx2, by2 = (int(v) for v in match["bin_bbox"])
        status = match.get("status", "UNKNOWN")
        color = colors.get(status, (128, 128, 128))

        cv2.rectangle(image, (bx1, by1), (bx2, by2), color, 2)
        cv2.putText(
            image,
            f"{match.get('bin_label', 'bin')} {status}",
            (bx1, max(16, by1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
        )

        # Draw garbage and matching lines
        for obj in match.get("garbage", []):
            gx1, gy1, gx2, gy2 = (int(v) for v in obj["bbox"])
            cv2.rectangle(image, (gx1, gy1), (gx2, gy2), color, 2)
            # Line from bin center bottom to garbage center
            cv2.line(
                image,
                ((bx1 + bx2) // 2, by2),
                ((gx1 + gx2) // 2, (gy1 + gy2) // 2),
                color,
                1,
            )
            cv2.putText(
                image,
                f"{obj.get('label', '?')} {obj.get('confidence', 0):.2f}",
                (gx1, max(16, gy1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

    # Draw unmatched waste
    for obj in report.get("unmatched", []):
        gx1, gy1, gx2, gy2 = (int(v) for v in obj["bbox"])
        cv2.rectangle(image, (gx1, gy1), (gx2, gy2), colors["UNKNOWN"], 2)

    # Status text at bottom center
    status_text = report.get("placement_status", "N/A")
    h, w = image.shape[:2]
    text_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
    tx = (w - text_size[0]) // 2
    ty = h - 20
    cv2.rectangle(image, (tx - 10, ty - text_size[1] - 10), (tx + text_size[0] + 10, ty + 10), (0, 0, 0), -1)
    cv2.putText(
        image, status_text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2
    )

    return image


# ---------------------------------------------------------------------------
# Traffic light state visualization
# ---------------------------------------------------------------------------


def draw_traffic_light_state(
    image: np.ndarray,
    detections: List[Dict[str, Any]],
    state: str = "UNKNOWN",
    frame_count: int = 0,
    timeout_s: Optional[float] = None,
) -> np.ndarray:
    """Draw traffic light boxes and large state display.

    state: "STOP" (red) or "GO" (green) or "UNKNOWN" (yellow)
    frame_count: consecutive frames with this state
    timeout_s: optional countdown timer
    """
    # Draw detection boxes
    for det in detections:
        x1, y1, x2, y2 = det["box"]
        label = det.get("label", "light")
        conf = det.get("confidence", 1.0)
        color = (0, 255, 255)  # Yellow for traffic light boxes
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image,
            f"{label} {conf:.2f}",
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    # Large state text in center
    h, w = image.shape[:2]
    state_colors = {"STOP": (0, 0, 255), "GO": (0, 255, 0), "UNKNOWN": (0, 255, 255)}
    color = state_colors.get(state, (128, 128, 128))

    state_text = state
    if timeout_s is not None:
        state_text += f" ({timeout_s:.1f}s)"

    text_size = cv2.getTextSize(state_text, cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4)[0]
    tx = (w - text_size[0]) // 2
    ty = h // 2

    # Background rectangle
    cv2.rectangle(
        image,
        (tx - 20, ty - text_size[1] - 20),
        (tx + text_size[0] + 20, ty + 20),
        (0, 0, 0),
        -1,
    )
    cv2.putText(image, state_text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 2.0, color, 4)

    # Frame count in corner
    cv2.putText(
        image,
        f"Frames: {frame_count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return image


# ---------------------------------------------------------------------------
# Thermal building D visualization
# ---------------------------------------------------------------------------


def draw_thermal_building(
    image: np.ndarray, zones: List[Dict[str, Any]]
) -> np.ndarray:
    """Draw four-zone thermal visualization.

    zones: [{"zone": "4F|3F|2F|1F", "max_temp": float, "status": "normal|confirming|alarm|nodata|timeout", "frames": int}, ...]
    """
    h, w = image.shape[:2]
    zone_width = w // 4

    status_colors = {
        "normal": (0, 255, 0),
        "confirming": (0, 255, 255),
        "alarm": (0, 0, 255),
        "nodata": (128, 128, 128),
        "timeout": (255, 165, 0),
    }

    for idx, zone_data in enumerate(zones):
        zone_name = zone_data.get("zone", f"Zone{idx}")
        max_temp = zone_data.get("max_temp", 0.0)
        status = zone_data.get("status", "nodata")
        frames = zone_data.get("frames", 0)

        x1 = idx * zone_width
        x2 = (idx + 1) * zone_width
        color = status_colors.get(status, (128, 128, 128))

        # Vertical line separator
        if idx > 0:
            cv2.line(image, (x1, 0), (x1, h), (255, 255, 255), 2)

        # Zone text
        text_y = 40
        cv2.putText(
            image,
            zone_name,
            (x1 + 10, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )
        cv2.putText(
            image,
            f"{max_temp:.1f}°C",
            (x1 + 10, text_y + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
        cv2.putText(
            image,
            f"{status} ({frames}f)",
            (x1 + 10, text_y + 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
        )

    return image


# ---------------------------------------------------------------------------
# Fall detection visualization
# ---------------------------------------------------------------------------


def draw_fall_count(
    image: np.ndarray, persons: List[Dict[str, Any]]
) -> np.ndarray:
    """Draw person tracking with standing/fallen state.

    persons: [{"box": (x1, y1, x2, y2), "track_id": int, "state": "standing|fallen", "fall_count": int}, ...]
    """
    for person in persons:
        x1, y1, x2, y2 = person["box"]
        track_id = person.get("track_id", 0)
        state = person.get("state", "unknown")
        fall_count = person.get("fall_count", 0)

        color = (0, 255, 0) if state == "standing" else (0, 0, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image,
            f"ID:{track_id} {state}",
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    # Total fall count in corner
    total_falls = sum(p.get("fall_count", 0) for p in persons)
    cv2.rectangle(image, (0, 0), (250, 30), (0, 0, 0), -1)
    cv2.putText(
        image,
        f"Falls: {total_falls}",
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return image


# ---------------------------------------------------------------------------
# License plate visualization
# ---------------------------------------------------------------------------


def draw_license_plate(
    image: np.ndarray, plates: List[Dict[str, Any]]
) -> np.ndarray:
    """Draw license plate boxes with OCR text.

    plates: [{"box": (x1, y1, x2, y2), "text": str, "confidence": float}, ...]
    """
    for plate in plates:
        x1, y1, x2, y2 = plate["box"]
        text = plate.get("text", "")
        conf = plate.get("confidence", 0.0)

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            image,
            f"{text} ({conf:.2f})",
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    return image


# ---------------------------------------------------------------------------
# Obstacle routing visualization
# ---------------------------------------------------------------------------


def draw_obstacle_routing(
    image: np.ndarray,
    detections: List[Dict[str, Any]],
    decision: str = "clear",
    center_x_ratio: Optional[float] = None,
) -> np.ndarray:
    """Draw obstacle boxes and route decision (blocked/clear).

    decision: "blocked" or "clear"
    center_x_ratio: optional X position ratio of main obstacle (0.0 = left, 1.0 = right)
    """
    for det in detections:
        x1, y1, x2, y2 = det["box"]
        label = det.get("label", "obstacle")
        conf = det.get("confidence", 1.0)
        color = (0, 0, 255) if decision == "blocked" else (0, 255, 0)

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image,
            f"{label} {conf:.2f}",
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

        if center_x_ratio is not None:
            cx = (x1 + x2) // 2
            cv2.circle(image, (cx, (y1 + y2) // 2), 5, (255, 0, 0), -1)

    # Decision arrow
    h, w = image.shape[:2]
    arrow_color = (0, 0, 255) if decision == "blocked" else (0, 255, 0)
    arrow_text = "BLOCKED" if decision == "blocked" else "CLEAR"

    cv2.putText(
        image,
        arrow_text,
        (w // 2 - 60, h - 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        arrow_color,
        3,
    )

    if decision == "blocked":
        # Draw X
        cv2.line(image, (w // 2 - 40, h - 80), (w // 2 + 40, h - 40), arrow_color, 4)
        cv2.line(image, (w // 2 - 40, h - 40), (w // 2 + 40, h - 80), arrow_color, 4)
    else:
        # Draw arrow up
        cv2.arrowedLine(
            image, (w // 2, h - 40), (w // 2, h - 100), arrow_color, 4, tipLength=0.3
        )

    return image
