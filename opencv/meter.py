"""Windows tools for reading a Deli 9011 with a monocular camera."""
import argparse
import json
import math
from pathlib import Path
from time import monotonic

import cv2 as cv
import numpy as np


SCALES = {"temperature": range(-30, 51, 10), "humidity": range(0, 101, 10)}
READING_STEPS = {"temperature": 1, "humidity": 2}
CAMERA_RESOLUTIONS = {"720p": (1280, 720), "1080p": (1920, 1080), "4k": (3840, 2160)}
DEFAULT_PROFILE = Path(__file__).parent / "example" / "profile.json"


def load_image(path):
    image = cv.imdecode(np.fromfile(path, np.uint8), cv.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Cannot decode {path}")
    return image


def save_image(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, data = cv.imencode(path.suffix, image)
    if not ok:
        raise ValueError(f"Cannot encode {path}")
    data.tofile(path)


def dial_geometry(spec):
    center = np.asarray(spec["center"], np.float32)
    offsets = np.asarray(spec["ticks"], np.float32) - center
    angles = np.unwrap(np.arctan2(offsets[:, 1], offsets[:, 0]))
    if not np.all(np.diff(angles) > 0) or angles[-1] - angles[0] >= 2 * np.pi:
        raise ValueError("Click major ticks clockwise in increasing value order")
    return center, angles, np.median(np.linalg.norm(offsets, axis=1))


def read_needle(gray, spec, values):
    center, ticks, radius = dial_geometry(spec)
    angles = np.linspace(ticks[0], ticks[-1], max(200, int(radius * 5)))
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    normals = np.column_stack((-directions[:, 1], directions[:, 0]))
    radii = radius * np.linspace(0.28, 0.79, 48)
    rays = center + directions[:, None, :] * radii[None, :, None]

    def sample(offset):
        points = (rays + offset * normals[:, None, :]).astype(np.float32)
        return cv.remap(gray, points[:, :, 0], points[:, :, 1],
                        cv.INTER_LINEAR).astype(np.float32)

    middle = sample(0)
    width = max(3, radius * 0.048)
    # Reward a dark shaft with two lighter flanks along both halves.
    contrast = np.minimum(sample(width), sample(-width)) - middle
    score = np.minimum(contrast[:, :24].mean(1), contrast[:, 24:].mean(1))
    best = int(np.argmax(score))
    other = np.abs(angles - angles[best]) > np.deg2rad(7)
    if score[best] < 20 or score[best] - score[other].max() < 8:
        return None, None
    point = np.rint(center + directions[best] * radius).astype(int)
    return float(np.interp(angles[best], ticks, values)), tuple(point)


def quantize_reading(name, value):
    if value is None:
        return None
    step = READING_STEPS[name]
    return math.floor(value / step) * step


def valid_projection(matrix, face_quad, frame_shape):
    # A projective pole crossing the face folds it over in the warped image.
    depth = face_quad @ matrix[2, :2] + matrix[2, 2]
    if not (np.all(depth > 0) or np.all(depth < 0)):
        return False

    quad = cv.perspectiveTransform(face_quad[None, :, :], matrix)[0]
    if not np.isfinite(quad).all() or not cv.isContourConvex(quad):
        return False
    height, width = frame_shape[:2]
    if np.any(quad < 0) or np.any(quad >= (width, height)):
        return False
    sides = np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)
    return cv.contourArea(quad, oriented=True) >= 1600 and sides.min() >= 0.35 * sides.max()


class Reader:
    def __init__(self, path):
        self.profile = json.loads(path.read_text(encoding="utf-8-sig"))
        self.template = load_image(path.parent / self.profile["template"])
        self.features = cv.SIFT_create(nfeatures=2200)
        self.matcher = cv.BFMatcher()
        mask = np.zeros(self.template.shape[:2], np.uint8)
        # Limit reference features to the static face around the temperature pivot.
        center, _, radius = dial_geometry(self.profile["temperature"])
        cv.circle(mask, tuple(center.astype(int)), int(radius * 1.03), 255, -1)
        self.face_area = cv.countNonZero(mask)
        # Interior of the dial, with room for the outer rim to touch the frame edge.
        corners = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]])
        self.face_quad = center + corners * (0.65 * radius)
        gray = cv.cvtColor(self.template, cv.COLOR_BGR2GRAY)
        self.reference_features = []
        for scale in (1.0, 0.5, 0.25):
            scaled = cv.resize(gray, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
            scaled_mask = cv.resize(mask, (scaled.shape[1], scaled.shape[0]),
                                    interpolation=cv.INTER_NEAREST)
            keys, desc = self.features.detectAndCompute(scaled, scaled_mask)
            if desc is not None and len(keys) >= 12:
                points = np.float32([key.pt for key in keys]) / scale
                self.reference_features.append((points, desc))
        if not self.reference_features:
            raise ValueError("Template has insufficient detail")
        self.tracking_anchor = None

    def match_face(self, gray):
        factor = min(1, 1400 / max(gray.shape))
        small = cv.resize(gray, None, fx=factor, fy=factor, interpolation=cv.INTER_AREA)
        keys, desc = self.features.detectAndCompute(small, None)
        if desc is None or len(desc) < 2:
            return None
        points = np.float32([key.pt for key in keys]) / factor
        for reference, descriptors in self.reference_features:
            matches = self.matcher.knnMatch(descriptors, desc, k=2)
            good = [a for a, b in matches if a.distance < 0.72 * b.distance]
            ref = reference[[m.queryIdx for m in good]]
            src = points[[m.trainIdx for m in good]]
            alignment = self.fit_face(ref, src, gray.shape, 3 / factor)
            if alignment is not None:
                return alignment
        return None

    def fit_face(self, ref, src, shape, tolerance):
        if len(ref) < 12:
            return None
        matrix, inliers = cv.findHomography(ref, src, cv.RANSAC, tolerance)
        if matrix is None or inliers.sum() < max(12, 0.55 * len(ref)):
            return None
        if not valid_projection(matrix, self.face_quad, shape):
            return None
        keep = inliers.ravel().astype(bool)
        if cv.contourArea(cv.convexHull(ref[keep])) < 0.10 * self.face_area:
            return None
        return matrix, ref[keep], src[keep]

    def track_face(self, gray, now):
        if self.tracking_anchor is None:
            return None
        anchored_at, anchor, ref, src = self.tracking_anchor
        if now - anchored_at > 2.0 or anchor.shape != gray.shape:
            return None
        points = src.reshape(-1, 1, 2)
        moved, forward_ok, _ = cv.calcOpticalFlowPyrLK(anchor, gray, points, None)
        forward = forward_ok.ravel().astype(bool)
        if forward.sum() < 12:
            return None
        moved = moved[forward]
        returned, backward_ok, _ = cv.calcOpticalFlowPyrLK(gray, anchor, moved, None)
        error = np.linalg.norm((returned - points[forward])[:, 0], axis=1)
        keep = backward_ok.ravel().astype(bool) & (error < 1.5)
        if keep.sum() < 0.5 * len(points):
            return None
        return self.fit_face(ref[forward][keep], moved[keep, 0], gray.shape, 2)

    def read(self, frame, *, camera_view=False):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        now = monotonic()
        alignment = self.match_face(gray)
        status = "aligned"
        if camera_view:
            if alignment is None:
                alignment = self.track_face(gray, now)
                status = "tracking"
                if alignment is not None:
                    anchored_at = self.tracking_anchor[0]
                    _, ref, src = alignment
                    self.tracking_anchor = anchored_at, gray, ref, src
            else:
                _, ref, src = alignment
                self.tracking_anchor = now, gray, ref, src
        if alignment is None:
            self.tracking_anchor = None
            return dict.fromkeys(SCALES), frame.copy(), "alignment_failed"
        matrix, _, _ = alignment
        h, w = self.template.shape[:2]
        image = gray if camera_view else frame
        border = 255 if camera_view else (255, 255, 255)
        aligned = cv.warpPerspective(image, matrix, (w, h),
                                     flags=cv.INTER_LINEAR | cv.WARP_INVERSE_MAP,
                                     borderMode=cv.BORDER_CONSTANT, borderValue=border)
        aligned_gray = aligned if camera_view else cv.cvtColor(aligned, cv.COLOR_BGR2GRAY)
        overlay = frame.copy() if camera_view else aligned
        result = {}
        for name, values in SCALES.items():
            spec = self.profile[name]
            value, tip = read_needle(aligned_gray, spec, values)
            result[name] = value
            if tip is not None:
                points = np.float32([[spec["center"], tip]])
                if camera_view:
                    points = cv.perspectiveTransform(points, matrix)
                start, end = np.rint(points[0]).astype(int)
                cv.line(overlay, tuple(start), tuple(end), (255, 0, 255), 2, cv.LINE_AA)
        return result, overlay, status


def display_image(image):
    factor = min(1, 850 / image.shape[0], 1100 / image.shape[1])
    return cv.resize(image, None, fx=factor, fy=factor), factor


def calibrate(image_path, folder):
    image = load_image(image_path)
    folder.mkdir(parents=True, exist_ok=True)
    if any((folder / name).exists() for name in ("face.png", "profile.json")):
        raise ValueError("Choose a fresh calibration directory")
    preview, factor = display_image(image)
    profile = {"template": "face.png"}
    for name, values in SCALES.items():
        points = []
        labels = ["pivot"] + [f"{v} tick" for v in values]
        window = "Calibration"
        cv.namedWindow(window)

        def click(event, x, y, flags, data):
            if event == cv.EVENT_LBUTTONDOWN and len(points) < len(labels):
                points.append([round(x / factor), round(y / factor)])

        cv.setMouseCallback(window, click)
        while True:
            canvas = preview.copy()
            for point in points:
                cv.circle(canvas, tuple(np.rint(np.array(point) * factor).astype(int)), 3, (255, 0, 255), -1)
            prompt = labels[len(points)] if len(points) < len(labels) else "Enter to accept"
            cv.putText(canvas, f"{name}: {prompt} | U undo, Esc cancel", (8, 26),
                       cv.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 220), 2)
            cv.imshow(window, canvas)
            key = cv.waitKey(25) & 255
            if key == 27 or cv.getWindowProperty(window, cv.WND_PROP_VISIBLE) < 1:
                return
            if key in (ord("u"), 8) and points:
                points.pop()
            if key in (10, 13) and len(points) == len(labels):
                break
        profile[name] = {"center": points[0], "ticks": points[1:]}
        dial_geometry(profile[name])
    save_image(folder / "face.png", image)
    (folder / "profile.json").write_text(json.dumps(profile, indent=2), encoding="utf-8")


def open_camera(index, width, height):
    stream = cv.VideoCapture(index, cv.CAP_DSHOW)
    if not stream.isOpened():
        stream.release()
        raise ValueError(f"Cannot open camera {index}")
    stream.set(cv.CAP_PROP_FRAME_WIDTH, width)
    stream.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    return stream


def capture_template(index, output):
    if output.exists():
        raise ValueError(f"Choose a new output path: {output}")
    width, height = CAMERA_RESOLUTIONS["4k"]
    stream = open_camera(index, width, height)
    window = "4K template capture"
    try:
        while True:
            ok, frame = stream.read()
            if not ok:
                raise ValueError("Camera frame unavailable")
            if frame.shape[1::-1] != (width, height):
                actual_width, actual_height = frame.shape[1::-1]
                raise ValueError(f"Requested 3840x2160, got {actual_width}x{actual_height}")
            preview, _ = display_image(frame)
            cv.putText(preview, "3840x2160 | S save, Esc cancel", (8, 28),
                       cv.FONT_HERSHEY_SIMPLEX, 0.64, (255, 0, 255), 2)
            cv.imshow(window, preview)
            key = cv.waitKey(1) & 255
            if key == 27 or cv.getWindowProperty(window, cv.WND_PROP_VISIBLE) < 1:
                return
            if key in (ord("s"), ord("S")):
                save_image(output, frame)
                print(f"Saved {width}x{height}: {output}")
                return
    finally:
        stream.release()


def camera(reader, index, resolution="720p"):
    stream = open_camera(index, *CAMERA_RESOLUTIONS[resolution])
    try:
        while True:
            ok, frame = stream.read()
            if not ok:
                raise ValueError("Camera frame unavailable")
            values, overlay, status = reader.read(frame, camera_view=True)
            shown = {name: quantize_reading(name, value) for name, value in values.items()}
            canvas, _ = display_image(overlay)
            height, width = frame.shape[:2]
            label = f"{width}x{height}  {status}  C: {shown['temperature']}  %RH: {shown['humidity']}"
            cv.putText(canvas, label,
                       (8, 28), cv.FONT_HERSHEY_SIMPLEX, 0.64, (255, 0, 255), 2)
            cv.imshow("Meter", canvas)
            if cv.waitKey(1) & 255 in (27, ord("q")) or cv.getWindowProperty("Meter", cv.WND_PROP_VISIBLE) < 1:
                break
    finally:
        stream.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    read = modes.add_parser("read")
    read.add_argument("image", type=Path)
    read.add_argument("--overlay", type=Path)
    live = modes.add_parser("camera")
    live.add_argument("--index", type=int, default=0)
    live.add_argument("--resolution", choices=CAMERA_RESOLUTIONS, default="720p",
                      help="requested capture resolution (default: 720p)")
    capture = modes.add_parser("capture", help="save a full-resolution 4K reference frame")
    capture.add_argument("output", type=Path)
    capture.add_argument("--index", type=int, default=0)
    for sub in (read, live):
        sub.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    setup = modes.add_parser("calibrate")
    setup.add_argument("image", type=Path)
    setup.add_argument("folder", type=Path)
    args = parser.parse_args()
    try:
        if args.mode == "capture":
            capture_template(args.index, args.output)
            return
        if args.mode == "calibrate":
            calibrate(args.image, args.folder)
            return
        reader = Reader(args.profile)
        if args.mode == "camera":
            camera(reader, args.index, args.resolution)
            return
        values, overlay, status = reader.read(load_image(args.image))
        if args.overlay:
            save_image(args.overlay, overlay)
        readings = {name: quantize_reading(name, value) for name, value in values.items()}
        print(json.dumps({"status": status, **readings}))
    except (OSError, ValueError, cv.error) as exc:
        parser.exit(1, f"{exc}\n")
    finally:
        if args.mode in ("capture", "camera", "calibrate"):
            cv.destroyAllWindows()


if __name__ == "__main__":
    main()
