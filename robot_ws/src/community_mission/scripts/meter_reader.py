"""Read temperature and humidity from images of a Deli 9011."""
import json
import math
from pathlib import Path
from time import monotonic

import cv2 as cv
import numpy as np


SCALES = {"temperature": range(-30, 51, 10), "humidity": range(0, 101, 10)}
READING_STEPS = {"temperature": 1, "humidity": 1}


def load_image(path):
    image = cv.imdecode(np.fromfile(path, np.uint8), cv.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Cannot decode {path}")
    return image


def save_image(path, image):
    ok, data = cv.imencode(path.suffix, image)
    if not ok:
        raise ValueError(f"Cannot encode {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    data.tofile(path)


def draw_inlier_matches(template, frame, reference_points, frame_points, max_side=1000):
    """Draw accepted matches between the template and the original frame."""
    previews = []
    keypoints = []
    indices = np.linspace(0, len(reference_points) - 1,
                          num=min(len(reference_points), 60), dtype=int)
    for image, points in ((template, reference_points), (frame, frame_points)):
        scale = min(1.0, max_side / max(image.shape[:2]))
        previews.append(cv.resize(image, None, fx=scale, fy=scale,
                                  interpolation=cv.INTER_AREA))
        keypoints.append([cv.KeyPoint(float(x), float(y), 5)
                          for x, y in points[indices] * scale])
    matches = [cv.DMatch(index, index, 0.0) for index in range(len(indices))]
    return cv.drawMatches(previews[0], keypoints[0], previews[1], keypoints[1],
                          matches, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)


def dial_geometry(spec):
    center = np.asarray(spec["center"], np.float32)
    offsets = np.asarray(spec["ticks"], np.float32) - center
    angles = np.unwrap(np.arctan2(offsets[:, 1], offsets[:, 0]))
    if not np.all(np.diff(angles) > 0) or angles[-1] - angles[0] >= 2 * np.pi:
        raise ValueError("Specify major ticks clockwise in increasing value order")
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
    def __init__(self, path, *, match_max_side=1400):
        self.profile = json.loads(path.read_text(encoding="utf-8-sig"))
        self.match_max_side = match_max_side
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
        factor = min(1, self.match_max_side / max(gray.shape))
        match_image = gray
        if factor < 1:
            match_image = cv.resize(gray, None, fx=factor, fy=factor, interpolation=cv.INTER_AREA)
        keys, desc = self.features.detectAndCompute(match_image, None)
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

    def read(self, frame, *, camera_view=False, debug=None):
        """Read one frame and optionally collect intermediate images and alignment data."""
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        if debug is not None:
            debug["gray"] = gray
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
        matrix, ref, src = alignment
        h, w = self.template.shape[:2]
        image = gray if camera_view else frame
        border = 255 if camera_view else (255, 255, 255)
        aligned = cv.warpPerspective(image, matrix, (w, h),
                                     flags=cv.INTER_LINEAR | cv.WARP_INVERSE_MAP,
                                     borderMode=cv.BORDER_CONSTANT, borderValue=border)
        aligned_gray = aligned if camera_view else cv.cvtColor(aligned, cv.COLOR_BGR2GRAY)
        if debug is not None:
            debug.update(matrix=matrix, reference_points=ref, frame_points=src,
                         aligned=aligned if camera_view else aligned.copy(), aligned_gray=aligned_gray)
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
