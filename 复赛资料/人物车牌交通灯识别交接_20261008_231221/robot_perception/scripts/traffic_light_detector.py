"""Image-only detector for the competition's horizontal three-lamp housing.

Scores are heuristics, not calibrated probabilities. No simulator state is used.
"""
import itertools
import math

import cv2
import numpy as np

COLORS = ('RED', 'YELLOW', 'GREEN')


def color_masks(hsv, saturation, value):
    red = cv2.inRange(hsv, (0, saturation, value), (10, 255, 255))
    red |= cv2.inRange(hsv, (170, saturation, value), (179, 255, 255))
    yellow = cv2.inRange(hsv, (11, saturation, value), (39, 255, 255))
    green = cv2.inRange(hsv, (40, saturation, value), (95, 255, 255))
    return [red, yellow, green]


def iou(a, b):
    width = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = width * height
    union = ((a[2]-a[0]) * (a[3]-a[1]) +
             (b[2]-b[0]) * (b[3]-b[1]) - intersection)
    return intersection / union if union > 0 else 0.0


class Detector:
    def __init__(self, params):
        self.p = params

    def analyze(self, hsv, box):
        x1, y1, x2, y2 = box
        crop = hsv[y1:y2, x1:x2]
        h, w = crop.shape[:2]
        yy, xx = np.ogrid[:h, :w]
        # Sample the lamp interior rather than its black surround/screws.
        disk = (((xx - (w-1)/2) / max(1, w*0.43)) ** 2 +
                ((yy - (h-1)/2) / max(1, h*0.43)) ** 2 <= 1)
        pixels = crop[disk]
        masks = color_masks(crop, self.p['min_saturation'], self.p['candidate_value'])
        fractions = [float(np.mean(mask[disk] > 0)) for mask in masks]
        bright = pixels[:, 2] >= self.p['bright_value']
        bright_ratio = float(np.mean(bright))
        mean_value = float(np.mean(pixels[:, 2]))
        # White/yellow highlights in a lit red lamp count as brightness, not hue.
        score = 0.55 * bright_ratio + 0.45 * mean_value / 255.0
        return {'bbox_xyxy': list(box), 'color': COLORS[int(np.argmax(fractions))],
                'color_fraction': max(fractions), 'mean_value': mean_value,
                'bright_fraction': bright_ratio, 'activation_score': score}

    def locate(self, hsv):
        height, width = hsv.shape[:2]
        masks = color_masks(hsv, self.p['min_saturation'], self.p['candidate_value'])
        mask = masks[0] | masks[1] | masks[2]
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lamps = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            perimeter = cv2.arcLength(contour, True)
            if (area < self.p['min_lamp_area'] or area > width*height*0.08
                    or min(w, h) < 4 or not 0.45 <= w/h <= 1.8
                    or perimeter <= 0 or 4*math.pi*area/perimeter**2 < 0.25):
                continue
            lamp = self.analyze(hsv, (x, y, x+w, y+h))
            if lamp['color_fraction'] < self.p['min_color_fraction']:
                continue
            lamps.append(lamp)
        lamps.sort(key=lambda lamp: (lamp['bbox_xyxy'][2]-lamp['bbox_xyxy'][0]) *
                   (lamp['bbox_xyxy'][3]-lamp['bbox_xyxy'][1]), reverse=True)
        lamps = lamps[:self.p['max_candidates']]
        groups = []
        for three in itertools.combinations(lamps, 3):
            three = sorted(three, key=lambda lamp: sum(lamp['bbox_xyxy'][::2]))
            order = [lamp['color'] for lamp in three]
            if order not in [list(COLORS), list(reversed(COLORS))]:
                continue
            boxes = [lamp['bbox_xyxy'] for lamp in three]
            cx = [(b[0]+b[2])/2 for b in boxes]
            cy = [(b[1]+b[3])/2 for b in boxes]
            diameters = [max(b[2]-b[0], b[3]-b[1]) for b in boxes]
            d = float(np.mean(diameters))
            gaps = [cx[1]-cx[0], cx[2]-cx[1]]
            if (max(diameters)/min(diameters) > 2.0 or max(cy)-min(cy) > d*0.7
                    or min(gaps) < d*1.05 or max(gaps) > d*3.0
                    or max(gaps)/min(gaps) > 1.6):
                continue
            bbox = [min(b[0] for b in boxes), min(b[1] for b in boxes),
                    max(b[2] for b in boxes), max(b[3] for b in boxes)]
            # Require dark material around the row, like the supplied housing.
            pad = max(2, round(d*0.15))
            expanded = [max(0, bbox[0]-pad), max(0, bbox[1]-pad),
                        min(width, bbox[2]+pad), min(height, bbox[3]+pad)]
            x1, y1, x2, y2 = expanded
            dark_ratio = float(np.mean(hsv[y1:y2, x1:x2, 2] < 100))
            if dark_ratio < self.p['min_dark_fraction']:
                continue
            quality = min(lamp['color_fraction'] for lamp in three)
            quality *= max(0, 1 - (max(cy)-min(cy))/(d*1.4))
            groups.append({'bbox_xyxy': expanded, 'lamps': three,
                           'localization_score': quality, 'mode': 'auto'})
        groups.sort(key=lambda group: group['localization_score'], reverse=True)
        selected = []
        for group in groups:
            if any(iou(group['bbox_xyxy'], other['bbox_xyxy']) > 0.3 for other in selected):
                continue
            selected.append(group)
        return selected[:self.p['max_groups']]

    def detect(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        groups = self.locate(hsv)
        for group in groups:
            ranked = sorted(group['lamps'], key=lambda lamp: lamp['activation_score'], reverse=True)
            best, second = ranked[:2]
            margin = best['activation_score'] - second['activation_score']
            failed = []
            if best['bright_fraction'] < self.p['min_bright_fraction']:
                failed.append('bright_fraction_low')
            if best['mean_value'] < self.p['min_mean_value']:
                failed.append('mean_value_low')
            if margin < self.p['min_activation_margin']:
                failed.append('activation_margin_low')
            active = not failed
            group['decision_reason'] = ','.join(failed) if failed else 'passed_brightness_gates'
            group['raw_state'] = best['color'] if active else 'UNKNOWN'
            group['score'] = float(min(group['localization_score'],
                                       max(0.0, margin) * 2.0)) if active else 0.0
            group['activation_margin'] = float(margin)
        return groups


class ConfirmStates:
    """Short-lived image tracks; GREEN requires consecutive fresh observations."""
    def __init__(self, frames=3, max_gap=0.6):
        self.frames, self.max_gap = frames, max_gap
        self.tracks, self.next_id = {}, 1

    def clear(self):
        self.tracks.clear()

    def update(self, groups, now):
        previous = self.tracks
        updated, used = {}, set()
        for group in groups:
            matches = [(iou(group['bbox_xyxy'], track['bbox']), key)
                       for key, track in previous.items()
                       if key not in used and now-track['time'] <= self.max_gap]
            overlap, key = max(matches, default=(0, None))
            if overlap < 0.3:
                key = self.next_id
                self.next_id += 1
                old = None
            else:
                old = previous[key]
                used.add(key)
            raw = group['raw_state']
            count = old['count']+1 if old and old['raw'] == raw else 1
            stable = raw if raw in ('RED', 'YELLOW') or (raw == 'GREEN' and count >= self.frames) else 'UNKNOWN'
            group.update(track_id=key, state=stable, valid=stable != 'UNKNOWN',
                         consecutive_frames=count)
            updated[key] = {'bbox': group['bbox_xyxy'], 'raw': raw,
                            'count': count, 'time': now}
        # A missed/ambiguous detection does not hold an old green state.
        self.tracks = updated
        return groups
