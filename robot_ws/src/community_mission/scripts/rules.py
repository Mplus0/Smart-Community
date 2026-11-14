"""Pure business rules; no ROS, camera or inference runtime required."""
from dataclasses import dataclass
from collections import Counter
import math


@dataclass(frozen=True)
class Detection:
    label: str
    box: tuple
    confidence: float = 0.0


def detections(response):
    """The service's per-object arrays share order; class_counts is only a summary."""
    labels = response.detected_classes
    if len(response.bboxes) != 4 * len(labels):
        raise ValueError('Detection labels and bounding boxes have different lengths')
    if len(response.confidences) != len(labels):
        raise ValueError('Detection labels and confidences have different lengths')
    return [Detection(label, tuple(response.bboxes[4*i:4*i+4]), response.confidences[i])
            for i, label in enumerate(labels)]


def validate_route(route):
    points = route.get('points') if isinstance(route, dict) else None
    if not isinstance(points, list) or not points:
        raise ValueError('points must be a nonempty list')
    ids = set()
    allowed = {'navigation', 'person', 'plate', 'traffic_light', 'fire', 'trash',
               'vehicle_photo', 'announcement', 'record_start', 'record_stop'}
    recording = False
    for point in points:
        if not isinstance(point, dict):
            raise ValueError('Every point must be a mapping')
        name = point.get('id')
        if not isinstance(name, str) or not name.strip() or name in ids:
            raise ValueError('Point ID must be nonempty and unique')
        ids.add(name)
        for key in ('x', 'y', 'yaw'):
            value = point.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'{name}: {key} must be finite')
        task = point.get('task')
        if not isinstance(task, str) or task not in allowed or 'tasks' in point:
            raise ValueError(f'{name}: task must name one supported task')
        if task == 'person' and point.get('area') not in ('A', 'B'):
            raise ValueError(f'{name}: person requires area A or B')
        if task == 'plate' and (type(point.get('parking_lot')) is not int or point['parking_lot'] < 1):
            raise ValueError(f'{name}: plate requires a positive parking_lot')
        if task == 'record_start':
            if recording: raise ValueError('Recording already started')
            recording = True
        if task == 'record_stop':
            if not recording: raise ValueError('Recording stop must follow recording start')
            recording = False
    if recording:
        raise ValueError('Recording requires a stop task')
    return route


def people_counts(objects, width, height, settings):
    """Retain legacy 45% and 40% ROI ratios, scaled to actual image dimensions."""
    people = [d for d in objects if d.label in ('community_person', 'outsider')]
    counts = Counter(d.label for d in people if d.box[3] >= height * settings['person_y_ratio'])
    roi = [d.box for d in people if d.box[1] >= height * settings['person_y_ratio'] and d.box[2] >= width * settings['person_x_ratio']]
    bounds = None if not roi else (min(b[0] for b in roi), min(b[1] for b in roi),
                                  max(b[2] for b in roi), max(b[3] for b in roi))
    return counts['community_person'] + counts['outsider'], counts['outsider'], bounds


def illegal_parking_count(objects, people_bounds, width, height, settings):
    if people_bounds is None:
        return 0
    margin = width * settings['parking_margin_px'] / settings['reference_width']
    # The old y threshold was 576 in a 960-high reference image.
    candidates = [d.box for d in objects if d.label in ('stand', 'fall')
                  and d.box[0] <= people_bounds[2] + margin
                  and d.box[2] >= people_bounds[0] - margin
                  and d.box[1] >= height * settings['parking_y_ratio']]
    unique = []
    for box in candidates:
        if not any(box[0] <= b[2] and b[0] <= box[2] and box[1] <= b[3] and b[1] <= box[3]
                   for b in unique):
            unique.append(box)
    areas = [max(0, b[2]-b[0]) * max(0, b[3]-b[1]) for b in unique]
    return sum(area > 0 and area >= max(areas) * settings['parking_area_ratio'] for area in areas) if areas else 0


def vertical_overlap(a, b):
    intersection = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = max(a[3], b[3]) - min(a[1], b[1])
    return intersection / union if union > 0 else 0.0


def vehicle_counts(frames, settings):
    """Deduplicate adjacent photo boundaries using the original 15%/30%/0.4 rules.

    Each entry is (objects, image_width, image_height). Coordinates are normalized
    so camera resolution changes do not alter the threshold meaning.
    """
    normalized = []
    for objects, width, height in frames:
        normalized.append([Detection(d.label, (d.box[0]/width, d.box[1]/height,
                                                d.box[2]/width, d.box[3]/height))
                           for d in objects if d.label in ('stand', 'fall')])
    counts = Counter()
    for i, objects in enumerate(normalized):
        for detection in objects:
            duplicate = i+1 < len(normalized) and detection.box[0] < settings['vehicle_left_ratio'] and any(
                other.box[2] > settings['vehicle_right_ratio'] and other.label == detection.label and
                vertical_overlap(detection.box, other.box) > settings['vehicle_overlap_ratio'] for other in normalized[i+1])
            if not duplicate:
                counts[detection.label] += 1
    return counts['stand'], counts['fall']




def waste_type(label, settings):
    return next((kind for prefix, kind in settings['waste_types'].items()
                 if label.startswith(prefix)), '其他')


def trash_announcement(bins, waste, width, settings):
    if not bins:
        return '未检测到垃圾桶'
    parts, all_correct = [], True
    for container in sorted(bins, key=lambda d: d.box[0]+d.box[2]):
        kind = waste_type(container.label, settings)
        status = '开启' if 'open' in container.label else '关闭'
        center = (container.box[0]+container.box[2])/2
        contents = []
        for obj in waste:
            if obj.label not in settings['waste_names']:
                continue
            if abs((obj.box[0]+obj.box[2])/2-center) <= width*settings['trash_match_px']/settings['reference_width']:
                correct = waste_type(obj.label, settings) == kind
                all_correct = all_correct and correct
                contents.append(settings['waste_names'][obj.label] + ('投放正确' if correct else '投放错误'))
        parts.append(f'{kind}垃圾桶状态为{status}，' + ('垃圾为'+'、'.join(contents) if contents else '无垃圾'))
    return '垃圾桶识别结果：'+'；'.join(parts)+('，所有投放正确' if all_correct else '，存在投放错误')


def vehicle_announcement(illegal, stand, fall):
    return (f'A街区有{illegal["A"]}辆电动车违停，B街区有{illegal["B"]}辆电动车违停。'
            f'停车区内电动车正常{stand}辆，倒伏{fall}辆')
