"""Pure business rules; no ROS, camera or inference runtime required."""
from dataclasses import dataclass
from collections import Counter
import math
import re


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


def detection_records(objects, source):
    """JSON-safe detector evidence from the response used by the task itself."""
    return [dict(label=obj.label, bbox=list(obj.box), confidence=obj.confidence,
                 source=source) for obj in objects]


def ocr_detection_records(response, label):
    if len(response.bboxes) != 4 * len(response.plate_texts) or len(response.confidences) != len(response.plate_texts):
        raise ValueError('OCR 文字、置信度与坐标数量不一致')
    records = []
    for index, text in enumerate(response.plate_texts):
        box = list(response.bboxes[4*index:4*index+4])
        if not all(math.isfinite(v) for v in box) or box[2] <= box[0] or box[3] <= box[1]:
            raise ValueError('OCR 返回无效坐标')
        records.append(dict(label=label, text=text, bbox=box,
                            confidence=float(response.confidences[index]), source='ocr'))
    return records


def validate_route(route):
    points = route.get('points') if isinstance(route, dict) else None
    if not isinstance(points, list) or not points:
        raise ValueError('points must be a nonempty list')
    ids = set()
    allowed = {'navigation', 'person', 'plate', 'traffic_light', 'fire', 'trash', 'obstacle',
               'meter', 'temperature', 'vehicle_photo', 'announcement', 'record_start', 'record_stop'}
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
        if 'camera' in point:
            camera = point['camera']
            if camera not in ('main', 'meter'):
                raise ValueError(f'{name}: camera must be main or meter')
        if task == 'obstacle' and not isinstance(route.get('navigation_branches'), dict):
            raise ValueError('obstacle task requires navigation_branches')
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
    validate_navigation_branches(route)
    return route


def validate_navigation_branches(route):
    branches = route.get('navigation_branches')
    if branches is None:
        return
    if not isinstance(branches, dict) or set(branches) != {'clear', 'blocked'}:
        raise ValueError('navigation_branches requires clear and blocked')
    points = route['points']
    triggers = [i for i, p in enumerate(points) if p['task'] == 'obstacle']
    if len(triggers) != 1:
        raise ValueError('Branched routes require exactly one obstacle task')
    split = triggers[0]
    remaining = {p['id']: p for p in points[split + 1:]}
    required = {name for name, p in remaining.items() if p['task'] != 'navigation'}
    if any(p['id'].startswith('via_') and p['task'] != 'navigation' for p in points):
        raise ValueError('Transition waypoints must use task navigation')
    clear = branches['clear']
    if not isinstance(clear, list) or not clear:
        raise ValueError('clear branch must be a nonempty list')
    finish = route.get('finish_waypoint', clear[-1])
    if not isinstance(finish, str) or finish not in remaining:
        raise ValueError('finish_waypoint must follow the obstacle task')
    referenced = set()
    for mode, order in branches.items():
        if (not isinstance(order, list) or not order
                or any(not isinstance(name, str) for name in order)):
            raise ValueError(f'{mode}: branch must be a nonempty waypoint ID list')
        selected = set(order)
        if len(order) != len(selected) or not selected <= set(remaining):
            raise ValueError(f'{mode}: branch must use unique known waypoints')
        referenced.update(selected)
        if order[-1] != finish:
            raise ValueError(f'{mode}: branch must end at finish_waypoint')
        recording = False
        for point in points[:split + 1] + [remaining[name] for name in order]:
            if point['task'] == 'record_start':
                if recording:
                    raise ValueError(f'{mode}: recording already started')
                recording = True
            elif point['task'] == 'record_stop':
                if not recording:
                    raise ValueError(f'{mode}: recording stop must follow recording start')
                recording = False
        if recording:
            raise ValueError(f'{mode}: recording requires a stop task')
    if not required <= referenced:
        raise ValueError('Every branch task must be referenced by at least one branch')


def obstacle_route(objects, width, clear_x_ratio):
    """左侧标志对应直行 clear，中间标志对应绕行 blocked。"""
    obstacles = [obj for obj in objects if obj.label == 'obstacle']
    if len(obstacles) != 1:
        raise ValueError('Expected exactly one obstacle; route cannot be selected')
    obj = obstacles[0]
    if (not math.isfinite(obj.confidence) or not 0 < obj.confidence <= 1
            or not all(math.isfinite(v) for v in obj.box)
            or obj.box[2] <= obj.box[0] or obj.box[3] <= obj.box[1]):
        raise ValueError('Invalid obstacle detection; route cannot be selected')
    center_x_ratio = (obj.box[0] + obj.box[2]) / (2 * width)
    return dict(mode='clear' if center_x_ratio < clear_x_ratio else 'blocked',
                center_x_ratio=center_x_ratio, clear_x_ratio=clear_x_ratio,
                bbox=list(obj.box), confidence=obj.confidence)


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




def trash_roi(bins, width, height, settings):
    """合并桶框，并保留桶前方放置垃圾的区域。"""
    if not bins:
        return None
    bw = max(d.box[2] - d.box[0] for d in bins)
    bh = max(d.box[3] - d.box[1] for d in bins)
    return (max(0, math.floor(min(d.box[0] for d in bins) - bw * settings['trash_roi_side'])),
            max(0, math.floor(min(d.box[1] for d in bins) - bh * settings['trash_roi_top'])),
            min(width, math.ceil(max(d.box[2] for d in bins) + bw * settings['trash_roi_side'])),
            min(height, math.ceil(max(d.box[3] for d in bins) + bh * settings['trash_roi_bottom'])))


def trash_result(bins, waste, settings):
    """每个垃圾只归属横向最近的桶；漏检和重复对应不判为正确。"""
    ordered = sorted(bins, key=lambda d: d.box[0] + d.box[2])
    matches = [dict(bin_label=d.label, bin_type=settings['bin_types'][d.label.split('_')[0]],
                    bin_state='开启' if d.label.endswith('_open') else '关闭',
                    bin_bbox=list(d.box), bin_confidence=d.confidence,
                    garbage=[], status='UNKNOWN') for d in ordered]
    unmatched = []
    for obj in waste:
        item = dict(label=obj.label, type=settings['waste_types'][obj.label],
                    bbox=list(obj.box), confidence=obj.confidence)
        if not ordered:
            unmatched.append(item)
            continue
        cx, cy = (obj.box[0] + obj.box[2]) / 2, (obj.box[1] + obj.box[3]) / 2
        index = min(range(len(ordered)), key=lambda i: abs(cx - (ordered[i].box[0] + ordered[i].box[2]) / 2))
        box = ordered[index].box
        bw, bh = box[2] - box[0], box[3] - box[1]
        if (abs(cx - (box[0] + box[2]) / 2) <= bw * settings['trash_match_x_ratio']
                and box[1] + bh * settings['trash_match_y_start'] <= cy
                <= box[3] + bh * settings['trash_roi_bottom']):
            matches[index]['garbage'].append(item)
        else:
            unmatched.append(item)
    parts = []
    for match in matches:
        garbage = match['garbage']
        if len(garbage) == 1:
            match['status'] = 'CORRECT' if garbage[0]['type'] == match['bin_type'] else 'INCORRECT'
            detail = garbage[0]['type'] + ('垃圾，投放正确' if match['status'] == 'CORRECT' else '垃圾，投放错误')
        else:
            detail = '未识别到对应垃圾' if not garbage else '多个垃圾对应，无法确认'
        parts.append(f"{match['bin_type']}垃圾桶{match['bin_state']}，{detail}")
    complete = len(matches) == 4 and {m['bin_type'] for m in matches} == set(settings['bin_types'].values())
    status = ('INCORRECT' if any(m['status'] == 'INCORRECT' for m in matches)
              else 'CORRECT' if complete and not unmatched and all(m['status'] == 'CORRECT' for m in matches)
              else 'UNKNOWN')
    summary = {'CORRECT': '所有投放正确', 'INCORRECT': '存在投放错误', 'UNKNOWN': '识别不完整，无法确认全部投放结果'}[status]
    text = '垃圾桶识别结果：' + '；'.join(parts) + '；' + summary if matches else '未检测到垃圾桶'
    return dict(bins=[d.label for d in ordered], waste=[d.label for d in waste],
                matches=matches, unmatched=unmatched, placement_status=status, announcement=text)



def vehicle_announcement(illegal, stand, fall):
    return (f'A街区有{illegal["A"]}辆电动车违停，B街区有{illegal["B"]}辆电动车违停。'
            f'停车区内电动车正常{stand}辆，倒伏{fall}辆')


def building_name(texts):
    """Accept a single plausible building sign from the fire frame."""
    candidates = set()
    signs = re.compile(r'楼宇[A-D]|(?<![A-Z0-9])[A-D](?:号)?(?:大楼|大厦|楼|栋)|[\u4e00-\u9fff]{1,12}(?:大楼|大厦)')
    for text in texts:
        candidates.update(signs.findall(''.join(text.upper().split())))
    if not candidates:
        candidates.update(signs.findall(''.join(''.join(text.upper().split()) for text in texts)))
    return next(iter(candidates)) if len(candidates) == 1 else ''
