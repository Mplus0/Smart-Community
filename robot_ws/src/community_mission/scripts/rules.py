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
    goals = route.get('goals', [])
    if not isinstance(goals, list) or not goals:
        raise ValueError('goals must be a nonempty list')
    for goal in goals:
        if (not isinstance(goal, (list, tuple)) or len(goal) != 3 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in goal)):
            raise ValueError('Every goal must contain finite [x, y, yaw]')
    keys = ('photo_idx', 'license_plate_idx', 'traffic_light_idx', 'fire_idx',
            'trash_can_idx', 'electrocar_idx')
    for key in keys:
        values = route.get(key)
        if not isinstance(values, list):
            raise ValueError(f'{key} must be a list')
        if len(values) != len(set(values)):
            raise ValueError(f'{key} contains duplicate indices')
        for index in values:
            if type(index) is not int or not 0 <= index < len(goals):
                raise ValueError(f'{key}: invalid index {index}')
    for key in ('announcement_index', 'record_start_index', 'record_stop_index'):
        index = route.get(key)
        if type(index) is not int or not 0 <= index < len(goals):
            raise ValueError(f'{key}: invalid index')
    if route['record_start_index'] >= route['record_stop_index']:
        raise ValueError('Recording stop must follow recording start')
    if route['electrocar_idx'] != sorted(route['electrocar_idx']):
        raise ValueError('Vehicle photo indices must follow route order')
    return route


def people_counts(objects, width, height):
    """Retain legacy 45% and 40% ROI ratios, scaled to actual image dimensions."""
    people = [d for d in objects if d.label in ('good', 'bad')]
    counts = Counter(d.label for d in people if d.box[3] >= height * 0.45)
    roi = [d.box for d in people if d.box[1] >= height * 0.45 and d.box[2] >= width * 0.4]
    bounds = None if not roi else (min(b[0] for b in roi), min(b[1] for b in roi),
                                  max(b[2] for b in roi), max(b[3] for b in roi))
    return counts['good'] + counts['bad'], counts['bad'], bounds


def illegal_parking_count(objects, people_bounds, width, height):
    if people_bounds is None:
        return 0
    margin = width * 50.0 / 1280.0
    # The old y threshold was 576 in a 960-high reference image.
    candidates = [d.box for d in objects if d.label in ('stand', 'fall')
                  and d.box[0] <= people_bounds[2] + margin
                  and d.box[2] >= people_bounds[0] - margin
                  and d.box[1] >= height * 0.6]
    unique = []
    for box in candidates:
        if not any(box[0] <= b[2] and b[0] <= box[2] and box[1] <= b[3] and b[1] <= box[3]
                   for b in unique):
            unique.append(box)
    areas = [max(0, b[2]-b[0]) * max(0, b[3]-b[1]) for b in unique]
    return sum(area > 0 and area >= max(areas) * 0.5 for area in areas) if areas else 0


def vertical_overlap(a, b):
    intersection = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = max(a[3], b[3]) - min(a[1], b[1])
    return intersection / union if union > 0 else 0.0


def vehicle_counts(frames):
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
            duplicate = i+1 < len(normalized) and detection.box[0] < 0.15 and any(
                other.box[2] > 0.7 and other.label == detection.label and
                vertical_overlap(detection.box, other.box) > 0.4 for other in normalized[i+1])
            if not duplicate:
                counts[detection.label] += 1
    return counts['stand'], counts['fall']


WASTE_NAMES = {}
for _prefix, _names in {
    'hazardous': '创可贴 注射器 医用手套 废旧电池 过期指甲油 漆桶 医用棉签 农药瓶',
    'kitchen': '橙皮 西红柿 废弃茶叶 香蕉皮 番茄酱 甘蔗 大蒜 饼干 面包 香菇 骨头',
    'orther': '脏污衣物 塑料袋 木质梳子 扫把 渣土 胶水 旧镜子 毛巾 牙膏皮 牙刷',
    'recoverable': '勺子 塑料瓶 旧夹子 玻璃瓶 坏垃圾桶 废电脑 易拉罐',
}.items():
    WASTE_NAMES.update({f'{_prefix}{i}': name for i, name in enumerate(_names.split(), 1)})


def waste_type(label):
    return next((kind for prefix, kind in [('red', '有害'), ('hazardous', '有害'),
                 ('green', '厨余'), ('kitchen', '厨余'), ('blue', '可回收'),
                 ('recoverable', '可回收')] if label.startswith(prefix)), '其他')


def trash_announcement(bins, waste, width):
    if not bins:
        return '未检测到垃圾桶'
    parts, all_correct = [], True
    for container in sorted(bins, key=lambda d: d.box[0]+d.box[2]):
        kind = waste_type(container.label)
        status = '开启' if 'open' in container.label else '关闭'
        center = (container.box[0]+container.box[2])/2
        contents = []
        for obj in waste:
            if obj.label not in WASTE_NAMES:
                continue
            if abs((obj.box[0]+obj.box[2])/2-center) <= width*60/1280:
                correct = waste_type(obj.label) == kind
                all_correct = all_correct and correct
                contents.append(WASTE_NAMES[obj.label] + ('投放正确' if correct else '投放错误'))
        parts.append(f'{kind}垃圾桶状态为{status}，' + ('垃圾为'+'、'.join(contents) if contents else '无垃圾'))
    return '垃圾桶识别结果：'+'；'.join(parts)+('，所有投放正确' if all_correct else '，存在投放错误')


def vehicle_announcement(illegal, stand, fall):
    return (f'A街区有{illegal["A"]}辆电动车违停，B街区有{illegal["B"]}辆电动车违停。'
            f'停车区内电动车正常{stand}辆，倒伏{fall}辆')
