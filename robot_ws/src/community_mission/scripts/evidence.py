"""Render per-task evidence from existing results; never run inference here."""
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from result_manager import task_summary


def evidence_font(size, configured=''):
    candidates = [configured] if configured else [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
        '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
        '/usr/share/fonts/truetype/arphic/uming.ttc',
        '/System/Library/Fonts/Supplemental/Arial Unicode.ttf',
    ]
    for path in candidates:
        if path and Path(path).is_file():
            return ImageFont.truetype(path, size)
    raise RuntimeError('未找到中文标注字体，请通过 evidence_font 指定中文 TTF/TTC 字体路径')


def wrapped_lines(text, font, width):
    lines, current = [], ''
    for character in str(text).replace('\n', ' '):
        if current and font.getlength(current + character) > width:
            lines.append(current)
            current = ''
        current += character
    return lines + [current]


def save_annotation(frame, record, path, font_path=''):
    """Save a copy with detector boxes and a separate footer for exact task results."""
    image = Image.fromarray(frame[:, :, ::-1].copy())  # cv_bridge BGR -> Pillow RGB
    width, height = image.size
    size = max(14, min(32, width // 60))
    font = evidence_font(size, font_path)
    draw = ImageDraw.Draw(image)
    data = record.get('result') or {}
    skipped = 0
    labels = []
    for item in data.get('detections', []):
        box = item['bbox']
        if len(box) != 4 or not all(math.isfinite(value) for value in box):
            skipped += 1
            continue
        x1, y1, x2, y2 = box
        x1, x2 = max(0, min(width - 1, x1)), max(0, min(width - 1, x2))
        y1, y2 = max(0, min(height - 1, y1)), max(0, min(height - 1, y2))
        if x2 <= x1 or y2 <= y1:
            skipped += 1
            continue
        color = (255, 100, 70) if item['label'] in ('fire', 'fall', 'outsider', 'obstacle') else (70, 230, 140)
        draw.rectangle((x1, y1, x2, y2), outline=color, width=max(2, size // 8))
        confidence = item['confidence']
        score = f'{confidence:.2f}' if math.isfinite(confidence) else '?'
        label = f"{item['source']}: {item['label']} {score}"
        while font.getlength(label) > width - 8 and label:
            label = label[:-1]
        labels.append((label, color, (x1, y1, x2, y2)))
    if data.get('people_roi') is not None:
        # This is a person-derived reference ROI, not a claim that every vehicle is illegal.
        box = data['people_roi']
        if len(box) == 4 and all(math.isfinite(value) for value in box):
            x1, y1, x2, y2 = box
            x1, x2 = max(0, min(width - 1, x1)), max(0, min(width - 1, x2))
            y1, y2 = max(0, min(height - 1, y1)), max(0, min(height - 1, y2))
            if x2 > x1 and y2 > y1:
                draw.rectangle((x1, y1, x2, y2), outline=(255, 210, 80), width=2)
    occupied = []
    for label, color, (x1, y1, x2, y2) in labels:
        label_width = font.getlength(label)
        tx = max(2, min(x1, width - label_width - 4))
        candidates = []
        for position in (y1 - size - 8, y2 + 2, y1 + 2, y1 - 2 * (size + 8)):
            ty = max(0, min(position, height - size - 8))
            box = (tx - 2, ty, tx + label_width + 2, ty + size + 8)
            overlap = sum(max(0, min(box[2], other[2]) - max(box[0], other[0])) *
                          max(0, min(box[3], other[3]) - max(box[1], other[1]))
                          for other in occupied)
            candidates.append((overlap, abs(ty - y1), box))
        _, _, box = min(candidates)
        occupied.append(box)
        draw.rectangle(box, fill=(20, 25, 30))
        draw.text((box[0] + 2, box[1]), label, font=font, fill=color)
    lines = [f"航点={record['waypoint_id']} 任务={record['task_type']} 状态={record['status']}"]
    if record['status'] != 'FAILED':
        lines.append(task_summary(record['task_type'], data))
    if record.get('reason'):
        lines.append(record['reason'])
    if record['task_type'] == 'plate':
        confidence = data.get('plate_confidence')
        lines.append('OCR 置信度=' + ('未提供' if confidence is None else f'{confidence:.3f}'))
    if record['task_type'] == 'person':
        lines.append('检测框为原始检测；人数和违停按任务 ROI 规则统计，黄框为人员参考区域')
    if skipped:
        lines.append(f'已忽略 {skipped} 个无法绘制的检测框')
    lines = [line for text in lines for line in wrapped_lines(text, font, width - 16)]
    line_height = size + 12
    canvas = Image.new('RGB', (width, height + len(lines) * line_height + 16), (20, 25, 30))
    canvas.paste(image, (0, 0))
    footer = ImageDraw.Draw(canvas)
    for index, line in enumerate(lines):
        footer.text((8, height + 8 + index * line_height), line, font=font, fill=(245, 245, 245))
    canvas.save(str(path), format='JPEG', quality=92)
    return Path(path)
