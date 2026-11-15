"""Convert real traffic-light rectangle JSON annotations to YOLO train/val data."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import random
import shutil

from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[2]
CLASSES = ('stop', 'go')


def train_path(value):
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def read_samples(source):
    images = sorted(p for p in source.iterdir() if p.suffix.lower() in {'.jpg', '.jpeg', '.png'})
    if not images:
        raise ValueError(f'No images: {source}')
    samples, stems = [], set()
    for image in images:
        if image.stem in stems:
            raise ValueError(f'Duplicate image stem: {image}')
        stems.add(image.stem)
        data = json.loads(image.with_suffix('.json').read_text(encoding='utf-8'))
        width, height = data['imageWidth'], data['imageHeight']
        with Image.open(image) as opened:
            if opened.size != (width, height):
                raise ValueError(f'Annotation/image size mismatch: {image}')
        lines, labels = [], []
        for shape in data['shapes']:
            label = shape['label']
            points = shape['points']
            if label not in CLASSES or shape['shape_type'] != 'rectangle' or len(points) not in (2, 4):
                raise ValueError(f'Unsupported label or shape: {image}: {label}')
            xs, ys = zip(*points)
            if not all(math.isfinite(v) for v in (*xs, *ys)):
                raise ValueError(f'Non-finite rectangle: {image}')
            x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
            if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
                raise ValueError(f'Invalid rectangle: {image}')
            values = ((x0+x1)/(2*width), (y0+y1)/(2*height), (x1-x0)/width, (y1-y0)/height)
            lines.append(str(CLASSES.index(label)) + ' ' + ' '.join(f'{v:.8f}' for v in values))
            labels.append(label)
        samples.append((image, lines, labels))
    return samples


def split_samples(samples, val_ratio, seed):
    # Stratify by the set of labels present, retaining empty images as backgrounds.
    groups = defaultdict(list)
    for sample in samples:
        groups[tuple(sorted(set(sample[2])))].append(sample)
    rng = random.Random(seed)
    splits = {'train': [], 'val': []}
    for key in sorted(groups):
        group = groups[key]
        rng.shuffle(group)
        count = min(len(group)-1, max(1, round(len(group)*val_ratio))) if len(group) > 1 else 0
        splits['val'].extend(group[:count])
        splits['train'].extend(group[count:])
    if not splits['val']:
        raise ValueError('Not enough images for a validation split')
    for name, group in splits.items():
        counts = Counter(label for _, _, labels in group for label in labels)
        if set(counts) != set(CLASSES):
            raise ValueError(f'{name} does not contain every class: {counts}')
    return splits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='datasets/traffic_light_real/images')
    parser.add_argument('--output', default='datasets/traffic_light_real_yolo')
    parser.add_argument('--val-ratio', type=float, default=0.1)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--check-only', action='store_true', help='Validate and report without writing files')
    args = parser.parse_args()
    if not 0 < args.val_ratio < 1:
        parser.error('--val-ratio must be between 0 and 1')
    splits = split_samples(read_samples(train_path(args.source)), args.val_ratio, args.seed)
    for name, group in splits.items():
        counts = Counter(label for _, _, labels in group for label in labels)
        print(f'{name}: {len(group)} images, boxes={dict(counts)}, backgrounds={sum(not s[1] for s in group)}')
    if args.check_only:
        return
    target = train_path(args.output)
    # A fresh directory prevents stale files from leaking between dataset splits.
    target.mkdir(parents=True, exist_ok=False)
    for name, group in splits.items():
        image_dir, label_dir = target / name / 'images', target / name / 'labels'
        image_dir.mkdir(parents=True)
        label_dir.mkdir(parents=True)
        for image, lines, _ in group:
            shutil.copy2(image, image_dir / image.name)
            (label_dir / (image.stem+'.txt')).write_text('\n'.join(lines) + ('\n' if lines else ''), encoding='utf-8')
    data = dict(path=target.as_posix(), train='train/images', val='val/images', names=dict(enumerate(CLASSES)))
    (target / 'data.yaml').write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    print(f'Dataset: {target}')


if __name__ == '__main__':
    main()
