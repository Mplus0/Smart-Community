"""Build a 20:1 fire dataset with two lighting variants per training image."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import random
import shutil

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "datasets/fire/images"
OUTPUT = ROOT / "datasets/fire_aug"
SEED = 42


def read_image(path):
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Cannot decode image: {path}")
    return image


def yolo_labels(annotation):
    width, height = annotation["imageWidth"], annotation["imageHeight"]
    lines = []
    for shape in annotation["shapes"]:
        if shape["label"] != "fire" or shape["shape_type"] != "rectangle":
            raise ValueError(f"Unsupported shape: {shape}")
        points = np.asarray(shape["points"], dtype=float)
        if not np.isfinite(points).all():
            raise ValueError("Non-finite annotation coordinates")
        x1, y1 = np.maximum(points.min(axis=0), [0, 0])
        x2, y2 = np.minimum(points.max(axis=0), [width, height])
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError("Invalid annotation box")
        lines.append(f"0 {(x1+x2)/(2*width):.8f} {(y1+y2)/(2*height):.8f} "
                     f"{(x2-x1)/width:.8f} {(y2-y1)/height:.8f}")
    return "\n".join(lines) + ("\n" if lines else "")


def lighting_variant(image, kind, rng):
    ev = float(rng.uniform(-0.70, -0.25) if kind == "dark" else rng.uniform(0.20, 0.55))
    gamma = float(rng.uniform(1.00, 1.15) if kind == "dark" else rng.uniform(0.90, 1.00))
    contrast = float(rng.uniform(0.92, 1.08))
    temperature = float(rng.uniform(-0.025, 0.025))
    noise = float(rng.uniform(0.3, 1.0))
    pixels = np.arange(256, dtype=np.float32) / 255.0
    table = np.clip((np.power(pixels, gamma) * (2.0 ** ev) - 0.5)
                    * contrast + 0.5, 0, 1)
    table = np.round(table * 255).astype(np.uint8)
    transformed = cv2.LUT(image, table).astype(np.float32)
    transformed *= np.array([1-temperature, 1, 1+temperature], dtype=np.float32)
    transformed += rng.normal(0, noise, image.shape).astype(np.float32)
    transformed = np.clip(np.rint(transformed), 0, 255).astype(np.uint8)
    return transformed, dict(ev=ev, gamma=gamma, contrast=contrast,
                             temperature=temperature, noise_std=noise)


def main():
    if OUTPUT.exists():
        raise FileExistsError(f"Output already exists; refusing to overwrite: {OUTPUT}")
    images = sorted(p for p in SOURCE.iterdir()
                    if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"})
    annotations, labels, hashes = {}, {}, set()
    # Validate all labels before creating output. Identical originals cannot cross splits.
    for image in images:
        annotation = json.loads(image.with_suffix(".json").read_text(encoding="utf-8-sig"))
        labels[image.name] = yolo_labels(annotation)
        annotations[image.name] = annotation
        digest = hashlib.sha256(image.read_bytes()).digest()
        if digest in hashes:
            raise ValueError(f"Duplicate original image: {image}")
        hashes.add(digest)
    shuffled = list(range(len(images)))
    random.Random(SEED).shuffle(shuffled)
    validation = set(shuffled[:round(len(images) / 21)])
    for split in ("train", "val"):
        for folder in ("images", "labels"):
            (OUTPUT / split / folder).mkdir(parents=True)
    (OUTPUT / "images").mkdir()
    rng = np.random.default_rng(SEED)
    split_rows, augmentation_rows, preview_rows = [], [], []

    def save_entry(name, split, source, variant):
        shutil.copy2(OUTPUT / "images" / name, OUTPUT / split / "images" / name)
        (OUTPUT / split / "labels" / (Path(name).stem + ".txt")).write_text(
            labels[source], encoding="utf-8")
        split_rows.append(dict(source_name=source, image_name=name, split=split, variant=variant))

    for index, image in enumerate(images):
        split = "val" if index in validation else "train"
        shutil.copy2(image, OUTPUT / "images" / image.name)
        annotation = annotations[image.name]
        original = copy.deepcopy(annotation)
        original["imagePath"] = image.name
        (OUTPUT / "images" / (image.stem + ".json")).write_text(
            json.dumps(original, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        save_entry(image.name, split, image.name, "original")
        if split == "val":
            continue
        pixels = read_image(image)
        if (pixels.shape[1], pixels.shape[0]) != (annotation["imageWidth"], annotation["imageHeight"]):
            raise ValueError(f"Image dimensions disagree with annotation: {image}")
        previews = [pixels]
        for kind in ("dark", "bright"):
            name = f"{image.stem}__aug_{kind}.jpg"
            changed, params = lighting_variant(pixels, kind, rng)
            ok, encoded = cv2.imencode(".jpg", changed, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise ValueError(f"JPEG encoding failed: {name}")
            encoded.tofile(str(OUTPUT / "images" / name))
            augmented = copy.deepcopy(annotation)
            augmented.update(imagePath=name, imageData=None, checked=False)
            for shape in augmented["shapes"]:
                shape["review_status"] = "pending"
            (OUTPUT / "images" / (Path(name).stem + ".json")).write_text(
                json.dumps(augmented, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            save_entry(name, split, image.name, kind)
            augmentation_rows.append(dict(source_name=image.name, image_name=name, **params))
            previews.append(changed)
        if len(preview_rows) < 4:
            tiles = []
            for title, pixels in zip(("Original", "Darker exposure", "Brighter exposure"), previews):
                tile = cv2.resize(pixels, (480, 270))
                cv2.rectangle(tile, (0, 0), (480, 28), (25, 25, 25), -1)
                cv2.putText(tile, title, (12, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1)
                tiles.append(tile)
            preview_rows.append(np.hstack(tiles))
        if (index + 1) % 100 == 0:
            print(f"Processed {index+1}/{len(images)} originals", flush=True)
    for filename, rows in (("split_manifest.csv", split_rows),
                           ("augmentation_manifest.csv", augmentation_rows)):
        with (OUTPUT / filename).open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    (OUTPUT / "data.yaml").write_text(
        "path: yolo_train/datasets/fire_aug\ntrain: train/images\nval: val/images\nnames:\n  0: fire\n",
        encoding="utf-8")
    ok, preview = cv2.imencode(".jpg", np.vstack(preview_rows))
    if ok:
        preview.tofile(str(OUTPUT / "augmentation_preview.jpg"))
    print(f"Finished: {len(images)-len(validation)} training originals, "
          f"{len(augmentation_rows)} augmented images, {len(validation)} validation originals")


if __name__ == "__main__":
    main()
