"""Copy existing class folders into raw/ and reproducible train/val/test splits."""
import argparse
import filecmp
import math
from pathlib import Path
import random
import shutil

import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = "scripts/traffic_light/config.yaml"
CLASSES = ("red", "yellow", "green")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}


def repo_path(value):
    path = Path(value)
    return (ROOT / path).resolve() if not path.is_absolute() else path.resolve()


def load_config(path=DEFAULT_CONFIG):
    with repo_path(path).open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if config["model"] not in {"yolo11n-cls.pt", "yolo26n-cls.pt"}:
        raise ValueError("model must be yolo11n-cls.pt or yolo26n-cls.pt")
    return config


def images(folder):
    return sorted(p for p in folder.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS)


def check_classes(folder):
    actual = {p.name for p in folder.iterdir() if p.is_dir()}
    if actual != set(CLASSES):
        raise ValueError(f"Expected exactly {CLASSES} in {folder}, found {sorted(actual)}")
    for name in CLASSES:
        if not images(folder / name):
            raise ValueError(f"Empty class: {folder / name}")


def prepare(config, source=None):
    source = repo_path(source or config["data"]["source"])
    target = repo_path(config["data"]["root"])
    ratios = config["data"]["ratios"]
    if len(ratios) != 3 or any(r <= 0 for r in ratios) or not math.isclose(sum(ratios), 1.0):
        raise ValueError("data.ratios must contain three positive values summing to 1")
    # Only explicit class names are accepted; numerical labels are never guessed.
    for name in CLASSES:
        if not (source / name).is_dir():
            raise ValueError(f"Missing source class directory: {source / name}")
    rng = random.Random(config["train"]["seed"])
    copies, counts = [], {}
    for name in CLASSES:
        class_root = source / name
        files = images(class_root)
        rng.shuffle(files)
        n_train, n_val = int(len(files) * ratios[0]), int(len(files) * ratios[1])
        groups = (files[:n_train], files[n_train:n_train + n_val], files[n_train + n_val:])
        if any(not group for group in groups):
            raise ValueError(f"Not enough images to populate all splits for {name}: {len(files)}")
        counts[name] = [len(group) for group in groups]
        copies.extend((p, target / "raw" / name / p.relative_to(class_root)) for p in files)
        for split, group in zip(("train", "val", "test"), groups):
            copies.extend((p, target / split / name / p.relative_to(class_root)) for p in group)
    # Preflight every destination before writing. Existing identical copies are safe to reuse.
    for src, dst in copies:
        if dst.exists() and (not dst.is_file() or not filecmp.cmp(src, dst, shallow=False)):
            raise FileExistsError(f"Refusing to overwrite a different existing file: {dst}")
    expected = {dst for _, dst in copies}
    for split in ("train", "val", "test"):
        extra = set(images(target / split)) - expected
        if extra:
            raise FileExistsError(f"Existing split differs from this configuration: {next(iter(extra))}")
    for src, dst in copies:
        if not dst.exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    print("Class       train  val  test")
    for name, (train, val, test) in counts.items():
        print(f"{name:10} {train:5} {val:4} {test:5}")
    print(f"Dataset: {target}; seed={config['train']['seed']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--source", help="Repository-relative folder containing red/yellow/green")
    args = parser.parse_args()
    prepare(load_config(args.config), args.source)


if __name__ == "__main__":
    main()
