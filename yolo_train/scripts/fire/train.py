"""Fine-tune YOLO26 on the fire detection train/val dataset."""
import argparse
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[2]
REPO = ROOT.parent
DEFAULT_CONFIG = Path(__file__).with_name("config.yaml")


def train_path(value):
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--model", help="Weights path, absolute or relative to yolo_train/")
    parser.add_argument("--device", help="Override automatic selection: cpu, 0, or 0,1")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))

    data_file = train_path(config["data"])
    data = yaml.safe_load(data_file.read_text(encoding="utf-8"))
    # Dataset YAML paths in this repository are relative to the repository root.
    data_root = Path(data["path"])
    if not data_root.is_absolute():
        data_root = REPO / data_root
    data["path"] = str(data_root.resolve())
    for split in ("train", "val"):
        image_dir = data_root / data[split]
        label_dir = image_dir.parent / "labels"
        images = [p for p in image_dir.iterdir()
                  if p.suffix.lower() in {".jpg", ".jpeg", ".png"}]
        if not images or any(not (label_dir / (p.stem + ".txt")).is_file() for p in images):
            raise ValueError(f"Empty dataset or missing labels: {image_dir}")

    import torch
    from ultralytics import YOLO

    project = train_path(config["output"]["project"])
    weights = train_path(args.model or config["model"])
    weights.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights), task="detect")
    device = args.device if args.device is not None else (0 if torch.cuda.is_available() else "cpu")
    print(f"Model: {config['model']}; device: {device}")
    # An absolute dataset path avoids dependence on cwd or Ultralytics settings.
    with tempfile.TemporaryDirectory(prefix="fire_train_") as temp:
        resolved_data = Path(temp) / "data.yaml"
        resolved_data.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
        model.train(
            data=str(resolved_data), device=device,
            project=str(project), name=config["output"]["name"],
            exist_ok=False, save=True, plots=True, val=True,
            **config["train"],
        )
    print(f"Best weights: {model.trainer.best}")
    print(f"Last weights: {model.trainer.last}")


if __name__ == "__main__":
    main()
