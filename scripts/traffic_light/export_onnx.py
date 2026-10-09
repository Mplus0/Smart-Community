"""Export trained best.pt to ONNX and check the generated model."""
import argparse
from pathlib import Path
import shutil

from prepare_dataset import CLASSES, DEFAULT_CONFIG, load_config, repo_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--weights", help="Repository-relative best.pt override")
    args = parser.parse_args()
    config = load_config(args.config)

    import onnx
    from ultralytics import YOLO

    weights = repo_path(args.weights or config["output"]["best"])
    if not weights.is_file():
        raise FileNotFoundError(f"Train a model first: {weights}")
    model = YOLO(str(weights), task="classify")
    if set(model.names.values()) != set(CLASSES) or len(model.names) != 3:
        raise ValueError(f"Model has unexpected classes: {model.names}")
    exported = Path(model.export(format="onnx", imgsz=config["train"]["imgsz"],
                                 opset=config["export"]["opset"], device="cpu",
                                 simplify=False, dynamic=False, batch=1))
    if not exported.is_file():
        raise FileNotFoundError(f"ONNX export did not produce a file: {exported}")
    onnx.checker.check_model(str(exported))
    destination = repo_path(config["export"]["path"])
    destination.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != destination:
        shutil.copy2(exported, destination)
    print(f"Validated ONNX model: {destination}")


if __name__ == "__main__":
    main()
