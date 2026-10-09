"""Evaluate best.pt on test/ or predict one image using --image."""
import argparse
import csv

from prepare_dataset import CLASSES, DEFAULT_CONFIG, check_classes, images, load_config, repo_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--weights", help="Repository-relative best.pt override")
    parser.add_argument("--image", help="Predict a single repository-relative image")
    args = parser.parse_args()
    config = load_config(args.config)

    import torch
    from ultralytics import YOLO

    weights = repo_path(args.weights or config["output"]["best"])
    if not weights.is_file():
        raise FileNotFoundError(f"Train a model first: {weights}")
    model = YOLO(str(weights), task="classify")
    if set(model.names.values()) != set(CLASSES) or len(model.names) != 3:
        raise ValueError(f"Model has unexpected classes: {model.names}")
    options = dict(imgsz=config["train"]["imgsz"], device=0 if torch.cuda.is_available() else "cpu", verbose=False)
    if args.image:
        result = model.predict(str(repo_path(args.image)), **options)[0]
        print(f"{result.names[result.probs.top1]} {float(result.probs.top1conf):.6f}")
        return

    test_root = repo_path(config["data"]["root"]) / "test"
    check_classes(test_root)
    matrix = [[0] * 3 for _ in CLASSES]
    # Read names from the model instead of assuming its numeric class order.
    for actual in CLASSES:
        for path in images(test_root / actual):
            result = model.predict(str(path), **options)[0]
            predicted = result.names[result.probs.top1]
            matrix[CLASSES.index(actual)][CLASSES.index(predicted)] += 1
    total = sum(map(sum, matrix))
    accuracy = sum(matrix[i][i] for i in range(3)) / total
    print(f"Test accuracy: {accuracy:.4%} ({total} images)")
    print("Confusion matrix: rows=true, columns=predicted; order=" + ", ".join(CLASSES))
    for name, row in zip(CLASSES, matrix):
        print(f"{name:8} {row}")
    output = repo_path(config["output"]["project"]) / "test"
    output.mkdir(parents=True, exist_ok=True)
    with (output / "confusion_matrix.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["true/predicted", *CLASSES])
        writer.writerows([name, *row] for name, row in zip(CLASSES, matrix))
    print(f"Confusion matrix saved: {output / 'confusion_matrix.csv'}")


if __name__ == "__main__":
    main()
