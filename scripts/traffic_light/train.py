"""Fine-tune official pretrained YOLO classification weights; never train on test/."""
import argparse
from pathlib import Path
import shutil

from prepare_dataset import DEFAULT_CONFIG, check_classes, load_config, repo_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = load_config(args.config)

    import torch
    import ultralytics
    from ultralytics import YOLO
    from ultralytics.utils.downloads import GITHUB_ASSETS_NAMES

    if config["model"] not in GITHUB_ASSETS_NAMES:
        raise RuntimeError(f"Ultralytics {ultralytics.__version__} does not support {config['model']}; upgrade it.")
    data = repo_path(config["data"]["root"])
    for split in ("train", "val"):
        check_classes(data / split)
    destination = repo_path(config["output"]["best"])
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Keep automatically downloaded official weights under models/, independent of cwd.
    pretrained = destination.parent / config["model"]
    model = YOLO(str(pretrained), task="classify")
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Ultralytics {ultralytics.__version__}, torch {torch.__version__}, device={device}")
    model.train(
        data=str(data), device=device, pretrained=True,
        project=str(repo_path(config["output"]["project"])),
        name=config["output"]["name"], exist_ok=False,
        save=True, plots=True, **config["train"],
    )
    best = Path(model.trainer.best)
    if not best.is_file():
        raise FileNotFoundError(f"Training did not produce best.pt: {best}")
    shutil.copy2(best, destination)
    print(f"Best model: {destination}")


if __name__ == "__main__":
    main()
