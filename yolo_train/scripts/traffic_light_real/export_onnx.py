"""Export real traffic-light detection weights to ONNX for external NMS."""
import argparse
from pathlib import Path

from train import DEFAULT_CONFIG, train_path
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--weights', default='model/traffic_light.pt',
                        help='Weights path, absolute or relative to yolo_train/')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    weights = train_path(args.weights)
    if not weights.is_file():
        raise FileNotFoundError(weights)

    import onnx
    from ultralytics import YOLO

    model = YOLO(str(weights), task='detect')
    if model.names != {0: 'stop', 1: 'go'}:
        raise ValueError(f'Unexpected traffic-light classes: {model.names}')
    exported = Path(model.export(format='onnx', imgsz=config['train']['imgsz'],
                                 opset=17, device='cpu', half=False, simplify=False,
                                 dynamic=False, batch=1, nms=False, end2end=False))
    onnx.checker.check_model(str(exported))
    print(f'Validated ONNX model: {exported}')


if __name__ == '__main__':
    main()
