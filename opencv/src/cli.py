"""calibrate、read、demo 命令的参数解析与执行。"""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2

from .core import load_config, read_frame, read_image, write_image


MODULE_ROOT = Path(__file__).resolve().parents[1]
OUTPUTS_ROOT = MODULE_ROOT / "outputs"


def csv_numbers(value):
    try:
        return [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("请输入逗号分隔的数值") from exc


def roi_pixels(value):
    try:
        values = [int(item.strip()) for item in value.split(",")]
        if len(values) != 4:
            raise ValueError()
        return values
    except ValueError as exc:
        raise argparse.ArgumentTypeError("ROI 格式为整数 x,y,width,height") from exc


def process(image_path, config_path, output=None):
    config = load_config(config_path)
    result, debug = read_frame(read_image(image_path), config)
    result["image"] = str(Path(image_path).resolve())
    result["config"] = str(Path(config_path).resolve())
    result["processed_at_utc"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    output = Path(output) if output is not None else OUTPUTS_ROOT / "read" / Path(image_path).stem
    output.mkdir(parents=True, exist_ok=True)
    (output / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    write_image(output / "debug.png", debug)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("结果与调试图：{}".format(output.resolve()), file=sys.stderr)
    return 2 if result["status"] == "rejected" else 0


def main():
    parser = argparse.ArgumentParser(prog="python -m opencv",
                                     description="仪表离线读数：点击标定 → 径向检测 → 整数输出")
    commands = parser.add_subparsers(dest="command", required=True)
    cal = commands.add_parser("calibrate", help="交互式标定，需本地图形桌面")
    cal.add_argument("image", type=Path)
    cal.add_argument("--output", type=Path, default=MODULE_ROOT / "configs/meter.json",
                     help="标定文件保存位置，默认 opencv/configs/meter.json")
    cal.add_argument("--values", type=csv_numbers,
                     help="默认 -30,-20,-10,0,10,20,30,40,50；可自定义中间点，须包含两端")
    cal.add_argument("--direction", choices=("clockwise", "counterclockwise"), default="clockwise",
                     help="刻度数值增大的方向")
    cal.add_argument("--roi", type=roi_pixels, help="可选：提前指定原图 ROI，跳过框选")
    cal.add_argument("--exclude", type=csv_numbers, action="append", default=[],
                     help="可重复；屏蔽圆 x,y,r，坐标相对于裁剪后的表盘")
    read = commands.add_parser("read", help="读取一张照片，无窗口，可供 ROS 程序调用相同核心函数")
    read.add_argument("image", type=Path)
    read.add_argument("--config", type=Path, required=True)
    read.add_argument("--output", type=Path,
                      help="可选：自定义输出目录；默认 outputs/read/图片名，重复运行更新结果")
    demo = commands.add_parser("demo", help="对已有比赛照片验证候选指针方向；不输出未确认的刻度")
    demo.add_argument("--output", type=Path, help="可选：自定义演示目录；默认 outputs/demo")
    args = parser.parse_args()
    try:
        if args.command == "calibrate":
            from .calibration import DEFAULT_VALUES, calibrate
            values = list(DEFAULT_VALUES) if args.values is None else args.values
            config = calibrate(args.image, args.output, values, args.direction, args.roi, args.exclude)
            print("已保存标定：{}；已标定 {} 个刻度".format(args.output.resolve(), len(config["ticks"])))
            return 0
        if args.command == "demo":
            return process(MODULE_ROOT / "photo/clear3.png",
                           MODULE_ROOT / "configs/competition_geometry.json", args.output or OUTPUTS_ROOT / "demo")
        return process(args.image, args.config, args.output)
    except (OSError, ValueError, KeyError, TypeError, cv2.error, argparse.ArgumentTypeError) as exc:
        print(json.dumps({"status": "error", "reading": None, "rounded": None, "message": str(exc)},
                         ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
