#!/usr/bin/env python3
"""从 ROS 相机话题手动采集 YOLO 待标注图像。"""

import argparse
import os
import re
from datetime import datetime
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="数据集名称，例如 person 或 bicycle")
    parser.add_argument("--topic", default="/usb_cam/image_raw", help="相机 Image 话题")
    parser.add_argument(
        "--datasets-dir", type=Path,
        default=Path(__file__).resolve().parents[1] / "datasets",
        help="数据集根目录，默认 robot_ws/datasets",
    )
    parser.add_argument("--no-preview", action="store_true", help="无窗口模式，适用于 SSH")
    args = parser.parse_args()
    if (not args.name.strip() or args.name in (".", "..")
            or any(c in args.name for c in '/\\<>:"|?*')
            or any(ord(c) < 32 for c in args.name)):
        parser.error("数据集名称不能为空，也不能包含路径分隔符或非法文件名字符")
    return args


def prepare_directories(args):
    dataset = args.datasets_dir.expanduser().resolve() / args.name
    (dataset / "images").mkdir(parents=True, exist_ok=True)
    return dataset


def main():
    args = parse_args()
    # 延迟导入，--help 不依赖 ROS 或图形环境。
    import cv2
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image

    if not args.no_preview:
        no_gui = re.search(r"^\s*GUI:\s*NONE\s*$", cv2.getBuildInformation(), re.M)
        no_display = not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if no_gui or no_display:
            reason = "当前 OpenCV 不支持图形窗口" if no_gui else "当前终端没有图形显示环境"
            print(reason + "，改用终端采集模式。", flush=True)
            print("桌面预览可使用 python3 -s tools/collect_dataset.py ...；SSH 可加 --no-preview。", flush=True)
            args.no_preview = True

    dataset = prepare_directories(args)
    rospy.init_node("collect_dataset", anonymous=True)
    bridge = CvBridge()
    count = 0
    subscriber = None
    window_created = False

    def save(image):
        nonlocal count
        output = dataset / "images"
        filename = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
        path = output / filename
        if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise OSError("图像保存失败：" + str(path))
        count += 1
        print("已保存 [{}] {}".format(count, path), flush=True)

    def receive():
        message = rospy.wait_for_message(args.topic, Image, timeout=5.0)
        return bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

    print("相机话题：{}\n数据集目录：{}".format(args.topic, dataset), flush=True)
    try:
        if args.no_preview:
            print("回车采集一张最新图像；输入 q 后回车退出。", flush=True)
            while not rospy.is_shutdown():
                command = input().strip().lower()
                if command == "q":
                    break
                if command:
                    continue
                try:
                    save(receive())
                except (rospy.ROSException, CvBridgeError) as exc:
                    print("未采集到有效图像：{}".format(exc), flush=True)
        else:
            print("在预览窗口按空格或 S 保存原图，按 Q 或 Esc 退出。", flush=True)
            latest = [None]
            subscriber = rospy.Subscriber(
                args.topic, Image, lambda message: latest.__setitem__(0, message),
                queue_size=1, buff_size=16 * 1024 * 1024,
            )
            window = "Dataset capture"
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            window_created = True
            last_saved = None
            while not rospy.is_shutdown():
                message = latest[0]
                if message is None:
                    message = rospy.wait_for_message(args.topic, Image, timeout=5.0)
                image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                cv2.imshow(window, image)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                # GTK 旧版本返回 -1 表示不支持查询，并非窗口已关闭。
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                    break
                if key in (ord("s"), ord("S"), 32):
                    age = (rospy.Time.now() - message.header.stamp).to_sec()
                    if age > 2.0 or message is last_saved:
                        print("当前帧已过期或已保存，请等待相机新帧。", flush=True)
                        continue
                    save(image)
                    last_saved = message
    except (KeyboardInterrupt, EOFError, rospy.ROSInterruptException):
        pass
    except (rospy.ROSException, CvBridgeError, OSError, cv2.error) as exc:
        print("采集失败：{}".format(exc), flush=True)
        return 1
    finally:
        if subscriber is not None:
            subscriber.unregister()
        if window_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        print("采集结束，本次保存 {} 张。".format(count), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
