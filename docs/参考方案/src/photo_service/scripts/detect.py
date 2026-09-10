#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线批量检测：自动增强亮度后，对 photo/photo1 下的所有图片做 YOLOv5 检测，
结果保存到 photo/output/results_<时间戳> 目录。
"""
import os
import glob
import torch
import cv2
import numpy as np
from datetime import datetime

try:
    import rospkg
    _WS_SRC = os.path.dirname(rospkg.RosPack().get_path('photo_service'))
except Exception:
    # scripts/ -> photo_service/ -> src/
    _WS_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def enhance_brightness(image, alpha=1.4, beta=40):
    """亮度 + 对比度增强：new = image * alpha + beta"""
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)


def main():
    # 路径配置（基于工作空间自动推导）
    weights = os.path.join(_WS_SRC, "yolov5_ros", "weights", "shequ.pt")
    source = os.path.join(_WS_SRC, "photo_service", "photo", "photo1")
    base_output = os.path.join(_WS_SRC, "photo_service", "photo", "output")
    yolov5_home = os.path.join(_WS_SRC, "yolov5_ros", "yolov5")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = os.path.join(base_output, "results_" + timestamp)

    if not os.path.exists(weights):
        print("模型权重不存在: %s" % weights)
        return
    if not os.path.exists(source):
        print("图片文件夹不存在: %s" % source)
        return
    os.makedirs(output, exist_ok=True)

    print("正在加载模型...")
    model = torch.hub.load(yolov5_home, 'custom', path=weights, source='local')
    model.conf = 0.6  # 置信度阈值

    img_files = sorted(
        glob.glob(os.path.join(source, "*.jpg")) +
        glob.glob(os.path.join(source, "*.png")) +
        glob.glob(os.path.join(source, "*.jpeg"))
    )

    if not img_files:
        print("文件夹中没有找到图片: %s" % source)
        return

    print("模型加载完成，检测到 %d 张图片，开始亮度增强 + 检测..." % len(img_files))

    for i, img_path in enumerate(img_files, 1):
        print("[%d/%d] 正在增强亮度并检测: %s" % (i, len(img_files), os.path.basename(img_path)))

        img = cv2.imread(img_path)
        if img is None:
            print("无法读取图片: %s" % img_path)
            continue

        bright_img = enhance_brightness(img, alpha=1.4, beta=40)
        temp_path = os.path.join(output, "bright_" + os.path.basename(img_path))
        cv2.imwrite(temp_path, bright_img)

        results = model(temp_path)
        results.save(save_dir=output)

    print("检测完成！所有结果已保存到：%s" % output)


if __name__ == "__main__":
    main()
