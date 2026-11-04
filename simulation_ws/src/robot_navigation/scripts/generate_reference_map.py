#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "maps"

GAZEBO_IMAGE = MAP_DIR / "map.png"
LOCALIZATION_MAP = MAP_DIR / "localization_map.pgm"
OUTPUT_IMAGE = MAP_DIR / "gazebo_reference.png"


def main():
    gazebo = cv2.imread(
        str(GAZEBO_IMAGE),
        cv2.IMREAD_GRAYSCALE
    )

    localization = cv2.imread(
        str(LOCALIZATION_MAP),
        cv2.IMREAD_GRAYSCALE
    )

    if gazebo is None:
        raise RuntimeError(f"Cannot read: {GAZEBO_IMAGE}")

    if localization is None:
        raise RuntimeError(f"Cannot read: {LOCALIZATION_MAP}")

    gazebo_h, gazebo_w = gazebo.shape
    map_h, map_w = localization.shape

    print(f"Gazebo image: {gazebo_w} x {gazebo_h}")
    print(f"Localization map: {map_w} x {map_h}")

    # =====================================================
    # Gazebo 整张原始图片的四个外角
    #
    # 左上、右上、右下、左下
    # =====================================================
    src_points = np.float32([
        [0, 0],
        [gazebo_w - 1, 0],
        [gazebo_w - 1, gazebo_h - 1],
        [0, gazebo_h - 1],
    ])

    # =====================================================
    # Cartographer 黑色外边框的四个角
    #
    # 当前先使用近似位置：
    #
    # 左上 ≈ (4, 5)
    # 右上 ≈ (216, 5)
    # 右下 ≈ (216, 218)
    # 左下 ≈ (4, 218)
    #
    # 因为 Gazebo 与 map 相差 180°，
    # 所以对应关系需要反过来。
    # =====================================================
    dst_points = np.float32([
        [216, 218],  # Gazebo 左上 -> map 右下
        [4,   218],  # Gazebo 右上 -> map 左下
        [4,   5],    # Gazebo 右下 -> map 左上
        [216, 5],    # Gazebo 左下 -> map 右上
    ])

    matrix = cv2.getPerspectiveTransform(
        src_points,
        dst_points
    )

    # Gazebo 原图是黑底白线。
    # 先执行几何变换。
    reference = cv2.warpPerspective(
        gazebo,
        matrix,
        (map_w, map_h),
        flags=cv2.INTER_LINEAR,

        # 变换区域之外先填黑，
        # 后面反相后会变成白色。
        borderValue=0
    )

    # Gazebo：
    # 黑底 + 白线
    #
    # RViz Map：
    # 白底 + 黑线
    reference = 255 - reference

    cv2.imwrite(
        str(OUTPUT_IMAGE),
        reference
    )

    print(f"Saved: {OUTPUT_IMAGE}")


if __name__ == "__main__":
    main()