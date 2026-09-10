#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
车牌识别（EasyOCR）：读取车牌图片，左侧识别省份汉字、右侧识别字母数字。
运行：python3 ceshi2.py <图片路径>
"""
import sys
import re
import cv2
import numpy as np
import easyocr


def brighten_and_enhance(img):
    """提亮 + CLAHE 对比度增强"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.add(v, 80)
    v = np.clip(v, 0, 255)
    hsv = cv2.merge((h, s, v))
    img_bright = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    lab = cv2.cvtColor(img_bright, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def recognize_plate(image_path):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError("找不到图片: %s" % image_path)

    img = brighten_and_enhance(img)
    h, w = img.shape[:2]

    # 按比例分成两部分：左侧为省份汉字，右侧为其余字符
    x_split = int(w * 0.19)
    left = img[:, :x_split]
    right = img[:, x_split:]

    cv2.imwrite("left_su.jpg", left)
    cv2.imwrite("right_rest.jpg", right)

    reader_ch = easyocr.Reader(['ch_sim'])
    reader_en = easyocr.Reader(['en'])

    # 左边：只允许识别汉字（省份简称）
    res_ch = reader_ch.readtext(left, allowlist="苏皖浙京沪津渝湘粤鲁豫鄂赣川陕黑吉辽晋蒙桂贵云甘宁青新藏闽冀琼")
    province = res_ch[0][1] if res_ch else "苏"

    # 右边：英数字
    res_en = reader_en.readtext(right, allowlist="ABCDEFGHJKLMNPQRSTUVWXYZ0123456789")
    rest = ''.join([r[1] for r in res_en])
    rest = re.sub(r'[^A-Z0-9]', '', rest)
    plate = province + rest

    # 自动纠错：EasyOCR 常将 W 识别为 M
    if len(plate) > 2 and plate[1] == 'M':
        plate = plate[0] + 'W' + plate[2:]

    print("识别结果: %s" % plate)
    return plate


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 ceshi2.py <图片路径>")
        sys.exit(1)
    recognize_plate(sys.argv[1])
