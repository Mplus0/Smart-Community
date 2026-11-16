#!/usr/bin/env python3
"""按场地图白线生成禁行区，以四角刚体标定叠加到实车扫描地图。"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw


REFERENCE_IMAGE = (Path(__file__).resolve().parents[1] /
                   'src/community_bringup/maps/field_reference.png')
FIELD_SIZE = 4.2


def line_centers(values):
    """读取连续白线的中心，忽略单像素外框和抗锯齿边缘。"""
    white = values > 180
    edges = np.diff(np.r_[False, white, False].astype(np.int8))
    starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0]
    return [(a + b - 1) / 2.0 for a, b in zip(starts, ends) if b - a >= 3]


def reference_zones(path=REFERENCE_IMAGE):
    # 原图停车区在右侧；旋转 180°后对应停车区在左、起点在左下的场地方向。
    image = np.rot90(np.array(Image.open(path).convert('L')), 2)
    height, width = image.shape
    vertical = line_centers(image[height // 4, :])
    lower_vertical = line_centers(image[height * 3 // 4, :])
    horizontal = line_centers(image[:, width * 7 // 10])
    parking_horizontal = line_centers(image[:, width // 10])
    if (width != height or
            tuple(map(len, (vertical, lower_vertical, horizontal, parking_horizontal)))
            != (7, 5, 7, 6)):
        raise ValueError('参考图片布局不匹配，请使用随脚本提供的 field_reference.png')

    left, parking_right, l_left, l_right, rect_left, rect_right, _ = vertical
    top, zone_top, _, rect_bottom, l_inner_bottom, l_bottom, _ = horizontal
    parking_bottom = parking_horizontal[-2]
    l_far_right = lower_vertical[-2]
    pixels = {
        'parking_area': [[left, top], [parking_right, top],
                         [parking_right, parking_bottom], [left, parking_bottom]],
        'rectangle': [[rect_left, zone_top], [rect_right, zone_top],
                      [rect_right, rect_bottom], [rect_left, rect_bottom]],
        'l_area': [[l_left, zone_top], [l_right, zone_top],
                   [l_right, l_inner_bottom], [l_far_right, l_inner_bottom],
                   [l_far_right, l_bottom], [l_left, l_bottom]],
    }
    # 整张纹理对应 4.2 m；图片内白色外框约 4.0 m，不能当作 4.2 m 再缩放。
    zones = {}
    for name, points in pixels.items():
        points = np.asarray(points, dtype=float)
        zones[name] = np.column_stack(((points[:, 0] + .5) / width,
                                      (height - .5 - points[:, 1]) / height)) * FIELD_SIZE
    return zones


def fit_field_transform(corners, max_error=.08):
    """四角顺序：场地左下、右下、右上、左上；只拟合旋转和平移。"""
    source = np.array([[0., 0.], [FIELD_SIZE, 0.],
                       [FIELD_SIZE, FIELD_SIZE], [0., FIELD_SIZE]])
    target = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.isfinite(target).all() or not cv2.isContourConvex(target.astype('float32')):
        raise ValueError('四角必须是有限坐标，并按左下、右下、右上、左上顺序提供')
    signed_area = np.sum(target[:, 0] * np.roll(target[:, 1], -1) -
                         target[:, 1] * np.roll(target[:, 0], -1)) / 2
    if signed_area <= 0:
        raise ValueError('四角顺序反向；请按场地左下、右下、右上、左上提供 map 坐标')
    u, _, vt = np.linalg.svd((source - source.mean(0)).T @ (target - target.mean(0)))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        raise ValueError('四角对应发生镜像，请检查顺序')
    translation = target.mean(0) - source.mean(0) @ rotation.T
    residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    if residual.max() > max_error:
        raise ValueError('4.2 m 正方形标定最大误差 %.3f m 超过 %.3f m；'
                         '请检查角点、场地尺寸及扫描变形，不会强行拉伸地图' %
                         (residual.max(), max_error))
    return rotation, translation, residual


def world_to_pixels(points, info, height):
    origin = np.asarray(info['origin'], dtype=float)
    yaw = origin[2]
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    grid = (np.asarray(points) - origin[:2]) @ rotation / float(info['resolution'])
    return np.column_stack((grid[:, 0] - .5, height - .5 - grid[:, 1]))


def generate_map(map_path, output, corners, reference=REFERENCE_IMAGE, max_error=.08):
    map_path, output = Path(map_path).expanduser().resolve(), Path(output).expanduser().resolve()
    outputs = {key: output.parent / (output.name + suffix) for key, suffix in
               [('image', '.pgm'), ('map', '.yaml'),
                ('zones', '_zones.yaml'), ('preview', '_preview.png')]}
    for path in outputs.values():
        if path.exists():
            raise ValueError('输出已存在，不覆盖：' + str(path))
    info = yaml.safe_load(map_path.read_text())
    if float(info['resolution']) <= 0 or len(info['origin']) != 3:
        raise ValueError('地图 resolution 或 origin 无效')
    image_path = Path(info['image']).expanduser()
    if not image_path.is_absolute():
        image_path = map_path.parent / image_path
    scan = np.array(Image.open(image_path).convert('L'))
    height, width = scan.shape
    rotation, translation, residual = fit_field_transform(corners, max_error)
    zones = reference_zones(reference)
    mask = np.zeros_like(scan)
    preview = Image.fromarray(scan).convert('RGB')
    draw = ImageDraw.Draw(preview)
    records = {}
    colors = {'parking_area': '#ff9900', 'rectangle': '#ff3366', 'l_area': '#0099ff'}
    for name, points in zones.items():
        world = points @ rotation.T + translation
        pixels = world_to_pixels(world, info, height)
        if (pixels[:, 0].min() < -.5 or pixels[:, 0].max() > width - .5 or
                pixels[:, 1].min() < -.5 or pixels[:, 1].max() > height - .5):
            raise ValueError('禁行区超出扫描图范围：' + name)
        vertices = np.rint(pixels).astype(np.int32)
        cv2.fillPoly(mask, [vertices], 255)
        polygon = [tuple(map(int, point)) for point in vertices]
        draw.line(polygon + [polygon[0]], fill=colors[name], width=2)
        label = tuple(map(int, vertices.mean(0)))
        draw.text(label, name, fill=colors[name])
        records[name] = {'field_points_m': points.tolist(), 'map_points_m': world.tolist()}
    for i, point in enumerate(world_to_pixels(np.asarray(corners).reshape(4, 2), info, height)):
        x, y = point
        draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill='#00aa00')
        draw.text((x + 3, y + 3), str(i + 1), fill='#00aa00')
    result = scan.copy()
    result[mask != 0] = 255 if int(info.get('negate', 0)) else 0
    output_info = dict(info, image=outputs['image'].name)
    report = {
        'source_map': str(map_path), 'reference_image': str(Path(reference).resolve()),
        'field_size_m': FIELD_SIZE,
        'corner_order': ['lower_left', 'lower_right', 'upper_right', 'upper_left'],
        'map_corners_m': np.asarray(corners).reshape(4, 2).tolist(),
        'field_to_map_yaw_rad': float(math.atan2(rotation[1, 0], rotation[0, 0])),
        'field_to_map_translation_m': translation.tolist(),
        'corner_residuals_m': residual.tolist(), 'forbidden_zones': records,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(result).save(outputs['image'])
    outputs['map'].write_text(yaml.safe_dump(output_info, sort_keys=False), encoding='utf-8')
    outputs['zones'].write_text(yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
    preview.save(outputs['preview'])
    return outputs, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path, help='新扫描地图 YAML')
    parser.add_argument('--output', required=True, type=Path, help='新导航地图文件前缀，不含扩展名')
    parser.add_argument('--corners', required=True, nargs=8, type=float, metavar='M',
                        help='场地左下、右下、右上、左上四角的 map 坐标 x y，共 8 个数；以左下停车区定方向')
    parser.add_argument('--reference', type=Path, default=REFERENCE_IMAGE, help='场地原图，默认随附图片')
    parser.add_argument('--max-corner-error', type=float, default=.08,
                        help='四角刚体标定最大允许误差，默认 0.08 m')
    args = parser.parse_args()
    if not math.isfinite(args.max_corner_error) or args.max_corner_error <= 0:
        parser.error('--max-corner-error 必须为有限正数')
    try:
        outputs, report = generate_map(args.map, args.output, args.corners,
                                       args.reference, args.max_corner_error)
    except (ValueError, OSError, KeyError, yaml.YAMLError) as error:
        parser.exit(1, '未生成导航图：' + str(error) + '\n')
    print('角度 %.3f°；角点最大误差 %.3f m' %
          (math.degrees(report['field_to_map_yaw_rad']), max(report['corner_residuals_m'])))
    for name, path in outputs.items():
        print(name + ': ' + str(path))
    print('扫描图与航点未修改。先核对 preview 中角点和禁行区，再显式指定 navigation_map 启动导航。')


if __name__ == '__main__':
    main()
