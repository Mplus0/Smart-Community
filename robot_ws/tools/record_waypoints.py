#!/usr/bin/env python3
"""从 map TF 记录当前位置和任务，保存为巡检路线 YAML。"""

import argparse
import math
from pathlib import Path

import yaml


TASKS = (
    'navigation', 'person', 'plate', 'traffic_light', 'fire', 'trash',
    'vehicle_photo', 'announcement', 'record_start', 'record_stop',
)


def parse_task(command, points):
    parts = command.split() or ['navigation']
    task = parts[0]
    if task not in TASKS:
        raise ValueError('未知任务：' + task)
    fields = {'task': task}
    if task == 'person':
        if len(parts) != 2 or parts[1].upper() not in ('A', 'B'):
            raise ValueError('用法：person A 或 person B')
        fields['area'] = parts[1].upper()
    elif task == 'plate':
        if len(parts) != 2 or not parts[1].isdigit() or int(parts[1]) < 1:
            raise ValueError('用法：plate 1（车位号为正整数）')
        fields['parking_lot'] = int(parts[1])
    elif len(parts) != 1:
        raise ValueError('该任务不需要额外参数')
    recording = recording_open(points)
    if task == 'record_start' and recording:
        raise ValueError('已开始录像，请先记录 record_stop')
    if task == 'record_stop' and not recording:
        raise ValueError('请先记录 record_start')
    return fields


def recording_open(points):
    for point in reversed(points):
        if point['task'] in ('record_start', 'record_stop'):
            return point['task'] == 'record_start'
    return False


def save_route(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(yaml.safe_dump({'points': points}, allow_unicode=True,
                                        sort_keys=False), encoding='utf-8')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('~/maps/route.yaml'),
                        help='新建路线文件，默认 ~/maps/route.yaml；不覆盖已有文件')
    parser.add_argument('--base-frame', default='base_footprint', help='小车坐标系')
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f'文件已存在，请用 --output 指定新文件：{output}')

    import rospy
    import tf2_ros
    from tf.transformations import euler_from_quaternion

    rospy.init_node('record_waypoints', anonymous=True, disable_signals=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    points = []
    print('保持建图或定位运行，将小车停在目标位置后输入任务。')
    print('任务：' + ', '.join(TASKS))
    print('示例：person A、plate 1；回车记录 navigation；u 撤销；q 退出。')
    print('保存位置：' + str(output))
    try:
        while not rospy.is_shutdown():
            command = input(f'航点 {len(points) + 1} > ').strip()
            if command == 'q':
                break
            if command == 'u':
                if points:
                    remaining = points[:-1]
                    save_route(output, remaining)
                    removed = points.pop()
                    print('已撤销：' + removed['id'])
                continue
            try:
                fields = parse_task(command, points)
                transform = buffer.lookup_transform('map', args.base_frame,
                                                    rospy.Time(0), rospy.Duration(2.0))
                age = (rospy.Time.now() - transform.header.stamp).to_sec()
                if age < -0.1 or age > 1.0:
                    raise ValueError('定位 TF 已过期或时间异常，请检查建图/定位节点')
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
                if not all(math.isfinite(v) for v in quaternion) or sum(v*v for v in quaternion) < 1e-12:
                    raise ValueError('定位四元数无效')
                yaw = euler_from_quaternion(quaternion)[2]
                if not all(math.isfinite(v) for v in (translation.x, translation.y, yaw)):
                    raise ValueError('定位坐标无效')
                point = {'id': f'wp_{len(points) + 1:02d}',
                         'x': round(translation.x, 4), 'y': round(translation.y, 4),
                         'yaw': round(yaw, 4), **fields}
                save_route(output, points + [point])
                points.append(point)
                print(f"已保存 {point['id']}: x={point['x']}, y={point['y']}, "
                      f"yaw={point['yaw']}, {fields}")
            except (ValueError, tf2_ros.TransformException) as error:
                print(f'未记录：{error}')
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        rospy.signal_shutdown('航点记录结束')
        print(f'已保存 {len(points)} 个航点：{output}' if output.exists() else '未保存航点。')
        if not points and output.exists():
            print('路线为空，需记录航点后才能用于巡检。')
        if recording_open(points):
            print('路线尚缺 record_stop，使用前请补齐录像结束航点。')


if __name__ == '__main__':
    main()
