#!/usr/bin/env python3
"""在导航底图上叠加一格厚的分支封口，不修改定位图和磁盘地图。"""
import copy
import math
import threading
from pathlib import Path

import rospy
import yaml
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


def apply_closure(grid, closure):
    """封口法向仅占一格；span 两端接入原有静态禁行区域。"""
    result = copy.deepcopy(grid)
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    width, height = grid.info.width, grid.info.height
    axis = closure['axis']
    if axis not in ('x', 'y'):
        raise ValueError('Closure axis must be x or y')
    fixed_origin = origin.x if axis == 'x' else origin.y
    span_origin = origin.y if axis == 'x' else origin.x
    fixed_size, span_size = (width, height) if axis == 'x' else (height, width)
    fixed = math.floor((closure['coordinate'] - fixed_origin) / resolution)
    start, end = closure['span']
    if not 0 <= fixed < fixed_size or start >= end:
        raise ValueError('Closure is outside the navigation map or has an invalid span')
    first = max(0, math.ceil((start - span_origin) / resolution - 0.5))
    last = min(span_size, math.ceil((end - span_origin) / resolution - 0.5))
    values = list(grid.data)
    for index in range(first, last):
        cell = index * width + fixed if axis == 'x' else fixed * width + index
        values[cell] = 100
    result.data = values
    return result


class NavigationMap:
    def __init__(self):
        config = yaml.safe_load(Path(rospy.get_param('~route')).read_text())
        self.closures = config.get('navigation_closures', {})
        self.base = None
        self.mode = None
        self.lock = threading.RLock()
        self.publisher = rospy.Publisher('/navigation_map', OccupancyGrid, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber('/navigation_map_base', OccupancyGrid,
                                           self.receive, queue_size=1)
        self.reset_service = rospy.Service('/navigation_map/reset', Trigger, self.reset)
        self.select_service = rospy.Service('/navigation_map/set_blocked', SetBool, self.select)

    def publish(self):
        grid = (apply_closure(self.base, self.closures[self.mode])
                if self.mode else copy.deepcopy(self.base))
        grid.header.stamp = rospy.Time.now()
        self.publisher.publish(grid)

    def receive(self, grid):
        with self.lock:
            self.base = grid
            self.publish()

    def reset(self, request):
        with self.lock:
            if self.base is None:
                return TriggerResponse(False, 'Navigation base map is not ready')
            self.mode = None
            self.publish()
        return TriggerResponse(True, 'Navigation map reset to base map')

    def select(self, request):
        mode = 'blocked' if request.data else 'clear'
        with self.lock:
            if self.base is None or mode not in self.closures:
                return SetBoolResponse(False, 'Navigation base map or closure is not configured')
            # 先生成再切换状态，配置错误时保留原来的有效地图。
            try:
                grid = apply_closure(self.base, self.closures[mode])
            except (KeyError, TypeError, ValueError) as error:
                return SetBoolResponse(False, str(error))
            self.mode = mode
            grid.header.stamp = rospy.Time.now()
            self.publisher.publish(grid)
        rospy.loginfo('[导航封口] mode=%s，厚度一格 %.3f m', mode, grid.info.resolution)
        return SetBoolResponse(True, mode)


if __name__ == '__main__':
    rospy.init_node('navigation_map')
    NavigationMap()
    rospy.spin()
