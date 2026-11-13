"""标定窗口的交互状态；鼠标只修改候选，Enter 才确认。"""

import math


class RectangleSelection:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.anchor = None
        self.rectangle = None
        self.dragging = False

    def _clamp(self, x, y):
        return max(0, min(self.width, x)), max(0, min(self.height, y))

    def start(self, x, y):
        self.anchor = self._clamp(x, y)
        self.rectangle = None
        self.dragging = True

    def move(self, x, y):
        if self.dragging:
            x, y = self._clamp(x, y)
            ax, ay = self.anchor
            self.rectangle = (min(ax, x), min(ay, y), max(ax, x), max(ay, y))

    def finish(self, x, y):
        self.move(x, y)
        self.dragging = False

    def reset(self):
        self.anchor = self.rectangle = None
        self.dragging = False

    def confirm(self):
        if self.dragging or self.rectangle is None:
            return None
        x1, y1, x2, y2 = self.rectangle
        return self.rectangle if x2 - x1 >= 2 and y2 - y1 >= 2 else None


class PointSelection:
    def __init__(self, count):
        self.points = [None] * count
        self.index = 0

    @property
    def reviewing(self):
        return self.index == len(self.points)

    def select(self, x, y):
        if not self.reviewing:
            self.points[self.index] = [x, y]

    def confirm(self):
        if self.reviewing or self.points[self.index] is None:
            return False
        self.index += 1
        return True

    def back(self):
        if self.index > 0:
            self.index -= 1

    def reset_current(self):
        if not self.reviewing:
            self.points[self.index] = None


def rectangle_to_roi(rectangle, image_size, preview_size):
    width, height = image_size
    pw, ph = preview_size
    x1, y1, x2, y2 = rectangle
    left, top = max(0, math.floor(x1 * width / pw)), max(0, math.floor(y1 * height / ph))
    right, bottom = min(width, math.ceil(x2 * width / pw)), min(height, math.ceil(y2 * height / ph))
    return [left, top, right - left, bottom - top]
