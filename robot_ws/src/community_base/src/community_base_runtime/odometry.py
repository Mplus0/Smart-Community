"""Pure planar integration, retaining the original Euler update and scaling."""
import math


class OdometryIntegrator:
    def __init__(self, scale_x=1.0, scale_y=1.0, max_interval=1.0):
        self.scale_x, self.scale_y, self.max_interval = scale_x, scale_y, max_interval
        self.x = self.y = self.heading = 0.0
        self.previous_time = None

    def update(self, timestamp, vx, vy, yaw_rate):
        if not all(math.isfinite(v) for v in (timestamp, vx, vy, yaw_rate)):
            return None
        previous = self.previous_time
        self.previous_time = timestamp
        if previous is None or not 0 < timestamp-previous <= self.max_interval:
            return None
        dt = timestamp-previous
        vx, vy = vx*self.scale_x, vy*self.scale_y
        self.x += (vx*math.cos(self.heading)-vy*math.sin(self.heading))*dt
        self.y += (vx*math.sin(self.heading)+vy*math.cos(self.heading))*dt
        self.heading += yaw_rate*dt
        return self.x, self.y, self.heading, vx, vy, yaw_rate
