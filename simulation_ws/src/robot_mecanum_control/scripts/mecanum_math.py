#!/usr/bin/env python3
"""Pure-Python X-layout mecanum kinematics. No ROS dependency."""
from dataclasses import dataclass

@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius: float = 0.037230282
    half_wheelbase: float = 0.106000002
    half_track: float = 0.125000000

    @property
    def k(self) -> float:
        return self.half_wheelbase + self.half_track


def inverse_kinematics(vx, vy, wz, g=MecanumGeometry()):
    """Return [FL, FR, RL, RR] wheel angular velocities in rad/s."""
    r, k = g.wheel_radius, g.k
    return [
        (vx - vy - k * wz) / r,
        (vx + vy + k * wz) / r,
        (vx + vy - k * wz) / r,
        (vx - vy + k * wz) / r,
    ]


def forward_kinematics(wheels, g=MecanumGeometry()):
    """Return (vx, vy, wz) from [FL, FR, RL, RR] rad/s."""
    fl, fr, rl, rr = wheels
    r, k = g.wheel_radius, g.k
    vx = r * (fl + fr + rl + rr) / 4.0
    vy = r * (-fl + fr + rl - rr) / 4.0
    wz = r * (-fl + fr - rl + rr) / (4.0 * k)
    return vx, vy, wz


def scale_to_limit(wheels, max_abs_speed):
    peak = max(abs(v) for v in wheels)
    if max_abs_speed <= 0 or peak <= max_abs_speed:
        return list(wheels), 1.0
    scale = max_abs_speed / peak
    return [v * scale for v in wheels], scale
