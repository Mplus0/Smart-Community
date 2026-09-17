#!/usr/bin/env python3
import os, random, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mecanum_math import MecanumGeometry, inverse_kinematics, forward_kinematics

g = MecanumGeometry()
cases = [
    ('forward', (0.2, 0.0, 0.0), (+1,+1,+1,+1)),
    ('left',    (0.0, 0.2, 0.0), (-1,+1,+1,-1)),
    ('ccw',     (0.0, 0.0, 1.0), (-1,+1,-1,+1)),
]
for name, twist, signs in cases:
    w = inverse_kinematics(*twist, g)
    got = tuple(0 if abs(x)<1e-12 else (1 if x>0 else -1) for x in w)
    assert got == signs, (name, w, got, signs)
    back = forward_kinematics(w, g)
    assert max(abs(a-b) for a,b in zip(twist, back)) < 1e-10
    print(f'{name:8s}: wheels={[round(x,6) for x in w]} -> twist={[round(x,6) for x in back]}')
for _ in range(1000):
    twist=(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-3,3))
    back=forward_kinematics(inverse_kinematics(*twist,g),g)
    assert max(abs(a-b) for a,b in zip(twist,back)) < 1e-10
print('1000 randomized IK/FK round trips: PASS')
print('X-layout mecanum kinematics validation: PASS')
