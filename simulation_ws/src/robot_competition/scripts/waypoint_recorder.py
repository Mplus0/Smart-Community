#!/usr/bin/env python3
"""Interactive map -> base_footprint recorder; explicit append, atomic saves."""
import argparse
import fcntl
import math
import os
import shlex
import sys
import tempfile
import time

import rospy
import tf2_ros
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from waypoint_manager import RouteError, load_route, positive_number, validate_route


def append_point(path, point, allow_append=False):
    path = os.path.abspath(path)
    # Serialize cooperating recorders; re-read under lock to avoid lost updates.
    with open(path + ".lock", "a", encoding="utf-8") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        exists = os.path.lexists(path)
        if exists and not allow_append:
            raise RouteError("file exists; use --append explicitly (never overwrites a route)")
        data = load_route(path) if exists else {"route": {"frame_id": "map", "points": []}}
        data["route"]["points"].append(point)
        validate_route(data)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=os.path.dirname(path),
                                             prefix=".waypoints-", delete=False) as stream:
                temp = stream.name
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, path)
            temp = None
        finally:
            if temp is not None:
                os.unlink(temp)
    return data


def capture_pose(buffer, base_frame, timeout, max_age):
    deadline = time.monotonic() + timeout
    error = "TF unavailable; initialize AMCL in RViz"
    initial_clock = rospy.Time.now().to_sec()
    use_sim_time = rospy.get_param("/use_sim_time", False)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            now = rospy.Time.now()
            if now.to_sec() <= 0:
                raise ValueError("/clock not initialized")
            if use_sim_time and now.to_sec() <= initial_clock:
                raise ValueError("/clock paused or reset; unpause simulation before recording")
            # No TF timeout argument: its internal ROS-time wait can hang on paused /clock.
            transform = buffer.lookup_transform("map", base_frame, rospy.Time(0))
            age = (now - transform.header.stamp).to_sec()
            if transform.header.stamp.to_sec() <= 0 or age < -0.5 or age > max_age:
                raise ValueError("stale or invalid map TF; check AMCL and /clock")
            p = transform.transform.translation
            q = transform.transform.rotation
            values = (p.x, p.y, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                raise ValueError("non-finite TF pose")
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if norm < 1e-6:
                raise ValueError("invalid TF quaternion")
            x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
            return {"x": p.x, "y": p.y, "yaw": math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))}
        except (tf2_ros.TransformException, ValueError) as exc:
            error = str(exc)
        time.sleep(0.05)
    raise RouteError("pose capture failed: " + error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--append", action="store_true", help="explicitly append to an existing valid route")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--tf-timeout", type=float, default=3.0)
    parser.add_argument("--max-tf-age", type=float, default=1.0)
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        positive_number(args.tf_timeout, "tf-timeout")
        positive_number(args.max_tf_age, "max-tf-age")
        if os.path.lexists(args.output):
            if not args.append:
                raise RouteError("output exists: use --append to add points; no overwrite supported")
            load_route(args.output)
    except (RouteError, OSError) as exc:
        parser.error(str(exc))
    rospy.init_node("waypoint_recorder")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)  # Keep subscriber alive throughout input loop.
    print("Check AMCL alignment and whole-body clearance in RViz before recording.")
    print('Commands: add ID TYPE TASK [area=NAME] [description="TEXT"] [stop_before_line=true|false]; quit')
    allow_append = args.append
    while not rospy.is_shutdown():
        try:
            tokens = shlex.split(input("waypoint> "))
            if not tokens:
                continue
            if tokens == ["quit"]:
                break
            if len(tokens) < 4 or tokens[0] != "add":
                raise RouteError("expected: add ID waypoint|task none|person|plate|traffic_light [key=value]")
            point = dict(zip(("id", "type", "task"), tokens[1:4]))
            for token in tokens[4:]:
                key, sep, value = token.partition("=")
                if not sep or key not in ("area", "description", "stop_before_line") or key in point:
                    raise RouteError("invalid or duplicate metadata: " + token)
                if key == "stop_before_line":
                    if value not in ("true", "false"):
                        raise RouteError("stop_before_line must be true or false")
                    value = value == "true"
                point[key] = value
            point.update(capture_pose(buffer, args.base_frame, args.tf_timeout, args.max_tf_age))
            append_point(args.output, point, allow_append)
            allow_append = True
            rospy.loginfo("Saved %s: x=%.6f y=%.6f yaw=%.6f", point["id"], point["x"], point["y"], point["yaw"])
        except (EOFError, KeyboardInterrupt):
            break
        except (RouteError, OSError, ValueError) as exc:
            rospy.logerr("Not saved: %s", exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
