#!/usr/bin/env python3
"""Shared, ROS-independent route schema and validation."""
import argparse
import math
import sys

import yaml


class RouteError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys rather than silently overriding safety fields."""


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise RouteError("YAML keys must be unique strings: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def positive_number(value, name):
    if (type(value) not in (int, float) or not math.isfinite(value) or value <= 0):
        raise RouteError("{} must be a finite positive number".format(name))
    return float(value)


def retry_count(value):
    if type(value) is not int or value < 0:
        raise RouteError("retries must be a nonnegative integer")
    return value


def validate_route(data):
    if not isinstance(data, dict) or set(data) != {"route"}:
        raise RouteError("expected top-level mapping containing only route")
    route = data["route"]
    if not isinstance(route, dict) or set(route) != {"frame_id", "points"}:
        raise RouteError("route requires frame_id and points")
    if route["frame_id"] != "map":
        raise RouteError("P1 requires frame_id: map")
    if not isinstance(route["points"], list):
        raise RouteError("route.points must be a list (empty list is allowed)")
    ids = set()
    required = {"id", "type", "task", "x", "y", "yaw"}
    optional = {"area", "description", "stop_before_line", "navigation_timeout", "retries"}
    for index, point in enumerate(route["points"], 1):
        prefix = "point {}".format(index)
        if not isinstance(point, dict) or not required <= set(point):
            raise RouteError(prefix + ": missing required fields " + str(sorted(required)))
        if set(point) - required - optional:
            raise RouteError(prefix + ": unknown fields " + str(set(point) - required - optional))
        if not isinstance(point["id"], str) or not point["id"].strip():
            raise RouteError(prefix + ": id must be a nonempty string")
        if point["id"] in ids:
            raise RouteError(prefix + ": duplicate id " + point["id"])
        ids.add(point["id"])
        if not ((point["type"] == "waypoint" and point["task"] == "none") or
                (point["type"] == "task" and point["task"] in ("person", "plate", "traffic_light"))):
            raise RouteError(prefix + ": invalid type/task combination")
        for field in ("x", "y", "yaw"):
            value = point[field]
            if type(value) not in (int, float) or not math.isfinite(value):
                raise RouteError(prefix + ": " + field + " must be a finite number")
        for field in ("area", "description"):
            if field in point and not isinstance(point[field], str):
                raise RouteError(prefix + ": " + field + " must be a string")
        if "stop_before_line" in point:
            if type(point["stop_before_line"]) is not bool or point["task"] != "traffic_light":
                raise RouteError(prefix + ": stop_before_line must be boolean on traffic_light only")
        if "navigation_timeout" in point:
            positive_number(point["navigation_timeout"], prefix + ".navigation_timeout")
        if "retries" in point:
            retry_count(point["retries"])
    return data


def load_route(path):
    try:
        with open(path, encoding="utf-8") as stream:
            return validate_route(yaml.load(stream, Loader=UniqueLoader))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RouteError("cannot read route {}: {}".format(path, exc)) from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file")
    args = parser.parse_args()
    try:
        route = load_route(args.file)["route"]
        print("VALID: {} points, frame={}".format(len(route["points"]), route["frame_id"]))
        if not route["points"]:
            print("EMPTY_ROUTE: no navigation will be sent")
        return 0
    except RouteError as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
