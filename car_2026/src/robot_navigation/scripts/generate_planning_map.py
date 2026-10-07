#!/usr/bin/env python3

from pathlib import Path

import yaml
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]

MAP_DIR = ROOT / "maps"
CONFIG_DIR = ROOT / "config"

LOCALIZATION_MAP = MAP_DIR / "localization_map.pgm"
LOCALIZATION_YAML = MAP_DIR / "localization_map.yaml"

PLANNING_MAP = MAP_DIR / "planning_map.pgm"
PLANNING_YAML = MAP_DIR / "planning_map.yaml"

FORBIDDEN_ZONES = CONFIG_DIR / "forbidden_zones.yaml"


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def world_to_pixel(x, y, origin_x, origin_y, resolution, height):
    """
    ROS map coordinate -> image pixel coordinate

    ROS:
        x right
        y up

    Image:
        x right
        y down
    """

    px = (x - origin_x) / resolution
    py = height - 1 - (y - origin_y) / resolution

    return int(round(px)), int(round(py))


def main():

    map_info = load_yaml(LOCALIZATION_YAML)
    zones_info = load_yaml(FORBIDDEN_ZONES)

    resolution = float(map_info["resolution"])

    origin_x = float(map_info["origin"][0])
    origin_y = float(map_info["origin"][1])

    image = Image.open(LOCALIZATION_MAP).convert("L")

    width, height = image.size

    print(f"Map size: {width} x {height}")
    print(f"Resolution: {resolution}")
    print(f"Origin: ({origin_x}, {origin_y})")

    draw = ImageDraw.Draw(image)

    zones = zones_info.get("forbidden_zones", {})

    for zone_name, zone_data in zones.items():

        world_points = zone_data["points"]

        pixel_points = []

        print(f"\nZone: {zone_name}")

        for x, y in world_points:

            px, py = world_to_pixel(
                x,
                y,
                origin_x,
                origin_y,
                resolution,
                height
            )

            pixel_points.append((px, py))

            print(
                f"  world ({x:.3f}, {y:.3f})"
                f" -> pixel ({px}, {py})"
            )

        # 黑色 = occupied
        draw.polygon(
            pixel_points,
            fill=0
        )

    image.save(PLANNING_MAP)

    # 创建 planning_map.yaml
    planning_info = dict(map_info)

    planning_info["image"] = "planning_map.pgm"

    with open(
        PLANNING_YAML,
        "w",
        encoding="utf-8"
    ) as f:
        yaml.safe_dump(
            planning_info,
            f,
            default_flow_style=False,
            sort_keys=False
        )

    print("\nGenerated:")
    print(f"  {PLANNING_MAP}")
    print(f"  {PLANNING_YAML}")


if __name__ == "__main__":
    main()
