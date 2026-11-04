#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generate two random Chinese blue license plates by calling the API referenced in:
https://blog.csdn.net/lzl640/article/details/125312808

Targets:
  models/car_plate2/materials/textures/car_plate2.jpg
  models/car_plate3/materials/textures/car_plate3.jpg

Only Python standard library is required.
"""

import argparse
import base64
import json
import random
import re
import sys
import time
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import Request, urlopen


DEFAULT_API_URL = "http://new.hdsxsc.com:10086/server.php"

# API's normal blue/yellow plate regex:
# [province][A-Z][A-HJ-NP-Z0-9]{5}
PROVINCES = list("京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼")
REGION_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
SUFFIX_CHARS = "ABCDEFGHJKLMNPQRSTUVWXYZ0123456789"

BASE64_PATTERN = re.compile(
    r'data:image/jpeg;base64,([A-Za-z0-9+/=\r\n]+)',
    re.IGNORECASE,
)


def generate_plate_number():
    """Generate one plate number accepted by the API's normal-blue-plate regex."""
    province = random.choice(PROVINCES)
    region = random.choice(REGION_LETTERS)
    suffix = "".join(random.choice(SUFFIX_CHARS) for _ in range(5))
    return province + region + suffix


def request_plate_image(api_url, plate_number, timeout):
    """Call the API and return JPEG bytes."""
    params = urlencode({
        "cphm": plate_number,
        "cpys": "0",     # blue plate
        "double": "0",   # single row
    })
    url = f"{api_url}?{params}"

    request = Request(
        url,
        headers={
            "User-Agent": "Mozilla/5.0 Smart-Community-Gazebo/1.0",
            "Accept": "text/html,*/*",
        },
    )

    with urlopen(request, timeout=timeout) as response:
        body = response.read().decode("utf-8", errors="replace")

    if "号码规则无效" in body:
        raise RuntimeError(f"API rejected plate number: {plate_number}")

    match = BASE64_PATTERN.search(body)
    if not match:
        raise RuntimeError("No Base64 JPEG image found in API response")

    image_bytes = base64.b64decode(match.group(1), validate=False)

    # Basic JPEG integrity check.
    if len(image_bytes) < 4 or image_bytes[:2] != b"\xff\xd8":
        raise RuntimeError("API response did not decode to a valid JPEG image")

    return image_bytes


def write_atomic(path, data):
    """Avoid leaving a half-written texture if the process is interrupted."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    tmp.replace(path)


def generate_one(api_url, target, used_numbers, timeout, retries):
    last_error = None

    for attempt in range(1, retries + 1):
        plate_number = generate_plate_number()
        while plate_number in used_numbers:
            plate_number = generate_plate_number()

        try:
            image = request_plate_image(api_url, plate_number, timeout)
            write_atomic(target, image)
            used_numbers.add(plate_number)
            print(f"[OK] {target.name}: {plate_number}")
            return plate_number
        except Exception as exc:
            last_error = exc
            print(
                f"[WARN] attempt {attempt}/{retries} failed for "
                f"{target.name}: {exc}",
                file=sys.stderr,
            )
            if attempt < retries:
                time.sleep(1.0)

    # Never overwrite a valid previous texture when the online API fails.
    if target.exists():
        print(
            f"[WARN] API unavailable. Keeping existing texture: {target}",
            file=sys.stderr,
        )
        return None

    raise RuntimeError(
        f"Failed to generate {target.name}, and no fallback texture exists. "
        f"Last error: {last_error}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate random blue license-plate textures for Gazebo."
    )
    parser.add_argument(
        "--api-url",
        default=DEFAULT_API_URL,
        help=f"Plate API endpoint (default: {DEFAULT_API_URL})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=6.0,
        help="HTTP timeout in seconds (default: 6)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="Attempts per plate (default: 3)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional random seed for repeatable testing",
    )
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    # scripts/generate_random_plates.py -> robot_gazebo/
    package_root = Path(__file__).resolve().parents[1]

    targets = {
        "car_plate2": (
            package_root
            / "models/car_plate2/materials/textures/car_plate2.jpg"
        ),
        "car_plate3": (
            package_root
            / "models/car_plate3/materials/textures/car_plate3.jpg"
        ),
    }

    used_numbers = set()
    results = {}

    print("========================================")
    print("Generating random Gazebo license plates")
    print("========================================")

    for model_name, target in targets.items():
        number = generate_one(
            api_url=args.api_url,
            target=target,
            used_numbers=used_numbers,
            timeout=args.timeout,
            retries=max(1, args.retries),
        )
        results[model_name] = number

    results_dir = package_root / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    result_file = results_dir / "generated_plates.json"

    payload = {
        "car_plate2": results["car_plate2"],
        "car_plate3": results["car_plate3"],
        "api_url": args.api_url,
        "note": (
            "null means the API failed and the previous texture was retained."
        ),
    }
    result_file.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print("----------------------------------------")
    print(f"Result record: {result_file}")
    print("========================================")


if __name__ == "__main__":
    main()
