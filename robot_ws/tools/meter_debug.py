#!/usr/bin/env python3
"""Export intermediate images from the meter reader for a single photo."""

import argparse
import json
from pathlib import Path
import sys

import cv2 as cv


MISSION_SCRIPTS = Path(__file__).resolve().parents[1] / "src/community_mission/scripts"
sys.path.insert(0, str(MISSION_SCRIPTS))
from meter_reader import Reader, draw_inlier_matches, load_image, quantize_reading, save_image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Captured meter image")
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory for debug files")
    parser.add_argument("--profile", type=Path, default=MISSION_SCRIPTS / "meter_profile/profile.json",
                        help="Calibration profile (default: bundled 4K profile)")
    args = parser.parse_args()
    try:
        frame = load_image(args.image)
        reader = Reader(args.profile)
        debug = {}
        values, overlay, status = reader.read(frame, debug=debug)
        output = args.output_dir
        output.mkdir(parents=True, exist_ok=False)
        save_image(output / "00_input.png", frame)
        save_image(output / "01_gray.png", debug["gray"])
        report = {
            "status": status,
            "temperature_c": quantize_reading("temperature", values["temperature"]),
            "humidity_percent": quantize_reading("humidity", values["humidity"]),
            "inlier_count": 0,
        }
        if status == "aligned":
            matches = draw_inlier_matches(reader.template, frame, debug["reference_points"],
                                          debug["frame_points"])
            save_image(output / "02_matches.jpg", matches)
            save_image(output / "03_aligned.png", debug["aligned"])
            save_image(output / "04_aligned_gray.png", debug["aligned_gray"])
            save_image(output / "05_needles.png", overlay)
            report["inlier_count"] = len(debug["frame_points"])
            report["template_to_frame"] = debug["matrix"].tolist()
        result = json.dumps(report, indent=2)
        (output / "result.json").write_text(result + "\n", encoding="utf-8")
        print(result)
        return 0 if status == "aligned" else 1
    except (OSError, ValueError, cv.error) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    sys.exit(main())
