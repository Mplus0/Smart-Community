#!/usr/bin/env python3
"""ROS entry point; implementation lives in community_perception_yolo."""
import runpy

if __name__ == "__main__":
    runpy.run_module("community_perception_yolo.yolo_detector_service", run_name="__main__")
