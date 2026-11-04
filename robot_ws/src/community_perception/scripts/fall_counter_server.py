#!/usr/bin/env python3
"""ROS entry point; implementation lives in community_perception_fall."""
import runpy

if __name__ == "__main__":
    runpy.run_module("community_perception_fall.fall_counter_server", run_name="__main__")
