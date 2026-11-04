#!/usr/bin/env python3
"""ROS entry point; implementation lives in community_mission_video."""
import runpy

if __name__ == "__main__":
    runpy.run_module("community_mission_video.video_recorder_node", run_name="__main__")
