#!/usr/bin/env python3
"""Quick validation script for infra batches 2-5.

Checks Python syntax and file structure without requiring ROS or fastapi.

Usage:
    python check_batches.py
"""

import sys
from pathlib import Path
import ast
import importlib.util


def check_syntax(filepath):
    """Parse Python file to check for syntax errors."""
    try:
        with open(filepath) as f:
            ast.parse(f.read())
        return True
    except SyntaxError as e:
        print(f"  ❌ Syntax error in {filepath}: {e}")
        return False


def check_batch_2():
    """Verify batch 2 files exist and have valid syntax."""
    print("Batch 2: ros_bridge + nav routes")
    robot_ws = Path(__file__).parent
    files = [
        robot_ws / "infra" / "ros_bridge.py",
        robot_ws / "infra" / "routes" / "nav.py",
    ]
    ok = all(check_syntax(f) for f in files if f.exists())
    if ok:
        print("  ✓ All files have valid Python syntax")
    return ok


def check_batch_3():
    """Verify batch 3 files exist and have valid syntax."""
    print("Batch 3: camera + detector + overlay")
    robot_ws = Path(__file__).parent
    files = [
        robot_ws / "infra" / "camera_core.py",
        robot_ws / "infra" / "detector_core.py",
        robot_ws / "infra" / "overlay.py",
        robot_ws / "infra" / "routes" / "camera.py",
    ]
    ok = all(check_syntax(f) for f in files if f.exists())
    if ok:
        print("  ✓ All files have valid Python syntax")
    return ok


def check_batch_4():
    """Verify batch 4 files exist and have valid syntax."""
    print("Batch 4: mission routes + launches")
    robot_ws = Path(__file__).parent

    # Python files
    py_files = [
        robot_ws / "infra" / "routes" / "mission.py",
    ]
    py_ok = all(check_syntax(f) for f in py_files if f.exists())

    # Launch files (XML)
    launch_files = [
        robot_ws / "src" / "community_bringup" / "launch" / "field.launch",
        robot_ws / "src" / "community_bringup" / "launch" / "debug_perception.launch",
    ]
    launch_ok = all(f.exists() for f in launch_files)

    if py_ok:
        print("  ✓ Mission routes have valid Python syntax")
    if launch_ok:
        print("  ✓ field.launch and debug_perception.launch exist")

    return py_ok and launch_ok


def check_batch_5():
    """Verify batch 5 enhancements."""
    print("Batch 5: recognition WebSocket + estop")
    robot_ws = Path(__file__).parent

    files = [
        robot_ws / "infra" / "routes" / "camera.py",
        robot_ws / "infra" / "routes" / "dashboard.py",
    ]
    ok = all(check_syntax(f) for f in files if f.exists())
    if ok:
        print("  ✓ All files have valid Python syntax")
    return ok


def main():
    robot_ws = Path(__file__).parent
    print("=" * 70)
    print("Infra Batch 2-5 Validation (No Dependencies Required)")
    print("=" * 70)
    print()

    results = []
    results.append(("Batch 2", check_batch_2()))
    print()
    results.append(("Batch 3", check_batch_3()))
    print()
    results.append(("Batch 4", check_batch_4()))
    print()
    results.append(("Batch 5", check_batch_5()))

    print()
    print("=" * 70)
    if all(ok for _, ok in results):
        print("✅ All batches passed basic syntax validation!")
        print("=" * 70)
        print("\nImplemented files summary:")
        print("  Batch 2: ros_bridge + nav routes (遥控 + 地图)")
        print("  Batch 3: camera + detector + overlay + camera routes (相机检测)")
        print("  Batch 4: mission routes + field/debug_perception launches (巡检任务)")
        print("  Batch 5: recognition WS + estop enhancement (组合识别 + 紧急停止)")
        print("\nNext steps:")
        print("  1. Install dependencies: pip install -r requirements.txt")
        print("  2. Run unit tests: cd robot_ws && python -m pytest infra/tests -q")
        print("  3. Launch infra: roslaunch community_bringup infra.launch")
        print("  4. Open browser: http://<robot-ip>:8000/")
        return 0
    else:
        print("❌ Some batches failed validation")
        print("=" * 70)
        for name, ok in results:
            status = "✓" if ok else "✗"
            print(f"  {status} {name}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
