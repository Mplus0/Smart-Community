"""Tests for :mod:`infra.launch_manager`.

Uses ``sleep`` as a stand-in for ``roslaunch`` so the tests work on any
POSIX host without ROS installed. Skipped on Windows where process-group
behaviour differs.
"""

from __future__ import annotations

import os
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(os.name != "posix", reason="POSIX-only pgid behaviour")

from infra import launch_manager, state_bus
from infra.launch_manager import build_roslaunch_cmd


@pytest.fixture(autouse=True)
def _reset(tmp_path, monkeypatch):
    monkeypatch.setenv("INFRA_RUNTIME_DIR", str(tmp_path))
    # Reload config so the new runtime dir is picked up.
    import importlib

    from infra import config
    importlib.reload(config)
    importlib.reload(launch_manager)
    state_bus.reset_bus_for_tests()
    yield
    launch_manager.reset_manager_for_tests()
    state_bus.reset_bus_for_tests()


def _wait_until(pred, timeout=2.0, step=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(step)
    return False


def test_start_and_stop_sleep():
    mgr = launch_manager.LaunchManager()
    record = mgr.start("demo", [sys.executable, "-c", "import time; time.sleep(30)"])
    assert record.status == "running"
    assert mgr.is_running("demo")

    final = mgr.stop("demo")
    assert final.status in ("exited", "failed")
    assert final.exit_code is not None
    assert not mgr.is_running("demo")


def test_duplicate_start_rejected():
    mgr = launch_manager.LaunchManager()
    mgr.start("dup", [sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        with pytest.raises(launch_manager.AlreadyRunning):
            mgr.start("dup", [sys.executable, "-c", "pass"])
    finally:
        mgr.stop("dup")


def test_stop_unknown_raises():
    mgr = launch_manager.LaunchManager()
    with pytest.raises(launch_manager.NotRunning):
        mgr.stop("ghost")


def test_log_lines_reach_bus():
    bus = state_bus.get_bus()
    q = bus.subscribe_log()
    mgr = launch_manager.LaunchManager(bus=bus)
    mgr.start(
        "chatty",
        [sys.executable, "-c", "import time; print('hello from child', flush=True); time.sleep(5)"],
    )
    # Wait for the reader thread to pick up the line.
    assert _wait_until(lambda: not q.empty())
    payload = q.get_nowait()
    assert payload["name"] == "chatty"
    assert "hello" in payload["line"]
    mgr.stop("chatty")


def test_self_exit_updates_history():
    mgr = launch_manager.LaunchManager()
    mgr.start("short", [sys.executable, "-c", "pass"])
    assert _wait_until(lambda: not mgr.is_running("short"))
    rec = mgr.status("short")
    assert rec is not None
    assert rec.status in ("exited", "failed")
    assert rec.stop_reason == "self-exit"


def test_build_roslaunch_cmd_filters_empty():
    cmd = build_roslaunch_cmd("pkg", "foo.launch", {"a": "1", "b": "", "c": None})
    assert cmd == ["roslaunch", "pkg", "foo.launch", "a:=1"]
