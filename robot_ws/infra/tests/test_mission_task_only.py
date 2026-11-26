"""Exercise mission API launch validation without ROS or starting processes."""
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

from infra.routes import control, mission


@pytest.fixture
def launch_environment(monkeypatch):
    started = []
    manager = SimpleNamespace(
        is_running=lambda name: False, all_records=lambda: [],
        start=lambda name, cmd, args: started.append((name, cmd, args)) or
        SimpleNamespace(to_dict=lambda: {'name': name, 'args': args}))
    bridge = SimpleNamespace(get_ros_nodes=lambda: ['/usb_cam', '/speech_server'])
    monkeypatch.setattr(control, 'get_manager', lambda: manager)
    monkeypatch.setattr(control, 'get_bridge', lambda: bridge)
    return started


def test_task_only_starts_with_camera_and_services_without_move_base(launch_environment):
    result = mission.start_mission(mission.MissionStart(task_only=True, waypoint='wp_06'))
    assert result['ok']
    name, cmd, args = launch_environment[0]
    assert name == 'mission'
    assert args['task_only'] == 'true'
    assert args['single_waypoint'] == 'wp_06'
    assert 'task_only:=true' in cmd


def test_normal_mode_still_requires_hardware_and_navigation(launch_environment):
    with pytest.raises(HTTPException) as error:
        mission.start_mission(mission.MissionStart(waypoint='wp_06'))
    assert error.value.status_code == 409
    assert 'navigation' in error.value.detail
    assert not launch_environment


@pytest.mark.parametrize('payload', [
    mission.MissionStart(task_only=True, navigation_only=True),
    mission.MissionStart(task_only=True, waypoint='wp_35'),
    mission.MissionStart(task_only=True, waypoint='missing'),
])
def test_invalid_task_only_requests_start_nothing(launch_environment, payload):
    with pytest.raises(HTTPException) as error:
        mission.start_mission(payload)
    assert error.value.status_code == 400
    assert not launch_environment
