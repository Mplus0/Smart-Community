"""Tests for launch argument validation and component conflicts."""

from __future__ import annotations

from infra import config


def test_base_protocol_required():
    spec = config.get_launch("hardware")
    assert "base_protocol" in config.validate_launch_args(spec, dict(spec.args))
    assert config.validate_launch_args(spec, dict(spec.args, base_protocol="aa_bb")) is None
    assert "无效" in config.validate_launch_args(spec, dict(spec.args, base_protocol="x"))


def test_missing_map_file_rejected():
    spec = config.get_launch("navigation")
    assert "不存在" in config.validate_launch_args(spec, dict(spec.args, map="/no/such/map.yaml"))


def test_field_overlaps_hardware():
    hw = config.launch_components("hardware", {})
    field = config.launch_components("field", {})
    assert hw & field
    assert config.launch_components("navigation", {}) & config.launch_components("mapping", {})


def test_mission_requirements():
    assert "services" in config.launch_requires("mission", {"navigation_only": "false"})
    assert "services" not in config.launch_requires("mission", {"navigation_only": "true"})
    assert "mission" in config.launch_components("community", {"run_mission": "true"})
    assert "mission" not in config.launch_components("community", {"run_mission": "false"})
