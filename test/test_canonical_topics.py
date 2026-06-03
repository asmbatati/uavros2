"""Validate the canonical topic contract YAML schema."""

import os
import yaml
import pytest


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONTRACT_PATH = os.path.join(PKG_ROOT, "config", "topics", "canonical_topics.yaml")


@pytest.fixture(scope="module")
def contract():
    with open(CONTRACT_PATH) as f:
        return yaml.safe_load(f)


def test_contract_loads(contract):
    assert isinstance(contract, dict)


def test_contract_has_required_sections(contract):
    for section in ("namespace", "sensors", "state", "control", "frames"):
        assert section in contract, f"missing section: {section}"


def test_sensor_topics_present(contract):
    for sensor in ("imu", "gps", "air_pressure", "front_lidar", "front_stereo"):
        assert sensor in contract["sensors"], f"missing sensor: {sensor}"


def test_state_topics_present(contract):
    for state in ("local_pose", "local_odom", "joint_states"):
        assert state in contract["state"], f"missing state topic: {state}"


def test_control_topics_present(contract):
    for ctrl in ("setpoint_position", "setpoint_velocity",
                 "actuators", "joint_command"):
        assert ctrl in contract["control"], f"missing control topic: {ctrl}"


def test_topic_strings_use_ns_template(contract):
    """Every topic string must contain {ns} so downstream code can substitute."""
    flat = []

    def _walk(obj):
        if isinstance(obj, str) and obj.startswith("/"):
            flat.append(obj)
        elif isinstance(obj, dict):
            for v in obj.values():
                _walk(v)
        elif isinstance(obj, list):
            for v in obj:
                _walk(v)

    for section in ("sensors", "state", "control"):
        _walk(contract[section])

    for topic in flat:
        assert "{ns}" in topic, f"topic {topic!r} does not template the namespace"
