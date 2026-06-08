"""Unit tests for runtime_sensor_manager — mocks the gz CLI.

We test the catalog-resolution + SDF-string generation paths, plus the
add/remove/list bookkeeping. The actual subprocess.run() is patched so
the tests don't require a running Gazebo.
"""

from __future__ import annotations

import pathlib
import subprocess
from unittest.mock import patch

import pytest

# Skip the whole module gracefully when rclpy isn't importable (eg. a
# pure-python pytest run in a dev shell without /opt/ros sourced).
rclpy = pytest.importorskip("rclpy")

try:
    from uavros2_msgs.srv import AddSensor, ListSensors, RemoveSensor
    HAVE_MSGS = True
except ImportError:
    HAVE_MSGS = False


REPO = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture
def node():
    """Spin up a RuntimeSensorManager backed by the in-tree catalog."""
    if not HAVE_MSGS:
        pytest.skip("uavros2_msgs not built — colcon build it first")
    from uavros2.runtime_sensor_manager import RuntimeSensorManager
    rclpy.init()
    try:
        n = RuntimeSensorManager()
        # Override the catalog to point at the source tree (skips share/).
        n._catalog = type(n._catalog)(REPO / "assets")
        yield n
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _fake_proc(stdout: str = "data: true", returncode: int = 0):
    cp = subprocess.CompletedProcess(args=[], returncode=returncode,
                                      stdout=stdout, stderr="")
    return cp


def test_add_unknown_sensor_ref_fails(node):
    req = AddSensor.Request(
        uav_namespace="drone", name="x", sensor_ref="nonexistent",
        mount="base_link", pose=[0.0] * 6, overrides_yaml="",
    )
    resp = node._cb_add(req, AddSensor.Response())
    assert resp.success is False
    assert "Unknown sensor ref" in resp.message


def test_add_embedded_sensor_fails(node):
    """IMU/GPS/baro are embedded in the chassis SDF; nothing to spawn."""
    req = AddSensor.Request(
        uav_namespace="drone", name="extra_imu", sensor_ref="imu_default",
        mount="base_link", pose=[0.0] * 6, overrides_yaml="",
    )
    with patch("uavros2.runtime_sensor_manager._have_gz_cli", return_value=True):
        resp = node._cb_add(req, AddSensor.Response())
    assert resp.success is False
    assert "embedded sensor" in resp.message


@patch("uavros2.runtime_sensor_manager._have_gz_cli", return_value=True)
@patch("uavros2.runtime_sensor_manager.subprocess.run", return_value=_fake_proc())
def test_add_then_remove_roundtrip(mock_run, _mock_have, node):
    add_req = AddSensor.Request(
        uav_namespace="drone", name="extra_lidar", sensor_ref="velodyne_16",
        mount="base_link", pose=[0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
        overrides_yaml="",
    )
    resp = node._cb_add(add_req, AddSensor.Response())
    assert resp.success is True, resp.message
    assert "extra_lidar" in node._live

    # list should report 1
    lresp = node._cb_list(ListSensors.Request(uav_namespace="drone"),
                          ListSensors.Response())
    assert lresp.sensor_names == ["extra_lidar"]
    assert lresp.sensor_refs == ["velodyne_16"]

    # adding the same name again fails
    resp2 = node._cb_add(add_req, AddSensor.Response())
    assert resp2.success is False
    assert "already live" in resp2.message

    # remove
    rresp = node._cb_remove(
        RemoveSensor.Request(uav_namespace="drone", name="extra_lidar"),
        RemoveSensor.Response(),
    )
    assert rresp.success is True
    assert "extra_lidar" not in node._live


@patch("uavros2.runtime_sensor_manager._have_gz_cli", return_value=False)
def test_no_gz_cli_returns_clear_error(_mock_have, node):
    req = AddSensor.Request(
        uav_namespace="drone", name="lidar", sensor_ref="velodyne_16",
        mount="base_link", pose=[0.0] * 6, overrides_yaml="",
    )
    resp = node._cb_add(req, AddSensor.Response())
    assert resp.success is False
    assert "gz" in resp.message.lower()
