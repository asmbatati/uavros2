"""MAVROS namespace choice for the fork (relative topics) vs upstream (~/ topics)."""

from uavros2.mavros_layout import _version, mavros_namespace, private_topics


def test_version_parse():
    assert _version("2.16.0") == (2, 16, 0)
    assert _version("2.6.0") == (2, 6, 0)


def test_fork_runs_in_ns_mavros(monkeypatch):
    monkeypatch.delenv("UAVROS2_MAVROS_PRIVATE_TOPICS", raising=False)
    assert mavros_namespace("drone", (2, 6, 0)) == "drone/mavros"


def test_upstream_runs_in_ns(monkeypatch):
    monkeypatch.delenv("UAVROS2_MAVROS_PRIVATE_TOPICS", raising=False)
    assert mavros_namespace("/drone", (2, 16, 0)) == "drone"


def test_env_override(monkeypatch):
    monkeypatch.setenv("UAVROS2_MAVROS_PRIVATE_TOPICS", "0")
    assert not private_topics((2, 16, 0))
    monkeypatch.setenv("UAVROS2_MAVROS_PRIVATE_TOPICS", "1")
    assert private_topics((2, 6, 0))
