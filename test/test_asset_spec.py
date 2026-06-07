"""Asset descriptor schema + loader + validator tests.

Smoke-level: load the catalog, assert it validates, assert ref resolution
works, assert a bad descriptor fails loudly.
"""

import pathlib

import pytest

from uavros2.asset_spec import (
    Catalog, MultirotorChassis, SensorDescriptor, UAVDescriptor,
    load_any, validate_catalog,
)


REPO = pathlib.Path(__file__).resolve().parent.parent
ASSETS = REPO / "assets"


@pytest.fixture(scope="module")
def catalog() -> Catalog:
    return Catalog(ASSETS)


def test_catalog_loads(catalog):
    # We expect at least the seed primitives and the one UAV we've authored.
    assert "x500" in catalog.chassis
    assert "imu_default" in catalog.sensors
    assert "stereo_camera" in catalog.sensors
    assert "velodyne_16" in catalog.sensors
    assert "x500_stereo_cam_3d_lidar" in catalog.uavs


def test_catalog_validates_clean(catalog):
    problems = validate_catalog(catalog)
    assert problems == [], "\n".join(problems)


def test_x500_chassis_shape(catalog):
    c = catalog.chassis["x500"]
    assert isinstance(c, MultirotorChassis)
    assert c.layout == "quad_x"
    assert set(c.rotors) == {"fl", "fr", "rl", "rr"}
    # CCW vs CW assignment matches PX4 conventions for x500
    assert c.rotors["fl"].spin == "ccw"
    assert c.rotors["fr"].spin == "cw"
    assert c.rotors["rl"].spin == "cw"
    assert c.rotors["rr"].spin == "ccw"


def test_uav_resolves_chassis_and_sensors(catalog):
    uav = catalog.uavs["x500_stereo_cam_3d_lidar"]
    assert uav.chassis.ref == "x500"
    assert uav.px4.airframe_id == 4023
    sensor_names = {s.name for s in uav.sensors}
    # canonical names must be unique
    assert len(sensor_names) == len(uav.sensors)
    assert "front_stereo" in sensor_names
    assert "front_lidar" in sensor_names


def test_unknown_kind_raises(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("version: 1\nkind: nonsense\nname: bogus\n")
    with pytest.raises(ValueError, match="kind"):
        load_any(bad)


def test_unknown_chassis_ref_caught_by_validator(tmp_path, catalog):
    # Build a UAV that points at a nonexistent chassis; validator must flag it.
    bad = tmp_path / "ghost_uav.yaml"
    bad.write_text(
        "version: 1\nkind: uav\nname: ghost_uav\n"
        "chassis: { ref: nonexistent_chassis }\n"
        "px4: { airframe_id: 4999 }\n"
    )
    bad_uav = load_any(bad)
    catalog.uavs[bad_uav.name] = bad_uav
    problems = validate_catalog(catalog)
    assert any("nonexistent_chassis" in p for p in problems)
    # Clean up so other tests in the same fixture scope still see a clean catalog
    catalog.uavs.pop(bad_uav.name)


def test_airframe_id_collision_caught(tmp_path, catalog):
    bad = tmp_path / "twin_uav.yaml"
    bad.write_text(
        "version: 1\nkind: uav\nname: twin_uav\n"
        "chassis: { ref: x500 }\n"
        "px4: { airframe_id: 4023 }\n"   # collides with x500_stereo_cam_3d_lidar
    )
    twin = load_any(bad)
    catalog.uavs[twin.name] = twin
    problems = validate_catalog(catalog)
    assert any("airframe_id collision" in p for p in problems)
    catalog.uavs.pop(twin.name)
