"""Generator output stability tests.

These tests assert the generator is deterministic and that round-tripping
a UAV descriptor → built artefacts produces the same files we ship.
"""

import pathlib

import pytest

from uavros2.asset_builder import build_gazebo, build_px4
from uavros2.asset_spec import Catalog


REPO = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def catalog() -> Catalog:
    return Catalog(REPO / "assets")


def test_gazebo_build_idempotent(catalog, tmp_path):
    """Build the same UAV twice → identical output."""
    uav = catalog.uavs["x500_stereo_cam_3d_lidar"]
    a = tmp_path / "run_a"
    b = tmp_path / "run_b"
    build_gazebo(uav, catalog, a)
    build_gazebo(uav, catalog, b)
    for rel in ["models/x500_stereo_cam_3d_lidar/model.sdf",
                "models/x500_stereo_cam_3d_lidar/model.config"]:
        assert (a / rel).read_text() == (b / rel).read_text()


def test_gazebo_build_matches_committed(catalog, tmp_path):
    """Built SDF matches the file we ship in the repo."""
    uav = catalog.uavs["x500_stereo_cam_3d_lidar"]
    build_gazebo(uav, catalog, tmp_path)
    rel = "models/x500_stereo_cam_3d_lidar/model.sdf"
    committed = (REPO / rel).read_text()
    built = (tmp_path / rel).read_text()
    assert committed == built, (
        "Generated SDF drifted from committed.\n"
        "Run: uavros2-asset build x500_stereo_cam_3d_lidar"
    )


def test_px4_build_matches_committed(catalog, tmp_path):
    uav = catalog.uavs["x500_stereo_cam_3d_lidar"]
    build_px4(uav, catalog, tmp_path)
    rel = "config/px4/4023_gz_x500_stereo_cam_3d_lidar"
    committed = (REPO / rel).read_text()
    built = (tmp_path / rel).read_text()
    assert committed == built, (
        "Generated PX4 airframe drifted from committed.\n"
        "Run: uavros2-asset build x500_stereo_cam_3d_lidar --target px4"
    )


def test_px4_rotor_order_matches_convention(catalog, tmp_path):
    """ROTOR0..3 PX/PY must follow PX4's expected (fl, rr, fr, rl) order."""
    uav = catalog.uavs["x500_stereo_cam_3d_lidar"]
    build_px4(uav, catalog, tmp_path)
    text = (tmp_path / "config/px4/4023_gz_x500_stereo_cam_3d_lidar").read_text()
    # Front-left ccw: ROTOR0 at (+0.13, +0.22), km positive
    assert "param set-default CA_ROTOR0_PX 0.13" in text
    assert "param set-default CA_ROTOR0_PY 0.22" in text
    assert "param set-default CA_ROTOR0_KM +0.05" in text
    # Rear-right ccw: ROTOR1 at (-0.13, -0.20), km positive
    assert "param set-default CA_ROTOR1_PX -0.13" in text
    assert "param set-default CA_ROTOR1_PY -0.20" in text
    assert "param set-default CA_ROTOR1_KM +0.05" in text
    # Front-right cw: ROTOR2, km negative
    assert "param set-default CA_ROTOR2_PX 0.13" in text
    assert "param set-default CA_ROTOR2_PY -0.22" in text
    assert "param set-default CA_ROTOR2_KM -0.05" in text
