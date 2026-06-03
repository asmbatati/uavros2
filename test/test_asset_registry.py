"""Validate arms/<arm>/asset.yaml schemas and arms/manifest.yaml consistency."""

import os
import yaml
import pytest


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ARMS_DIR = os.path.join(PKG_ROOT, "arms")
MANIFEST = os.path.join(ARMS_DIR, "manifest.yaml")

REQUIRED_KEYS = {"name", "dof", "mount_frame", "mount_xyz", "mount_rpy",
                 "base_link", "ee_link", "joints", "default_controller", "sims"}


@pytest.fixture(scope="module")
def manifest():
    with open(MANIFEST) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def arm_names(manifest):
    return list(manifest["arms"].keys())


def test_manifest_loads(manifest):
    assert "arms" in manifest
    assert isinstance(manifest["arms"], dict)


def test_each_arm_has_asset_yaml(arm_names):
    for arm in arm_names:
        asset = os.path.join(ARMS_DIR, arm, "asset.yaml")
        assert os.path.isfile(asset), f"missing {asset}"


def test_each_arm_has_controllers_yaml(arm_names):
    for arm in arm_names:
        ctl = os.path.join(ARMS_DIR, arm, "config", "controllers.yaml")
        assert os.path.isfile(ctl), f"missing {ctl}"


@pytest.mark.parametrize("arm", ["three_dof", "openmanip_x", "panda", "ur5"])
def test_asset_yaml_has_required_keys(arm):
    asset_path = os.path.join(ARMS_DIR, arm, "asset.yaml")
    with open(asset_path) as f:
        asset = yaml.safe_load(f)
    missing = REQUIRED_KEYS - set(asset.keys())
    assert not missing, f"{arm}: missing keys {missing}"


@pytest.mark.parametrize("arm", ["three_dof", "openmanip_x", "panda", "ur5"])
def test_asset_yaml_joint_count_matches_dof(arm):
    asset_path = os.path.join(ARMS_DIR, arm, "asset.yaml")
    with open(asset_path) as f:
        asset = yaml.safe_load(f)
    assert len(asset["joints"]) == asset["dof"], (
        f"{arm}: dof={asset['dof']} but joints={asset['joints']}"
    )


def test_each_arm_has_mount_xacro(arm_names):
    for arm in arm_names:
        mount = os.path.join(ARMS_DIR, "mounts", f"x500_{arm}.urdf.xacro")
        assert os.path.isfile(mount), f"missing mount file: {mount}"


def test_each_arm_has_px4_airframe(manifest):
    px4_dir = os.path.join(PKG_ROOT, "config", "px4")
    for arm, meta in manifest["arms"].items():
        airframe_id = meta["px4_airframe"]
        # Find a file beginning with "<id>_gz_x500_with_<arm>"
        prefix = f"{airframe_id}_gz_x500_with_{arm}"
        matches = [f for f in os.listdir(px4_dir) if f.startswith(prefix)]
        assert matches, f"no PX4 airframe at {px4_dir}/{prefix}*"
