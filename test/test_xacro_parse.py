"""Verify that every URDF/Xacro source parses without error.

xacro doesn't need to fully resolve $(find ...) for upstream-only arm
mounts, but the file should at least be valid XML.
"""

import glob
import os
import xml.etree.ElementTree as ET
import pytest


PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _all_xacros():
    patterns = [
        os.path.join(PKG_ROOT, "arms", "*", "urdf", "*.urdf.xacro"),
        os.path.join(PKG_ROOT, "arms", "mounts", "*.urdf.xacro"),
        os.path.join(PKG_ROOT, "models", "*", "urdf", "*.urdf.xacro"),
    ]
    files = []
    for p in patterns:
        files.extend(glob.glob(p))
    return files


@pytest.mark.parametrize("path", _all_xacros())
def test_xacro_is_valid_xml(path):
    """Every xacro file must at least parse as XML."""
    try:
        ET.parse(path)
    except ET.ParseError as exc:
        pytest.fail(f"{path}: not valid XML: {exc}")


@pytest.mark.parametrize("path", _all_xacros())
def test_xacro_has_robot_root(path):
    """Every xacro must have a <robot> root element."""
    root = ET.parse(path).getroot()
    # Strip xmlns namespace from tag for the comparison.
    tag = root.tag.rsplit("}", 1)[-1]
    assert tag == "robot", f"{path}: root is <{tag}>, expected <robot>"
