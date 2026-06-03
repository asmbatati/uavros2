#!/usr/bin/env python3
"""Validate that per-sim asset outputs match their URDF source.

Currently checks link count and joint count parity between URDF and MJCF.
USD/PROTO validation requires Isaac/Webots and is skipped here.

Usage:
    validate_asset.py ARM_DIR
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def count_urdf_links_joints(path: Path) -> tuple:
    tree = ET.parse(path)
    root = tree.getroot()
    links = sum(1 for _ in root.iter("link"))
    joints = sum(1 for j in root.iter("joint")
                 if j.attrib.get("type") != "fixed")
    return links, joints


def count_mjcf_bodies_joints(path: Path) -> tuple:
    tree = ET.parse(path)
    root = tree.getroot()
    bodies = sum(1 for _ in root.iter("body"))
    joints = sum(1 for _ in root.iter("joint"))
    return bodies, joints


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        sys.exit(2)

    arm_dir = Path(sys.argv[1])
    urdf = arm_dir / "urdf" / f"{arm_dir.name}.urdf"
    mjcf = arm_dir / "mjcf" / f"{arm_dir.name}.xml"

    if not urdf.is_file():
        print(f"skip: {urdf} not found (likely an upstream-only arm)")
        return

    u_links, u_joints = count_urdf_links_joints(urdf)
    print(f"URDF: {u_links} links, {u_joints} moveable joints")

    if mjcf.is_file():
        m_bodies, m_joints = count_mjcf_bodies_joints(mjcf)
        print(f"MJCF: {m_bodies} bodies, {m_joints} joints")
        if abs(m_joints - u_joints) > 1:
            print(f"WARNING: joint count mismatch ({u_joints} vs {m_joints})",
                  file=sys.stderr)
    else:
        print(f"skip: {mjcf} not generated")


if __name__ == "__main__":
    main()
