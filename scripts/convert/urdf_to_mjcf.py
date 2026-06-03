#!/usr/bin/env python3
"""URDF -> MJCF converter (best-effort).

Uses MuJoCo's URDF importer. Some hand-patching of <actuator> and contact
parameters is typically required afterward.

Usage:
    urdf_to_mjcf.py INPUT.urdf OUTPUT.xml
"""

import sys
from pathlib import Path


def main():
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(2)

    in_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])

    if not in_path.is_file():
        print(f"error: input file not found: {in_path}", file=sys.stderr)
        sys.exit(1)

    try:
        import mujoco
    except ImportError:
        print("error: mujoco not installed. pip install mujoco", file=sys.stderr)
        sys.exit(1)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(in_path))
    mujoco.mj_saveLastXML(str(out_path), model)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
