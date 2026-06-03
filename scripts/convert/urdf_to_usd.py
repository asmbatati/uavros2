#!/usr/bin/env python3
"""URDF -> USD converter — MUST be run inside Isaac Sim's Python.

Cannot be imported in a normal Python interpreter; the Isaac importer
lives at omni.importer.urdf and depends on the running Omniverse stack.

Usage (from inside Isaac):
    ./python.sh path/to/urdf_to_usd.py INPUT.urdf OUTPUT.usd
"""

import sys
from pathlib import Path


def main():
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(2)

    in_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])

    try:
        from omni.isaac.urdf import _urdf as urdf
        from omni.isaac.core.utils.stage import open_stage, save_stage
    except ImportError:
        print(
            "error: omni.isaac.urdf not available. This script must run "
            "inside Isaac Sim's Python interpreter (./python.sh).",
            file=sys.stderr,
        )
        sys.exit(1)

    out_path.parent.mkdir(parents=True, exist_ok=True)

    cfg = urdf.ImportConfig()
    cfg.merge_fixed_joints = False
    cfg.fix_base = False
    cfg.make_default_prim = True
    _, prim_path = urdf.parse_and_import_urdf(str(in_path), cfg)
    save_stage(str(out_path))
    print(f"wrote {out_path} (prim {prim_path})")


if __name__ == "__main__":
    main()
