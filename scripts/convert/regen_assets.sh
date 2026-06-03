#!/usr/bin/env bash
# Regenerate per-simulator asset outputs from the Xacro-URDF sources.
#
# This is a CI helper, not a runtime tool. It assumes the converter scripts
# alongside it are installed (urdf_to_mjcf.py, urdf_to_usd.py). Per-sim
# outputs are committed to the repo — never regenerate at launch.
#
# Usage:
#   scripts/convert/regen_assets.sh [arm_name ...]
# If no args given, regenerates all arms listed in arms/manifest.yaml.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ARMS_DIR="$PKG_ROOT/arms"

if [ "$#" -eq 0 ]; then
    ARMS=$(ls -1 "$ARMS_DIR" | grep -v '^mounts$\|^manifest.yaml$' || true)
else
    ARMS="$*"
fi

for arm in $ARMS; do
    urdf_xacro="$ARMS_DIR/$arm/urdf/$arm.urdf.xacro"
    if [ ! -f "$urdf_xacro" ]; then
        echo "[regen] $arm: no urdf/$arm.urdf.xacro — skipping (likely an upstream arm)"
        continue
    fi
    echo "[regen] $arm"

    # 1. Xacro -> URDF
    urdf="$ARMS_DIR/$arm/urdf/$arm.urdf"
    xacro "$urdf_xacro" > "$urdf"

    # 2. URDF -> MJCF (best-effort; may need hand patching)
    mjcf_dir="$ARMS_DIR/$arm/mjcf"
    mkdir -p "$mjcf_dir"
    python3 "$SCRIPT_DIR/urdf_to_mjcf.py" "$urdf" "$mjcf_dir/$arm.xml" || \
        echo "[regen] $arm: MJCF conversion failed (see urdf_to_mjcf.py log)"

    # 3. URDF -> USD: must be run inside Isaac's Python; we just touch a
    # marker file so CI can flag missing USDs.
    usd_dir="$ARMS_DIR/$arm/usd"
    mkdir -p "$usd_dir"
    if [ ! -f "$usd_dir/$arm.usd" ]; then
        echo "TODO: run scripts/convert/urdf_to_usd.py inside Isaac" > "$usd_dir/REGEN_PENDING"
    fi

    # 4. URDF -> PROTO: requires webots_ros2_importer; same marker pattern.
    proto_dir="$ARMS_DIR/$arm/proto"
    mkdir -p "$proto_dir"
    if [ ! -f "$proto_dir/$arm.proto" ]; then
        echo "TODO: run 'ros2 run webots_ros2_importer urdf2proto $urdf'" > "$proto_dir/REGEN_PENDING"
    fi

    echo "[regen] $arm: done"
done

echo "regeneration complete"
