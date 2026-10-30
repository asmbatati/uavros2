#!/usr/bin/env bash
# Copy uavros2's Gazebo models, worlds and PX4 airframes into a PX4-Autopilot
# tree and register the airframes in PX4's ROMFS CMakeLists.
#
# PX4 SITL only sees models/worlds under Tools/simulation/gz and airframes
# compiled into its ROMFS, so after editing anything under models/, worlds/
# or config/px4/ run this and rebuild PX4 (`make px4_sitl`). install.sh runs
# it for you.
#
# Usage: sync_px4_assets.sh [PX4_DIR]
#   PX4_DIR defaults to $DEV_DIR/PX4-Autopilot, else the PX4-Autopilot next
#   to the ros2_ws that holds this package.
set -e

UAVROS2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_SRC="$(dirname "$UAVROS2_DIR")"
if [ -n "$1" ]; then
    PX4_DIR="$1"
elif [ -n "$DEV_DIR" ]; then
    PX4_DIR="$DEV_DIR/PX4-Autopilot"
else
    PX4_DIR="$(cd "$UAVROS2_DIR/../../.." && pwd)/PX4-Autopilot"
fi
if [ ! -d "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes" ]; then
    echo "[ERROR] $PX4_DIR is not a PX4-Autopilot tree" >&2
    exit 1
fi

print_status()  { echo -e "\033[0;32m[OK]\033[0m $1"; }
print_error()   { echo -e "\033[0;31m[ERROR]\033[0m $1" >&2; }
print_info()    { echo -e "\033[0;34m[INFO]\033[0m $1"; }
print_warning() { echo -e "\033[1;33m[WARN]\033[0m $1"; }

print_info "Syncing uavros2 assets from $UAVROS2_DIR into $PX4_DIR"

# Copy files to $PX4_DIR
print_info "Copying models to ${PX4_DIR}/Tools/simulation/gz/models/"
if [ -d "${UAVROS2_DIR}/models" ]; then
    cp -r ${UAVROS2_DIR}/models/* ${PX4_DIR}/Tools/simulation/gz/models/ || {
        print_error "Failed to copy models"
        exit 1
    }
    print_status "Models copied"
else
    print_error "Models directory not found at ${UAVROS2_DIR}/models"
    exit 1
fi

print_info "Copying worlds to ${PX4_DIR}/Tools/simulation/gz/worlds/"
if [ -d "${UAVROS2_DIR}/worlds" ]; then
    cp -r ${UAVROS2_DIR}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/ || {
        print_error "Failed to copy worlds"
        exit 1
    }
    print_status "Worlds copied"
else
    print_error "Worlds directory not found at ${UAVROS2_DIR}/worlds"
    exit 1
fi

# RViz uses Ogre which doesn't have a TIFF codec — meshes that reference
# .tif textures (the urban1/2/3 terrain DAEs do) render as untextured.
# Convert any .tif texture referenced by a .dae into .png and patch the
# .dae to point at the new path. We do this on the PX4-side copy, not on
# the LFS source, so the repo stays clean.
print_info "Converting TIFF textures referenced by urban* terrain DAEs for RViz..."
python3 - "${PX4_DIR}/Tools/simulation/gz/models" <<'PY' || \
    print_warning "TIFF→PNG conversion failed; urban1/2/3 meshes may render untextured"
import os, sys, re
try:
    from PIL import Image
except ImportError:
    print("Pillow not installed; skipping TIFF conversion")
    sys.exit(0)

models_root = sys.argv[1]
converted = 0
for entry in sorted(os.listdir(models_root)):
    if not entry.startswith("urban") or not entry.endswith("_terrain"):
        continue
    model_dir = os.path.join(models_root, entry)
    # Walk only this terrain dir
    for dirpath, _, files in os.walk(model_dir):
        for f in files:
            if not f.lower().endswith(".dae"):
                continue
            dae_path = os.path.join(dirpath, f)
            with open(dae_path) as df:
                text = df.read()
            # Find all <init_from>...tif</init_from> references
            refs = re.findall(
                r"<init_from>\s*([^<\s]+\.tif)\s*</init_from>",
                text, flags=re.IGNORECASE)
            for ref in set(refs):
                # Resolve relative path against the DAE's dir
                src = os.path.normpath(os.path.join(dirpath, ref))
                if not os.path.isfile(src):
                    continue
                dst = os.path.splitext(src)[0] + ".png"
                if not os.path.isfile(dst):
                    try:
                        Image.open(src).convert("RGB").save(dst)
                        converted += 1
                        print(f"  converted {src} → {dst}")
                    except Exception as exc:
                        print(f"  failed to convert {src}: {exc}")
                        continue
                # Patch the DAE: <init_from>foo.tif</init_from> →
                # <init_from>foo.png</init_from>
                new_ref = re.sub(r"\.tif$", ".png", ref, flags=re.IGNORECASE)
                text = text.replace(ref, new_ref)
            with open(dae_path, "w") as df:
                df.write(text)
print(f"TIFF→PNG conversion complete: {converted} new PNG(s)")
PY
print_status "TIFF textures converted to PNG (terrain meshes now render in RViz)"

print_info "Copying airframe configs to ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
if [ -d "${UAVROS2_DIR}/config/px4" ]; then
    AIRFRAMES_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"
    MANIFEST="${PX4_DIR}/.uavros2_airframes_manifest"
    # Drop airframes we installed on a prior run. This prevents stale files
    # (e.g. after a uavros2 descriptor rename) from accumulating and
    # confusing PX4's `*_gz_*` glob — which would otherwise see two files
    # for the same model name and fail with a CMake duplicate-target error.
    # We track our own files in a manifest so we never delete PX4 stock
    # files even when their numeric ID range overlaps with ours.
    if [ -f "$MANIFEST" ]; then
        print_info "Cleaning up airframes from previous uavros2 install..."
        while IFS= read -r prev; do
            [ -n "$prev" ] && rm -f "${AIRFRAMES_DIR}/${prev}"
        done < "$MANIFEST"
    fi
    cp -r ${UAVROS2_DIR}/config/px4/* "${AIRFRAMES_DIR}/" || {
        print_error "Failed to copy airframe configurations"
        exit 1
    }
    # Record what we just installed so we can clean it up next time.
    ls "${UAVROS2_DIR}/config/px4" | grep -E '^[0-9]+_gz_' > "$MANIFEST" || true
    print_status "Airframe configurations copied (manifest: $MANIFEST)"
else
    print_error "PX4 config directory not found at ${UAVROS2_DIR}/config/px4"
    exit 1
fi

# PX4 keeps an explicit airframe list in CMakeLists.txt that is used to embed
# the ROMFS tarball at build time. Copying files alone is not enough — they
# must also appear in that list. Strip any prior entries that match our
# airframe FILENAMES exactly (so stock PX4 IDs like 4010_gz_x500_mono_cam
# are preserved), then re-inject the current list after the
# "# [22000, 22999] Reserve for custom models" marker.
print_info "Registering airframes in PX4 CMakeLists.txt"
CMAKE_AIRFRAMES="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
UAV_AIRFRAMES=$(ls "${UAVROS2_DIR}/config/px4" | grep -E '^[0-9]+_gz_' | sort)
if [ -f "$CMAKE_AIRFRAMES" ] && [ -n "$UAV_AIRFRAMES" ]; then
    python3 - "$CMAKE_AIRFRAMES" <<PY
import sys, re
path = sys.argv[1]
ours = """${UAV_AIRFRAMES}""".strip().splitlines()
src = open(path).read()
# Strip ONLY the exact airframe filenames we own, plus any stray COLCON_IGNORE
# (some prior install runs left one behind). Stock PX4 entries are untouched.
for name in ours:
    src = re.sub(r'(?m)^[ \t]*' + re.escape(name) + r'[ \t]*\n', '', src)
src = re.sub(r'(?m)^[ \t]*COLCON_IGNORE[ \t]*\n', '', src)
new_block = '\n'.join(ours) + '\n'
marker = '# [22000, 22999] Reserve for custom models\n'
if marker in src:
    src = src.replace(marker, marker + new_block, 1)
else:
    # Fallback: append at end-of-list (before final closing paren)
    src = re.sub(r'\)\s*$', new_block + ')\n', src)
open(path, 'w').write(src)
print('updated', path)
PY
    print_status "Airframe list registered in PX4 CMakeLists.txt"
else
    print_warning "Could not patch $CMAKE_AIRFRAMES — PX4 build may not include new airframes"
fi
