# Worlds

| World | Description | Gazebo | Webots | MuJoCo | Isaac | PyBullet | Genesis |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|
| `empty` | Bare physics, default | ✓ | ✓ | ✓ | — | ✓ | — |
| `tugbot_depot` | Warehouse with obstacles, AMRs | ✓ | — | — | — | — | — |
| `dem_world` | Original DEM testbed (Mt. Wilder area) | ✓ | — | — | — | — | — |
| `taif_world` | TAIF DEM (Taif city, KSA; center 21.27 N, 40.35 E) | ✓ | — | — | — | — | — |
| `taif1_world` | TAIF DEM variant 1 (larger area, 120 MB heightmap) | ✓ | — | — | — | — | — |
| `taif_test` | TAIF test area (TERCOM benchmark) | ✓ | — | — | — | — | — |
| `taif_test4` | TAIF test 4 (with `tercom_dem.json` metadata) | ✓ | — | — | — | — | — |

Per-simulator availability is declared in `worlds/manifest.yaml`. Asking the launch dispatcher for an unsupported world combination fails fast with a clear error message.

## DEM worlds (TAIF / Mt. Wilder)

These are heightmap-based outdoor terrains ported from `gps_denied_navigation_sim`, useful for:
- GPS-denied navigation algorithms (TERCOM, MINS, OpenVINS, FAST-LIO, FAST-LIVO2, RTAB-Map, ORB-SLAM3)
- High-altitude flight benchmarks
- TERCOM (TERrain COntour Matching) testbeds — `taif_test4` ships a `tercom_dem.json` companion file

Heightmap binaries (`.tif`, `.dae`, `.png` textures) live in the matching `models/<name>_dem/` directory and are tracked via **git-lfs**. The total LFS payload across the 5 DEM worlds is ≈ 200 MB:

| Model | Size |
|---|---|
| `models/dem` | 13 MB |
| `models/taif_dem` | 12 MB |
| `models/taif1_dem` | 120 MB |
| `models/taif_test` | 16 MB |
| `models/taif_test4` | 41 MB |

After `git clone`, run `git lfs pull` inside the package to materialize the binaries. The `install.sh` script does this automatically.

## Launching a DEM world

Via the dispatcher (recommended):
```bash
ros2 launch uav_gz_sim sim.launch.py \
    simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=taif_test4
```

Via the `bash.sh` aliases (mirrors the gps_denied pattern):
```
mono_dem    mono_taif    mono_taif1    mono_taif4
stereo_dem  stereo_taif  stereo_taif1  stereo_taif4
            twin_taif                  twin_taif4
```

Each alias is `simulator:=gazebo uav:=<camera_rig> world:=<terrain>`.

Note that TAIF worlds spawn the UAV at high altitude (PX4 airframe pose offset ≈ 2000 m AMSL to match the DEM's reference elevation). The `world:=taif_test4` runs are the most validated.

## File format notes

- **SDF (Gazebo)**: world + physics + lights + spawned models. Heightmaps must use `git-lfs`.
- **WBT (Webots)**: world is a flat protobuf-like scene file; reference PROTO models.
- **XML (MuJoCo)**: world = top-level MJCF; can `<include>` model MJCFs.
- **USD (Isaac)**: scene composition; reference USDs for terrain, props, lighting.
- **None (PyBullet/Genesis)**: world is procedural — typically a flat plane + `loadURDF` calls in the adapter.

## Adding a new world

1. Author the world file in the native format of your primary target sim (SDF for Gazebo, WBT for Webots, XML for MuJoCo, USD for Isaac).
2. Add it to `worlds/manifest.yaml` under the right simulator key.
3. (Optional) Convert/port for other sims as needed — there is no universal world format. Most conversions require hand work.
