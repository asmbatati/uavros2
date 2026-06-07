# Worlds

| World | Description | Gazebo | Webots | MuJoCo | Isaac | PyBullet | Genesis |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|
| `empty` | Bare physics, default | ✓ | ✓ | ✓ | — | ✓ | — |
| `warehouse` | Indoor warehouse with obstacles + AMRs (formerly `tugbot_depot`) | ✓ | — | — | — | — | — |
| `urban1` | Outdoor heightmap testbed (formerly `dem_world`, Mt. Wilder area) | ✓ | — | — | — | — | — |
| `urban2` | Urban heightmap 2 (formerly `taif_world`) | ✓ | — | — | — | — | — |
| `urban3` | Urban heightmap 3 (formerly `taif1_world`, larger area) | ✓ | — | — | — | — | — |
| `urban4` | Urban heightmap 4 (formerly `taif_test`, terrain-correlation testbed) | ✓ | — | — | — | — | — |
| `urban5` | Urban heightmap 5 (formerly `taif_test4`, ships `tercom_dem.json`) | ✓ | — | — | — | — | — |

Per-simulator availability is declared in `worlds/manifest.yaml`. Asking the launch dispatcher for an unsupported world combination fails fast with a clear error message.

## Urban / outdoor heightmap worlds

These are heightmap-based outdoor terrains, useful for:
- High-altitude flight benchmarks
- GPS-denied navigation algorithms (TERCOM, MINS, OpenVINS, FAST-LIO, …)
- Terrain-correlation testbeds — `urban5` ships a `tercom_dem.json` companion file in its terrain model

Heightmap binaries (`.tif`, `.dae`, `.png` textures) live in the matching `models/urban<N>_terrain/` directory and are tracked via **git-lfs**. Total payload ≈ 200 MB:

| Terrain model | Size |
|---|---|
| `models/urban1_terrain` | 13 MB |
| `models/urban2_terrain` | 12 MB |
| `models/urban3_terrain` | 120 MB |
| `models/urban4_terrain` | 16 MB |
| `models/urban5_terrain` | 41 MB |

After `git clone`, run `git lfs pull` inside the package to materialize the binaries. `install.sh` does this automatically.

## Launching

Via the dispatcher (recommended):
```bash
ros2 launch uav_gz_sim sim.launch.py \
    simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=urban5
```

Via `bash.sh` aliases (camera-rig × world matrix):
```
mono_urban1   mono_urban2   mono_urban3   mono_urban4   mono_urban5
stereo_urban1 stereo_urban2 stereo_urban3 stereo_urban4 stereo_urban5
              twin_urban2                               twin_urban5
warehouse                                  # x500_stereo_cam_3d_lidar in the warehouse
```

The urban worlds spawn the UAV at high altitude (PX4 airframe pose offset ≈ 2000 m AMSL to match the heightmap's reference elevation).

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
