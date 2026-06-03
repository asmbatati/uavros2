# Worlds

| World | Description | Gazebo | Webots | MuJoCo | Isaac | PyBullet | Genesis |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|
| `empty` | Bare physics, default | ✓ | ✓ | ✓ | — | ✓ | — |
| `tugbot_depot` | Warehouse with obstacles, AMRs | ✓ | — | — | — | — | — |

The per-simulator availability is declared in `worlds/manifest.yaml`. Asking the launch dispatcher for an unsupported world combination fails fast with a clear error message rather than silently falling back.

## Adding a new world

1. Author the world file in the native format of your primary target sim (SDF for Gazebo, WBT for Webots, XML for MuJoCo, USD for Isaac).
2. Add it to `worlds/manifest.yaml` under the right simulator key.
3. (Optional) Convert/port for other sims as needed — there is no universal world format. Most conversions require hand work.

## File format notes

- **SDF (Gazebo)**: world + physics + lights + spawned models. Heightmaps must use git-lfs.
- **WBT (Webots)**: world is a flat protobuf-like scene file; reference PROTO models.
- **XML (MuJoCo)**: world = top-level MJCF; can `<include>` model MJCFs.
- **USD (Isaac)**: scene composition; reference USDs for terrain, props, lighting.
- **None (PyBullet/Genesis)**: world is procedural — typically a flat plane + `loadURDF` calls in the adapter.
