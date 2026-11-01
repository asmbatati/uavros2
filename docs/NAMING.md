# Naming

**Decided, Nov 2026: the name stays `uavros2`.** The candidates below are kept for the record; there is no rename planned.

`uavros2` is the working name (Python package and repo). These are the brand
candidates collected so far; renaming is an open decision for the owner.

The brand should make the ROS 2 dependency immediately obvious — these
candidates all carry "ros2" / "r2" in the name itself:

| Candidate | Vibe | Notes |
|---|---|---|
| **`r2flight`** ⭐ *(recommended)* | Playful + clear | `r2` instantly reads as ROS 2 (also a Star Wars wink). Short, single-word-feeling, easy as a Python package and GitHub repo. |
| **`uavros2`** | Portmanteau | Very crisp; the existing `mavros` precedent makes the `*ros2` suffix legible at a glance |
| **`ros2_aerolab`** | Descriptive lab | "ROS 2 aerial lab" — explicit about both pieces |
| **`r2sky`** | Short + lyrical | Alliterative, slightly more brand-y than r2flight |
| **`skybench_ros2`** | Benchmark / academic | Strong fit for the testbed framing if you want to lead with the function |
| **`flock_ros2`** | Swarm | Future-proofs a multi-UAV roadmap |
| **`aero_ros2`** | Maximally generic | If you'd rather the brand stay non-cute |

Avoid: `rosflight` (an existing autopilot project), `roscopter` (existing
Python package).

When you pick one, the rename is:
1. Repo: `gh repo rename <new>` (or via the web UI)
2. Python package: `setup.py.name`, `package.xml.name`, `uavros2/` dir
3. Every `launch_arguments` / `FindPackageShare("uavros2")` reference

A single search-and-replace pass nails most of it.
