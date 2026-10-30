"""Tests for the partition-scoped stale-process sweep (uavros2.sim_cleanup).

Builds a fake /proc tree so the matching rules are checked without touching
real processes: another sim's gz/px4 (other partition, other PX4 tree) must
never be selected.
"""

import os

from uavros2.sim_cleanup import find_stale


MY_PX4 = "/ws/mine/PX4-Autopilot"
OTHER_PX4 = "/ws/other/PX4-Autopilot"


def _proc(root, pid, argv, env, cwd, title=False):
    d = root / str(pid)
    d.mkdir()
    if title:
        # gz's Ruby launcher rewrites its title: one space-separated string.
        (d / "cmdline").write_bytes(" ".join(argv).encode())
    else:
        (d / "cmdline").write_bytes(b"\x00".join(a.encode() for a in argv) + b"\x00")
    (d / "environ").write_bytes(
        b"\x00".join(f"{k}={v}".encode() for k, v in env.items()) + b"\x00")
    os.symlink(cwd, d / "cwd")


def _tree(tmp_path):
    root = tmp_path / "proc"
    root.mkdir()
    mine = {"GZ_PARTITION": "uavsim", "ROS_DOMAIN_ID": "24"}
    other = {"GZ_PARTITION": "ihunter", "ROS_DOMAIN_ID": "21"}
    rootfs = "/build/px4_sitl_default/rootfs/"
    # this sim, instance 4
    _proc(root, 101, ["./bin/px4", "-i", "4", "-d"], mine, MY_PX4 + rootfs + "4")
    _proc(root, 102, ["gz", "sim", "-r", "-s", MY_PX4 + "/worlds/warehouse.sdf"],
          mine, MY_PX4 + rootfs + "4", title=True)
    _proc(root, 103, ["/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge", "/clock"],
          mine, "/home/u")
    # this sim's tree and partition but another instance: not ours
    _proc(root, 104, ["./bin/px4", "-i", "5", "-d"], mine, MY_PX4 + rootfs + "5")
    # another sim on the same machine
    _proc(root, 201, ["./bin/px4", "-i", "4", "-d"], other, OTHER_PX4 + rootfs + "4")
    _proc(root, 202, ["gz", "sim", "-r", "-s", OTHER_PX4 + "/worlds/x.sdf"],
          other, OTHER_PX4 + rootfs + "4", title=True)
    _proc(root, 203, ["/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge", "/clock"],
          other, "/home/u")
    # unrelated process in our partition
    _proc(root, 301, ["python3", "node.py"], mine, "/home/u")
    return root


def test_selects_only_this_sims_processes(tmp_path):
    root = _tree(tmp_path)
    got = find_stale(MY_PX4, 4, "uavsim", "24", proc_root=str(root), self_pid=1)
    assert got == [101, 102, 103]


def test_other_partition_never_selected(tmp_path):
    root = _tree(tmp_path)
    # Even pointing at the other tree, our partition excludes its processes.
    got = find_stale(OTHER_PX4, 4, "uavsim", "24", proc_root=str(root), self_pid=1)
    assert 201 not in got and 202 not in got and 203 not in got


def test_unset_partition_requires_our_px4_tree(tmp_path):
    root = tmp_path / "proc"
    root.mkdir()
    rootfs = "/build/px4_sitl_default/rootfs/0"
    _proc(root, 11, ["./bin/px4", "-i", "0"], {}, MY_PX4 + rootfs)
    _proc(root, 12, ["gz", "sim", "-g"], {}, MY_PX4 + rootfs, title=True)
    _proc(root, 21, ["./bin/px4", "-i", "0"], {}, OTHER_PX4 + rootfs)
    _proc(root, 22, ["gz", "sim", "-s", OTHER_PX4 + "/w.sdf"], {}, "/tmp", title=True)
    # bridges are never swept without a named partition
    _proc(root, 23, ["parameter_bridge", "/clock"], {}, "/tmp")
    got = find_stale(MY_PX4, 0, "", "", proc_root=str(root), self_pid=1)
    assert got == [11, 12]


def test_self_is_skipped(tmp_path):
    root = _tree(tmp_path)
    got = find_stale(MY_PX4, 4, "uavsim", "24", proc_root=str(root), self_pid=101)
    assert 101 not in got
