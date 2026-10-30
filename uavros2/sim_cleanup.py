"""Find and kill stale Gazebo / PX4 SITL processes left by an earlier launch.

Several simulations can share one machine, each in its own ``GZ_PARTITION``
with its own PX4 instance number. A crashed run of *this* sim leaves an
orphan ``gz sim`` server or ``px4`` that blocks the next launch (PX4 sees
"gazebo already running" and refuses to spawn), so ``gazebo.launch.py``
sweeps them first. The sweep must never touch another sim, so a process is
only stale when all of these hold:

* its environment carries the same ``GZ_PARTITION`` as ours (both unset
  counts as equal), and
* it is one of ours:
    - ``px4``: runs from our PX4 tree (cwd) with our ``-i <instance>``;
    - ``gz sim``: its command line or cwd points into our PX4 tree, or the
      partition is set and non-empty (a server in our named partition is
      ours whatever tree started it);
    - ``parameter_bridge``: the partition is set and non-empty and the
      bridge runs in our ``ROS_DOMAIN_ID``.
"""

import os
import signal


def _read_cmdline(proc_root, pid):
    try:
        with open(os.path.join(proc_root, pid, "cmdline"), "rb") as f:
            raw = f.read()
    except OSError:
        return []
    argv = [a.decode("utf-8", "replace") for a in raw.split(b"\x00") if a]
    # The gz CLI is a Ruby script that rewrites its process title, so the
    # whole command line arrives as one space-separated string.
    if len(argv) == 1 and " " in argv[0]:
        argv = argv[0].split()
    return argv


def _read_environ(proc_root, pid):
    try:
        with open(os.path.join(proc_root, pid, "environ"), "rb") as f:
            raw = f.read()
    except OSError:
        return None
    env = {}
    for item in raw.split(b"\x00"):
        key, sep, value = item.partition(b"=")
        if sep:
            env[key.decode("utf-8", "replace")] = value.decode("utf-8", "replace")
    return env


def _read_cwd(proc_root, pid):
    try:
        return os.readlink(os.path.join(proc_root, pid, "cwd"))
    except OSError:
        return ""


def _is_px4(argv):
    return bool(argv) and os.path.basename(argv[0]) == "px4"


def _px4_instance(argv):
    for flag, value in zip(argv, argv[1:]):
        if flag == "-i":
            return value
    return "0"


def _is_gz_sim(argv):
    return len(argv) >= 2 and os.path.basename(argv[0]) == "gz" and argv[1] == "sim"


def _is_bridge(argv):
    return bool(argv) and os.path.basename(argv[0]) == "parameter_bridge"


def _inside(path, root):
    return bool(path) and (path == root or path.startswith(root.rstrip("/") + "/"))


def find_stale(px4_dir, instance, partition=None, domain_id=None,
               proc_root="/proc", self_pid=None):
    """Return the pids of stale processes that belong to this sim.

    ``partition`` and ``domain_id`` default to this process's
    ``GZ_PARTITION`` / ``ROS_DOMAIN_ID``.
    """
    if partition is None:
        partition = os.environ.get("GZ_PARTITION", "")
    if domain_id is None:
        domain_id = os.environ.get("ROS_DOMAIN_ID", "")
    if self_pid is None:
        self_pid = os.getpid()
    px4_dir = os.path.realpath(px4_dir)
    instance = str(instance)

    stale = []
    for pid in os.listdir(proc_root):
        if not pid.isdigit() or int(pid) == self_pid:
            continue
        argv = _read_cmdline(proc_root, pid)
        if not (_is_px4(argv) or _is_gz_sim(argv) or _is_bridge(argv)):
            continue
        env = _read_environ(proc_root, pid)
        if env is None or env.get("GZ_PARTITION", "") != partition:
            continue
        cwd = _read_cwd(proc_root, pid)
        if _is_px4(argv):
            mine = _inside(cwd, px4_dir) and _px4_instance(argv) == instance
        elif _is_gz_sim(argv):
            mine = (bool(partition) or _inside(cwd, px4_dir)
                    or any(px4_dir in a for a in argv))
        else:
            mine = bool(partition) and env.get("ROS_DOMAIN_ID", "") == domain_id
        if mine:
            stale.append(int(pid))
    return sorted(stale)


def kill_stale(px4_dir, instance, partition=None, domain_id=None):
    """SIGKILL this sim's stale processes; return the pids that were killed."""
    killed = []
    for pid in find_stale(px4_dir, instance, partition, domain_id):
        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(pid)
        except OSError:
            pass
    return killed
