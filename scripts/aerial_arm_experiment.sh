#!/usr/bin/env bash
# Run one aerial-arm flight experiment end to end, headless:
#   launch PX4 SITL + Gazebo + MAVROS + arm controllers, wait until the arm
#   controller is up, fly `aerial_arm_demo` (takeoff, hover, arm poses, land),
#   then stop the sim.
#
# Usage: aerial_arm_experiment.sh <arm> [instance] [output_dir] [extra demo params...]
#   arm       three_dof | openmanip_x | panda | ur5 | none (plain x500 baseline)
#   instance  PX4 SITL instance (default 0); MAVLink ports shift with it
#   output_dir  results directory (default ~/uavros2_results)
#
# Env: UAV overrides the model, WORLD the world (default warehouse),
# NAMESPACE the ROS namespace (default drone), DEMO the mission node
# (default aerial_arm_demo; wall_contact_demo with UAV=x500_with_openmanip_x_tool
# WORLD=wall).
#
# Source your workspace first. On a shared machine also export your own
# ROS_DOMAIN_ID and GZ_PARTITION so the run is isolated from other sims.
set -euo pipefail
# Job control: without it bash starts background jobs with SIGINT ignored,
# so the launch would never see the SIGINT that stops it cleanly.
set -m

arm=${1:?usage: $0 <arm> [instance] [output_dir]}
instance=${2:-0}
out=${3:-$HOME/uavros2_results}
shift $(( $# < 3 ? $# : 3 ))
case $arm in
  three_dof) uav=x500_with_three_dof_arm ;;
  openmanip_x|panda|ur5) uav=x500_with_$arm ;;
  none) uav=x500 ;;
  *) echo "unknown arm: $arm" >&2; exit 2 ;;
esac
uav=${UAV:-$uav}   # e.g. UAV=x500_d435 for a no-arm control with another airframe
ns=${NAMESPACE:-drone}
mkdir -p "$out"
log="$out/${arm}_sim.log"

launch_pid=
start_sim() {
  ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:="$uav" arm:="$arm" \
    world:="${WORLD:-warehouse}" namespace:="$ns" instance:="$instance" \
    headless:=1 use_rviz:=false > "$log" 2>&1 &
  launch_pid=$!
}
stop_sim() {
  [ -n "$launch_pid" ] || return 0
  kill -INT "$launch_pid" 2>/dev/null || true
  for _ in $(seq 30); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  kill -KILL "$launch_pid" 2>/dev/null || true
  launch_pid=
  # Anything that still survived (this sim's partition / PX4 instance only).
  python3 -c "from uavros2.px4_paths import find_px4_dir
from uavros2.sim_cleanup import kill_stale
k = kill_stale(find_px4_dir(), $instance)
print('[experiment] killed leftovers:', k) if k else None" || true
}
trap stop_sim EXIT

# Ready = the arm controller is active (if any) and MAVROS hears PX4. PX4
# occasionally gives up at startup ("gz_bridge: timed out waiting for clock
# message", about 1 launch in 15 here); then restart the sim once.
sim_ready() {
  local need=('CON: Got HEARTBEAT')
  [ "$arm" = none ] || need+=('Configured and activated joint_trajectory_controller')
  for _ in $(seq 60); do
    grep -q 'px4 -i .*process has died' "$log" && return 1
    local ok=1
    for pat in "${need[@]}"; do grep -q "$pat" "$log" || ok=0; done
    [ $ok = 1 ] && return 0
    sleep 2
  done
  return 1
}

echo "[experiment] $arm on instance $instance; sim log: $log"
for attempt in 1 2; do
  start_sim
  sim_ready && break
  echo "[experiment] sim did not come up (attempt $attempt); see $log" >&2
  stop_sim
  [ $attempt = 2 ] && exit 1
  mv "$log" "${log%.log}_failed$attempt.log"
done
# PX4 needs a few seconds after the controller spawns for EKF2 to align.
sleep 10
ros2 run uavros2 "${DEMO:-aerial_arm_demo}" --ros-args -p arm:="$arm" -p namespace:="$ns" \
  -p output_dir:="$out" "$@" 2>&1 | tee "$out/${arm}_demo.log"
