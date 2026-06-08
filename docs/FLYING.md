# Flying a uavros2 drone

A `sim.launch.py` run brings up Gazebo + PX4 SITL + MAVROS + RViz, but
the drone sits idle. This doc covers the three common ways to actually
fly it.

## TL;DR — bash aliases (recommended)

`scripts/bash.sh` ships helper aliases for the common service calls.
After sourcing the workspace (done automatically by `install.sh` via
`~/.bashrc`):

```bash
# T1: zenoh & sim (RViz visible)
zenoh
ros2 launch uavros2 sim.launch.py simulator:=gazebo

# T2 (or T3): fly
state            # see connected / armed / mode
arm              # arm motors → props idle at ~50% mavros/vfr_hud throttle
takeoff          # AUTO.TAKEOFF: climb to ~2.5 m and loiter
hold             # AUTO.LOITER: hover in place
land             # AUTO.LAND: descend + disarm at touchdown
disarm           # force disarm

offboard         # OFFBOARD mode (you must be streaming setpoints)
position         # POSCTL: position-control with rc input

qgc              # spawn QGroundControl AppImage
```

All aliases honor `NS=<namespace>` for multi-drone setups:

```bash
NS=interceptor takeoff
```

## What you should see

| Action  | In Gazebo                | In RViz (drone_view.rviz)             |
|---------|--------------------------|---------------------------------------|
| `arm`   | rotor cones turn         | marker props start spinning           |
| `takeoff` | drone lifts ~2.5 m     | body markers rise, GT path traces up  |
| `hold`  | hovers in place          | markers steady; GT path stops growing |
| `land`  | drone descends           | markers settle to ground              |
| `disarm`| props stop               | marker props stop spinning            |

The propeller spin in RViz is driven by **real PX4 throttle**
(`mavros/vfr_hud.throttle`), not a fixed animation. Zero throttle =
zero spin.

## Path A — Raw MAVROS services (no aliases)

Useful for scripting:

```bash
# Arm
ros2 service call /drone/mavros/cmd/arming \
    mavros_msgs/srv/CommandBool "{value: true}"

# Takeoff
ros2 service call /drone/mavros/set_mode \
    mavros_msgs/srv/SetMode "{custom_mode: 'AUTO.TAKEOFF'}"

# Land
ros2 service call /drone/mavros/set_mode \
    mavros_msgs/srv/SetMode "{custom_mode: 'AUTO.LAND'}"
```

## Path B — QGroundControl (GUI)

`install.sh` downloads the QGC AppImage to `$DEV_DIR/`:

```bash
qgc                                    # alias
# or:
~/drone_arm_ws/QGroundControl.AppImage
```

QGC auto-discovers PX4 SITL on localhost (UDP 14550). Use the on-screen
*Arm* / *Takeoff* / *Land* buttons. Best UX for manual flight, mission
planning, and parameter tuning.

## Path C — PX4 shell (advanced)

PX4 SITL exposes a `pxh>` prompt mid-launch. With `ros2 launch`'s
output mixing in, it's awkward, but works:

```
pxh> commander arm
pxh> commander takeoff
pxh> commander land
pxh> commander mode AUTO.LOITER
pxh> param show MPC_THR_HOVER
```

## OFFBOARD mode (programmatic flight)

For waypoint missions or velocity control from your own ROS 2 node:

1. Arm the drone.
2. Start streaming setpoints to one of:
   - `/drone/mavros/setpoint_position/local` (geometry_msgs/PoseStamped)
   - `/drone/mavros/setpoint_velocity/cmd_vel` (geometry_msgs/TwistStamped)
   - `/drone/mavros/setpoint_raw/local` (mavros_msgs/PositionTarget — most flexible)
3. Setpoint stream must be running for **at least 1 s** before mode switch.
4. Switch mode to `OFFBOARD`:
   ```bash
   offboard          # alias
   ```

PX4 immediately abandons OFFBOARD if setpoints stop for >0.5 s.

Example minimal Python OFFBOARD streamer:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Hover(Node):
    def __init__(self):
        super().__init__('hover')
        self.pub = self.create_publisher(
            PoseStamped, '/drone/mavros/setpoint_position/local', 10)
        self.create_timer(0.05, self.tick)   # 20 Hz
    def tick(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.z = 5.0
        self.pub.publish(msg)

rclpy.init(); rclpy.spin(Hover())
```

## One-shot smoke test (no GUI)

Paste into a third terminal once the sim is up — flies the drone for
10 s and lands. Asserts the full chain works end-to-end.

```bash
# wait for MAVROS to connect to PX4
until ros2 topic echo /drone/mavros/state --once 2>/dev/null \
      | grep -q "connected: true"; do sleep 1; done
echo "MAVROS connected."

arm
takeoff
sleep 10
land
```

In RViz: marker prop spin → drone rises → GT path traces upward →
drone descends → props stop.

## Troubleshooting

**`armed: false` won't flip to `true`:**
- Check `state` output for the `system_status` and `mode` fields. PX4
  refuses to arm in some modes (e.g. mid-mission). Try `hold` first.
- Pre-arm health checks: `pxh> commander check` in the PX4 console
  reports which gates are blocking.

**`AUTO.TAKEOFF` returns `success: false`:**
- The drone must be armed first. The `mode_sent` field tells you
  whether PX4 accepted the request.

**Drone takes off, then drifts and crashes:**
- Likely a hover-thrust mismatch with the actual mass. Check
  `param show MPC_THR_HOVER` in the PX4 console. Our airframes ship
  with `MPC_THR_HOVER` defaults appropriate to each UAV — for x500
  it's `0.60`; for the with-arm variants `0.62`–`0.68`.

**`OFFBOARD` keeps rejecting:**
- You're not streaming setpoints, or stream rate is too low. PX4
  requires the setpoint stream to be already running BEFORE the mode
  switch, and the rate must be >2 Hz.

**`RTT too high for timesync` warnings in MAVROS log:**
- Harmless at startup. PX4 takes 10–30 s to fully synchronize. After
  that the warnings should subside.
