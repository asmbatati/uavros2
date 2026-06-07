"""Unified sim_control_bridge node.

Loads the per-simulator adapter selected by the ``simulator`` parameter.
For inert backends (gazebo, webots, isaac) this node is essentially a
no-op — MAVROS or the native sim bridge owns state publication.
For active backends (mujoco, pybullet, genesis) this node steps the
physics engine from setpoints and synthesizes MAVROS-shaped state.

Public ROS interface (the canonical topic contract):

Subscribes (under the node's namespace):
  setpoint_position/pose         geometry_msgs/PoseStamped
  setpoint_velocity/cmd_vel      geometry_msgs/TwistStamped
  setpoint_raw/attitude          mavros_msgs/AttitudeTarget  (best-effort)
  actuators                      std_msgs/Float32MultiArray
  joint_command                  trajectory_msgs/JointTrajectory

Publishes (under the node's namespace):
  mavros/local_position/pose     geometry_msgs/PoseStamped
  mavros/local_position/odom     nav_msgs/Odometry
  imu                            sensor_msgs/Imu
  joint_states                   sensor_msgs/JointState
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SimControlBridge(Node):
    def __init__(self):
        super().__init__("sim_control_bridge")

        self.declare_parameter("simulator", "gazebo")
        self.declare_parameter("uav", "x500")
        self.declare_parameter("arm", "none")
        self.declare_parameter("world", "empty")
        self.declare_parameter("step_rate_hz", 200.0)

        self._simulator = self.get_parameter("simulator").value
        self._uav = self.get_parameter("uav").value
        self._arm = self.get_parameter("arm").value
        self._world = self.get_parameter("world").value
        self._step_rate = float(self.get_parameter("step_rate_hz").value)

        from .adapters import load as load_adapter
        self._adapter = load_adapter(self._simulator)
        self._adapter.spawn(self._uav, self._arm, self._world)

        # Setpoint state (latest received)
        self._setpoint = {
            "position": None,
            "velocity": None,
            "actuators": None,
            "joint_command": None,
        }

        self._sub_pos = self.create_subscription(
            PoseStamped, "setpoint_position/pose",
            lambda m: self._setpoint.__setitem__("position", m), SENSOR_QOS,
        )
        self._sub_vel = self.create_subscription(
            TwistStamped, "setpoint_velocity/cmd_vel",
            lambda m: self._setpoint.__setitem__("velocity", m), SENSOR_QOS,
        )
        self._sub_act = self.create_subscription(
            Float32MultiArray, "actuators",
            lambda m: self._setpoint.__setitem__("actuators", list(m.data)),
            SENSOR_QOS,
        )

        if not self._adapter.is_inert:
            self._pub_pose = self.create_publisher(PoseStamped, "mavros/local_position/pose", 10)
            self._pub_odom = self.create_publisher(Odometry, "mavros/local_position/odom", 10)
            self._pub_imu = self.create_publisher(Imu, "imu", SENSOR_QOS)
            self._pub_joints = self.create_publisher(JointState, "joint_states", 10)

            period = 1.0 / max(1.0, self._step_rate)
            self._last_step = time.monotonic()
            self._timer = self.create_timer(period, self._on_step)

            self.get_logger().info(
                f"sim_control_bridge ACTIVE: simulator={self._simulator} "
                f"uav={self._uav} arm={self._arm}"
            )
        else:
            self.get_logger().info(
                f"sim_control_bridge INERT: simulator={self._simulator} "
                "(state owned by MAVROS / native sim bridge)"
            )

    # --- Active mode: step physics, publish state ---

    def _on_step(self) -> None:
        now = time.monotonic()
        dt = now - self._last_step
        self._last_step = now

        out = self._adapter.step(dt, self._setpoint)
        if not out:
            return

        stamp = self.get_clock().now().to_msg()

        if "pose" in out:
            x, y, z, qx, qy, qz, qw = out["pose"]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            self._pub_pose.publish(pose_msg)

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose = pose_msg.pose
            if "vel" in out:
                odom.twist.twist.linear.x = out["vel"][0]
                odom.twist.twist.linear.y = out["vel"][1]
                odom.twist.twist.linear.z = out["vel"][2]
            self._pub_odom.publish(odom)

        if "imu" in out:
            ax, ay, az, gx, gy, gz = out["imu"]
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "base_link"
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            self._pub_imu.publish(imu)

        if "joint_states" in out:
            js = JointState()
            js.header.stamp = stamp
            for name, (pos, vel, eff) in out["joint_states"].items():
                js.name.append(name)
                js.position.append(pos)
                js.velocity.append(vel)
                js.effort.append(eff)
            self._pub_joints.publish(js)

    def destroy_node(self):
        try:
            self._adapter.shutdown()
        except Exception as exc:
            self.get_logger().warn(f"adapter shutdown failed: {exc}")
        super().destroy_node()


# Re-export pure helpers for backwards compatibility / convenience.
from .control_math import quad_attitude_pid, rotor_mixer  # noqa: F401,E402


def main(args=None):
    rclpy.init(args=args)
    node = SimControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
