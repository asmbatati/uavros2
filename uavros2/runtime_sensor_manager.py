"""runtime_sensor_manager — hot-swap sensors into a running Gazebo sim.

Exposes three ROS 2 services under the UAV namespace:

  /<ns>/runtime/add_sensor      uavros2_msgs/srv/AddSensor
  /<ns>/runtime/remove_sensor   uavros2_msgs/srv/RemoveSensor
  /<ns>/runtime/list_sensors    uavros2_msgs/srv/ListSensors

Add / remove are wrappers around Gazebo Sim's transport-level services:

  /world/<world>/create   ← spawn a new model from SDF
  /world/<world>/remove   ← remove a model by name

The wrapper authors a small SDF that ``<include>``s the same sensor
model dir we'd reference at build time (per the catalog), so the
spawned sensor is functionally identical to one declared in the UAV
descriptor. The new sensor appears under the canonical topic name
through the same ros_gz_bridge that other sensors use - the bridge
re-discovers Gazebo topics on a periodic timer.

Limitations (v1)
----------------
- Gazebo Sim only. MuJoCo / Webots / Isaac return "not supported".
- The new sensor is attached to the UAV's ``base_link`` via Gazebo's
  spawn pose (relative_to behavior depends on Gazebo build).
- No automatic re-wiring of ros_gz_bridge YAML - the bridge picks up
  new topics by re-running the discovery pass, OR launch a new
  parameter_bridge for the added sensor (left to the user for v1).
"""

from __future__ import annotations

import json
import shutil
import subprocess
import textwrap
from pathlib import Path
from typing import Dict, List, Optional

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from uavros2_msgs.srv import AddSensor, ListSensors, RemoveSensor

from uavros2.asset_spec import Catalog


def _have_gz_cli() -> bool:
    return shutil.which("gz") is not None


class RuntimeSensorManager(Node):
    """ROS 2 node that adds / removes / lists sensors at runtime."""

    def __init__(self):
        super().__init__("runtime_sensor_manager")

        self.declare_parameter("namespace", "drone")
        self.declare_parameter("world", "warehouse")
        self.declare_parameter("uav_model_name", "x500_stereo_cam_3d_lidar")
        # Allow overriding the catalog root; default to the installed share path.
        self.declare_parameter("catalog_root", "")

        ns = self.get_parameter("namespace").value
        self._world = self.get_parameter("world").value
        self._uav_model_name = self.get_parameter("uav_model_name").value

        catalog_root = self.get_parameter("catalog_root").value or \
            str(Path(get_package_share_directory("uavros2")) / "assets")
        self._catalog = Catalog(catalog_root)
        self.get_logger().info(
            f"runtime_sensor_manager up — ns={ns} world={self._world} "
            f"uav={self._uav_model_name} catalog={catalog_root} "
            f"({len(self._catalog.sensors)} sensor types known)"
        )

        # Tracks instance_name -> sensor_ref so list/remove know what's live.
        self._live: Dict[str, str] = {}

        self.create_service(AddSensor, f"/{ns}/runtime/add_sensor", self._cb_add)
        self.create_service(RemoveSensor, f"/{ns}/runtime/remove_sensor", self._cb_remove)
        self.create_service(ListSensors, f"/{ns}/runtime/list_sensors", self._cb_list)

    # ----- service callbacks ------------------------------------------------

    def _cb_add(self, req: AddSensor.Request, resp: AddSensor.Response):
        if not _have_gz_cli():
            resp.success = False
            resp.message = "Gazebo `gz` CLI not on PATH — cannot spawn"
            return resp
        if req.sensor_ref not in self._catalog.sensors:
            resp.success = False
            resp.message = (
                f"Unknown sensor ref {req.sensor_ref!r}. Known: "
                f"{sorted(self._catalog.sensors)}"
            )
            return resp
        if req.name in self._live:
            resp.success = False
            resp.message = f"Sensor named {req.name!r} is already live"
            return resp

        sd = self._catalog.sensors[req.sensor_ref]
        if not sd.gazebo.gz_model:
            resp.success = False
            resp.message = (
                f"sensor_ref={req.sensor_ref!r} is an embedded sensor "
                "(no separate model dir) — nothing to spawn"
            )
            return resp

        # Build a minimal SDF that includes the sensor model + sets its pose.
        # The spawned entity is parented to the world; if a fixed joint to
        # the UAV body is required, that's a follow-up (would need to author
        # a wrapper model SDF and use `gz model --rejoin`).
        x, y, z, r, p, yw = list(req.pose) + [0.0] * (6 - len(req.pose))
        spawn_name = f"{req.uav_namespace}_{req.name}"
        sdf = textwrap.dedent(f"""\
            <?xml version="1.0" encoding="UTF-8"?>
            <sdf version='1.9'>
              <model name='{spawn_name}'>
                <include>
                  <name>{req.name}</name>
                  <uri>model://{sd.gazebo.gz_model}</uri>
                  <pose>{x:g} {y:g} {z:g} {r:g} {p:g} {yw:g}</pose>
                </include>
              </model>
            </sdf>
            """)

        ok, msg = self._gz_spawn(sdf)
        if ok:
            self._live[req.name] = req.sensor_ref
            resp.message = f"spawned {spawn_name!r}"
        else:
            resp.message = msg
        resp.success = ok
        self.get_logger().info(
            f"add_sensor {req.name!r}={req.sensor_ref!r} → {resp.success} ({resp.message})"
        )
        return resp

    def _cb_remove(self, req: RemoveSensor.Request, resp: RemoveSensor.Response):
        if not _have_gz_cli():
            resp.success = False
            resp.message = "Gazebo `gz` CLI not on PATH"
            return resp
        if req.name not in self._live:
            resp.success = False
            resp.message = f"No sensor named {req.name!r} is live"
            return resp
        spawn_name = f"{req.uav_namespace}_{req.name}"
        ok, msg = self._gz_remove(spawn_name)
        if ok:
            self._live.pop(req.name, None)
        resp.success = ok
        resp.message = msg if not ok else f"removed {spawn_name!r}"
        self.get_logger().info(
            f"remove_sensor {req.name!r} → {resp.success} ({resp.message})"
        )
        return resp

    def _cb_list(self, req: ListSensors.Request, resp: ListSensors.Response):
        resp.success = True
        resp.message = f"{len(self._live)} active"
        resp.sensor_names = list(self._live.keys())
        resp.sensor_refs = list(self._live.values())
        return resp

    # ----- Gazebo CLI wrappers ----------------------------------------------

    def _gz_spawn(self, sdf_text: str) -> tuple[bool, str]:
        """Call /world/<world>/create with an SDF string payload."""
        # gz expects a single-quoted protobuf message; embed the SDF as the
        # `sdf:` field.
        req_msg = f'sdf: "{sdf_text.replace(chr(34), chr(92)+chr(34))}"'
        cmd = [
            "gz", "service", "-s", f"/world/{self._world}/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", req_msg,
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        except subprocess.TimeoutExpired as exc:
            return False, f"gz service timeout: {exc}"
        if proc.returncode != 0:
            return False, f"gz service failed: {proc.stderr.strip() or proc.stdout.strip()}"
        # gz prints e.g. `data: true` on success
        if "data: true" in proc.stdout:
            return True, proc.stdout.strip()
        return False, f"gz responded but spawn failed: {proc.stdout.strip()}"

    def _gz_remove(self, entity_name: str) -> tuple[bool, str]:
        req_msg = f'name: "{entity_name}" type: MODEL'
        cmd = [
            "gz", "service", "-s", f"/world/{self._world}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", req_msg,
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        except subprocess.TimeoutExpired as exc:
            return False, f"gz service timeout: {exc}"
        if proc.returncode != 0:
            return False, f"gz service failed: {proc.stderr.strip() or proc.stdout.strip()}"
        if "data: true" in proc.stdout:
            return True, proc.stdout.strip()
        return False, f"gz responded but remove failed: {proc.stdout.strip()}"


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeSensorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
