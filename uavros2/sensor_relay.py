"""Sensor relay: republish native simulator topics under canonical names.

Used as a fallback when the per-simulator launch can't directly produce
canonical-named topics (likely for Webots' fixed device naming and possibly
some community Isaac graphs). Keep it one-hop and lightweight — no
transforms, no conversions. If you find yourself adding type conversions
here, add a dedicated bridge node instead.

Configuration is a YAML file mapping native -> canonical names:

    relays:
      - native: /webots/imu
        canonical: /drone/imu
        type: sensor_msgs/msg/Imu
      - native: /webots/camera/left/image
        canonical: /drone/front_stereo/left_cam/image_raw
        type: sensor_msgs/msg/Image
"""

import importlib
from typing import Any

import rclpy
from rclpy.node import Node


def _resolve_msg(type_str: str) -> Any:
    """Resolve 'pkg/msg/Type' to a Python class."""
    pkg, _, name = type_str.replace(".", "/").partition("/msg/")
    if not name:
        raise ValueError(f"bad msg type: {type_str!r}")
    mod = importlib.import_module(f"{pkg}.msg")
    return getattr(mod, name)


class SensorRelay(Node):
    def __init__(self):
        super().__init__("sensor_relay")
        self.declare_parameter("relays", [])  # list of dicts via YAML param file
        relays = self.get_parameter("relays").value or []

        self._pubs = {}
        for r in relays:
            native = r["native"]
            canonical = r["canonical"]
            msg_cls = _resolve_msg(r["type"])
            pub = self.create_publisher(msg_cls, canonical, 10)
            self._pubs[native] = pub
            self.create_subscription(
                msg_cls, native,
                (lambda m, p=pub: p.publish(m)), 10,
            )
            self.get_logger().info(f"relay: {native} -> {canonical}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
