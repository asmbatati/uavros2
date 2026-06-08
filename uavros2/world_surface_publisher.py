"""Publish the active world's terrain in RViz.

uavros2 worlds are loaded by Gazebo, which renders the terrain natively
(COLLADA meshes for urban1/2/3, heightmap .tif for urban4/5, etc.) — but
RViz can't see Gazebo's scene, so without this node the RViz view shows
only the drone markers floating in empty space.

This node fills that gap by republishing the terrain in a way RViz can
render. Two modes (dispatched on the ``mode:`` parameter):

  mode == 'mesh'
      Publishes a single ``visualization_msgs/Marker`` of type
      ``MESH_RESOURCE`` on ``/world/surface_mesh`` (transient_local QoS).
      The marker's ``mesh_resource`` is a ``file://`` URI to the COLLADA
      file; RViz fetches and renders it like any other mesh marker.

  mode == 'heightmap'
      Publishes a coloured ``sensor_msgs/PointCloud2`` on
      ``/world/surface_cloud`` (transient_local QoS). The cloud is built
      by sampling the heightmap .tif on a decimated grid, scaling each
      point to the configured world ``size``, and (optionally) sampling
      an aerial PNG for per-point RGB. Falls back to a jet elevation
      colormap when no texture is provided.

Adapted from the DEM-rendering function in
gps_denied_navigation_sim / tercom_nav's diagnostics_node, but stripped
of any TERCOM / ESKF / GeoTIFF-CRS dependencies. PIL is the only
non-stdlib requirement.
"""

from __future__ import annotations

import array as arr
import os
import struct
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker


def _file_uri(path: str) -> str:
    """Turn a model:// or absolute path into file:// for RViz."""
    if path.startswith(("file://", "package://")):
        return path
    if path.startswith("model://"):
        # Try common roots in priority order: PX4 tree, then uavros2 share.
        rest = path[len("model://"):]
        dev_dir = os.environ.get("DEV_DIR", os.path.expanduser("~/drone_arm_ws"))
        candidates = [
            os.environ.get("PX4_DIR", ""),
            os.path.join(dev_dir, "PX4-Autopilot/Tools/simulation/gz/models"),
        ]
        for base in candidates:
            if not base:
                continue
            if "models" not in base.split(os.sep)[-2:]:
                base = os.path.join(base, "Tools/simulation/gz/models")
            full = os.path.join(base, rest)
            if os.path.isfile(full) or os.path.isdir(os.path.dirname(full)):
                return "file://" + full
        return "file://" + os.path.join(dev_dir, "PX4-Autopilot/Tools/simulation/gz/models", rest)
    if path.startswith("/"):
        return "file://" + path
    return "file://" + os.path.abspath(path)


class WorldSurfacePublisher(Node):
    """Publish a mesh marker or a heightmap point cloud for the active world."""

    def __init__(self):
        super().__init__("world_surface_publisher")

        # Common params
        self.declare_parameter("mode", "")              # 'mesh' | 'heightmap' | ''
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # mesh-mode params
        self.declare_parameter("mesh", "")              # model:// or file:// or abs path
        self.declare_parameter("mesh_scale", [1.0, 1.0, 1.0])
        self.declare_parameter("mesh_color", [0.6, 0.6, 0.6, 1.0])

        # heightmap-mode params
        self.declare_parameter("heightmap", "")         # path to .tif
        self.declare_parameter("texture", "")           # optional aerial PNG
        self.declare_parameter("size", [100.0, 100.0, 10.0])  # x, y, z in metres
        self.declare_parameter("decimation", 4)         # downsample factor

        mode = self.get_parameter("mode").value
        if not mode:
            self.get_logger().info(
                "mode parameter is empty — world has no surface viz configured. "
                "This is normal for worlds like 'empty' / 'warehouse'."
            )
            return

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        if mode == "mesh":
            self._pub = self.create_publisher(Marker, "/world/surface_mesh", latched)
            # Re-publish on a timer so RViz late-subscribers still see it.
            self.create_timer(2.0, self._publish_mesh_once)
            self._published_mesh = False
        elif mode == "heightmap":
            self._pub = self.create_publisher(
                PointCloud2, "/world/surface_cloud", latched)
            self.create_timer(2.0, self._publish_heightmap_once)
            self._published_cloud = False
        else:
            self.get_logger().error(
                f"unknown mode {mode!r} (expected 'mesh' or 'heightmap')")

    # ---- mesh mode ---------------------------------------------------------

    def _publish_mesh_once(self):
        if self._published_mesh:
            return
        mesh = self.get_parameter("mesh").value
        if not mesh:
            self.get_logger().warn("mesh path is empty; nothing to publish")
            self._published_mesh = True
            return
        uri = _file_uri(mesh)
        pose = list(self.get_parameter("pose").value)
        scale = list(self.get_parameter("mesh_scale").value)
        color = list(self.get_parameter("mesh_color").value)
        frame_id = self.get_parameter("frame_id").value

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "world_surface"
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = uri
        m.mesh_use_embedded_materials = True
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pose[:3]
        # Default identity quat (mesh in upright pose).
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale[:3]
        m.color.r, m.color.g, m.color.b, m.color.a = color[:4]
        self._pub.publish(m)
        self.get_logger().info(f"published world mesh: {uri}")
        self._published_mesh = True

    # ---- heightmap mode ----------------------------------------------------

    def _publish_heightmap_once(self):
        if self._published_cloud:
            return
        hm = self.get_parameter("heightmap").value
        if not hm:
            self.get_logger().warn("heightmap path is empty; nothing to publish")
            self._published_cloud = True
            return
        # Resolve to a local file path.
        hm_path = hm
        if hm.startswith("model://"):
            hm_path = _file_uri(hm)[len("file://"):]
        elif hm.startswith("file://"):
            hm_path = hm[len("file://"):]
        if not os.path.isfile(hm_path):
            self.get_logger().error(f"heightmap not found: {hm_path}")
            self._published_cloud = True
            return

        try:
            from PIL import Image
        except ImportError:
            self.get_logger().error(
                "Pillow not installed; cannot read heightmap. "
                "Install with: pip install Pillow")
            self._published_cloud = True
            return

        size = list(self.get_parameter("size").value)
        pose = list(self.get_parameter("pose").value)
        decimation = max(1, int(self.get_parameter("decimation").value))
        frame_id = self.get_parameter("frame_id").value

        sx, sy, sz = (float(s) for s in size[:3])
        px, py, pz = (float(p) for p in pose[:3])

        # Read heightmap (single-channel TIFF). Normalise to [0..1].
        img = Image.open(hm_path)
        arr_h = np.asarray(img)
        if arr_h.ndim == 3:
            arr_h = arr_h[..., 0]
        arr_h = arr_h.astype(np.float32)
        hmin, hmax = float(arr_h.min()), float(arr_h.max())
        hrange = max(hmax - hmin, 1.0)
        norm = (arr_h - hmin) / hrange   # [0..1]
        h, w = norm.shape

        # Decimate
        rows = np.arange(0, h, decimation, dtype=np.int32)
        cols = np.arange(0, w, decimation, dtype=np.int32)
        c_grid, r_grid = np.meshgrid(cols, rows)

        # Map pixel (row, col) → world (x, y, z) with the heightmap pose+size.
        # Gazebo convention: heightmap centred at <pos>, spanning <size> X,Y.
        # Row 0 is top of the image (positive Y in Gazebo's convention).
        u = c_grid.astype(np.float32) / max(w - 1, 1)         # 0..1 across X
        v = r_grid.astype(np.float32) / max(h - 1, 1)         # 0..1 across Y
        xs = (px + (u - 0.5) * sx).astype(np.float32)
        ys = (py + (0.5 - v) * sy).astype(np.float32)         # flip: row 0 = +Y
        zs = (pz + norm[r_grid, c_grid] * sz).astype(np.float32)

        # Colour from texture if present, else jet colormap.
        rgb_packed: np.ndarray
        tex = self.get_parameter("texture").value
        if tex:
            tex_path = tex
            if tex.startswith("model://"):
                tex_path = _file_uri(tex)[len("file://"):]
            elif tex.startswith("file://"):
                tex_path = tex[len("file://"):]
            try:
                tex_img = Image.open(tex_path).convert("RGB")
                tex_arr = np.asarray(tex_img, dtype=np.uint8)
                th, tw = tex_arr.shape[:2]
                # Sample at the SAME (u, v) used for xyz. Texture row 0 = +Y.
                tex_cols = np.clip((u * (tw - 1)).astype(np.int32), 0, tw - 1)
                tex_rows = np.clip((v * (th - 1)).astype(np.int32), 0, th - 1)
                rgb_u8 = tex_arr[tex_rows, tex_cols]                # (R, C, 3)
                rgb_packed = (
                    (rgb_u8[..., 0].astype(np.uint32) << 16)
                    | (rgb_u8[..., 1].astype(np.uint32) << 8)
                    | rgb_u8[..., 2].astype(np.uint32)
                )
                self.get_logger().info(f"DEM coloured with texture: {tex_path}")
            except Exception as exc:
                self.get_logger().warn(
                    f"texture load failed ({exc}); falling back to elevation colormap")
                rgb_packed = self._jet_colormap(norm[r_grid, c_grid])
        else:
            rgb_packed = self._jet_colormap(norm[r_grid, c_grid])

        # Flatten and assemble PointCloud2 (xyz + packed-RGB float).
        xs = xs.ravel()
        ys = ys.ravel()
        zs = zs.ravel()
        rgb_packed = rgb_packed.ravel()
        rgb_float = rgb_packed.view(np.float32)
        cloud = np.column_stack([xs, ys, zs, rgb_float]).astype(np.float32)
        n_points = cloud.shape[0]

        pc2 = PointCloud2()
        pc2.header.frame_id = frame_id
        pc2.header.stamp = self.get_clock().now().to_msg()
        pc2.height = 1
        pc2.width = n_points
        pc2.is_dense = True
        pc2.is_bigendian = False
        pc2.point_step = 16
        pc2.row_step = 16 * n_points
        pc2.fields = []
        for fname, foffset, ftype in [
            ("x",   0,  PointField.FLOAT32),
            ("y",   4,  PointField.FLOAT32),
            ("z",   8,  PointField.FLOAT32),
            ("rgb", 12, PointField.FLOAT32),
        ]:
            f = PointField()
            f.name = fname
            f.offset = foffset
            f.datatype = ftype
            f.count = 1
            pc2.fields.append(f)
        pc2.data = cloud.tobytes()
        self._pub.publish(pc2)
        self.get_logger().info(
            f"published DEM pointcloud: {n_points} pts "
            f"(decimation={decimation}, size={size}, frame={frame_id})")
        self._published_cloud = True

    # ---- helpers -----------------------------------------------------------

    @staticmethod
    def _jet_colormap(t: np.ndarray) -> np.ndarray:
        """t in [0..1] → packed-RGB uint32, classic 'jet' colormap."""
        r = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0)
        g = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0)
        b = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0)
        return (
            (r * 255).astype(np.uint32) << 16
            | (g * 255).astype(np.uint32) << 8
            | (b * 255).astype(np.uint32)
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorldSurfacePublisher()
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
