#!/usr/bin/env python3
"""Publish RViz markers for a uavros2 drone, attached to its base_link TF frame.

uavros2 drones spawn from PX4 SDF (no ROS robot_description / URDF), so RViz
has nothing to render for the bodies. This node fills that gap by publishing a
MarkerArray parented to ``frame_id`` (e.g. ``drone/base_link``); the markers
then move with the drone through TF — no URDF needed.

Three rendering modes, in priority order:

1. ``model_sdf`` set — parse the model SDF and render EVERY visual mesh at
   its true pose/scale (body, motors, legs, props), spinning the propellers
   on ``*rotor*`` links. This reproduces the full Gazebo model.
2. ``mesh_resource`` set — a single body mesh + geometric spinning prop blades.
3. neither — a simple geometric quadcopter (body + arms + spinning blades).

Propeller spin is driven by real flight data (``mavros/state`` armed +
``mavros/vfr_hud`` throttle); the rate is proportional to throttle and zero
when disarmed.

Adapted from drone_interception_sim/drone_markers.py.
"""
import math
import os
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point
from mavros_msgs.msg import State, VfrHud
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_rpy(r, p, y):
    """SDF roll-pitch-yaw (XYZ intrinsic) -> (x, y, z, w) quaternion."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _qrot(q, v):
    """Rotate vector v (3-tuple) by quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    rx, ry, rz, _ = _qmul(_qmul(q, qv), _qconj(q))
    return (rx, ry, rz)


def _parse_pose(text):
    """SDF '<pose>' text -> (position 3-tuple, quaternion (x,y,z,w))."""
    vals = [float(x) for x in (text or '').split()]
    while len(vals) < 6:
        vals.append(0.0)
    return (vals[0], vals[1], vals[2]), _quat_from_rpy(vals[3], vals[4], vals[5])


class DroneMarkers(Node):
    """Publish a quadcopter MarkerArray on the drone's base_link frame."""

    def __init__(self):
        super().__init__('drone_markers')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('marker_ns', 'drone')
        self.declare_parameter('color', [0.1, 0.4, 1.0])   # RGB 0..1 (geom modes)
        self.declare_parameter('arm_length', 0.25)
        self.declare_parameter('rate', 10.0)
        # Full-model mode: render every visual of this model SDF.
        self.declare_parameter('model_sdf', '')             # path to model.sdf
        self.declare_parameter('model_dir', '')             # base for model:// URIs
        # Single-mesh / geometric fallbacks.
        self.declare_parameter('mesh_resource', '')         # file://... single body mesh
        self.declare_parameter('mesh_scale', 1.0)
        # Propeller animation driven by the real PX4 throttle (VFR_HUD).
        self.declare_parameter('max_spin_rate', 80.0)       # rad/s at full throttle
        self.declare_parameter('idle_spin_rate', 12.0)      # rad/s when armed, ~0 throttle
        self.declare_parameter('show_props', True)          # geometric/single-mesh modes
        self.declare_parameter('prop_z', 0.10)
        self.declare_parameter('prop_len', 0.22)

        self.frame_id = self.get_parameter('frame_id').value
        self.ns = self.get_parameter('marker_ns').value
        self.color = list(self.get_parameter('color').value)
        self.arm = self.get_parameter('arm_length').value
        self.mesh = self.get_parameter('mesh_resource').value
        self.mesh_scale = self.get_parameter('mesh_scale').value
        self.max_spin = self.get_parameter('max_spin_rate').value
        self.idle_spin = self.get_parameter('idle_spin_rate').value
        self.show_props = self.get_parameter('show_props').value
        self.prop_z = self.get_parameter('prop_z').value
        self.prop_len = self.get_parameter('prop_len').value

        model_sdf = self.get_parameter('model_sdf').value
        model_dir = self.get_parameter('model_dir').value
        self.visuals = self._parse_sdf(model_sdf, model_dir) if model_sdf else []
        if model_sdf and not self.visuals:
            self.get_logger().warn(
                f'no visuals parsed from {model_sdf}; falling back to geometry')

        # Real flight state for the propeller spin.
        self.armed = False
        self.throttle = 0.0          # 0..1 from VFR_HUD
        self.spin_angle = 0.0
        self._last_t = None
        state_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(State, 'mavros/state', self._state_cb, state_qos)
        self.create_subscription(VfrHud, 'mavros/vfr_hud', self._vfr_cb,
                                 qos_profile_sensor_data)

        self.pub = self.create_publisher(MarkerArray, 'markers', 1)
        period = 1.0 / max(self.get_parameter('rate').value, 1e-3)
        self.create_timer(period, self._publish)

    # ---- SDF parsing -------------------------------------------------------

    def _resolve_uri(self, uri, model_dir, sdf_dir):
        if uri.startswith('model://'):
            return 'file://' + os.path.join(model_dir, uri[len('model://'):])
        if uri.startswith(('file://', 'package://')):
            return uri
        if uri.startswith('/'):
            return 'file://' + uri
        return 'file://' + os.path.join(sdf_dir, uri)

    def _parse_sdf(self, sdf_path, model_dir):
        """Return a list of visual specs in the base_link frame.

        Each spec is (p_link, q_link, p_vis, q_vis, scale, uri, spin_dir) so
        the per-frame marker pose is T(base_link<-link) . Rz(spin) . T(visual).
        spin_dir is +1 (CCW) / -1 (CW) for '*rotor*' links, else 0 (static).

        Follows ``<include>`` references recursively so composed UAV SDFs
        (e.g. our generated assemblies that ``<include>`` the chassis SDF +
        sensor SDFs) get all visuals from every included model.
        """
        visuals = []
        self._sdf_walk(sdf_path, model_dir, visuals,
                       p_off=(0.0, 0.0, 0.0), q_off=(0.0, 0.0, 0.0, 1.0))
        return visuals

    def _sdf_walk(self, sdf_path, model_dir, out, p_off, q_off):
        """Recurse one SDF, accumulating visuals into ``out`` with parent offset."""
        try:
            root = ET.parse(sdf_path).getroot()
        except (OSError, ET.ParseError) as exc:
            self.get_logger().warn(f'cannot parse {sdf_path}: {exc}')
            return
        model = root.find('model') if root.tag != 'model' else root
        if model is None:
            return
        sdf_dir = os.path.dirname(sdf_path)
        if not model_dir:
            model_dir = os.path.dirname(sdf_dir)

        # base_link link pose in the model frame (markers are in the base_link frame)
        base_pos, base_quat = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        for link in model.findall('link'):
            if link.get('name') == 'base_link':
                pe = link.find('pose')
                if pe is not None:
                    base_pos, base_quat = _parse_pose(pe.text)
        bq_inv = _qconj(base_quat)

        # Follow <include> references (depth-first).
        for inc in model.findall('include'):
            ue = inc.find('uri')
            if ue is None or not ue.text:
                continue
            uri = ue.text.strip()
            if uri.startswith('model://'):
                inc_path = os.path.join(model_dir, uri[len('model://'):], 'model.sdf')
            elif uri.startswith('file://'):
                inc_path = uri[len('file://'):]
                if os.path.isdir(inc_path):
                    inc_path = os.path.join(inc_path, 'model.sdf')
            elif uri.startswith('/'):
                inc_path = uri
                if os.path.isdir(inc_path):
                    inc_path = os.path.join(inc_path, 'model.sdf')
            else:
                inc_path = os.path.join(sdf_dir, uri, 'model.sdf')
            if not os.path.isfile(inc_path):
                continue
            ipe = inc.find('pose')
            if ipe is not None:
                ipos, iquat = _parse_pose(ipe.text)
            else:
                ipos, iquat = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
            # Compose the include's pose with the running offset.
            child_q = _qmul(q_off, iquat)
            child_p = tuple(p_off[i] + _qrot(q_off, ipos)[i] for i in range(3))
            self._sdf_walk(inc_path, model_dir, out, child_p, child_q)

        # Now walk this SDF's own links.
        for link in model.findall('link'):
            lname = (link.get('name') or '').lower()
            pe = link.find('pose')
            lpos, lquat = _parse_pose(pe.text) if pe is not None else (
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            q_link_local = _qmul(bq_inv, lquat)
            p_link_local = _qrot(bq_inv, (lpos[0] - base_pos[0],
                                          lpos[1] - base_pos[1],
                                          lpos[2] - base_pos[2]))
            # Apply parent offset (from includes).
            q_link = _qmul(q_off, q_link_local)
            p_link = tuple(p_off[i] + _qrot(q_off, p_link_local)[i] for i in range(3))
            spin_dir = 0.0
            if 'rotor' in lname:
                spin_dir = 1.0
                for vis in link.findall('visual'):
                    ue = vis.find('.//mesh/uri')
                    fn = (ue.text or '').lower() if ue is not None else ''
                    if 'ccw' in fn:
                        spin_dir = 1.0
                        break
                    if 'cw' in fn:
                        spin_dir = -1.0
                        break
            for vis in link.findall('visual'):
                mesh = vis.find('.//mesh')
                if mesh is None:
                    continue   # skip plane/box decals (FMU labels etc.)
                ue = mesh.find('uri')
                if ue is None or not ue.text:
                    continue
                uri = self._resolve_uri(ue.text.strip(), model_dir, sdf_dir)
                se = mesh.find('scale')
                if se is not None and se.text:
                    sv = [float(x) for x in se.text.split()]
                    while len(sv) < 3:
                        sv.append(sv[-1] if sv else 1.0)
                    scale = (sv[0], sv[1], sv[2])
                else:
                    scale = (1.0, 1.0, 1.0)
                ve = vis.find('pose')
                vpos, vquat = _parse_pose(ve.text) if ve is not None else (
                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
                out.append((p_link, q_link, vpos, vquat, scale, uri, spin_dir))

    # ---- callbacks ---------------------------------------------------------

    def _state_cb(self, msg):
        self.armed = msg.armed

    def _vfr_cb(self, msg):
        # VFR_HUD.throttle is 0..100 (percent) in MAVLink/mavros.
        self.throttle = max(0.0, min(1.0, msg.throttle / 100.0))

    # ---- markers -----------------------------------------------------------

    def _base(self, marker_id, mtype):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = self.ns
        m.id = marker_id
        m.type = mtype
        m.action = Marker.ADD
        m.color.r, m.color.g, m.color.b = (float(c) for c in self.color[:3])
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        return m

    def _model_markers(self):
        """Full model: every parsed visual at its true pose/scale, rotors spinning."""
        markers = []
        for i, (p_link, q_link, p_vis, q_vis, scale, uri, spin_dir) in \
                enumerate(self.visuals):
            if spin_dir != 0.0:
                s = spin_dir * self.spin_angle
                qs = (0.0, 0.0, math.sin(s / 2.0), math.cos(s / 2.0))
            else:
                qs = (0.0, 0.0, 0.0, 1.0)
            q_ls = _qmul(q_link, qs)
            q_marker = _qmul(q_ls, q_vis)
            off = _qrot(q_ls, p_vis)
            m = self._base(i, Marker.MESH_RESOURCE)
            m.mesh_resource = uri
            m.mesh_use_embedded_materials = True   # .dae materials; STL uses color
            m.pose.position.x = p_link[0] + off[0]
            m.pose.position.y = p_link[1] + off[1]
            m.pose.position.z = p_link[2] + off[2]
            m.pose.orientation.x, m.pose.orientation.y = q_marker[0], q_marker[1]
            m.pose.orientation.z, m.pose.orientation.w = q_marker[2], q_marker[3]
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r = m.color.g = m.color.b = 0.15   # for material-less STL meshes
            m.color.a = 1.0
            markers.append(m)
        return markers

    def _prop_markers(self, z):
        """Geometric spinning blade bars at the X-quad rotor positions."""
        markers = []
        d = self.arm / math.sqrt(2.0)
        rotors = [(d, d), (-d, -d), (d, -d), (-d, d)]
        spin_dir = [1.0, 1.0, -1.0, -1.0]
        for i, (x, y) in enumerate(rotors):
            r = self._base(2 + i, Marker.CUBE)
            r.pose.position.x, r.pose.position.y, r.pose.position.z = x, y, z
            yaw = spin_dir[i] * self.spin_angle
            r.pose.orientation.z = math.sin(yaw / 2.0)
            r.pose.orientation.w = math.cos(yaw / 2.0)
            r.scale.x, r.scale.y, r.scale.z = self.prop_len, 0.03, 0.01
            r.color.a = 0.95
            markers.append(r)
        return markers

    def _mesh_marker(self):
        m = self._base(0, Marker.MESH_RESOURCE)
        m.mesh_resource = self.mesh
        m.mesh_use_embedded_materials = True
        m.scale.x = m.scale.y = m.scale.z = float(self.mesh_scale)
        markers = [m]
        if self.show_props:
            markers += self._prop_markers(self.prop_z)
        return markers

    def _quad_markers(self):
        markers = []
        body = self._base(0, Marker.CUBE)
        body.scale.x, body.scale.y, body.scale.z = 0.18, 0.18, 0.06
        markers.append(body)

        arms = self._base(1, Marker.LINE_LIST)
        arms.scale.x = 0.03
        d = self.arm / math.sqrt(2.0)
        for (x, y) in [(d, d), (-d, -d), (d, -d), (-d, d)]:
            arms.points.append(Point(x=0.0, y=0.0, z=0.0))
            arms.points.append(Point(x=x, y=y, z=0.0))
        markers.append(arms)

        markers += self._prop_markers(0.02)
        return markers

    def _spin_rate(self):
        """Angular rate (rad/s) for the props, from real armed/throttle state."""
        if not self.armed:
            return 0.0
        return self.idle_spin + (self.max_spin - self.idle_spin) * self.throttle

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is not None:
            self.spin_angle = (self.spin_angle + self._spin_rate() *
                               (now - self._last_t)) % (2.0 * math.pi)
        self._last_t = now
        if self.visuals:
            markers = self._model_markers()
        elif self.mesh:
            markers = self._mesh_marker()
        else:
            markers = self._quad_markers()
        self.pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = DroneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
