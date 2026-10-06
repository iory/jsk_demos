"""Forward kinematics on a URDF and the leader -> K_ARM wrist orientation map.

The leader's wrist (pitch + roll) and the K_ARM's wrist (z, y, x axes) are
built differently, so the wrist joints cannot be copied one to one. Instead
the leader hand's rotation relative to its forearm is carried over to the
K_ARM forearm and split into the K_ARM wrist angles.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """URDF convention: fixed-axis X, then Y, then Z (R = Rz Ry Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_angle_to_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1.0 - c) * k.dot(k)


class _Joint(object):
    __slots__ = ('name', 'type', 'parent', 'child', 'xyz', 'rotation', 'axis')

    def __init__(self, element):
        self.name = element.get('name')
        self.type = element.get('type')
        self.parent = element.find('parent').get('link')
        self.child = element.find('child').get('link')
        origin = element.find('origin')
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        self.xyz = np.array(xyz)
        self.rotation = rpy_to_matrix(*rpy)
        axis = element.find('axis')
        self.axis = np.array([float(v) for v in axis.get('xyz').split()]) \
            if axis is not None else np.array([1.0, 0.0, 0.0])


class UrdfKinematics(object):
    """Rotations of URDF frames for given joint angles (positions are not needed here)."""

    def __init__(self, urdf_string):
        root = ET.fromstring(urdf_string)
        self.joints = {}
        self.by_child = {}
        for element in root.findall('joint'):
            joint = _Joint(element)
            self.joints[joint.name] = joint
            self.by_child[joint.child] = joint

    def _chain(self, link):
        """Joints from the root down to ``link``."""
        chain = []
        while link in self.by_child:
            joint = self.by_child[link]
            chain.append(joint)
            link = joint.parent
        return chain[::-1]

    def _motion(self, joint, positions):
        if joint.type in ('revolute', 'continuous'):
            return axis_angle_to_matrix(joint.axis, positions.get(joint.name, 0.0))
        return np.eye(3)

    def link_rotation(self, link, positions):
        """Rotation of ``link`` in the root link's frame."""
        rotation = np.eye(3)
        for joint in self._chain(link):
            rotation = rotation.dot(joint.rotation).dot(self._motion(joint, positions))
        return rotation

    def joint_frame_rotation(self, joint_name, positions):
        """Rotation of a joint's frame before its own motion (parent link * origin)."""
        joint = self.joints[joint_name]
        return self.link_rotation(joint.parent, positions).dot(joint.rotation)

    def relative_rotation(self, first_joint, last_joint, positions):
        """Child link of ``last_joint`` seen from the frame of ``first_joint``."""
        child = self.joints[last_joint].child
        return self.joint_frame_rotation(first_joint, positions).T.dot(
            self.link_rotation(child, positions))


def zyx_angles(rotation):
    """Split R = Rz(a) Ry(b) Rx(c) into (a, b, c)."""
    b = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    a = math.atan2(rotation[1, 0], rotation[0, 0])
    c = math.atan2(rotation[2, 1], rotation[2, 2])
    return a, b, c


class WristMap(object):
    """Leader wrist joints -> follower wrist joints by matching the hand rotation.

    Parameters
    ----------
    leader : UrdfKinematics
    follower : UrdfKinematics
    leader_joints, follower_joints : list of str
        The arm joints (mapped one to one) followed by the three wrist
        joints, base to tip, for each robot.
    leader_zero : dict
        Leader angles of the calibration pose, where the follower is at 0.
    leader_base_rotation : numpy.ndarray
        Rotation from the leader base frame to the follower base frame.

    The follower wrist must turn about z, y, x with no rotation in the joint
    origins between them (true for K_ARM); this is checked.
    """

    def __init__(self, leader, follower, leader_joints, follower_joints, leader_zero,
                 leader_base_rotation):
        self.leader = leader
        self.l_wrist = leader_joints[-3:]
        self.f_wrist = follower_joints[-3:]
        for name, axis in zip(self.f_wrist, ([0, 0, 1], [0, 1, 0], [1, 0, 0])):
            joint = follower.joints[name]
            if not np.allclose(joint.axis / np.linalg.norm(joint.axis), axis):
                raise ValueError('{} turns about {}, expected {}'.format(name, joint.axis, axis))
        for name in self.f_wrist[1:]:
            if not np.allclose(follower.joints[name].rotation, np.eye(3)):
                raise ValueError('{} has a rotated origin; z-y-x split does not apply'.format(name))

        zero = dict(leader_zero)
        # forearm frames at the calibration pose -> constant leader->follower forearm offset
        leader_forearm = leader_base_rotation.dot(
            leader.joint_frame_rotation(self.l_wrist[0], zero))
        follower_forearm = follower.joint_frame_rotation(self.f_wrist[0], {})
        self.forearm_offset = leader_forearm.T.dot(follower_forearm)
        self.leader_wrist_zero = leader.relative_rotation(self.l_wrist[0], self.l_wrist[-1], zero)
        self._last = None

    def reset(self):
        """Forget the last wrist solution (call when following restarts)."""
        self._last = None

    def follower_wrist(self, leader_positions, limits=None, previous=None):
        """Follower wrist angles (J4, J5, J6) for the leader's current wrist.

        The K_ARM wrist (J4 +-165 deg, J5 +-90 deg, J6 +-90 deg) cannot reach
        every hand rotation of the leader, so somewhere the target must be cut.
        This keeps the cuts where they do the least harm:

        - J5 stays within +-90 deg (the z-y-x branch the K_ARM can reach).
        - J4 and J6 follow the last solution continuously across +-180 deg up
          to WRAP_HYSTERESIS beyond it (the caller clamps them to their
          limits, so the robot just stops there), and only then wrap. A wrap
          therefore happens only after the leader has turned well outside
          the K_ARM range, and no full turn is ever wound up.
        - Near the singularity (J5 -> +-90 deg, J4 and J6 axes line up) J4 is
          held at its last value and J5 / J6 take the rotation, instead of J4
          and J6 swinging around.

        ``previous`` (``{joint: angle}``) seeds the continuity when there is no
        last solution (after reset()).
        """
        leader_rel = self.leader.relative_rotation(self.l_wrist[0], self.l_wrist[-1],
                                                   leader_positions)
        c = self.forearm_offset
        rot = c.T.dot(leader_rel).dot(self.leader_wrist_zero.T).dot(c)
        b_limits = limits.get(self.f_wrist[1], (-math.pi / 2, math.pi / 2)) if limits else \
            (-math.pi / 2, math.pi / 2)

        last = self._last
        if last is None and previous is not None and all(n in previous for n in self.f_wrist):
            last = tuple(previous[n] for n in self.f_wrist)

        a = math.atan2(rot[1, 0], rot[0, 0])
        cos_b = math.hypot(rot[0, 0], rot[1, 0])
        if last is not None:
            a = _continue(a, last[0])
            if cos_b < SINGULAR_COS:
                # J4 is ill-defined here: move it only as far as cos(J5) allows
                a = last[0] + (cos_b / SINGULAR_COS) ** 2 * (a - last[0])
        a, b, cc = _split_given_z(rot, a, b_limits)
        if last is not None:
            cc = _continue(cc, last[2])
        self._last = (a, b, cc)
        return dict(zip(self.f_wrist, self._last))


# Below this |cos(J5)| (about 76 deg) J4 is held back near the wrist singularity.
SINGULAR_COS = 0.25
# J4 / J6 may run this far past +-180 deg (following the last value) before
# they wrap back [rad].
WRAP_HYSTERESIS = 0.5


def _continue(angle, last):
    """The 2 pi copy of ``angle`` nearest ``last``, kept within +-(pi + WRAP_HYSTERESIS)."""
    nearest = angle + 2.0 * math.pi * round((last - angle) / (2.0 * math.pi))
    if abs(nearest) > math.pi + WRAP_HYSTERESIS:
        return math.atan2(math.sin(nearest), math.cos(nearest))
    return nearest


def _split_given_z(rot, a, b_limits):
    """(a, b, c) with Rz(a) Ry(b) Rx(c) as close to ``rot`` as a fixed ``a`` allows.

    ``b`` is clamped to ``b_limits``.
    """
    rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    rest = rz.T.dot(rot)  # = Ry(b) Rx(c) when a is exact
    b = max(b_limits[0], min(b_limits[1], math.atan2(-rest[2, 0], rest[0, 0])))
    c = math.atan2(-rest[1, 2], rest[1, 1])
    return a, b, c
