#!/usr/bin/env python3
"""Faithful dummy plant for hydrus -- verifies trajectry_real.py on ROS (headless).

Models the *ideal* aerial-robot behaviour the controller assumes:

  * a flight controller that drives the robot COG to the commanded
    (x, y, z) and the body yaw to the commanded target_yaw
    (FlightNav, target=COG, WORLD_FRAME);
  * three joint servos that drive joint1/2/3 to /hydrus/joints_ctrl.

Both are first-order followers, slew-rate limited well ABOVE the
controller's own rate limits, so the bottleneck is the controller --
exactly like the real machine.

The plant uses skrobot's own ``centroid()`` as the COG model, so a
tracking failure is NOT a COG-model mismatch: it isolates the
controller's geometry / frame / rate-limit logic.

Broadcasts tf  world -> hydrus/{root, leg5, fc, cog}
Publishes      /hydrus/joint_states

Parameters (private ~):
  ~urdf            : path to a URDF file (else read /robot_description param)
  ~rate            : plant update rate [Hz]              (default 200)
  ~q_slew          : joint slew rate    [rad/s]          (default 3.0)
  ~yaw_slew        : yaw slew rate      [rad/s]          (default 2.0)
  ~cog_slew        : COG slew rate      [m/s]            (default 1.0)
  ~init_cog        : initial COG [x,y,z]                 (default [0,0,1])
  ~init_q          : initial [j1,j2,j3]                  (default [1,0.6,1])
"""
import numpy as np

import rospy
import tf
import tf.transformations as tft

from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav

from skrobot.models.urdf import RobotModelFromURDF
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import rpy_matrix


def load_robot():
    """Build a skrobot model from ~urdf (path) or /robot_description (xml)."""
    urdf_path = rospy.get_param('~urdf', '')
    if urdf_path:
        rospy.loginfo('dummy_plant: loading URDF from %s', urdf_path)
        return RobotModelFromURDF(urdf_file=urdf_path)
    xml = rospy.get_param('robot_description', '')
    if not xml:
        raise RuntimeError(
            'neither ~urdf nor /robot_description is set')
    rospy.loginfo('dummy_plant: loading URDF from /robot_description param')
    import tempfile
    import os
    fd, tmp = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(xml)
    return RobotModelFromURDF(urdf_file=tmp)


def Rz(yaw):
    # skrobot rpy_matrix(yaw, pitch, roll)
    return rpy_matrix(yaw, 0.0, 0.0)


class DummyPlant(object):
    def __init__(self):
        self.robot = load_robot()
        self.q = np.array(rospy.get_param('~init_q', [1.0, 0.6, 1.0]),
                          dtype=float)
        self.cog = np.array(rospy.get_param('~init_cog', [0.0, 0.0, 1.0]),
                            dtype=float)
        self.yaw = 0.0
        self.q_cmd = self.q.copy()
        self.cog_cmd = self.cog.copy()
        self.yaw_cmd = 0.0

        self.hz = float(rospy.get_param('~rate', 200.0))
        self.dt = 1.0 / self.hz
        self.q_slew = float(rospy.get_param('~q_slew', 3.0))
        self.yaw_slew = float(rospy.get_param('~yaw_slew', 2.0))
        self.cog_slew = float(rospy.get_param('~cog_slew', 1.0))

        self.br = tf.TransformBroadcaster()
        self.js_pub = rospy.Publisher('/hydrus/joint_states', JointState,
                                      queue_size=1)
        rospy.Subscriber('/hydrus/joints_ctrl', JointState,
                         self._joint_cb, queue_size=1)
        rospy.Subscriber('/hydrus/uav/nav', FlightNav,
                         self._nav_cb, queue_size=1)

    def _joint_cb(self, msg):
        d = dict(zip(msg.name, msg.position))
        self.q_cmd = np.array([d.get('joint1', self.q_cmd[0]),
                               d.get('joint2', self.q_cmd[1]),
                               d.get('joint3', self.q_cmd[2])])

    def _nav_cb(self, msg):
        if msg.target == FlightNav.COG:
            self.cog_cmd = np.array([msg.target_pos_x, msg.target_pos_y,
                                     msg.target_pos_z])
            self.yaw_cmd = float(msg.target_yaw)

    @staticmethod
    def _slew(cur, tgt, max_d):
        diff = tgt - cur
        n = np.linalg.norm(diff)
        if n <= max_d or n < 1e-12:
            return np.array(tgt, dtype=float)
        return cur + diff / n * max_d

    def _cog_in_root(self):
        self.robot.root_link.newcoords(Coordinates())
        self.robot.joint1.joint_angle(self.q[0])
        self.robot.joint2.joint_angle(self.q[1])
        self.robot.joint3.joint_angle(self.q[2])
        return np.asarray(self.robot.centroid(), dtype=float)

    def step(self):
        self.q = self._slew(self.q, self.q_cmd, self.q_slew * self.dt)
        self.cog = self._slew(self.cog, self.cog_cmd, self.cog_slew * self.dt)
        self.yaw += np.clip(self.yaw_cmd - self.yaw,
                            -self.yaw_slew * self.dt, self.yaw_slew * self.dt)

        cog_root = self._cog_in_root()
        R = Rz(self.yaw)
        root_xyz = self.cog - R.dot(cog_root)
        self.robot.root_link.newcoords(
            Coordinates(pos=root_xyz, rot=R), relative_coords='world')

        now = rospy.Time.now()
        js = JointState()
        js.header.stamp = now
        js.name = ['joint1', 'joint2', 'joint3']
        js.position = [float(v) for v in self.q]
        self.js_pub.publish(js)

        # Broadcast every URDF link as world -> hydrus/<link> (flat) from the
        # posed skrobot model. This is the single tf source: it feeds the
        # controller (hydrus/root, hydrus/leg5, hydrus/fc) AND lets rviz's
        # RobotModel (TF Prefix = hydrus) render the whole robot -- no
        # robot_state_publisher needed (this ros-o RSP ignores frame_prefix).
        for link in self.robot.link_list:
            self._tf('hydrus/' + link.name, link.worldpos(), link.worldrot(),
                     now)
        # cog is not a URDF link; publish it separately (body yaw, COG pos).
        self._tf('hydrus/cog', self.cog, R, now)

    def _tf(self, child, pos, rot3, stamp):
        T = np.eye(4)
        T[:3, :3] = np.asarray(rot3)
        T[:3, 3] = np.asarray(pos)
        quat = tft.quaternion_from_matrix(T)
        self.br.sendTransform(tuple(np.asarray(pos, dtype=float)),
                              tuple(quat), stamp, child, 'world')


def main():
    rospy.init_node('dummy_plant')
    plant = DummyPlant()
    rospy.loginfo('dummy_plant up: q=%s yaw=%.2f cog=%s',
                  plant.q, plant.yaw, plant.cog)
    rate = rospy.Rate(plant.hz)
    while not rospy.is_shutdown():
        plant.step()
        rate.sleep()


if __name__ == '__main__':
    main()
