#!/usr/bin/env python3
"""Kinematic stand-in for the K_ARM JointTrajectoryControllers.

Serves ``<controller>/<action>`` (teleop.yaml targets.<~target>.action) and ``<controller>/command``
for every controller in the ``arms`` section of teleop.yaml and publishes
``joint_states`` for all movable joints of ``robot_description``. Joints
follow the commanded trajectory exactly (linear interpolation between
points), so it shows what the commands are, not how the hardware responds.
"""

import threading

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from k_arm_teleop.mapping import clamp
from k_arm_teleop.urdf_utils import parse_joints


class _Segment(object):
    """A trajectory being executed: absolute times and positions per joint."""

    __slots__ = ('joints', 'times', 'positions')

    def __init__(self, joints, times, positions):
        self.joints = joints
        self.times = times
        self.positions = positions

    def sample(self, now):
        times = self.times
        if now >= times[-1]:
            return self.positions[-1]
        index = 1
        while times[index] < now:
            index += 1
        t0, t1 = times[index - 1], times[index]
        ratio = 0.0 if t1 <= t0 else (now - t0) / (t1 - t0)
        p0, p1 = self.positions[index - 1], self.positions[index]
        return [a + ratio * (b - a) for a, b in zip(p0, p1)]

    def end_time(self):
        return self.times[-1]


class FakeTrajectoryController(object):

    def __init__(self):
        self.limits = parse_joints(rospy.get_param('robot_description'))
        initial = rospy.get_param('~initial_positions', {})
        self.lock = threading.Lock()
        self.positions = {name: clamp(float(initial.get(name, 0.0)), lower, upper)
                          for name, (lower, upper) in self.limits.items()}
        self.segments = {}
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        # serve the action name of the target this node stands in for
        target = rospy.get_param('~target', 'virtual')
        self.action = rospy.get_param('~targets/{}/action'.format(target), 'follow_joint_trajectory')
        self.servers = []
        for controller, joints in self._controllers().items():
            missing = [j for j in joints if j not in self.limits]
            if missing:
                raise RuntimeError('{}: joints {} are not in robot_description'.format(
                    controller, missing))
            server = actionlib.SimpleActionServer(
                '{}/{}'.format(controller, self.action), FollowJointTrajectoryAction,
                execute_cb=lambda goal, c=controller, j=joints: self._execute(c, j, goal),
                auto_start=False)
            server.start()
            self.servers.append((controller, server))
            rospy.Subscriber(controller + '/command', JointTrajectory,
                             lambda msg, c=controller, j=joints: self._start(c, j, msg),
                             queue_size=1)
            rospy.loginfo('fake controller %s: %s', controller, ', '.join(joints))

    @staticmethod
    def _controllers():
        controllers = {}
        for arm in rospy.get_param('~arms').values():
            controllers[arm['controller']] = ([j['follower'] for j in arm['joints']]
                                              + list(arm.get('wrist', {}).get('follower', [])))
            gripper = arm.get('gripper')
            if gripper is not None:
                controllers[gripper['controller']] = [gripper['follower']]
        return controllers

    def _start(self, controller, controller_joints, trajectory):
        """Replace the controller's trajectory, starting from where the joints are now."""
        unknown = [j for j in trajectory.joint_names if j not in controller_joints]
        if unknown or not trajectory.points:
            rospy.logerr('%s: rejected trajectory (unknown joints %s, %d points)',
                         controller, unknown, len(trajectory.points))
            return None
        now = rospy.get_time()
        start = trajectory.header.stamp.to_sec() or now
        start = max(start, now)
        with self.lock:
            joints = list(trajectory.joint_names)
            times = [now]
            positions = [[self.positions[j] for j in joints]]
            for point in trajectory.points:
                t = start + point.time_from_start.to_sec()
                if t <= times[-1]:
                    continue
                times.append(t)
                positions.append([clamp(p, *self.limits[j])
                                  for j, p in zip(joints, point.positions)])
            if len(times) == 1:
                # every point lies in the past: jump to the last one
                times.append(now)
                positions.append([clamp(p, *self.limits[j]) for j, p in
                                  zip(joints, trajectory.points[-1].positions)])
            segment = _Segment(joints, times, positions)
            self.segments[controller] = segment
        return segment

    def _execute(self, controller, joints, goal):
        server = dict(self.servers)[controller]
        segment = self._start(controller, joints, goal.trajectory)
        result = FollowJointTrajectoryResult()
        if segment is None:
            result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            server.set_aborted(result)
            return
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if server.is_preempt_requested():
                with self.lock:
                    if self.segments.get(controller) is segment:
                        del self.segments[controller]  # hold where it is
                server.set_preempted()
                return
            with self.lock:
                replaced = self.segments.get(controller) is not segment
            if replaced:
                # a /command message took over; the goal no longer runs
                server.set_preempted()
                return
            if rospy.get_time() >= segment.end_time():
                result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                server.set_succeeded(result)
                return
            rate.sleep()

    def spin(self):
        rate = rospy.Rate(rospy.get_param('~rate', 100.0))
        names = sorted(self.positions)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            with self.lock:
                for controller, segment in list(self.segments.items()):
                    for joint, value in zip(segment.joints, segment.sample(now)):
                        self.positions[joint] = value
                    if now > segment.end_time() + 1.0:
                        del self.segments[controller]
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = names
                msg.position = [self.positions[n] for n in names]
            self.pub.publish(msg)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


def main():
    rospy.init_node('fake_trajectory_controller')
    FakeTrajectoryController().spin()


if __name__ == '__main__':
    main()
