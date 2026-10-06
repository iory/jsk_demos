#!/usr/bin/env python3
"""Find a streaming setup the real K_ARM follows without building up lag.

Teleop streams small trajectories at a high rate. On the robot the servo
reference was seen replaying them later and later (lag growing while
moving), although one slow goal (wrist_check.py) is followed fine. This
streams a small slow sine on one joint with several setups and reports, for
each, the lag of the measured joint (/joint_states) behind the command at
the start and at the end of the stream: a growing lag means the robot
buffers the commands.

Stop the teleop first. Each setup moves --joint by at most --amplitude
(default 0.05 rad, about 3 deg) around where it is, then waits for it to
settle.

    export ROS_MASTER_URI=http://<robot>:11311 ROS_IP=<this PC>
    rosrun k_arm_teleop stream_test.py --dry-run
    rosrun k_arm_teleop stream_test.py
"""

import argparse
import math
import sys
import threading

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

MAX_AMPLITUDE = 0.1  # [rad]

# (label, interface, rate [Hz], time_from_start [s])
SETUPS = [
    ('topic 50Hz / 0.04s (teleop now)', 'topic', 50.0, 0.04),
    ('topic 50Hz / 0.02s', 'topic', 50.0, 0.02),
    ('topic 10Hz / 0.10s', 'topic', 10.0, 0.10),
    ('action 50Hz / 0.04s', 'action', 50.0, 0.04),
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controller', default='rarm_controller')
    parser.add_argument('--action', default='follow_joint_trajectory_action')
    parser.add_argument('--joint', default='RARM_JOINT6')
    parser.add_argument('--amplitude', type=float, default=0.05, help='[rad]')
    parser.add_argument('--period', type=float, default=4.0, help='[s] of the sine')
    parser.add_argument('--duration', type=float, default=12.0, help='[s] of streaming per setup')
    parser.add_argument('--settle', type=float, default=4.0, help='[s] to wait after each setup')
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args(rospy.myargv()[1:])


class Recorder(object):

    def __init__(self, joint):
        self.joint = joint
        self.lock = threading.Lock()
        self.measured = []
        rospy.Subscriber('/joint_states', JointState, self._cb, queue_size=100)

    def _cb(self, msg):
        if self.joint in msg.name:
            with self.lock:
                self.measured.append((rospy.get_time(), msg.position[msg.name.index(self.joint)]))

    def latest(self):
        with self.lock:
            return self.measured[-1][1] if self.measured else None

    def between(self, t0, t1):
        with self.lock:
            return np.array([m for m in self.measured if t0 <= m[0] <= t1])


def lag_of(command, measured, maxlag=3.0):
    """Delay [s] that best aligns measured to command, and the error left."""
    if len(command) < 10 or len(measured) < 10:
        return float('nan'), float('nan')
    lo = max(command[0, 0], measured[0, 0]) + 0.5
    hi = min(command[-1, 0], measured[-1, 0])
    if hi - lo < 1.0:
        return float('nan'), float('nan')
    grid = np.arange(lo, hi, 0.005)
    meas = np.interp(grid, measured[:, 0], measured[:, 1])
    best = min((np.abs(np.interp(grid - lag, command[:, 0], command[:, 1]) - meas).mean(), lag)
               for lag in np.arange(0.0, maxlag, 0.005))
    return best[1], best[0]


def main():
    args = parse_args()
    if abs(args.amplitude) > MAX_AMPLITUDE:
        sys.exit('--amplitude above {} rad; this is only a test'.format(MAX_AMPLITUDE))
    rospy.init_node('stream_test', anonymous=True)
    state = rospy.wait_for_message('/{}/state'.format(args.controller),
                                   JointTrajectoryControllerState, timeout=5.0)
    joints = list(state.joint_names)
    if args.joint not in joints:
        sys.exit('{} is not in {}'.format(args.joint, joints))
    index = joints.index(args.joint)
    recorder = Recorder(args.joint)
    js = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
    hold = [js.position[js.name.index(j)] for j in joints]
    center = hold[index]
    print('{} stays around {:+.4f} (+-{:.3f} rad, period {:.1f} s); the other joints hold.'.format(
        args.joint, center, args.amplitude, args.period))
    for label, _, rate, tfs in SETUPS:
        print('  {:<34} {:.0f} s'.format(label, args.duration))
    if args.dry_run:
        print('dry run: nothing sent')
        return
    if input('send it? [y/N] ').strip().lower() != 'y':
        print('nothing sent')
        return

    pub = rospy.Publisher('/{}/command'.format(args.controller), JointTrajectory, queue_size=1)
    client = actionlib.SimpleActionClient('/{}/{}'.format(args.controller, args.action),
                                          FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        sys.exit('no action server')
    rospy.sleep(1.0)  # let the command topic connect

    results = []
    for label, interface, rate, tfs in SETUPS:
        command = []
        r = rospy.Rate(rate)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - t0
            if t > args.duration:
                break
            # sine that starts and ends at the centre with zero slope at the ends
            q = center + args.amplitude * math.sin(2.0 * math.pi * t / args.period) * \
                min(1.0, t / 1.0, (args.duration - t) / 1.0)
            positions = list(hold)
            positions[index] = q
            traj = JointTrajectory(joint_names=joints, points=[JointTrajectoryPoint(
                positions=positions, time_from_start=rospy.Duration(tfs))])
            if interface == 'topic':
                pub.publish(traj)
            else:
                client.send_goal(FollowJointTrajectoryGoal(trajectory=traj))
            command.append((rospy.get_time(), q))
            r.sleep()
        t_end = rospy.get_time()
        rospy.sleep(args.settle)
        command = np.array(command)
        half = t0 + args.duration / 2.0
        early = lag_of(command[command[:, 0] < half], recorder.between(t0, half + 3.0))
        late = lag_of(command[command[:, 0] >= half], recorder.between(half, t_end + args.settle))
        left = recorder.latest() - center
        results.append((label, early, late, left))
        print('{:<34} lag first half {:5.2f} s, second half {:5.2f} s (error {:.4f} / {:.4f} rad); '
              '{:+.4f} rad from the centre after {:.0f} s'.format(
                  label, early[0], late[0], early[1], late[1], left, args.settle))
    # hold where it started
    client.send_goal(FollowJointTrajectoryGoal(trajectory=JointTrajectory(
        joint_names=joints, points=[JointTrajectoryPoint(positions=hold,
                                                         time_from_start=rospy.Duration(2.0))])))
    client.wait_for_result(rospy.Duration(6.0))
    print('\nA lag that grows from the first half to the second means the commands pile up.')


if __name__ == '__main__':
    main()
