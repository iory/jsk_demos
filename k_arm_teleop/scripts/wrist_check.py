#!/usr/bin/env python3
"""Move one K_ARM joint a little and back, to check the real arm responds.

Reads the arm's current joint angles from /joint_states, sends a
FollowJointTrajectory goal that moves only ``--joint`` by ``--delta`` over
``--duration`` (all other joints of the controller stay where they are),
waits, then sends it back. Nothing is sent before you type "y".

Stop the teleop (or keep its target away from the real robot) first, so it
does not command the same controller.

    export ROS_MASTER_URI=http://<robot>:11311 ROS_IP=<this PC>
    rosrun k_arm_teleop wrist_check.py --dry-run          # show what it would send
    rosrun k_arm_teleop wrist_check.py                    # RARM_JOINT6 +0.05 rad and back
    rosrun k_arm_teleop wrist_check.py --joint RARM_JOINT4 --delta -0.03
"""

import argparse
import sys
import xml.etree.ElementTree as ET

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

MAX_DELTA = 0.1  # [rad] refuse larger test moves: this is a small wiggle


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controller', default='rarm_controller')
    parser.add_argument('--action', default='follow_joint_trajectory_action',
                        help='action name under the controller (the K_ARM bringup uses this one)')
    parser.add_argument('--joint', default='RARM_JOINT6', help='the one joint to move')
    parser.add_argument('--delta', type=float, default=0.05,
                        help='[rad] (default 0.05, about 3 deg), at most {}'.format(MAX_DELTA))
    parser.add_argument('--duration', type=float, default=3.0, help='[s] for each way')
    parser.add_argument('--dry-run', action='store_true', help='print the goals, send nothing')
    return parser.parse_args(rospy.myargv()[1:])


def joint_limits():
    limits = {}
    if not rospy.has_param('/robot_description'):
        return limits
    for joint in ET.fromstring(rospy.get_param('/robot_description')).findall('joint'):
        limit = joint.find('limit')
        if limit is not None and joint.get('type') in ('revolute', 'prismatic'):
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    return limits


def main():
    args = parse_args()
    rospy.init_node('wrist_check', anonymous=True)
    if abs(args.delta) > MAX_DELTA:
        sys.exit('--delta {} is more than {} rad; this is only a check'.format(args.delta, MAX_DELTA))

    # the joints the controller owns, from its state topic
    state_topic = '/{}/state'.format(args.controller)
    try:
        from control_msgs.msg import JointTrajectoryControllerState
        state = rospy.wait_for_message(state_topic, JointTrajectoryControllerState, timeout=5.0)
        joints = list(state.joint_names)
    except rospy.ROSException:
        sys.exit('no {} within 5 s: is the robot bringup running and ROS_MASTER_URI set?'.format(
            state_topic))
    if args.joint not in joints:
        sys.exit('{} is not a joint of {} ({})'.format(args.joint, args.controller, joints))

    js = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
    positions = dict(zip(js.name, js.position))
    missing = [j for j in joints if j not in positions]
    if missing:
        sys.exit('/joint_states has no {}'.format(missing))
    start = [positions[j] for j in joints]
    target = list(start)
    index = joints.index(args.joint)
    target[index] += args.delta

    limits = joint_limits()
    if args.joint in limits:
        lower, upper = limits[args.joint]
        if not lower <= target[index] <= upper:
            sys.exit('{} would go to {:.3f}, outside its limits [{:.3f}, {:.3f}]'.format(
                args.joint, target[index], lower, upper))
    else:
        print('warning: no limits for {} in /robot_description'.format(args.joint))

    print('controller: /{}/{}'.format(args.controller, args.action))
    for j, a, b in zip(joints, start, target):
        print('  {:<12} {:+.4f} -> {:+.4f}{}'.format(j, a, b, '   <- moves' if a != b else ''))
    print('{:.3f} rad in {:.1f} s, then back in {:.1f} s'.format(
        args.delta, args.duration, args.duration))
    if args.dry_run:
        print('dry run: nothing sent')
        return
    if input('send it? [y/N] ').strip().lower() != 'y':
        print('nothing sent')
        return

    client = actionlib.SimpleActionClient('/{}/{}'.format(args.controller, args.action),
                                          FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        sys.exit('no action server /{}/{}'.format(args.controller, args.action))

    for label, goal_positions in (('there', target), ('back', start)):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=goal_positions, velocities=[0.0] * len(joints),
            time_from_start=rospy.Duration(args.duration))]
        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(args.duration + 5.0))
        now = dict(zip(*_joint_snapshot()))
        reached = now.get(args.joint, float('nan'))
        print('{}: {} (state {}), {} commanded {:+.4f}, measured {:+.4f} (error {:+.4f} rad)'.format(
            label, 'done' if finished else 'TIMED OUT', client.get_state(), args.joint,
            goal_positions[index], reached, reached - goal_positions[index]))
        if not finished:
            client.cancel_goal()
            sys.exit('goal did not finish; cancelled')


def _joint_snapshot():
    js = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
    return js.name, js.position


if __name__ == '__main__':
    main()
