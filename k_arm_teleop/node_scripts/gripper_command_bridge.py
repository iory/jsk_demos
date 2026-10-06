#!/usr/bin/env python3
"""Drive a real effort-controlled gripper from a 0..1 close command.

For nodes other than the teleop (e.g. the learned picking policy, whose
gripper output is ``std_msgs/Float64`` 0 = open .. 1 = closed). The command
goes through the same open / close logic as the teleop trigger
(``k_arm_teleop.mapping.gripper_effort``, the arm's ``gripper.real``
section of teleop.yaml): closing torque while closed, a push until the
gripper stops when it opens, then 0.

Nothing is sent before the first command; the servo is switched on with
it. The effort is re-evaluated at ``~rate`` from the last command, so an
opening push ends on its own. When commands stop, the last state is kept
(a closed gripper keeps holding what it holds); ``~release`` (Trigger)
opens it.

Parameters
----------
~arm : str
    Arm in teleop.yaml ``arms`` (default ``rarm``).
~rate : float
    [Hz] effort update rate (default 50).
"""

import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from k_arm_teleop.mapping import gripper_effort


class GripperCommandBridge(object):

    def __init__(self):
        arm = rospy.get_param('~arm', 'rarm')
        arms = rospy.get_param('~arms')
        if arm not in arms or not (arms[arm].get('gripper') or {}).get('real'):
            raise RuntimeError('teleop.yaml has no arms/{}/gripper/real section'.format(arm))
        self.real = arms[arm]['gripper']['real']
        self.lock = threading.Lock()
        self.fraction = None
        self.velocity = None
        self.state = {}
        self.effort_pub = rospy.Publisher(self.real['command_topic'], Float64MultiArray, queue_size=1)
        self.servo_pub = rospy.Publisher(self.real['servo_topic'], Bool, queue_size=1, latch=True)
        rospy.Subscriber(self.real['state_topic'], JointState, self._state_cb, queue_size=1)
        rospy.Subscriber('~command', Float64, self._command_cb, queue_size=1)
        rospy.Service('~release', Trigger, self._release)
        rospy.loginfo('gripper bridge: %s (0..1) -> %s', rospy.resolve_name('~command'),
                      self.real['command_topic'])

    def _state_cb(self, msg):
        with self.lock:
            self.velocity = msg.velocity[0] if msg.velocity else None

    def _command_cb(self, msg):
        with self.lock:
            if self.fraction is None:
                self.servo_pub.publish(Bool(data=True))
                rospy.loginfo('gripper bridge: first command, servo on')
            self.fraction = min(max(float(msg.data), 0.0), 1.0)

    def _release(self, _request):
        with self.lock:
            if self.fraction is None:
                return TriggerResponse(False, 'no command received yet; the gripper was not touched')
            self.fraction = 0.0
        return TriggerResponse(True, 'opening')

    def step(self):
        with self.lock:
            if self.fraction is None:
                return
            effort = gripper_effort(self.fraction, self.real, self.state, rospy.get_time(),
                                    self.velocity)
        self.effort_pub.publish(Float64MultiArray(data=[effort]))


def main():
    rospy.init_node('gripper_command_bridge')
    bridge = GripperCommandBridge()
    rate = rospy.Rate(float(rospy.get_param('~rate', 50.0)))
    while not rospy.is_shutdown():
        bridge.step()
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    main()
