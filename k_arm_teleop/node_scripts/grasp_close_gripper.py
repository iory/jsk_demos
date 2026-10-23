#!/usr/bin/env python3
"""Close the real gripper on the grasp_close model's decision.

Turns ``/grasp_close/close`` (std_msgs/Bool, the k_arm_grasp_close
grasp_close node) into the 0 (open) .. 1 (closed) command of
``gripper_command_bridge.py``, which drives the effort-controlled gripper.

The first True closes the gripper and the node keeps it closed whatever the
model says afterwards: the model was trained only up to just after the
operator's close, so its output while holding means nothing, and a closed
gripper must not drop what it holds. ``~open`` (std_srvs/Trigger) opens it;
the node closes again only after the decision has gone False once (a hand or
an apple still in front of the sensors does not close it right away).
``~enable`` (std_srvs/SetBool) switches the closing on / off; a disabled node
ignores the decision but still opens on ``~open``.

``~toggle_gripper`` (std_srvs/Trigger, the AtomS3 long press through
rec_button_bridge.py ``~gripper_service``) opens a closed gripper like
``~open``, and closes an open one by hand; a gripper closed by hand stays
closed until opened, also with ``~open_on_release``.

With ``~open_on_release`` the gripper instead follows the decision: it opens
once the decision has stayed False for ``~open_delay``. For trying the model
by hand; while holding something the decision means nothing, so it may open
and drop it.

Nothing is sent before the first close, so the gripper is not touched until
the model decides. The latest command is re-sent at ``~rate`` so the bridge
gets it even if it starts later.

On shutdown (Ctrl-C) a node that has sent anything releases the gripper:
zero effort straight to the effort controller (the arm's
``gripper.real.command_topic`` in teleop.yaml), since the controller keeps
its last command and the bridge in the same launch may already be gone. A
gripper holding something drops it.

Parameters
----------
~arm : str
    Arm in teleop.yaml ``arms`` (default ``rarm``).
~arms : dict
    teleop.yaml ``arms``, for the effort command topic.
~enabled : bool
    Close on the decision from the start (default True).
~open_on_release : bool
    Open when the decision goes False (default False: stay closed).
~open_delay : float
    [s] how long the decision must stay False before opening with
    ``~open_on_release`` (default 0.3).
~rate : float
    [Hz] re-send rate of the latest command (default 10).

Topics
------
~close (std_msgs/Bool, in)
    Remap to the model's decision, ``/grasp_close/close``.
~command (std_msgs/Float64, out)
    Remap to the bridge's ``~command``.
~state (std_msgs/String, out, latched)
    ``disabled`` / ``armed`` (open, waiting for a close) / ``closed`` /
    ``closed by hand`` / ``open, waiting for the decision to go False``.
"""

import signal
import threading
import time

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

OPEN = 0.0
CLOSED = 1.0
RELEASE_EFFORT = 0.0      # zero torque: the gripper goes limp and opens by itself
RELEASE_REPEAT = 5        # zero effort sent this many times on shutdown ...
RELEASE_PERIOD = 0.05     # [s] ... this far apart, past the bridge's last command


class GraspCloseGripper(object):

    def __init__(self):
        arm = rospy.get_param('~arm', 'rarm')
        arms = rospy.get_param('~arms')
        if arm not in arms or not (arms[arm].get('gripper') or {}).get('real'):
            raise RuntimeError('teleop.yaml has no arms/{}/gripper/real section'.format(arm))
        effort_topic = arms[arm]['gripper']['real']['command_topic']
        self.effort_pub = rospy.Publisher(effort_topic, Float64MultiArray, queue_size=1)
        self.lock = threading.Lock()
        self.enabled = bool(rospy.get_param('~enabled', True))
        self.open_on_release = bool(rospy.get_param('~open_on_release', False))
        self.open_delay = float(rospy.get_param('~open_delay', 0.3))
        self.false_since = None   # first False of the current run while closed
        self.command = None       # nothing sent yet
        self.wait_false = False   # after ~open: close only once the decision went False
        self.by_hand = False      # closed by ~toggle_gripper, not by the model
        self.command_pub = rospy.Publisher('~command', Float64, queue_size=1)
        self.state_pub = rospy.Publisher('~state', String, queue_size=1, latch=True)
        rospy.Subscriber('~close', Bool, self._close_cb, queue_size=1)
        rospy.Service('~open', Trigger, self._open)
        rospy.Service('~enable', SetBool, self._enable)
        rospy.Service('~toggle_gripper', Trigger, self._toggle)
        self._publish_state()
        rospy.loginfo('grasp_close_gripper: %s -> %s (%s)', rospy.resolve_name('~close'),
                      rospy.resolve_name('~command'), self._state())

    def _state(self):
        if self.command == CLOSED:
            return 'closed by hand' if self.by_hand else 'closed'
        if not self.enabled:
            return 'disabled'
        if self.wait_false:
            return 'open, waiting for the decision to go False'
        return 'armed'

    def _publish_state(self):
        self.state_pub.publish(String(data=self._state()))

    def _send(self, command):
        self.command = command
        self.command_pub.publish(Float64(data=command))
        self._publish_state()

    def _close_cb(self, msg):
        with self.lock:
            if not msg.data:
                if self.wait_false:
                    self.wait_false = False
                    self._publish_state()
                if self.open_on_release and self.command == CLOSED and not self.by_hand:
                    now = rospy.get_time()
                    if self.false_since is None:
                        self.false_since = now
                    if now - self.false_since >= self.open_delay:
                        rospy.loginfo('grasp_close_gripper: the decision went False, opening the gripper')
                        self.false_since = None
                        self._send(OPEN)
                return
            self.false_since = None
            if self.command == CLOSED or not self.enabled or self.wait_false:
                return
            rospy.loginfo('grasp_close_gripper: the model decided to close, closing the gripper')
            self._send(CLOSED)

    def _open_locked(self):
        self.wait_false = True
        self.by_hand = False
        self._send(OPEN)
        rospy.loginfo('grasp_close_gripper: opening (%s)', self._state())

    def _open(self, _request):
        with self.lock:
            if self.command is None:
                return TriggerResponse(False, 'nothing sent yet; the gripper was not touched')
            self._open_locked()
        return TriggerResponse(True, 'opening')

    def _toggle(self, _request):
        with self.lock:
            if self.command == CLOSED:
                self._open_locked()
                return TriggerResponse(True, 'gripper open')
            self.by_hand = True
            self.false_since = None
            self._send(CLOSED)
            rospy.loginfo('grasp_close_gripper: closed by hand')
            return TriggerResponse(True, 'gripper closed')

    def _enable(self, request):
        with self.lock:
            self.enabled = bool(request.data)
            self._publish_state()
            return SetBoolResponse(True, self._state())

    def release(self):
        """Zero effort to the gripper if this node has sent anything; call
        before rospy shuts down (its own shutdown closes the topics first)."""
        with self.lock:
            if self.command is None:
                return
            self.command = OPEN
            self.command_pub.publish(Float64(data=OPEN))
        for _ in range(RELEASE_REPEAT):
            self.effort_pub.publish(Float64MultiArray(data=[RELEASE_EFFORT]))
            time.sleep(RELEASE_PERIOD)
        rospy.loginfo('grasp_close_gripper: shutting down, released the gripper (effort %.1f)',
                      RELEASE_EFFORT)

    def step(self):
        with self.lock:
            if self.command is not None:
                self.command_pub.publish(Float64(data=self.command))


def main():
    # SIGINT / SIGTERM are handled here, not by rospy: rospy's shutdown closes
    # the publishers before any hook could send the release
    rospy.init_node('grasp_close_gripper', disable_signals=True)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: stop.set())
    node = GraspCloseGripper()
    period = 1.0 / float(rospy.get_param('~rate', 10.0))
    while not stop.is_set() and not rospy.is_shutdown():
        node.step()
        stop.wait(period)
    node.release()
    rospy.signal_shutdown('stopped')


if __name__ == '__main__':
    main()
