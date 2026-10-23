#!/usr/bin/env python3
"""Publish the close-the-gripper decision of a train.py model.

Builds a feature frame at the model's ``rate_hz`` from the latest message
of each input (the config stored in the model), keeps the last ``window_s``
and runs the model. It only publishes; k_arm_teleop grasp_close_gripper.py
turns ``~close`` into the gripper's command.

Parameters
----------
~model : str
    train.py's model.pt.

Topics
------
~close (std_msgs/Bool)
    True once ``debounce`` consecutive probabilities reach ``threshold``.
~close_probability (std_msgs/Float64)
    Every tick with a full window.
~status (std_msgs/String)
    JSON: state (ok / filling / stale / bad input), the stale inputs, probability.
"""

import json
import os
import threading
from collections import deque

import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import UInt32MultiArray

from k_arm_grasp_close.features import FeatureBuilder
from k_arm_grasp_close.features import input_topics
from k_arm_grasp_close.model import GraspCloseModel


class GraspCloseNode(object):

    def __init__(self):
        path = os.path.expanduser(rospy.get_param('~model'))
        self.model = GraspCloseModel(path)
        cfg = self.model.cfg
        self.fb = FeatureBuilder(cfg)
        if self.fb.names != self.model.names:
            raise RuntimeError('the model config gives other features than the model was trained on')
        self.window = int(round(float(cfg['window_s']) * float(cfg['rate_hz'])))
        self.max_age = float(cfg['max_age_s'])
        self.threshold = float(cfg['decision']['threshold'])
        self.debounce = int(cfg['decision']['debounce'])
        f = cfg['features']
        types = [(t, Range) for t in f['ranges']] + [(t, UInt32) for t in f['intensities']]
        types += [(t, UInt32MultiArray) for t in f['arrays']] + [(f['joint_states'], JointState)]
        self.inputs = input_topics(cfg)
        self.lock = threading.Lock()
        self.latest = {}
        for t, cls in types:
            rospy.Subscriber(t, cls, self._store(t), queue_size=1)
        self.frames = deque(maxlen=self.window)
        self.run = 0
        self.pub_close = rospy.Publisher('~close', Bool, queue_size=1)
        self.pub_prob = rospy.Publisher('~close_probability', Float64, queue_size=1)
        self.pub_status = rospy.Publisher('~status', String, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / float(cfg['rate_hz'])), self._tick)
        rospy.loginfo('grasp_close: %s, %d features, window %d', path, len(self.fb.names), self.window)

    def _store(self, topic):
        def cb(msg):
            with self.lock:
                self.latest[topic] = (rospy.get_time(), msg)
        return cb

    def _clear(self):
        self.frames.clear()
        self.fb.reset()
        self.run = 0

    def _tick(self, _event):
        now = rospy.get_time()
        with self.lock:
            latest = dict(self.latest)
        stale = [t for t in self.inputs if t not in latest or now - latest[t][0] > self.max_age]
        status = dict(state='ok', stale=stale, probability=None)
        if stale:
            self._clear()
            status['state'] = 'stale'
        else:
            try:
                self.frames.append(self.fb.frame(dict((t, latest[t][1]) for t in self.inputs)))
            except ValueError as e:
                rospy.logwarn_throttle(5.0, 'grasp_close: %s', e)
                self._clear()
                status['state'] = 'bad input: {}'.format(e)
            if len(self.frames) == self.window:
                p = self.model.prob(list(self.frames))
                self.run = self.run + 1 if p >= self.threshold else 0
                status['probability'] = round(p, 3)
                self.pub_prob.publish(Float64(data=p))
            elif status['state'] == 'ok':
                status['state'] = 'filling'
        close = self.run >= self.debounce
        status['close'] = close
        self.pub_close.publish(Bool(data=close))
        self.pub_status.publish(String(data=json.dumps(status)))


def main():
    rospy.init_node('grasp_close')
    GraspCloseNode()
    rospy.spin()


if __name__ == '__main__':
    main()
