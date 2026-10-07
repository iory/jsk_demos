#!/usr/bin/env python3
"""Record rosbags without the teleop: the AtomS3 (or rosservice) starts / stops it.

For recording while the robot is driven some other way. Offers the teleop
manager's recording interface, so rec_button_bridge.py works unchanged with
``~manager`` pointing here:

- ``~start_recording``, ``~stop_recording`` (std_srvs/Trigger)
- ``~set_record_directory`` (k_arm_teleop/SetString)
- ``~status`` (k_arm_teleop/TeleopStatus, 5 Hz) with phase ``record_only``;
  only the recording fields mean anything (no leader, no target).

Topics, directory and prefix come from the ``recording`` section of
teleop.yaml (``~recording``). Nothing is sent to the robot.
"""

import threading

import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from k_arm_teleop.msg import TeleopStatus
from k_arm_teleop.recorder import Recorder
from k_arm_teleop.srv import SetString
from k_arm_teleop.srv import SetStringResponse

PHASE = 'record_only'
STATUS_PERIOD = 0.2  # [s]


class RecordOnly(object):

    def __init__(self):
        recording = rospy.get_param('~recording')
        self.recorder = Recorder(recording['directory'], recording['prefix'], recording['topics'])
        self.lock = threading.Lock()
        self.message = 'ready ({} topics)'.format(len(recording['topics']))
        self.status_pub = rospy.Publisher('~status', TeleopStatus, queue_size=1)
        rospy.Service('~start_recording', Trigger, self._start)
        rospy.Service('~stop_recording', Trigger, self._stop)
        rospy.Service('~set_record_directory', SetString, self._set_directory)
        rospy.on_shutdown(self._shutdown)

    def _start(self, _request):
        with self.lock:
            try:
                self.recorder.start()
            except (RuntimeError, OSError) as error:
                return TriggerResponse(False, str(error))
            self.message = 'recording to {}'.format(self.recorder.bag_path)
            return TriggerResponse(True, self.message)

    def _stop(self, _request):
        with self.lock:
            try:
                path = self.recorder.stop()
            except RuntimeError as error:
                return TriggerResponse(False, str(error))
            self.message = 'saved {}'.format(path)
            return TriggerResponse(True, self.message)

    def _set_directory(self, request):
        with self.lock:
            try:
                self.recorder.set_directory(request.data)
            except (RuntimeError, OSError) as error:
                return SetStringResponse(False, str(error))
            return SetStringResponse(True, 'recording into {}'.format(self.recorder.directory))

    def _shutdown(self):
        with self.lock:
            if self.recorder.process is not None:
                self.recorder.stop()

    def publish(self):
        with self.lock:
            error = self.recorder.poll()
            if error:
                rospy.logerr(error)
                self.message = error
            msg = TeleopStatus()
            msg.header.stamp = rospy.Time.now()
            msg.phase = PHASE
            msg.message = self.message
            recorder = self.recorder
            msg.recording = recorder.process is not None
            msg.record_directory = recorder.directory
            if msg.recording:
                msg.current_bag = recorder.bag_path
                msg.recording_duration = rospy.get_time() - recorder.started
                msg.current_bag_size = recorder.current_size()
            msg.bag_count, msg.total_bag_size, msg.disk_free = recorder.directory_stats()
        self.status_pub.publish(msg)


def main():
    rospy.init_node('k_arm_recorder')
    node = RecordOnly()
    rate = rospy.Rate(1.0 / STATUS_PERIOD)
    while not rospy.is_shutdown():
        node.publish()
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    main()
