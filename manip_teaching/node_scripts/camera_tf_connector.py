#!/usr/bin/env python

# you need to run.
# rosrun dynamic_tf_publisher tf_publish.py

import rospy
import tf
import tf2_ros
import skrobot

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import *
from dynamic_tf_publisher.srv import SetDynamicTF


class CameraTFConnector(object):

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def estimate_tf(self, parent_camera_link, parent_camera_checkerboard_frame,
                    child_camera_link, child_camera_checkerboard_frame,):
        tf_buffer = self.tf_buffer
        try:
            parent_camera_to_marker = tf_buffer.lookup_transform(
                parent_camera_link, parent_camera_checkerboard_frame,
                rospy.Time(), rospy.Duration(3))
            marker_to_child_camera = tf_buffer.lookup_transform(
                child_camera_checkerboard_frame,
                child_camera_link, rospy.Time(), rospy.Duration(3))
            parent_camera_to_marker_transform = skrobot.coordinates.Coordinates(
                [parent_camera_to_marker.transform.translation.x,
                 parent_camera_to_marker.transform.translation.y,
                 parent_camera_to_marker.transform.translation.z],
                [parent_camera_to_marker.transform.rotation.w,
                 parent_camera_to_marker.transform.rotation.x,
                 parent_camera_to_marker.transform.rotation.y,
                 parent_camera_to_marker.transform.rotation.z])
            marker_to_child_camera_transform = skrobot.coordinates.Coordinates(
                [marker_to_child_camera.transform.translation.x,
                 marker_to_child_camera.transform.translation.y,
                 marker_to_child_camera.transform.translation.z],
                [marker_to_child_camera.transform.rotation.w,
                 marker_to_child_camera.transform.rotation.x,
                 marker_to_child_camera.transform.rotation.y,
                 marker_to_child_camera.transform.rotation.z])
            parent_to_child = parent_camera_to_marker_transform.transform(
                marker_to_child_camera_transform)
            new_tf = TransformStamped()
            new_tf.header.frame_id = parent_camera_link
            new_tf.child_frame_id = child_camera_link
            new_tf.transform.translation.x = parent_to_child.translation[0]
            new_tf.transform.translation.y = parent_to_child.translation[1]
            new_tf.transform.translation.z = parent_to_child.translation[2]
            new_tf.transform.rotation.x = parent_to_child.quaternion[1]
            new_tf.transform.rotation.y = parent_to_child.quaternion[2]
            new_tf.transform.rotation.z = parent_to_child.quaternion[3]
            new_tf.transform.rotation.w = parent_to_child.quaternion[0]
            return new_tf
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            rospy.logwarn("Failed to get transform")

    def set_estimated_tf(self, estimated_tf, tf_hz=10):
        rospy.wait_for_service("/set_dynamic_tf")
        try:
            client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
            res = client(tf_hz, estimated_tf)
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            new_tf = self.estimate_tf(
                't265_link',
                't265_chessboard',
                'd405_link',
                'd405_chessboard'
            )
            if new_tf is not None:
                self.set_estimated_tf(new_tf)


if __name__ == "__main__":
    rospy.init_node("camera_tf_connector")
    setter = CameraTFConnector()
    setter.run()
