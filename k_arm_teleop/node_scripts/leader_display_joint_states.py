#!/usr/bin/env python3
"""Leader model state for rviz (display only; the manager never reads it).

Publishes ``joint_states`` for every movable joint of the leader URDF and
the static transform world -> ``<frame_prefix>base_link`` from teleop.yaml
``leader_base``, so the leader is drawn the way the K_ARM is (front +x,
up +z).

Normal mode: republishes ``servo_joint_states`` (real readings) and fills
joints without a servo with the calibration pose (``leader_zero``).

Guide mode (``~guide: true``): always publishes the calibration pose and a
text label, to be drawn as a translucent model that shows how to hold the
leader when pressing "Calibrate leader zero".
"""

import threading

from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import JointState
import tf2_ros
from visualization_msgs.msg import Marker

from k_arm_teleop.kinematics import rpy_to_matrix
from k_arm_teleop.mapping import leader_zero_pose
from k_arm_teleop.urdf_utils import parse_joints


def matrix_to_quaternion(m):
    """Rotation matrix -> (x, y, z, w)."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * (trace + 1.0) ** 0.5
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: m[k, k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * (1.0 + m[i, i] - m[j, j] - m[k, k]) ** 0.5
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return tuple(q)


def base_transform(leader_base, world_frame, child_frame):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = world_frame
    t.child_frame_id = child_frame
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
        leader_base['xyz']
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
     t.transform.rotation.w) = matrix_to_quaternion(rpy_to_matrix(*leader_base['rpy']))
    return t


def label_marker(frame, text):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = 'calibration_guide'
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.text = text
    # base_link frame of the leader: +y is up
    marker.pose.position.y = 0.12
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.025
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.3, 0.8, 1.0, 1.0
    return marker


def main():
    rospy.init_node('leader_display_joint_states')
    guide = rospy.get_param('~guide', False)
    prefix = rospy.get_param('~frame_prefix', 'leader/')
    world_frame = rospy.get_param('~world_frame', 'world')
    zero_pose = leader_zero_pose(rospy.get_param('~arms'))
    names = sorted(parse_joints(rospy.get_param('robot_description')))
    positions = {name: zero_pose.get(name, 0.0) for name in names}
    lock = threading.Lock()

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(base_transform(rospy.get_param('~leader_base'), world_frame,
                                             prefix + 'base_link'))

    if not guide:
        def callback(msg):
            with lock:
                for name, value in zip(msg.name, msg.position):
                    if name in positions:
                        positions[name] = value

        rospy.Subscriber('servo_joint_states', JointState, callback, queue_size=10)
    marker_pub = rospy.Publisher('label', Marker, queue_size=1, latch=True) if guide else None
    if guide:
        marker_pub.publish(label_marker(
            prefix + 'base_link',
            'calibration pose: hang straight down'))

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(rospy.get_param('~rate', 10.0 if guide else 50.0))
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        with lock:
            msg.position = [positions[n] for n in names]
        pub.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    main()
