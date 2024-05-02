#!/usr/bin/env python3

import datetime
import os
import os.path as osp
import re
import threading
from collections import deque
from pathlib import Path
from threading import Lock

import cv2
import cv_bridge
import numpy as np
import rospy
import sensor_msgs.msg
import std_msgs.msg
import tf2_ros
from cameramodels import PinholeCameraModel
from cv import decompresse_imgmsg, msg_to_img
from pybsc import current_time_str, makedirs, save_json
from pybsc.image_utils import imread, imwrite
from pybsc.timestamp_utils import get_timestamp_indices_wrt_base
from pybsc.vision.blur_detection import estimate_blur
from sensor_msgs.msg import CompressedImage, Image
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords

np.set_printoptions(precision=2)


def get_latest_img_file(path):
    path = Path(path)
    files = list(sorted(path.glob('*.jpg')))
    timestamps = []
    for file in files:
        match = re.search(r"(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})", str(file))
        if match:
            timestamp_str = match.group(1)
            timestamp = datetime.datetime.strptime(
                timestamp_str, '%Y-%m-%d-%H-%M-%S')
            timestamps.append((timestamp, file))
    timestamps.sort(reverse=True)  # Sort in descending order
    return timestamps[0][1] if timestamps else None


class DataCollector(object):

    def __init__(self):
        self.lock = Lock()
        self.root_image_path = Path(rospy.get_param(
            '~root_image_path',
            osp.expanduser(osp.join('~', 'dataset', '2023-12-20'))
        ))
        rospy.loginfo('waiting camera info')
        camera_info = rospy.wait_for_message(
            '/remote/color/camera_info',
            sensor_msgs.msg.CameraInfo)
        rospy.loginfo('camera info received')
        self.cm = PinholeCameraModel.from_camera_info(camera_info)

        self.queue = deque(maxlen=200)
        self.stamp_queue = deque(maxlen=200)
        self.img_sub = rospy.Subscriber(
            '/remote/color/image_rect_color/compressed',
            CompressedImage, queue_size=10,
            callback=self.img_callback)

        self.vacuum_sub = rospy.Subscriber(
            '/vacuum_pressure',
            std_msgs.msg.Float32,
            queue_size=1,
            callback=self.vacuum_callback
        )

        self.blur_pub = rospy.Publisher('blur', std_msgs.msg.Float32,
                                        queue_size=1)

        self.suctioned = False
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.img_publish_thread = threading.Thread(target=self.publish_img)
        self.img_publish_thread.daemon = True  # terminate when main thread exit
        self.img_publish_thread.start()

    def publish_img(self):
        rate = rospy.Rate(1)
        pub = rospy.Publisher('get_image', Image, queue_size=1)
        prev_img_path = None
        img_msg = None
        bridge = cv_bridge.CvBridge()
        while not rospy.is_shutdown():
            rate.sleep()
            current_img_path = get_latest_img_file(
                self.root_image_path / 'viz')
            if current_img_path != prev_img_path:
                img = imread(current_img_path)
                img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
                prev_img_path = current_img_path
            if img_msg is not None:
                img_msg.header.stamp = rospy.Time.now()
                pub.publish(img_msg)

    def img_callback(self, msg):
        img = decompresse_imgmsg(msg)
        _, score = estimate_blur(img)
        self.blur_pub.publish(std_msgs.msg.Float32(data=float(score)))
        if score < 50:
            return
        with self.lock:
            self.queue.append(msg)
            self.stamp_queue.append(msg.header.stamp.to_sec())

    def vacuum_callback(self, msg):
        if self.suctioned is False and msg.data < 320:
            self.suctioned_time_stamp = rospy.Time.now()
            self.suctioned = True
            self.save_image()
            rospy.loginfo('suctioned')
        elif msg.data >= 320:
            self.suctioned = False

    def get_pose(self, from_frame, to_frame, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                from_frame, to_frame, stamp, rospy.Duration(0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            return None
        return tf_pose_to_coords(transform)

    def save_image(self):
        target_time_stamps = []
        points_list = []
        for timestep in [2, 3, 4, 5]:
            target_time_stamp = self.suctioned_time_stamp - rospy.Duration(timestep)
            world_to_suction_frame_transform = self.get_pose(
                't265_odom_frame',
                'suction_frame',
                self.suctioned_time_stamp)
            if world_to_suction_frame_transform is None:
                rospy.logwarn('Failed to get {} to {} transform'
                              .format('t265_odom_frame',
                                      'suction_frame'))
                continue

            world_to_camera_transform = self.get_pose(
                't265_odom_frame',
                'd405_color_optical_frame',
                target_time_stamp)
            if world_to_camera_transform is None:
                rospy.logwarn('Failed to get {} to {} transform'
                              .format('t265_odom_frame',
                                      'd405_color_optical_frame'))
                continue
            point_in_camera = world_to_camera_transform.inverse_transform_vector(
                world_to_suction_frame_transform.worldpos())
            point_2d = self.cm.project3d_to_pixel(point_in_camera)
            target_time_stamps.append(target_time_stamp.to_sec())
            points_list.append(point_2d)
        if len(target_time_stamps) == 0:
            rospy.logwarn('failed to save image.')
            return

        target_time_stamps = np.array(target_time_stamps)
        points_list = np.array(points_list)
        with self.lock:
            timestamps = np.array(self.stamp_queue)
            if len(timestamps) == 0:
                rospy.logwarn('failed to save image. no images.')
                return
            indices = get_timestamp_indices_wrt_base(
                timestamps,
                target_time_stamps,
                style='closest')

            diff = timestamps[indices] - target_time_stamps
            diff = np.abs(diff)
            points_list = points_list[diff < 0.1]
            indices = indices[diff < 0.1]
            imgs = [self.queue[idx] for idx in indices]
        if len(imgs) == 0:
            rospy.logwarn('failed to save image.')
            return

        for img, point_2d in zip(imgs, points_list):
            img = decompresse_imgmsg(img)
            viz = cv2.circle(
                img.copy(), (int(point_2d[0]), int(point_2d[1])),
                10, (0, 0, 255), -1)
            x, y = point_2d
            if not (0 <= x < self.cm.width and 0 <= y < self.cm.height):
                continue
            makedirs(self.root_image_path / 'viz')
            filename = '{}.jpg'.format(current_time_str())
            imwrite(self.root_image_path / 'viz' / filename, viz)
            imwrite(self.root_image_path / filename, img)
            save_json(
                {
                    'x': float(x),
                    'y': float(y),
                 },
                (self.root_image_path / filename).with_suffix('.json'))


if __name__ == '__main__':
    rospy.init_node('data_collector')
    act = DataCollector()
    rospy.spin()
