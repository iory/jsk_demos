#!/usr/bin/env python3

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import message_filters
from image_geometry import PinholeCameraModel

from pathlib import Path
from stereodemo import methods
from stereodemo.method_raft_stereo import RaftStereo
from stereodemo.methods import Config, EnumParameter, StereoMethod, InputPair, Calibration
import cv2


class StereoDepthEstimation(ConnectionBasedTransport):

    def __init__(self):
        super(StereoDepthEstimation, self).__init__()

        self.bridge = cv_bridge.CvBridge()

        default_models_path = Path.home() / ".cache" / "stereodemo" / "models"
        config = methods.Config(default_models_path)
        self.raft_stereo = RaftStereo(config)
        self.raft_stereo.parameters["Shape"].set_value("160x128")
        self.raft_stereo.parameters["Model"].set_value('fast-cpu')

        self.pub = self.advertise(
            '~output', Image, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 100)
        sub_left = message_filters.Subscriber(
            '~left/image_rect',
            Image, queue_size=1, buff_size=2**24)
        sub_right = message_filters.Subscriber(
            '~right/image_rect',
            Image, queue_size=1, buff_size=2**24)
        sub_left_info = message_filters.Subscriber(
            '~left/camera_info',
            CameraInfo, queue_size=1, buff_size=2**24)
        sub_right_info = message_filters.Subscriber(
            '~right/camera_info',
            CameraInfo, queue_size=1, buff_size=2**24)
        self.subs = [sub_left, sub_right, sub_left_info, sub_right_info]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, left_img_msg, right_img_msg, left_info_msg, right_info_msg):
        bridge = self.bridge

        left_img = bridge.imgmsg_to_cv2(left_img_msg)
        right_img = bridge.imgmsg_to_cv2(right_img_msg)
        cm = PinholeCameraModel()
        cm.fromCameraInfo(right_info_msg)

        baseline = - cm.Tx() / cm.fx()
        calib = Calibration(
            cm.width, cm.height, cm.fx(), cm.fy(),
            cm.cx(), cm.cx(), cm.cy(), baseline, depth_range=(0.2, 3.0),)
        tmp = InputPair(left_img, right_img, calib, '')
        stereo_output = self.raft_stereo.compute_disparity(tmp)

        x0,y0,x1,y1 = tmp.calibration.left_image_rect_normalized
        x0 = int(x0*stereo_output.disparity_pixels.shape[1] + 0.5)
        x1 = int(x1*stereo_output.disparity_pixels.shape[1] + 0.5)
        y0 = int(y0*stereo_output.disparity_pixels.shape[0] + 0.5)
        y1 = int(y1*stereo_output.disparity_pixels.shape[0] + 0.5)
        valid_mask = np.zeros(stereo_output.disparity_pixels.shape, dtype=np.uint8)
        valid_mask[y0:y1, x0:x1] = 1
        stereo_output.disparity_pixels[valid_mask == 0] = -1.0
        depth_meters = StereoMethod.depth_meters_from_disparity(
            stereo_output.disparity_pixels, tmp.calibration)

        depth_meters[depth_meters <= 0.0] = 0.0
        depth_meters[depth_meters > 3.0] = 0.0
        depth_msg = self.bridge.cv2_to_imgmsg(depth_meters.astype(np.float32))
        depth_msg.header = left_img_msg.header
        self.pub.publish(depth_msg)


if __name__ == '__main__':
    rospy.init_node('stereo_depth_estimation')
    node = StereoDepthEstimation()
    rospy.spin()
