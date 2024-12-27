#!/usr/bin/env python3

from threading import Lock
import os.path as osp

import cv_bridge
import numpy as np
import rospy
import sensor_msgs.msg
import torch
from dynamic_reconfigure.server import Server
from jsk_recognition_msgs.msg import (ClassificationResult,
                                      ClusterPointIndices, Rect, RectArray)
from jsk_topic_tools import ConnectionBasedTransport
from pcl_msgs.msg import PointIndices
from ultralytics import YOLO
import cv2

from jsk_teaching_object.cfg import InstanceSegmentationConfig as Config


def resize_masks(masks, wh):
    out = [cv2.resize(mask, wh, interpolation=cv2.INTER_NEAREST) for mask in masks]
    return np.array(out)


class ObjectDetectionNode(ConnectionBasedTransport):

    def __init__(self):
        super(ObjectDetectionNode, self).__init__()

        self.lock = Lock()

        self.classifier_name = rospy.get_param('~classifier_name', 'grape_segmentation')
        self.ignore_class_names = rospy.get_param(
            '~ignore_class_names', [''])

        weights = rospy.get_param(
            '~model_path', None)
        device = rospy.get_param('~device', -1)
        if device < 0:
            device = 'cpu'
        self.srv = Server(Config, self.config_callback)
        self.device = device

        self.model = None
        self.load_model(weights)

        self.bridge = cv_bridge.CvBridge()
        self.pub = self.advertise('~output', sensor_msgs.msg.Image, queue_size=1)
        self.pub_compressed = self.advertise(
            '{}/compressed'.format(rospy.resolve_name('~output')),
            sensor_msgs.msg.CompressedImage, queue_size=1)
        self.rects_pub = self.advertise('~output/rects', RectArray, queue_size=1)
        self.encoding = 'bgr8'

        self.pub_indices = self.advertise(
            '~output/cluster_indices', ClusterPointIndices, queue_size=1)
        self.pub_lbl_cls = self.advertise(
            '~output/label_cls', sensor_msgs.msg.Image, queue_size=1)
        self.pub_lbl_ins = self.advertise(
            '~output/label_ins', sensor_msgs.msg.Image, queue_size=1)
        self.pub_class = self.advertise(
            "~output/class", ClassificationResult,
            queue_size=1)

    def load_model(self, model_path):
        del self.model
        torch.cuda.empty_cache()
        with self.lock:
            rospy.loginfo('Loading model {}'.format(model_path))
            self.model = YOLO(model_path, task='segment')
            if 'ncnn' not in osp.basename(model_path):
                self.model = self.model.to(self.device)
        self.target_names = [name for _, name in self.model.names.items()]
        rospy.loginfo("Loaded {} labels. {}".format(
            len(self.target_names),
            self.target_names))

    def config_callback(self, config, level):
        self.score_thresh = config.score_thresh
        self.nms_thresh = config.nms_thresh
        self.max_det = config.max_det
        self.roi = (config.roi_x_min, config.roi_y_min,
                    config.roi_x_max, config.roi_y_max)
        return config

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input',
            sensor_msgs.msg.Image,
            self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):
        if abs(msg.header.stamp - rospy.Time.now()).to_sec() > 1.0:
            return
        bridge = self.bridge
        encoding = self.encoding
        im = bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        org_h, org_w = im.shape[:2]

        if self.roi is not None:
            x_min, y_min, x_max, y_max = self.roi
            roi_image = im[y_min:y_max, x_min:x_max]
        else:
            roi_image = im

        with self.lock:
            results = self.model(roi_image, verbose=False)
        if results:
            result = results[0]
        else:
            rospy.logerr("Error: The 'results' list is empty.")
            return

        if self.pub.get_num_connections():
            annotated_frame = result.plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding=encoding,
                                           header=msg.header)
            self.pub.publish(img_msg)

        rects_msg = RectArray(header=msg.header)
        msg_indices = ClusterPointIndices(header=msg.header)

        valid_indices = []
        labels = []
        scores = []
        for j, ((x1, y1, x2, y2), conf, cls) in enumerate(
                zip(result.boxes.xyxy, result.boxes.conf,
                    result.boxes.cls)):
            if self.target_names[int(cls)] in self.ignore_class_names:
                continue
            if conf < self.score_thresh:
                continue

            if self.roi is not None:
                x1 = x1 + x_min
                y1 = y1 + y_min
                x2 = x2 + x_min
                y2 = y2 + y_min

            valid_indices.append(j)
            rects_msg.rects.append(
                Rect(x=int(x1), y=int(y1),
                     width=int(x2 - x1), height=int(y2 - y1)))
            labels.append(int(cls))
            scores.append(float(conf))

        lbl_cls = np.zeros((im.shape[0], im.shape[1]), dtype=np.int32)
        lbl_ins = np.zeros((im.shape[0], im.shape[1]), dtype=np.int32)
        if result.masks is not None:
            masks = result.masks.data.cpu().numpy()
            masks = resize_masks(masks, (roi_image.shape[1], roi_image.shape[0]))
            masks = masks[valid_indices]
            R, H, W = masks.shape
            mask_indices = np.array(
                np.arange(H * W).reshape(H, W), dtype=np.int32)

            for mask in masks:
                indices = mask_indices[mask > 0]

                if self.roi is not None:
                    indices = [(i // W + y_min) * org_w + (i % W + x_min) for i in indices]

                indices_msg = PointIndices(header=msg.header, indices=indices)
                msg_indices.cluster_indices.append(indices_msg)

            labels = np.array(labels)
            # -1: label for background
            if len(masks) > 0:
                lbl_cls = np.max(
                    (masks > 0)
                    * (labels.reshape(-1, 1, 1) + 1) - 1, axis=0)
                lbl_cls = np.array(lbl_cls, dtype=np.int32)
                lbl_ins = np.max(
                    (masks > 0) * (np.arange(R).reshape(-1, 1, 1) + 1) - 1,
                    axis=0)
                lbl_ins = np.array(lbl_ins, dtype=np.int32)
            labels = labels.tolist()

        self.pub_indices.publish(msg_indices)
        self.rects_pub.publish(rects_msg)
        cls_msg = ClassificationResult(
            header=msg.header,
            classifier=self.classifier_name,
            target_names=self.target_names,
            labels=labels,
            label_names=[self.target_names[label] for label in labels],
            label_proba=scores,
        )
        self.pub_class.publish(cls_msg)

        msg_lbl_cls = bridge.cv2_to_imgmsg(lbl_cls, header=msg.header)
        msg_lbl_ins = bridge.cv2_to_imgmsg(lbl_ins, header=msg.header)
        self.pub_lbl_cls.publish(msg_lbl_cls)
        self.pub_lbl_ins.publish(msg_lbl_ins)


if __name__ == "__main__":
    rospy.init_node('object_detection_node')
    act = ObjectDetectionNode()
    rospy.spin()
