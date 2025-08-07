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

    def _process_segmentation(self, result, valid_indices, labels, rects,
                              roi_image, org_dims):
        """Processes masks or bounding boxes to generate segmentation data.

        This method generates segmentation data (cluster indices and label maps)
        by processing masks from the model output. If masks are not available,
        it falls back to using bounding boxes.

        Args:
            result: The inference result from the YOLO model.
            valid_indices (list): A list of indices for valid detections that
                                  passed the score threshold.
            labels (list): A list of integer class labels for valid detections.
            rects (list): A list of jsk_recognition_msgs.msg.Rect for valid
                          detections.
            roi_image (np.ndarray): The cropped region of interest image.
            org_dims (tuple): A tuple of (height, width) for the original
                              input image.

        Returns:
            tuple: A tuple containing:
                - list: A list of NumPy arrays, where each array contains the
                        pixel indices for a detected instance.
                - np.ndarray: The semantic segmentation map (label_cls).
                - np.ndarray: The instance segmentation map (label_ins).
        """
        org_h, org_w = org_dims
        cluster_indices_list = []

        # Initialize full-size label maps with -1 to denote the background.
        lbl_cls = np.full(org_dims, -1, dtype=np.int32)
        lbl_ins = np.full(org_dims, -1, dtype=np.int32)

        # --- Path 1: Process from masks if available ---
        if result.masks is not None and len(result.masks.data) > 0:
            roi_h, roi_w = roi_image.shape[:2]
            masks = result.masks.data.cpu().numpy()
            masks = resize_masks(masks, (roi_w, roi_h))
            masks = masks[valid_indices]

            if len(masks) > 0:
                # Generate cluster indices for each mask
                mask_indices_roi = np.arange(
                    roi_h * roi_w, dtype=np.int32).reshape(roi_h, roi_w)
                for mask in masks:
                    indices_in_roi = mask_indices_roi[mask > 0]
                    if self.roi is not None:
                        x_min, y_min = self.roi[0], self.roi[1]
                        indices = ((indices_in_roi // roi_w + y_min) * org_w +
                                   (indices_in_roi % roi_w + x_min))
                    else:
                        indices = indices_in_roi
                    cluster_indices_list.append(indices)

                # Generate label maps for the ROI using the stack of masks
                labels_np = np.array(labels)
                R = len(masks)
                roi_lbl_cls = np.max(
                    (masks > 0) * (labels_np.reshape(-1, 1, 1) + 1),
                    axis=0) - 1
                roi_lbl_ins = np.max(
                    (masks > 0) * (np.arange(R).reshape(-1, 1, 1) + 1),
                    axis=0) - 1

                # Place the ROI label maps into the full-size maps
                if self.roi is not None:
                    x_min, y_min, x_max, y_max = self.roi
                    lbl_cls[y_min:y_max, x_min:x_max] = roi_lbl_cls
                    lbl_ins[y_min:y_max, x_min:x_max] = roi_lbl_ins
                else:
                    lbl_cls, lbl_ins = roi_lbl_cls, roi_lbl_ins

        # --- Path 2: Fallback to using bounding boxes ---
        else:
            if len(rects) > 0:
                rospy.logwarn_once(
                    "result.masks not found. "
                    "Falling back to bounding boxes for segmentation."
                )
            for i, rect in enumerate(rects):
                x, y, w, h = rect.x, rect.y, rect.width, rect.height
                # Draw the instance and class IDs onto the label maps
                y_slice = slice(max(0, y), min(org_h, y + h))
                x_slice = slice(max(0, x), min(org_w, x + w))
                lbl_ins[y_slice, x_slice] = i
                lbl_cls[y_slice, x_slice] = labels[i]

                # Generate indices from bounding box coordinates
                if w > 0 and h > 0:
                    x_coords, y_coords = np.meshgrid(
                        np.arange(x, x + w), np.arange(y, y + h))
                    indices = y_coords.flatten() * org_w + x_coords.flatten()
                    cluster_indices_list.append(indices.astype(np.int32))

        return cluster_indices_list, lbl_cls, lbl_ins

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
                x_min, y_min, _, _ = self.roi
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

        cluster_indices_list, lbl_cls, lbl_ins = self._process_segmentation(
            result=result,
            valid_indices=valid_indices,
            labels=labels,
            rects=rects_msg.rects,
            roi_image=roi_image,
            org_dims=(org_h, org_w)
        )

        for indices in cluster_indices_list:
            indices_msg = PointIndices(header=msg.header, indices=indices.tolist())
            msg_indices.cluster_indices.append(indices_msg)

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
