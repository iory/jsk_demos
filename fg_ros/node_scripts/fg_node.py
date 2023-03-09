#!/usr/bin/env python3

import os
from pathlib import Path
import sys

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
from pcl_msgs.msg import PointIndices
import rospy
import sensor_msgs.msg
import torch
import torch.backends.cudnn as cudnn

from fg_ros.cfg import InstanceSegmentationConfig as Config


ROOT = Path(__file__).resolve().parent / 'fg_libs' / 'yolo7'
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import check_img_size
from utils.general import non_max_suppression
from utils.general import scale_coords
from utils.segment.general import process_mask
from utils.torch_utils import select_device


class ForegroundSegmentationNode(ConnectionBasedTransport):

    def __init__(self):
        super(ForegroundSegmentationNode, self).__init__()

        self.classifier_name = rospy.get_param('~classifier_name', 'fg')
        self.target_names = rospy.get_param(
            '~class_names', ['foreground'])

        weights = rospy.get_param('~model_path')
        imgsz = rospy.get_param('~img_size', (640, 640))
        device = rospy.get_param('~device', 0)
        if device < 0:
            device = 'cpu'
        classes = None  # filter by class: --class 0, or --class 0 2 3
        half = False  # use FP16 half-precision inference

        self.srv = Server(Config, self.config_callback)

        # Load model
        device = select_device(device)
        model = DetectMultiBackend(weights, device=device, dnn=False,
                                   data='', fp16=half)
        self.device = device
        model.eval()
        self.model = model
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Run inference
        self.model.warmup(imgsz=(1, 3, *imgsz))  # warmup
        self.imgsz = imgsz

        self.bridge = cv_bridge.CvBridge()
        self.pub = self.advertise('~output', sensor_msgs.msg.Image, queue_size=1)
        self.pub_compressed = self.advertise('{}/compressed'.format(rospy.resolve_name('~output')),
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

    def config_callback(self, config, level):
        self.score_thresh = config.score_thresh
        self.nms_thresh = config.nms_thresh
        self.max_det = config.max_det
        return config

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', sensor_msgs.msg.Image,
            self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, msg):
        bridge = self.bridge
        encoding = self.encoding
        im = bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        model = self.model
        imgsz = self.imgsz
        device = self.device

        classes = None
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size
        img_size = imgsz

        im0 = im.copy()
        im = letterbox(im, img_size, stride=stride, auto=True)[0]
        im = im[..., ::-1].transpose((2, 0, 1))  # BGR to RGB, BHWC to BCHW
        im = np.ascontiguousarray(im)  # contiguous
        im = torch.from_numpy(im).to(device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        with torch.no_grad():
            pred, out = model(im, augment=False, visualize=False)
            proto = out[1]

        det = non_max_suppression(pred, self.score_thresh, self.nms_thresh, classes,
                                   max_det=self.max_det, nm=32)[0]

        rects_msg = RectArray(header=msg.header)

        scores = []
        labels = []

        lbl_cls = - np.ones(im0.shape[:2], dtype=np.int32)
        lbl_ins = - np.ones(im0.shape[:2], dtype=np.int32)
        msg_indices = ClusterPointIndices(header=msg.header)
        if len(det):
            masks = process_mask(
                proto[0], det[:, 6:], det[:, :4], im.shape[2:], upsample=True)  # HWC
            R, H, W = masks.shape
            mask_indices = np.array(
                np.arange(H * W).reshape(H, W), dtype=np.int32)
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
            labels = []
            for j, (*xyxy, conf, cls) in enumerate(det[:, :6]):
                x1, y1, x2, y2 = map(int, xyxy)
                rects_msg.rects.append(Rect(x=x1, y=y1, width=x2 - x1, height=y2 - y1))
                labels.append(int(cls))
                scores.append(float(conf))
            masks = masks.cpu().numpy()
            labels = np.array(labels, dtype=np.int32)

            for mask in masks:
                indices = mask_indices[mask > 0]
                indices_msg = PointIndices(header=msg.header, indices=indices)
                msg_indices.cluster_indices.append(indices_msg)

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

        msg_lbl_cls = bridge.cv2_to_imgmsg(lbl_cls, header=msg.header)
        msg_lbl_ins = bridge.cv2_to_imgmsg(lbl_ins, header=msg.header)
        self.pub_lbl_cls.publish(msg_lbl_cls)
        self.pub_lbl_ins.publish(msg_lbl_ins)

        img_msg = bridge.cv2_to_imgmsg(im0, encoding=encoding,
                                       header=msg.header)
        self.pub.publish(img_msg)

        cls_msg = ClassificationResult(
            header=msg.header,
            classifier=self.classifier_name,
            target_names=self.target_names,
            labels=labels,
            label_names=[self.target_names[label] for label in labels],
            label_proba=scores,
        )
        self.pub_class.publish(cls_msg)


if __name__ == "__main__":
    rospy.init_node('fg_node')
    act = ForegroundSegmentationNode()
    rospy.spin()
