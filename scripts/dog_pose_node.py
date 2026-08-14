#!/usr/bin/env python
"""ROS node for dog detection and 24-keypoint pose estimation.

Runs an Ultralytics YOLO pose model trained on the ``dog-pose`` dataset
(https://docs.ultralytics.com/datasets/pose/dog-pose) on a
``sensor_msgs/Image`` stream and publishes, for every frame:

``~rects`` (``jsk_recognition_msgs/RectArray``)
    One rect per detected dog.
``~class`` (``jsk_recognition_msgs/ClassificationResult``)
    Label and score of each rect, in the same order.
``~pose`` (``jsk_recognition_msgs/PeoplePoseArray``)
    Per dog, the visible keypoints with their ``kpt_names`` and scores.
``~skeleton`` (``jsk_recognition_msgs/HumanSkeletonArray``)
    Per dog, the bones connecting those keypoints, ready for the
    ``jsk_rviz_plugin/HumanSkeletonArray`` display.
``~image_annotated`` (``sensor_msgs/Image``)
    Input frame with boxes, keypoints and bones drawn on it. Only computed
    while somebody subscribes.

Coordinates
-----------
Without depth the keypoints are image coordinates: ``x``/``y`` in pixels and
``z`` fixed to 0. Set ``~with_depth`` to true and feed ``depth_image`` plus
``camera_info`` to get metric 3D points in the camera optical frame instead;
keypoints whose depth is missing are then dropped from the message.

Model
-----
``~model`` takes a local path or an ``http(s)`` URL; a URL is downloaded once
into ``$ROS_HOME/dog_pose/`` and reused. The default points at the only public
dog-pose checkpoint we could find (see ``DEFAULT_MODEL_URL``). Ultralytics
ships ``dog-pose.yaml`` as a dataset, not as weights, so ``train_dog_pose.py``
is there to produce your own.

Four of the 24 keypoints are never annotated in the dog-pose dataset, so no
model trained on it can predict them -- see ``UNLABELLED_KEYPOINTS``.
"""

import hashlib
import os
import shutil
import tempfile
from urllib.parse import urlparse
from urllib.request import urlopen

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import Segment
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ultralytics import YOLO

# Keypoint order of the dog-pose dataset, taken verbatim from the ``kpt_names``
# block of ultralytics/cfg/datasets/dog-pose.yaml.
DOG_KEYPOINT_NAMES = [
    "front_left_paw",
    "front_left_knee",
    "front_left_elbow",
    "rear_left_paw",
    "rear_left_knee",
    "rear_left_elbow",
    "front_right_paw",
    "front_right_knee",
    "front_right_elbow",
    "rear_right_paw",
    "rear_right_knee",
    "rear_right_elbow",
    "tail_start",
    "tail_end",
    "left_ear_base",
    "right_ear_base",
    "nose",
    "chin",
    "left_ear_tip",
    "right_ear_tip",
    "left_eye",
    "right_eye",
    "withers",
    "throat",
]

# Measured over all 8476 annotated instances of the dog-pose dataset: these
# four keypoints are labelled 0.0% of the time. They exist in ``kpt_names`` but
# no model trained on dog-pose.yaml can ever predict them, so no bone below
# depends on them. They still travel through ~pose if some other model fills
# them in.
UNLABELLED_KEYPOINTS = ["left_eye", "right_eye", "withers", "throat"]

# dog-pose.yaml carries no edge list, so the bones below are our own. The torso
# is strung between the four elbows and tail_start rather than through the
# withers, because the withers is one of the never-annotated keypoints above;
# routing the spine through it would leave the body disconnected in practice.
DOG_BONES = [
    # head
    ("nose", "chin"),
    ("nose", "left_ear_base"),
    ("nose", "right_ear_base"),
    ("left_ear_base", "right_ear_base"),
    ("left_ear_base", "left_ear_tip"),
    ("right_ear_base", "right_ear_tip"),
    # neck and shoulder girdle
    ("left_ear_base", "front_left_elbow"),
    ("right_ear_base", "front_right_elbow"),
    ("front_left_elbow", "front_right_elbow"),
    # flanks, hip girdle and tail
    ("front_left_elbow", "rear_left_elbow"),
    ("front_right_elbow", "rear_right_elbow"),
    ("rear_left_elbow", "rear_right_elbow"),
    ("rear_left_elbow", "tail_start"),
    ("rear_right_elbow", "tail_start"),
    ("tail_start", "tail_end"),
    # limbs, elbow -> knee -> paw
    ("front_left_elbow", "front_left_knee"),
    ("front_left_knee", "front_left_paw"),
    ("front_right_elbow", "front_right_knee"),
    ("front_right_knee", "front_right_paw"),
    ("rear_left_elbow", "rear_left_knee"),
    ("rear_left_knee", "rear_left_paw"),
    ("rear_right_elbow", "rear_right_knee"),
    ("rear_right_knee", "rear_right_paw"),
]

# yolo26m-pose fine-tuned on dog-pose.yaml for 100 epochs at imgsz 640.
# Reported by its own checkpoint: box mAP50-95 0.901, pose mAP50-95 0.607.
# AGPL-3.0, like everything else derived from Ultralytics weights.
DEFAULT_MODEL_URL = (
    "https://huggingface.co/20-team-daeng-ddang-ai/dog-pose-estimation"
    "/resolve/main/best.pt"
)


PALETTE = [
    (255, 0, 255),
    (0, 255, 255),
    (0, 255, 0),
    (255, 128, 0),
    (0, 128, 255),
    (255, 255, 0),
    (128, 0, 255),
    (0, 0, 255),
]


def resolve_model_path(model, cache_dir):
    """Return a local path for ``model``, fetching it once when it is a URL.

    The cache file name carries a hash of the URL, so two checkpoints that are
    both called ``best.pt`` upstream cannot shadow each other.

    Parameters
    ----------
    model : str
        Local path, or an ``http://`` / ``https://`` URL.
    cache_dir : str
        Directory the downloaded weights are kept in. Created if missing.

    Returns
    -------
    str
        Path to the weights on disk.
    """
    if not model.startswith(("http://", "https://")):
        return model
    digest = hashlib.sha1(model.encode("utf-8")).hexdigest()[:12]
    basename = os.path.basename(urlparse(model).path) or "model.pt"
    path = os.path.join(cache_dir, "{}_{}".format(digest, basename))
    if os.path.exists(path):
        rospy.loginfo("Using cached weights: %s", path)
        return path

    if not os.path.isdir(cache_dir):
        os.makedirs(cache_dir)
    rospy.loginfo("Downloading weights from %s", model)
    # Download beside the target and rename, so an interrupted run cannot
    # leave a truncated file that later looks like a valid cache hit.
    handle, tmp_path = tempfile.mkstemp(dir=cache_dir, suffix=".part")
    os.close(handle)
    try:
        response = urlopen(model)
        try:
            with open(tmp_path, "wb") as f:
                shutil.copyfileobj(response, f)
        finally:
            response.close()
        os.rename(tmp_path, path)
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    rospy.loginfo("Saved weights to %s", path)
    return path


def _split_csv(s):
    return [t.strip() for t in s.split(",") if t.strip()]


def extract_detections(result, kpt_conf_thresh):
    """Collect every dog of a YOLO pose result.

    Parameters
    ----------
    result : ultralytics.engine.results.Results
        Single-image result returned by ``YOLO.predict`` or ``YOLO.track``.
    kpt_conf_thresh : float
        Keypoints scoring below this are marked invalid.

    Returns
    -------
    list of dict
        One dict per detection with keys ``xyxy`` (4 floats, pixels),
        ``cls`` (int), ``conf`` (float), ``track_id`` (int or None),
        ``keypoints`` (``(K, 2)`` float array, pixels), ``kpt_scores``
        (``(K,)`` float array) and ``kpt_valid`` (``(K,)`` bool array).
        Empty when the frame contains no dog.
    """
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    xyxy = boxes.xyxy.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()
    if boxes.id is None:
        track_ids = [None] * len(cls)
    else:
        track_ids = [int(i) for i in boxes.id.cpu().numpy()]

    keypoints = result.keypoints
    if keypoints is None:
        raise RuntimeError(
            "The model returned no keypoints. ~model must be a pose model "
            "trained on dog-pose, not a plain detection model."
        )
    kpt_xy = keypoints.xy.cpu().numpy()
    if keypoints.conf is None:
        kpt_scores = np.ones(kpt_xy.shape[:2], dtype=np.float32)
    else:
        kpt_scores = keypoints.conf.cpu().numpy()

    detections = []
    for i in range(len(cls)):
        # A keypoint the model considers absent comes back as exactly (0, 0).
        placed = np.any(kpt_xy[i] != 0.0, axis=1)
        detections.append(
            {
                "xyxy": [float(v) for v in xyxy[i]],
                "cls": int(cls[i]),
                "conf": float(confs[i]),
                "track_id": track_ids[i],
                "keypoints": kpt_xy[i],
                "kpt_scores": kpt_scores[i],
                "kpt_valid": placed & (kpt_scores[i] >= kpt_conf_thresh),
            }
        )
    return detections


def draw_detections(frame, detections, class_names, keypoint_names, bones):
    """Draw the box, keypoints and bones of every detection, in place.

    Parameters
    ----------
    frame : numpy.ndarray
        BGR image to draw on.
    detections : list of dict
        Detections from ``extract_detections``.
    class_names : list of str
        Name of each class index.
    keypoint_names : list of str
        Name of each keypoint index.
    bones : list of tuple
        ``(index_a, index_b)`` keypoint index pairs to connect.

    Returns
    -------
    numpy.ndarray
        The same array, annotated.
    """
    for det in detections:
        color = PALETTE[det["cls"] % len(PALETTE)]
        x1, y1, x2, y2 = map(int, det["xyxy"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cls_idx = det["cls"]
        label = class_names[cls_idx] if cls_idx < len(class_names) else "?"
        if det["track_id"] is not None:
            label = "{}#{}".format(label, det["track_id"])
        cv2.putText(
            frame,
            "{} {:.2f}".format(label, det["conf"]),
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
        )
        xy = det["keypoints"]
        valid = det["kpt_valid"]
        for a, b in bones:
            if not (valid[a] and valid[b]):
                continue
            cv2.line(
                frame,
                (int(xy[a][0]), int(xy[a][1])),
                (int(xy[b][0]), int(xy[b][1])),
                color,
                2,
            )
        for k in range(len(keypoint_names)):
            if not valid[k]:
                continue
            cv2.circle(
                frame, (int(xy[k][0]), int(xy[k][1])), 4, (255, 255, 255), -1
            )
    return frame


class DogPoseNode(object):
    def __init__(self):
        rospy.init_node("dog_pose")

        # Empty means "whatever the node ships with", so the launch
        # files do not have to repeat the URL.
        self.model_param = rospy.get_param("~model", "") or DEFAULT_MODEL_URL
        self.model_path = resolve_model_path(
            self.model_param,
            os.path.join(
                os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")),
                "dog_pose",
            ),
        )
        self.device = rospy.get_param("~device", "cpu")
        self.conf_thresh = float(rospy.get_param("~conf", 0.25))
        self.iou_thresh = float(rospy.get_param("~iou", 0.5))
        self.kpt_conf_thresh = float(rospy.get_param("~kpt_conf", 0.3))
        self.min_valid_keypoints = int(
            rospy.get_param("~min_valid_keypoints", 0)
        )
        self.track = bool(rospy.get_param("~track", True))
        self.tracker = rospy.get_param("~tracker", "bytetrack.yaml")
        self.with_depth = bool(rospy.get_param("~with_depth", False))
        self.approximate_sync = bool(
            rospy.get_param("~approximate_sync", True)
        )
        self.queue_size = int(rospy.get_param("~queue_size", 10))
        self.slop = float(rospy.get_param("~slop", 0.1))
        self.keypoint_names = (
            _split_csv(rospy.get_param("~keypoint_names", ""))
            or list(DOG_KEYPOINT_NAMES)
        )

        rospy.loginfo("Loading YOLO pose model: %s", self.model_path)
        self.model = YOLO(self.model_path)
        self.class_names = [
            self.model.names[i] for i in sorted(self.model.names)
        ]
        self._check_keypoint_shape()
        self.bones = self._build_bones()
        rospy.loginfo(
            "Model ready. classes=%s, %d keypoints, %d bones",
            self.class_names,
            len(self.keypoint_names),
            len(self.bones),
        )
        dead = [n for n in UNLABELLED_KEYPOINTS if n in self.keypoint_names]
        if dead:
            rospy.loginfo(
                "dog-pose never annotates %s, so a model trained on it leaves "
                "them at score 0 and they stay out of ~pose / ~skeleton.",
                ", ".join(dead),
            )

        self.bridge = CvBridge()
        self.rects_pub = rospy.Publisher("~rects", RectArray, queue_size=1)
        self.class_pub = rospy.Publisher(
            "~class", ClassificationResult, queue_size=1
        )
        self.pose_pub = rospy.Publisher("~pose", PeoplePoseArray, queue_size=1)
        self.skeleton_pub = rospy.Publisher(
            "~skeleton", HumanSkeletonArray, queue_size=1
        )
        self.image_pub = rospy.Publisher(
            "~image_annotated", Image, queue_size=1
        )

        self._subscribe()

    def _check_keypoint_shape(self):
        """Fail early when the weights do not match ``~keypoint_names``."""
        kpt_shape = getattr(self.model.model, "kpt_shape", None)
        if kpt_shape is None:
            raise RuntimeError(
                "{} has no kpt_shape: it is not a pose model. Train one on "
                "dog-pose.yaml and pass its best.pt as ~model.".format(
                    self.model_param
                )
            )
        if int(kpt_shape[0]) != len(self.keypoint_names):
            raise RuntimeError(
                "{} predicts {} keypoints but ~keypoint_names lists {}. Pass "
                "matching names, or use weights trained on dog-pose.yaml "
                "(24 keypoints).".format(
                    self.model_param, kpt_shape[0], len(self.keypoint_names)
                )
            )

    def _build_bones(self):
        """Map ``DOG_BONES`` onto ``~keypoint_names``, dropping unknown names.

        Returns
        -------
        list of tuple
            ``(index_a, index_b)`` pairs into the keypoint array.
        """
        index = {name: i for i, name in enumerate(self.keypoint_names)}
        bones = []
        for a, b in DOG_BONES:
            if a in index and b in index:
                bones.append((index[a], index[b]))
        if not bones:
            rospy.logwarn(
                "None of the dog bones match ~keypoint_names; ~skeleton will "
                "stay empty."
            )
        return bones

    def _subscribe(self):
        if not self.with_depth:
            self.sub = rospy.Subscriber(
                "image",
                Image,
                self.image_cb,
                queue_size=1,
                buff_size=2 ** 26,
            )
            rospy.loginfo(
                "Subscribed to 'image'. Keypoints are published in pixels "
                "with z=0; set ~with_depth for metric 3D."
            )
            return

        import message_filters

        subs = [
            message_filters.Subscriber(
                "image", Image, queue_size=1, buff_size=2 ** 26
            ),
            message_filters.Subscriber(
                "depth_image", Image, queue_size=1, buff_size=2 ** 26
            ),
            message_filters.Subscriber(
                "camera_info", CameraInfo, queue_size=1
            ),
        ]
        if self.approximate_sync:
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, queue_size=self.queue_size, slop=self.slop
            )
        else:
            sync = message_filters.TimeSynchronizer(
                subs, queue_size=self.queue_size
            )
        sync.registerCallback(self.image_depth_cb)
        self.sync = sync
        rospy.loginfo(
            "Subscribed to 'image' + 'depth_image' + 'camera_info'. Keypoints "
            "are published in metres in the camera optical frame."
        )

    def _infer(self, frame):
        if self.track:
            results = self.model.track(
                frame,
                device=self.device,
                conf=self.conf_thresh,
                iou=self.iou_thresh,
                tracker=self.tracker,
                persist=True,
                verbose=False,
            )
        else:
            results = self.model.predict(
                frame,
                device=self.device,
                conf=self.conf_thresh,
                iou=self.iou_thresh,
                verbose=False,
            )
        detections = extract_detections(results[0], self.kpt_conf_thresh)
        if self.min_valid_keypoints > 0:
            # A box with almost no keypoints on it is usually a false positive
            # rather than a badly-posed dog.
            detections = [
                d
                for d in detections
                if int(d["kpt_valid"].sum()) >= self.min_valid_keypoints
            ]
        return detections

    def publish_rects(self, header, detections, width, height):
        """Publish one rect and one label per detection.

        Both topics go out on every frame, empty included, so a subscriber can
        tell "no dog in this frame" apart from "no new message". The arrays
        share their ordering with ``~pose`` and ``~skeleton``.

        Parameters
        ----------
        header : std_msgs.msg.Header
            Header of the input image, copied onto both messages.
        detections : list of dict
            Detections from ``extract_detections``.
        width : int
            Input image width in pixels, used to clamp the boxes.
        height : int
            Input image height in pixels, used to clamp the boxes.
        """
        rects_msg = RectArray(header=header)
        class_msg = ClassificationResult(
            header=header,
            classifier=self.model_param,
            target_names=self.class_names,
        )
        for det in detections:
            x1 = int(round(min(max(det["xyxy"][0], 0.0), width)))
            y1 = int(round(min(max(det["xyxy"][1], 0.0), height)))
            x2 = int(round(min(max(det["xyxy"][2], 0.0), width)))
            y2 = int(round(min(max(det["xyxy"][3], 0.0), height)))
            rects_msg.rects.append(
                Rect(x=x1, y=y1, width=max(0, x2 - x1), height=max(0, y2 - y1))
            )
            class_msg.labels.append(det["cls"])
            class_msg.label_names.append(self.class_names[det["cls"]])
            class_msg.label_proba.append(det["conf"])
        self.rects_pub.publish(rects_msg)
        self.class_pub.publish(class_msg)

    def publish_pose(self, header, detections, points):
        """Publish the keypoints and the bones joining them.

        Parameters
        ----------
        header : std_msgs.msg.Header
            Header of the input image, copied onto both messages.
        detections : list of dict
            Detections from ``extract_detections``.
        points : list of dict
            Per detection, ``{keypoint_index: (x, y, z)}`` holding only the
            keypoints that survived the score and depth checks.
        """
        pose_array = PeoplePoseArray(header=header)
        skeleton_array = HumanSkeletonArray(header=header)
        for i, det in enumerate(detections):
            pose_msg = PeoplePose()
            for k in sorted(points[i]):
                x, y, z = points[i][k]
                pose_msg.limb_names.append(self.keypoint_names[k])
                pose_msg.scores.append(float(det["kpt_scores"][k]))
                pose_msg.poses.append(
                    Pose(
                        position=Point(x=x, y=y, z=z),
                        orientation=Quaternion(w=1),
                    )
                )
            pose_array.poses.append(pose_msg)

            skeleton_msg = HumanSkeleton(header=header)
            for a, b in self.bones:
                if a not in points[i] or b not in points[i]:
                    continue
                ax, ay, az = points[i][a]
                bx, by, bz = points[i][b]
                skeleton_msg.bone_names.append(
                    "{}->{}".format(
                        self.keypoint_names[a], self.keypoint_names[b]
                    )
                )
                skeleton_msg.bones.append(
                    Segment(
                        start_point=Point(x=ax, y=ay, z=az),
                        end_point=Point(x=bx, y=by, z=bz),
                    )
                )
            skeleton_array.skeletons.append(skeleton_msg)
            track_id = det["track_id"]
            skeleton_array.human_ids.append(
                Int32(data=i if track_id is None else track_id)
            )
        self.pose_pub.publish(pose_array)
        self.skeleton_pub.publish(skeleton_array)

    def publish_annotated(self, header, frame, detections):
        if self.image_pub.get_num_connections() == 0:
            return
        annotated = draw_detections(
            frame,
            detections,
            self.class_names,
            self.keypoint_names,
            self.bones,
        )
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except Exception as e:
            rospy.logwarn("publish error: %s", e)
            return
        out_msg.header = header
        self.image_pub.publish(out_msg)

    def _points_2d(self, detections):
        """Keypoints as image coordinates ``(u, v, 0)``."""
        points = []
        for det in detections:
            xy = det["keypoints"]
            points.append(
                {
                    k: (float(xy[k][0]), float(xy[k][1]), 0.0)
                    for k in range(len(self.keypoint_names))
                    if det["kpt_valid"][k]
                }
            )
        return points

    def _points_3d(self, detections, depth, camera_info):
        """Keypoints back-projected into the camera optical frame, in metres.

        Keypoints falling outside the depth image, or on a pixel with no depth
        reading, are dropped rather than guessed.

        Parameters
        ----------
        detections : list of dict
            Detections from ``extract_detections``.
        depth : numpy.ndarray
            Depth image aligned with the colour image, in metres (float) or
            millimetres (integer encodings).
        camera_info : sensor_msgs.msg.CameraInfo
            Intrinsics of the colour image.

        Returns
        -------
        list of dict
            Per detection, ``{keypoint_index: (x, y, z)}`` in metres.
        """
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        if fx == 0.0 or fy == 0.0:
            rospy.logwarn_throttle(
                10.0, "camera_info has a zero focal length; skipping 3D."
            )
            return [{} for _ in detections]
        scale = 1.0 if np.issubdtype(depth.dtype, np.floating) else 1e-3
        height, width = depth.shape[:2]

        points = []
        for det in detections:
            xy = det["keypoints"]
            per_dog = {}
            for k in range(len(self.keypoint_names)):
                if not det["kpt_valid"][k]:
                    continue
                u, v = int(round(xy[k][0])), int(round(xy[k][1]))
                if not (0 <= u < width and 0 <= v < height):
                    continue
                z = float(depth[v, u]) * scale
                if not np.isfinite(z) or z <= 0.0:
                    continue
                per_dog[k] = ((u - cx) * z / fx, (v - cy) * z / fy, z)
            points.append(per_dog)
        return points

    def image_cb(self, img_msg):
        frame = self._to_bgr(img_msg)
        if frame is None:
            return
        height, width = frame.shape[:2]
        detections = self._infer(frame)
        self.publish_rects(img_msg.header, detections, width, height)
        self.publish_pose(
            img_msg.header, detections, self._points_2d(detections)
        )
        self.publish_annotated(img_msg.header, frame, detections)

    def image_depth_cb(self, img_msg, depth_msg, info_msg):
        frame = self._to_bgr(img_msg)
        if frame is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg)
        except Exception as e:
            rospy.logwarn("cv_bridge error on depth_image: %s", e)
            return
        height, width = frame.shape[:2]
        detections = self._infer(frame)
        self.publish_rects(img_msg.header, detections, width, height)
        points = self._points_3d(detections, depth, info_msg)
        self.publish_pose(img_msg.header, detections, points)
        self.publish_annotated(img_msg.header, frame, detections)

    def _to_bgr(self, img_msg):
        try:
            return self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)
            return None


def main():
    DogPoseNode()
    rospy.spin()


if __name__ == "__main__":
    main()
