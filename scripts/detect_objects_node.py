"""ROS node for open-vocabulary object detection.

Subscribes to a ``sensor_msgs/Image`` topic (the ``image`` name is meant to be
remapped from launch), runs YOLO-WorldV2 with the text prompts given in
``~classes``, and publishes every detection of the frame as rects plus class
labels.
"""

import clip as _clip

_orig_clip_load = _clip.load


def _clip_load_no_jit(name, device="cpu", jit=False):
    return _orig_clip_load(name, device=device, jit=False)


def _clip_tokenize_compat(texts, context_length=77, truncate=False):
    import torch as _torch

    if isinstance(texts, str):
        texts = [texts]
    tokenizer = _clip.simple_tokenizer.SimpleTokenizer()
    sot = tokenizer.encoder["<|startoftext|>"]
    eot = tokenizer.encoder["<|endoftext|>"]
    all_tokens = [[sot] + tokenizer.encode(t) + [eot] for t in texts]
    result = _torch.zeros(len(all_tokens), context_length, dtype=_torch.long)
    for i, tokens in enumerate(all_tokens):
        if len(tokens) > context_length:
            if truncate:
                tokens = tokens[:context_length]
                tokens[-1] = eot
            else:
                raise RuntimeError(f"Text too long: {texts[i]}")
        result[i, : len(tokens)] = _torch.tensor(tokens)
    return result


_clip.load = _clip_load_no_jit
_clip.tokenize = _clip_tokenize_compat

import cv2  # noqa: E402
import rospy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from jsk_recognition_msgs.msg import ClassificationResult  # noqa: E402
from jsk_recognition_msgs.msg import Rect  # noqa: E402
from jsk_recognition_msgs.msg import RectArray  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from ultralytics import YOLO  # noqa: E402

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


def extract_all_boxes(result):
    """Collect every detection of a YOLO result.

    Parameters
    ----------
    result : ultralytics.engine.results.Results
        Single-image result returned by ``YOLO.predict``.

    Returns
    -------
    list of tuple
        One ``(xyxy, class_index, confidence)`` tuple per detection, where
        ``xyxy`` is a list of four floats in pixels. Empty when the frame
        contains no detection.
    """
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    cls = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()
    xyxy = boxes.xyxy.cpu().numpy()
    return [
        (list(map(float, box)), int(cls_i), float(conf_i))
        for box, cls_i, conf_i in zip(xyxy, cls, confs)
    ]


def draw_detections(frame, detections, class_names):
    """Draw one box and label per detection, in place.

    Parameters
    ----------
    frame : numpy.ndarray
        BGR image to draw on.
    detections : list of tuple
        ``(xyxy, class_index, confidence)`` tuples from ``extract_all_boxes``.
    class_names : list of str
        Prompt of each class index.

    Returns
    -------
    numpy.ndarray
        The same array, annotated.
    """
    for xyxy, cls_idx, conf in detections:
        color = PALETTE[cls_idx % len(PALETTE)]
        x1, y1, x2, y2 = map(int, xyxy)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.putText(
            frame,
            f"{class_names[cls_idx]} {conf:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
        )
    return frame


def _split_csv(s):
    return [t.strip() for t in s.split(",") if t.strip()]


class DetectObjectsNode:
    def __init__(self):
        rospy.init_node("detect_objects")

        self.model_path = rospy.get_param("~model", "yolov8x-worldv2.pt")
        self.device = rospy.get_param("~device", "cpu")
        self.conf_thresh = float(rospy.get_param("~conf", 0.25))
        self.iou_thresh = float(rospy.get_param("~iou", 0.5))
        self.classes = _split_csv(rospy.get_param("~classes", ""))
        if not self.classes:
            raise RuntimeError(
                "~classes is empty. Give a comma-separated list of YOLO-World "
                "prompts, e.g. classes:=\"red scarf,person\"."
            )

        rospy.loginfo("Loading YOLO model: %s", self.model_path)
        self.model = YOLO(self.model_path)
        self.model.set_classes(self.classes)
        rospy.loginfo("Model ready. classes=%s", self.classes)

        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("~image_annotated", Image, queue_size=1)
        self.rects_pub = rospy.Publisher("~rects", RectArray, queue_size=1)
        self.class_pub = rospy.Publisher(
            "~class", ClassificationResult, queue_size=1
        )

        self.sub = rospy.Subscriber(
            "image", Image, self.image_cb, queue_size=1, buff_size=2 ** 26,
        )
        rospy.loginfo("Subscribed (remap 'image' from launch).")

    def publish_detections(self, header, detections, width, height):
        """Publish every detection of one frame as rects plus class labels.

        ``~rects`` and ``~class`` are published on every frame, empty included,
        so a subscriber can tell "nothing detected" apart from "no new message".
        The two arrays share their ordering: the i-th rect is described by the
        i-th entry of ``label_names`` / ``label_proba``.

        Parameters
        ----------
        header : std_msgs.msg.Header
            Header of the input image, copied onto both messages.
        detections : list of tuple
            ``(xyxy, class_index, confidence)`` tuples from
            ``extract_all_boxes``.
        width : int
            Input image width in pixels, used to clamp the boxes.
        height : int
            Input image height in pixels, used to clamp the boxes.
        """
        rects_msg = RectArray(header=header)
        class_msg = ClassificationResult(
            header=header,
            classifier=self.model_path,
            target_names=self.classes,
        )
        for xyxy, cls_idx, conf in detections:
            x1 = int(round(min(max(xyxy[0], 0.0), width)))
            y1 = int(round(min(max(xyxy[1], 0.0), height)))
            x2 = int(round(min(max(xyxy[2], 0.0), width)))
            y2 = int(round(min(max(xyxy[3], 0.0), height)))
            rects_msg.rects.append(
                Rect(x=x1, y=y1, width=max(0, x2 - x1), height=max(0, y2 - y1))
            )
            class_msg.labels.append(cls_idx)
            class_msg.label_names.append(self.classes[cls_idx])
            class_msg.label_proba.append(conf)
        self.rects_pub.publish(rects_msg)
        self.class_pub.publish(class_msg)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)
            return
        h, w = frame.shape[:2]

        results = self.model.predict(
            frame,
            device=self.device,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            verbose=False,
        )
        detections = extract_all_boxes(results[0])
        self.publish_detections(msg.header, detections, w, h)

        if self.image_pub.get_num_connections() == 0:
            return
        annotated = draw_detections(frame, detections, self.classes)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except Exception as e:
            rospy.logwarn("publish error: %s", e)


def main():
    DetectObjectsNode()
    rospy.spin()


if __name__ == "__main__":
    main()
