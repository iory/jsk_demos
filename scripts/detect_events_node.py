"""ROS node for balloon hand-off / fly-away event detection.

Subscribes to a ``sensor_msgs/Image`` topic (the ``image`` name is meant to be
remapped from launch), runs YOLO-WorldV2 + ByteTrack + a wall-clock state
machine per frame, and publishes annotated images and event messages.
"""

import json
from collections import deque

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
import numpy as np  # noqa: E402
import rospy  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from ultralytics import YOLO  # noqa: E402

from img_utils import (  # noqa: E402
    compressed_imgmsg_to_cv2,
    cv2_to_imgmsg,
    imgmsg_to_cv2,
)

HELD_IOU_THRESH = 0.005
VEL_WINDOW_SECS = 0.27
HOLD_SECS_FOR_HELD = 1.0
RECEIVED_HOLD_SECS = 3.0
RELEASE_SECS = 0.5
LOST_SECS_FOR_FLEW = 1.0
FLEW_AWAY_DWELL_SECS = 2.0
EDGE_MARGIN_RATIO = 0.10
STABILITY_WINDOW_SECS = 3.0
STABILITY_RADIUS_RATIO = 0.10
INITIAL_RELATION_SECS = 1.0
HELD_DIST_MARGIN_RATIO = 0.08
STATE_COLORS = {
    "NO_BALLOON": (128, 128, 128),
    "APPROACHING": (0, 165, 255),
    "HELD": (0, 255, 0),
    "RELEASED": (0, 200, 255),
    "FLEW_AWAY": (0, 0, 255),
}


def iou_xyxy(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (area_a + area_b - inter)


def pick_top(boxes_cls, boxes_conf, boxes_xyxy, allowed_idx):
    best_conf = 0.0
    best_box = None
    for cls_i, conf_i, xyxy_i in zip(boxes_cls, boxes_conf, boxes_xyxy):
        if int(cls_i) in allowed_idx and conf_i > best_conf:
            best_conf = float(conf_i)
            best_box = list(map(float, xyxy_i))
    return best_box, best_conf


def extract_boxes(result, balloon_idx, teddy_idx):
    boxes = result.boxes
    balloon_box = balloon_conf = None
    teddy_box = teddy_conf = None
    if boxes is not None and len(boxes) > 0:
        cls = boxes.cls.cpu().numpy().astype(int)
        confs = boxes.conf.cpu().numpy()
        xyxy = boxes.xyxy.cpu().numpy()
        balloon_box, balloon_conf = pick_top(cls, confs, xyxy, balloon_idx)
        teddy_box, teddy_conf = pick_top(cls, confs, xyxy, teddy_idx)
    return balloon_box, balloon_conf, teddy_box, teddy_conf


class BalloonEventDetector:
    """Wall-clock state machine for balloon hand-off / fly-away events.

    All counters accumulate elapsed time (seconds) between successive
    ``update()`` calls, so the state machine is robust to dropped frames or
    variable rates. The caller supplies ``t`` (monotonic seconds).
    """

    def __init__(self, width, height):
        self.w = width
        self.h = height
        diag = (width * width + height * height) ** 0.5
        self.held_dist_thresh = 0.20 * diag
        self.vy_high = 0.10 * diag
        self.edge_margin = EDGE_MARGIN_RATIO * height
        self.stability_radius = STABILITY_RADIUS_RATIO * min(width, height)

        self.state = "NO_BALLOON"
        self.held_secs = 0.0
        self.released_secs = 0.0
        self.lost_secs = 0.0
        self.held_total_secs = 0.0
        self.flew_away_secs = 0.0
        self.received_emitted = False
        self.flew_away_emitted = False
        self.balloon_history = deque()
        self.stability_history = deque()
        self.last_balloon_xy = None
        self.last_t = None
        self.events = []
        self.initial_dist_samples = []
        self.initial_dist = None
        self.initial_relation_done = False
        self.held_dist_margin = HELD_DIST_MARGIN_RATIO * diag

    def update(self, frame_idx, t, balloon_box, teddy_box):
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t

        cx_b = cy_b = None
        vx = vy = speed = 0.0
        if balloon_box is not None:
            cx_b = (balloon_box[0] + balloon_box[2]) / 2
            cy_b = (balloon_box[1] + balloon_box[3]) / 2
            self.balloon_history.append((t, cx_b, cy_b))
            while self.balloon_history and t - self.balloon_history[0][0] > VEL_WINDOW_SECS:
                self.balloon_history.popleft()
            if len(self.balloon_history) >= 3:
                t0, x0, y0 = self.balloon_history[0]
                t1, x1, y1 = self.balloon_history[-1]
                dt_v = t1 - t0
                if dt_v > 0:
                    vx = (x1 - x0) / dt_v
                    vy = (y1 - y0) / dt_v
                    speed = (vx * vx + vy * vy) ** 0.5

        dist = float("inf")
        iou = 0.0
        if balloon_box is not None and teddy_box is not None:
            cx_t = (teddy_box[0] + teddy_box[2]) / 2
            cy_t = (teddy_box[1] + teddy_box[3]) / 2
            dist = ((cx_b - cx_t) ** 2 + (cy_b - cy_t) ** 2) ** 0.5
            iou = iou_xyxy(balloon_box, teddy_box)

        balloon_visible = balloon_box is not None

        # record distance between teddy and balloon first
        if balloon_box is not None and teddy_box is not None and not self.initial_relation_done:
            self.initial_dist_samples.append(dist)
            if t >= INITIAL_RELATION_SECS and self.initial_dist_samples:
                self.initial_dist = float(np.median(self.initial_dist_samples))
                self.initial_relation_done = True
                self.events.append(
                    {
                        "frame": frame_idx,
                        "time_sec": round(t, 2),
                        "event": "INITIAL_RELATION_SET",
                        "note": f"initial_dist={self.initial_dist:.1f}, margin={self.held_dist_margin:.1f}",
                    }
                )

        # judge if teddy is holding a balloon by comparing with initial distance
        if self.initial_dist is not None:
            balloon_close = dist <= self.initial_dist + self.held_dist_margin
        else:
            # if there is no initial distance, use threshold
            balloon_close = (iou > HELD_IOU_THRESH) or (dist < self.held_dist_thresh)

        prev_state = self.state

        if balloon_visible:
            self.lost_secs = 0.0
            self.stability_history.append((t, cx_b, cy_b))
            while (
                self.stability_history
                and t - self.stability_history[0][0] > STABILITY_WINDOW_SECS
            ):
                self.stability_history.popleft()
        else:
            self.lost_secs += dt

        was_in_top_half = (
            self.last_balloon_xy is not None and self.last_balloon_xy[1] < self.h * 0.5
        )

        is_stable = False
        if self.stability_history:
            span = t - self.stability_history[0][0]
            if span >= STABILITY_WINDOW_SECS / 2:
                xs = np.array([p[1] for p in self.stability_history])
                ys = np.array([p[2] for p in self.stability_history])
                spread = max(xs.max() - xs.min(), ys.max() - ys.min())
                is_stable = spread < self.stability_radius * 2

        if self.state == "NO_BALLOON":
            if balloon_visible:
                self.state = "APPROACHING"
                self.held_secs = dt if balloon_close else 0.0
        elif self.state == "APPROACHING":
            if not balloon_visible:
                if self.lost_secs >= LOST_SECS_FOR_FLEW:
                    self.state = "FLEW_AWAY" if was_in_top_half else "NO_BALLOON"
                    self.held_secs = 0.0
            elif balloon_close:
                self.held_secs += dt
                if self.held_secs >= HOLD_SECS_FOR_HELD:
                    self.state = "HELD"
                    self.held_total_secs = self.held_secs
                    self.received_emitted = False
            else:
                self.held_secs = max(0.0, self.held_secs - dt)
        elif self.state == "HELD":
            self.held_total_secs += dt
            if (
                not self.received_emitted
                and self.held_total_secs >= RECEIVED_HOLD_SECS
                and is_stable
            ):
                self.events.append(
                    {
                        "frame": frame_idx,
                        "time_sec": round(t, 2),
                        "event": "RECEIVED",
                        "note": f"held for {self.held_total_secs:.1f}s, stable",
                    }
                )
                self.received_emitted = True
            if not balloon_visible:
                if self.lost_secs >= LOST_SECS_FOR_FLEW:
                    self.state = "FLEW_AWAY" if was_in_top_half else "RELEASED"
                    self.held_secs = 0.0
                    self.released_secs = 0.0
            elif not balloon_close:
                self.released_secs += dt
                if self.released_secs >= RELEASE_SECS:
                    fly_signal = vy < -self.vy_high or (
                        cy_b is not None and cy_b < self.edge_margin * 2
                    )
                    self.state = "FLEW_AWAY" if fly_signal else "RELEASED"
                    self.released_secs = 0.0
            else:
                self.released_secs = max(0.0, self.released_secs - dt)
        elif self.state == "RELEASED":
            if not balloon_visible:
                if self.lost_secs >= LOST_SECS_FOR_FLEW:
                    self.state = "FLEW_AWAY" if was_in_top_half else "NO_BALLOON"
                    self.held_total_secs = 0.0
            elif balloon_close:
                self.released_secs = max(0.0, self.released_secs - dt)
                if self.released_secs <= 0.0:
                    self.state = "HELD"
                    self.held_secs = HOLD_SECS_FOR_HELD
            elif vy < -self.vy_high and cy_b is not None and cy_b < self.edge_margin * 4:
                self.state = "FLEW_AWAY"
                self.held_total_secs = 0.0
        elif self.state == "FLEW_AWAY":
            # if balloon_visible and balloon_close:
            #     self.state = "HELD"
            #     self.held_secs = HOLD_SECS_FOR_HELD
            #     self.held_total_secs = HOLD_SECS_FOR_HELD
            #     self.received_emitted = False
            if balloon_visible:
                self.state = "APPROACHING"
                self.held_secs = 0.0

        if self.state != prev_state:
            self.events.append(
                {
                    "frame": frame_idx,
                    "time_sec": round(t, 2),
                    "from": prev_state,
                    "to": self.state,
                }
            )

        if self.state == "FLEW_AWAY":
            self.flew_away_secs += dt
            if (
                not self.flew_away_emitted
                and self.flew_away_secs >= FLEW_AWAY_DWELL_SECS
            ):
                self.events.append(
                    {
                        "frame": frame_idx,
                        "time_sec": round(t, 2),
                        "event": "FLEW_AWAY",
                        "note": f"balloon absent for {self.flew_away_secs:.1f}s",
                    }
                )
                self.flew_away_emitted = True
        else:
            self.flew_away_secs = 0.0
            if self.state in ("HELD", "APPROACHING"):
                self.flew_away_emitted = False

        if balloon_box is not None:
            self.last_balloon_xy = (cx_b, cy_b)

        return {
            "cx_b": cx_b,
            "cy_b": cy_b,
            "vx": vx,
            "vy": vy,
            "speed": speed,
            "dist": dist,
            "iou": iou,
            "is_stable": is_stable,
        }

    def annotate(self, frame, balloon_box, balloon_conf, teddy_box, teddy_conf, info):
        cx_b = info["cx_b"]
        cy_b = info["cy_b"]
        vx = info["vx"]
        vy = info["vy"]
        speed = info["speed"]
        dist = info["dist"]
        iou = info["iou"]
        is_stable = info["is_stable"]

        if teddy_box is not None:
            x1, y1, x2, y2 = map(int, teddy_box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
            cv2.putText(
                frame,
                f"teddy {teddy_conf:.2f}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )
        if balloon_box is not None:
            x1, y1, x2, y2 = map(int, balloon_box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.putText(
                frame,
                f"balloon {balloon_conf:.2f}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 255),
                2,
            )
            if teddy_box is not None:
                cv2.line(
                    frame,
                    (int(cx_b), int(cy_b)),
                    (int((teddy_box[0] + teddy_box[2]) / 2),
                     int((teddy_box[1] + teddy_box[3]) / 2)),
                    (255, 255, 255),
                    1,
                )
            if speed > 1:
                end_x = int(cx_b + vx * 0.4)
                end_y = int(cy_b + vy * 0.4)
                cv2.arrowedLine(
                    frame, (int(cx_b), int(cy_b)), (end_x, end_y),
                    (255, 255, 0), 3, tipLength=0.3,
                )

        color = STATE_COLORS.get(self.state, (255, 255, 255))
        cv2.rectangle(frame, (10, 10), (700, 130), (0, 0, 0), -1)
        cv2.putText(
            frame, f"STATE: {self.state}", (20, 55),
            cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 3,
        )
        cv2.putText(
            frame,
            f"dist={dist:.0f}  iou={iou:.3f}  vy={vy:+.0f}px/s",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
        )
        cv2.putText(
            frame,
            f"held={self.held_total_secs:.1f}s  lost={self.lost_secs:.1f}s  stable={int(is_stable)}",
            (20, 115),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2,
        )
        if self.received_emitted:
            cv2.putText(
                frame, "RECEIVED", (20, self.h - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 4,
            )
        return frame


def _split_csv(s):
    return [t.strip() for t in s.split(",") if t.strip()]


class DetectEventsNode:
    def __init__(self):
        rospy.init_node("detect_events")

        self.model_path = rospy.get_param("~model", "yolov8x-worldv2.pt")
        self.device = rospy.get_param("~device", "cpu")
        self.conf_thresh = float(rospy.get_param("~conf", 0.05))
        self.iou_thresh = float(rospy.get_param("~iou", 0.5))
        self.use_header_stamp = bool(rospy.get_param("~use_header_stamp", True))
        balloon_prompts = _split_csv(
            rospy.get_param("~balloon_classes", "balloon,red balloon")
        )
        teddy_prompts = _split_csv(
            rospy.get_param(
                "~teddy_classes",
                "teddy bear,stuffed animal,Winnie the Pooh plush",
            )
        )
        if not balloon_prompts or not teddy_prompts:
            raise RuntimeError(
                "~balloon_classes and ~teddy_classes must be non-empty CSV lists."
            )
        classes = balloon_prompts + teddy_prompts
        self.balloon_idx = set(range(len(balloon_prompts)))
        self.teddy_idx = set(
            range(len(balloon_prompts), len(balloon_prompts) + len(teddy_prompts))
        )

        rospy.loginfo("Loading YOLO model: %s", self.model_path)
        self.model = YOLO(self.model_path)
        self.model.set_classes(classes)
        rospy.loginfo(
            "Model ready. balloon=%s teddy=%s", balloon_prompts, teddy_prompts
        )

        self.compressed = bool(rospy.get_param("~compressed", False))
        self.detector = None
        self.frame_idx = 0
        self.last_event_count = 0
        self.t_start = None

        self.image_pub = rospy.Publisher("~image_annotated", Image, queue_size=1)
        self.event_pub = rospy.Publisher("~events", String, queue_size=10)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.last_state_published = None

        if self.compressed:
            self.sub = rospy.Subscriber(
                "image", CompressedImage, self.image_cb,
                queue_size=1, buff_size=2 ** 26,
            )
            rospy.loginfo("Subscribed to CompressedImage (remap 'image' from launch).")
        else:
            self.sub = rospy.Subscriber(
                "image", Image, self.image_cb,
                queue_size=1, buff_size=2 ** 26,
            )
            rospy.loginfo("Subscribed to Image (remap 'image' from launch).")

    def image_cb(self, msg):
        try:
            if self.compressed:
                frame = compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                frame = imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("image decode error: %s", e)
            return
        h, w = frame.shape[:2]

        stamp = msg.header.stamp if self.use_header_stamp else rospy.Time(0)
        if stamp.to_sec() <= 0.0:
            stamp = rospy.Time.now()
        now = stamp.to_sec()

        if self.detector is None:
            rospy.loginfo("First frame received (%dx%d)", w, h)
            self.detector = BalloonEventDetector(w, h)
            self.t_start = now
        t = now - self.t_start

        results = self.model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            device=self.device,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            verbose=False,
        )
        r = results[0]
        balloon_box, balloon_conf, teddy_box, teddy_conf = extract_boxes(
            r, self.balloon_idx, self.teddy_idx
        )
        info = self.detector.update(self.frame_idx, t, balloon_box, teddy_box)
        annotated = self.detector.annotate(
            frame, balloon_box, balloon_conf, teddy_box, teddy_conf, info
        )

        if self.detector.state != self.last_state_published:
            self.state_pub.publish(self.detector.state)
            self.last_state_published = self.detector.state

        new_events = self.detector.events[self.last_event_count:]
        for e in new_events:
            self.event_pub.publish(json.dumps(e, ensure_ascii=False))
            if "event" in e:
                rospy.loginfo(
                    "*** %s @ %.2fs: %s", e["event"], e["time_sec"], e.get("note", "")
                )
            else:
                rospy.loginfo(
                    "transition @ %.2fs: %s -> %s",
                    e["time_sec"], e["from"], e["to"],
                )
        self.last_event_count = len(self.detector.events)

        try:
            out_msg = cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except Exception as e:
            rospy.logwarn("publish error: %s", e)

        self.frame_idx += 1


def main():
    DetectEventsNode()
    rospy.spin()


if __name__ == "__main__":
    main()
