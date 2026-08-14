# balloon_detection

Detect balloon hand-off (`RECEIVED`) and fly-away (`FLEW_AWAY`) events from
video using YOLO-WorldV2 + ByteTrack + a wall-clock state machine, exposed
as a ROS 1 node that subscribes to `sensor_msgs/Image`.

## TL;DR（日本語）

- **入力**は `sensor_msgs/Image` 1 本だけ。`image_topic:=` で実機カメラでも rosbag でも繋がる。
- **出力**は 3 トピック。他のプログラムから使うなら `/detect_events/events`（`std_msgs/String`、中身は JSON 1 行）を subscribe する。
- **検出したい物体は launch 引数で変えられる**。YOLO-WorldV2 のオープン語彙なので、`balloon_classes:="red scarf"` のように英語の単語を渡すだけ。学習は不要。
- **位置が欲しいなら** `/detect_events/rects` と `/detect_events/class` を見る。毎フレーム、検出した全 box とクラス名・確信度が流れる。

## Setup

Tested on ROS One (`/opt/ros/one`).

```bash
sudo apt install \
    ros-one-catkin-virtualenv \
    ros-one-video-stream-opencv \
    ros-one-cv-bridge \
    ros-one-image-view \
    ros-one-jsk-recognition-msgs
```

Clone into a catkin workspace and build:

```bash
mkdir -p ~/ros/balloon-detection/src
cd ~/ros/balloon-detection/src
git clone --single-branch https://github.com/iory/jsk_demos -b balloon_detection

cd ~/ros/balloon-detection
source /opt/ros/one/setup.bash
catkin build
```

The first build creates a per-package virtualenv and installs `ultralytics`,
`lap`, and the OpenAI `clip` module into it. This pulls torch and takes a few
minutes.

## Run

```bash
source /opt/ros/one/setup.bash
source ~/ros/balloon-detection/devel/setup.bash
```

Two launch files are provided.

### `video.launch` — sample-video demo

Plays an mp4 through `video_stream_opencv` and feeds it into the detector.

```bash
roslaunch balloon_detection video.launch video:=IMG_8615.mp4
```

Drop your own `*.mp4` files into `pooh/` and pass the basename via `video:=`,
or specify an arbitrary path (or `/dev/video0`) via `video_path:=`.

| Arg | Default | Notes |
|---|---|---|
| `video` | `IMG_8615.mp4` | File name under `pooh/`. |
| `video_path` | `$(find balloon_detection)/pooh/$(arg video)` | Override with absolute path. |
| `publish_fps` | `30` | `video_stream_opencv` publish rate. |
| `loop` | `false` | Loop the video file. |
| `visualize` | `true` | Open `image_view` on the annotated output. |
| `device` | `cpu` | e.g. `cuda:0`. |
| `balloon_classes` | `balloon,red balloon` | CSV YOLO-World prompts for the balloon. |
| `teddy_classes` | `teddy bear,stuffed animal,Winnie the Pooh plush` | CSV prompts for the receiver. |

### `detect_events.launch` — detection only

Use this when you already have an image source (a real camera, a rosbag,
another launch). Pass the topic via `image_topic:=`:

```bash
roslaunch balloon_detection detect_events.launch \
    image_topic:=/my_camera/color/image_raw visualize:=true
```

Accepts the same `balloon_classes`, `teddy_classes`, `device`, and `model`
args as `video.launch`.

## Outputs

Topics:

- `/detect_events/image_annotated` (`sensor_msgs/Image`) — annotated frames
- `/detect_events/state` (`std_msgs/String`, latched) — current state name (one of `NO_BALLOON`, `APPROACHING`, `HELD`, `RELEASED`, `FLEW_AWAY`), republished only when it changes
- `/detect_events/events` (`std_msgs/String`) — one JSON line per event (state transitions and `RECEIVED`/`FLEW_AWAY` triggers)
- `/detect_events/rects` (`jsk_recognition_msgs/RectArray`) — every detection of the frame, published on every frame (empty included)
- `/detect_events/class` (`jsk_recognition_msgs/ClassificationResult`) — class name and confidence for each rect, in the same order

To record the annotated stream or event log, use `rosbag record` on the
topics above.

## 検出結果を他のプログラムから使う（日本語）

### 何が subscribe できるか

| トピック | 型 | 中身 |
|---|---|---|
| `/detect_events/events` | `std_msgs/String` | イベント 1 件につき JSON 1 行 |
| `/detect_events/state` | `std_msgs/String`（latched） | 現在の状態名。変化したときだけ流れる |
| `/detect_events/rects` | `jsk_recognition_msgs/RectArray` | そのフレームの全検出 box。毎フレーム publish（0 件でも空で流れる） |
| `/detect_events/class` | `jsk_recognition_msgs/ClassificationResult` | 各 box のクラス名と確信度。`~rects` と同じ並び順 |
| `/detect_events/image_annotated` | `sensor_msgs/Image` | 枠と状態を描画した画像（デバッグ用） |

`~events` に流れる JSON は 2 種類ある。

```json
{"frame": 120, "time_sec": 4.0, "event": "RECEIVED", "note": "held for 3.0s, stable"}
{"frame": 88,  "time_sec": 2.9, "from": "APPROACHING", "to": "HELD"}
```

`event` キーがあるのが本命のイベント（`RECEIVED` / `FLEW_AWAY`）で、`from` / `to` は
内部の状態遷移ログ。使う側はまず `event` キーの有無で振り分ければ良い。

### 最小の subscriber

```python
import json

import rospy
from std_msgs.msg import String


def event_cb(msg):
    """Handle one event line from ``/detect_events/events``."""
    e = json.loads(msg.data)
    if e.get("event") == "RECEIVED":
        rospy.loginfo("受け取った: %s", e["note"])
    elif e.get("event") == "FLEW_AWAY":
        rospy.loginfo("飛んでいった: %s", e["note"])


rospy.init_node("my_consumer")
rospy.Subscriber("/detect_events/events", String, event_cb)
rospy.spin()
```

`/detect_events/state` は latch されているので、後から起動したノードでも
「いまどの状態か」を購読した瞬間に 1 回受け取れる。ポーリングは不要。

コマンドラインで中身を見るだけなら:

```bash
rostopic echo /detect_events/events
rostopic echo /detect_events/state
```

### 位置（bbox）が欲しいとき

`~rects` と `~class` は i 番目どうしが対応している。header は入力画像のものを
そのままコピーしているので、`message_filters` でそろえられる。

```python
import message_filters
import rospy
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray


def cb(rects_msg, class_msg):
    """Handle one frame of detections."""
    for rect, name, proba in zip(
        rects_msg.rects, class_msg.label_names, class_msg.label_proba
    ):
        if name.startswith("red scarf") and proba > 0.3:
            rospy.loginfo(
                "スカーフ: (%d, %d) %dx%d p=%.2f",
                rect.x, rect.y, rect.width, rect.height, proba,
            )


rospy.init_node("scarf_watcher")
sub_rects = message_filters.Subscriber("/detect_events/rects", RectArray)
sub_class = message_filters.Subscriber("/detect_events/class", ClassificationResult)
message_filters.TimeSynchronizer([sub_rects, sub_class], 10).registerCallback(cb)
rospy.spin()
```

box は画像サイズにクランプ済みで、`Rect` は `x` / `y` / `width` / `height`（ピクセル）。
`class_msg.target_names` に `balloon_classes` + `teddy_classes` の全プロンプトが入るので、
どのプロンプトに当たったかは `label_names` を見れば分かる。

`TimeSynchronizer` は入力画像の stamp が正しく入っていることが前提。
rosbag やカメラなら問題ないが、stamp が 0 のソースを使う場合は片方だけ
subscribe して自前で持ち回るほうが確実。

### 風船以外のものを検出したいとき（例: 赤いスカーフ）

YOLO-WorldV2 はテキストプロンプトで検出対象を指定するので、launch 引数を
差し替えるだけで別の物体に向けられる。再学習もモデル差し替えも要らない。

```bash
roslaunch balloon_detection detect_events.launch \
    image_topic:=/camera/color/image_raw \
    balloon_classes:="red scarf,red scarf held in hand" \
    teddy_classes:="person,teddy bear"
```

プロンプトは英語で、具体的な名詞句ほど当たりやすい。CSV で複数書くと
いずれかに当たれば同じクラス扱いになる。

### 注意: 状態機械は風船の受け渡し専用

クラスを差し替えても、`~state` / `~events` の意味までは変わらない。
`HELD` / `FLEW_AWAY` は「1 つ目のクラス群と 2 つ目のクラス群の距離・IoU」と
「上方向に消えたか」で決まる（`BalloonEventDetector.update`）。

単に「赤いスカーフが映っているか」を知りたいだけなら、状態機械は無視して
`~rects` / `~class` だけを subscribe すれば良い。`APPROACHING` を検出フラグ
代わりに使うのは筋が悪い。

## States and events

```
NO_BALLOON -> APPROACHING -> HELD --3s stable--> RECEIVED
                                \--lost / upward--> FLEW_AWAY --2s--> FLEW_AWAY
```

`RECEIVED` and `FLEW_AWAY` are the two events emitted on `~events`; everything
else is an internal state transition (also logged for debugging). All timing
is wall-clock (`rospy.Time`), so the state machine is robust to dropped frames.

## License

MIT. See [LICENSE](LICENSE).
