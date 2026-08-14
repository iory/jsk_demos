# balloon_detection

YOLO-WorldV2 で **「いま何が映っているか」** を検出して ROS トピックに流すノード。
検出したい物体は英語のテキストで指定するだけで、学習もモデル差し替えも要らない
（オープン語彙検出）。

## TL;DR

- **入力**は `sensor_msgs/Image` 1 本だけ。`image_topic:=` で実機カメラでも rosbag でも繋がる。
- **出力**は毎フレーム 2 本。`/detect_events/class` に「何が映ったか」（クラス名＋確信度）、
  `/detect_events/rects` に「どこに映ったか」（bbox）。並び順は i 番目どうしが対応。
- **検出対象**は launch 引数にプロンプトを渡すだけ。例: `balloon_classes:="red scarf"`。
- 確信度のしきい値がかなり低い（`~conf` = 0.05）ので、**購読側で `label_proba` を見て
  足切りする**のが前提。
- おまけとして「風船の受け渡し」を判定する状態機械が載っている（末尾の付録参照）。
  検出だけ使いたいなら無視して良い。

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

初回ビルドでパッケージ専用の virtualenv が作られ、`ultralytics`、`lap`、OpenAI の
`clip` が入る。torch を引くので数分かかる。

```bash
source /opt/ros/one/setup.bash
source ~/ros/balloon-detection/devel/setup.bash
```

## 何が映っているかを見る

検出したいものを英語で並べて起動する。

```bash
roslaunch balloon_detection detect_events.launch \
    image_topic:=/camera/color/image_raw \
    balloon_classes:="red scarf,red scarf held in hand" \
    teddy_classes:="person,chair"
```

映ったものはそのまま流れてくる。

```bash
rostopic echo /detect_events/class/label_names   # 何が映ったか
rostopic echo /detect_events/class/label_proba   # その確信度
rostopic echo /detect_events/rects               # どこに映ったか
```

引数が `balloon_classes` / `teddy_classes` の 2 つに分かれているのは付録の状態機械が
2 グループを必要とするため。**どちらに書いても `~class` / `~rects` には全部出る**ので、
検出だけが目的なら好きに振り分けて良い。ただし両方とも必須で、空にすると起動時に
エラーになる。

プロンプトは英語で、具体的な名詞句ほど当たりやすい。CSV で複数書いた場合、
同じ引数に書いたものは状態機械の上では同じグループ扱いになる。

## 出力トピック

| トピック | 型 | 中身 |
|---|---|---|
| `/detect_events/class` | `jsk_recognition_msgs/ClassificationResult` | 各検出のクラス名（`label_names`）と確信度（`label_proba`）。`target_names` に指定した全プロンプト |
| `/detect_events/rects` | `jsk_recognition_msgs/RectArray` | 各検出の bbox。画像サイズにクランプ済み |
| `/detect_events/image_annotated` | `sensor_msgs/Image` | 枠を描画した画像（目視確認用） |
| `/detect_events/state` | `std_msgs/String`（latched） | 付録の状態機械の現在状態 |
| `/detect_events/events` | `std_msgs/String` | 付録の状態機械のイベント（JSON 1 行） |

`~class` と `~rects` は **検出 0 件でも毎フレーム空で publish される**。
「何も映っていない」と「メッセージが来ていない」を区別できる。

header は入力画像のものをそのままコピーしているので、`message_filters` でそろう。

## 他のノードから使う

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
        if proba < 0.3:
            continue
        if name.startswith("red scarf"):
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

`Rect` は `x` / `y` / `width` / `height`（ピクセル）。
`TimeSynchronizer` は入力画像の stamp が正しく入っていることが前提。カメラや rosbag なら
問題ないが、stamp が 0 のソースを使う場合は片方だけ subscribe して自前で持ち回るほうが確実。

記録するなら上のトピックを `rosbag record` すれば良い。

## Launch ファイル

### `detect_events.launch` — 検出のみ

画像ソースが既にある場合（実機カメラ、rosbag、他の launch）はこちら。

| Arg | Default | Notes |
|---|---|---|
| `image_topic` | `image` | 入力 `sensor_msgs/Image` トピック。 |
| `model` | `yolov8x-worldv2.pt` | YOLO-World の重み。 |
| `device` | `cpu` | 例: `cuda:0`。 |
| `balloon_classes` | `balloon,red balloon` | 検出プロンプト（CSV）。 |
| `teddy_classes` | `teddy bear,stuffed animal,Winnie the Pooh plush` | 検出プロンプト（CSV）。 |
| `visualize` | `false` | `image_view` を開く。 |

### `video.launch` — 動画ファイルで試す

`video_stream_opencv` で mp4 を流し込んで `detect_events.launch` に繋ぐ。

```bash
roslaunch balloon_detection video.launch video:=IMG_8615.mp4
```

`pooh/` に置いた `*.mp4` を `video:=` でファイル名指定するか、`video_path:=` に
絶対パス（あるいは `/dev/video0`）を渡す。

| Arg | Default | Notes |
|---|---|---|
| `video` | `IMG_8615.mp4` | `pooh/` 以下のファイル名。 |
| `video_path` | `$(find balloon_detection)/pooh/$(arg video)` | 絶対パスで上書き。 |
| `publish_fps` | `30` | `video_stream_opencv` の publish レート。 |
| `loop` | `false` | 動画をループ再生。 |
| `visualize` | `true` | `image_view` を開く。 |

`model` / `device` / `balloon_classes` / `teddy_classes` は `detect_events.launch` と同じ。

## 付録: 風船の受け渡し検出

元々このパッケージは「風船がぬいぐるみに手渡されたか（`RECEIVED`）」「飛んでいったか
（`FLEW_AWAY`）」を判定するために書かれたもので、その状態機械が今も動いている。

```
NO_BALLOON -> APPROACHING -> HELD --3s 静止--> RECEIVED
                                \--見失う / 上方向--> FLEW_AWAY --2s--> FLEW_AWAY
```

`~state` に現在の状態名（`NO_BALLOON` / `APPROACHING` / `HELD` / `RELEASED` /
`FLEW_AWAY`）が、変化したときだけ latch 付きで流れる。`~events` には JSON が 1 行ずつ流れる。

```json
{"frame": 120, "time_sec": 4.0, "event": "RECEIVED", "note": "held for 3.0s, stable"}
{"frame": 88,  "time_sec": 2.9, "from": "APPROACHING", "to": "HELD"}
```

`event` キーがあるのが本命のイベントで、`from` / `to` は内部の状態遷移ログ。
判定は「`balloon_classes` 群と `teddy_classes` 群の距離・IoU」と「上方向に消えたか」で
決まる（`BalloonEventDetector.update`）ので、**クラスを別の物体に差し替えても
この意味は変わらない**。単に何が映っているか知りたいだけなら `~state` / `~events` は
無視して構わない。時間の計算は全て wall-clock（`rospy.Time`）なので、フレーム落ちには強い。

## License

MIT. See [LICENSE](LICENSE).
