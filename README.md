# balloon_detection

YOLO-WorldV2 で **「いま何が映っているか」** を検出して ROS トピックに流すノード。
検出したいものを英語のテキストで指定するだけで、学習もモデル差し替えも要らない
（オープン語彙検出）。

## TL;DR

- **入力**は `sensor_msgs/Image` 1 本だけ。`image_topic:=` で実機カメラでも rosbag でも繋がる。
- **検出対象**は `classes:=` に英語のプロンプトを CSV で渡すだけ。例: `classes:="red scarf,person"`。
- **出力**は毎フレーム 2 本。`/detect_objects/class` に「何が映ったか」（クラス名＋確信度）、
  `/detect_objects/rects` に「どこに映ったか」（bbox）。i 番目どうしが対応。
- 出す/出さないは `conf:=`（デフォルト 0.25）で調整。取りこぼすなら下げる。

```bash
roslaunch balloon_detection detect_objects.launch \
    image_topic:=/camera/color/image_raw \
    classes:="red scarf,person"

rostopic echo /detect_objects/class/label_names   # 何が映ったか
```

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

初回ビルドでパッケージ専用の virtualenv が作られ、`ultralytics` と OpenAI の `clip`
が入る。torch を引くので数分かかる。

```bash
source /opt/ros/one/setup.bash
source ~/ros/balloon-detection/devel/setup.bash
```

## 何が映っているかを見る

```bash
roslaunch balloon_detection detect_objects.launch \
    image_topic:=/camera/color/image_raw \
    classes:="red scarf,person,chair" \
    visualize:=true
```

映ったものはそのまま流れてくる。

```bash
rostopic echo /detect_objects/class/label_names   # 何が映ったか
rostopic echo /detect_objects/class/label_proba   # その確信度
rostopic echo /detect_objects/rects               # どこに映ったか
```

プロンプトは英語で、具体的な名詞句ほど当たりやすい（`scarf` より
`red scarf held in hand` のほうが効くことがある）。同じものに対して言い方を変えた
プロンプトを複数並べるのも有効で、`label_names` にはヒットしたプロンプトが
そのまま入る。

`conf` のデフォルトは 0.25。何も出てこないときは `conf:=0.05` あたりまで下げると
拾うようになるが、そのぶん誤検出も増える。購読側で `label_proba` を見て
足切りするのが確実。

## 出力トピック

| トピック | 型 | 中身 |
|---|---|---|
| `/detect_objects/class` | `jsk_recognition_msgs/ClassificationResult` | 各検出のクラス名（`label_names`）と確信度（`label_proba`）。`target_names` に `classes` の全プロンプト |
| `/detect_objects/rects` | `jsk_recognition_msgs/RectArray` | 各検出の bbox。画像サイズにクランプ済み |
| `/detect_objects/image_annotated` | `sensor_msgs/Image` | 枠を描画した画像（目視確認用。購読者がいるときだけ生成される） |

`~class` と `~rects` は **検出 0 件でも毎フレーム空で publish される**。
「何も映っていない」と「メッセージが来ていない」を区別できる。

header は入力画像のものをそのままコピーしているので、`message_filters` でそろう。
記録するなら上のトピックを `rosbag record` すれば良い。

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
sub_rects = message_filters.Subscriber("/detect_objects/rects", RectArray)
sub_class = message_filters.Subscriber("/detect_objects/class", ClassificationResult)
message_filters.TimeSynchronizer([sub_rects, sub_class], 10).registerCallback(cb)
rospy.spin()
```

`Rect` は `x` / `y` / `width` / `height`（ピクセル）。
`TimeSynchronizer` は入力画像の stamp が正しく入っていることが前提。カメラや rosbag なら
問題ないが、stamp が 0 のソースを使う場合は片方だけ subscribe して自前で持ち回るほうが確実。

## Launch ファイル

### `detect_objects.launch` — 検出のみ

画像ソースが既にある場合（実機カメラ、rosbag、他の launch）はこちら。

| Arg | Default | Notes |
|---|---|---|
| `classes` | （必須） | 検出プロンプト（CSV）。 |
| `image_topic` | `image` | 入力 `sensor_msgs/Image` トピック。 |
| `model` | `yolov8x-worldv2.pt` | YOLO-World の重み。 |
| `device` | `cpu` | 例: `cuda:0`。 |
| `conf` | `0.25` | 確信度のしきい値。 |
| `iou` | `0.5` | NMS の IoU しきい値。 |
| `visualize` | `false` | `image_view` を開く。 |

### `video.launch` — 動画ファイルで試す

`video_stream_opencv` で mp4 を流し込んで `detect_objects.launch` に繋ぐ。

```bash
roslaunch balloon_detection video.launch \
    video:=IMG_8615.mp4 classes:="balloon,teddy bear"
```

`pooh/` に置いた `*.mp4` を `video:=` でファイル名指定するか、`video_path:=` に
絶対パス（あるいは `/dev/video0`）を渡す。

| Arg | Default | Notes |
|---|---|---|
| `classes` | （必須） | 検出プロンプト（CSV）。 |
| `video` | `IMG_8615.mp4` | `pooh/` 以下のファイル名。 |
| `video_path` | `$(find balloon_detection)/pooh/$(arg video)` | 絶対パスで上書き。 |
| `publish_fps` | `30` | `video_stream_opencv` の publish レート。 |
| `loop` | `false` | 動画をループ再生。 |
| `visualize` | `true` | `image_view` を開く。 |

`model` / `device` / `conf` / `iou` は `detect_objects.launch` と同じ。

## License

MIT. See [LICENSE](LICENSE).
