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

Tested on Ubuntu 24.04 + ROS One (`/opt/ros/one`).

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

## 犬のポーズ推定（`dog_pose_node.py`）

YOLO-World とは独立したもう一組のノード + launch。Ultralytics の
[dog-pose](https://docs.ultralytics.com/ja/datasets/pose/dog-pose) データセット
（24 キーポイント）で学習した YOLO pose モデルを回して、犬の bbox と骨格を
`jsk_recognition_msgs/HumanSkeletonArray` などで publish する。

### 動かす

学習は不要。`model` を省略すると、初回起動時に公開の学習済み重みを
`$ROS_HOME/dog_pose/` に落としてきて以後それを使う。

```bash
roslaunch balloon_detection dog_pose.launch \
    image_topic:=/camera/color/image_raw \
    device:=cuda:0 visualize:=true
```

使っている重みは HuggingFace の
[`20-team-daeng-ddang-ai/dog-pose-estimation`](https://huggingface.co/20-team-daeng-ddang-ai/dog-pose-estimation)
（`yolo26m-pose` を `dog-pose.yaml` で 100 epochs, imgsz 640、AGPL-3.0）。
チェックポイントに記録されている val メトリクスは box mAP50-95 **0.901** /
pose mAP50-95 **0.607**。Ultralytics 公式は dog-pose の**データセットしか
配布していない**ので、公開されている 24 点の重みは事実上これだけ。

自分で学習し直す場合は同梱の `train_dog_pose.py`（初回にデータセット 337 MB
を落とす）。`model:=` にはローカルパスでも URL でも渡せる。

```bash
rosrun balloon_detection train_dog_pose.py --epochs 100 --device 0
roslaunch balloon_detection dog_pose.launch \
    model:=$HOME/runs/dog-pose/weights/best.pt
```

```bash
rostopic echo /dog_pose/skeleton   # 骨格（HumanSkeletonArray）
rostopic echo /dog_pose/pose       # キーポイント（PeoplePoseArray）
rostopic echo /dog_pose/rects      # bbox
```

rviz では `jsk_rviz_plugin/HumanSkeletonArray` ディスプレイをそのまま
`/dog_pose/skeleton` に向ければ骨が出る。

### 座標系（重要）

デフォルトはモノラル画像だけなので、キーポイントは **ピクセル座標**
（`x` = u, `y` = v, `z` = 0）で出る。実寸の 3D が欲しい場合は depth と
`camera_info` を渡す。

```bash
roslaunch balloon_detection dog_pose.launch \
    image_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    with_depth:=true model:=$HOME/runs/dog-pose/weights/best.pt
```

このとき座標はカメラ光学座標系のメートル。**depth が取れなかったキーポイントは
メッセージから落とす**（適当な値で埋めない）ので、`limb_names` を見て
どの関節が入っているかを確認すること。

### 出力トピック

| トピック | 型 | 中身 |
|---|---|---|
| `/dog_pose/skeleton` | `jsk_recognition_msgs/HumanSkeletonArray` | 犬 1 匹 = 1 `HumanSkeleton`。`bone_names` は `"withers->tail_start"` 形式、`bones` が線分。`human_ids` は ByteTrack の追跡 ID |
| `/dog_pose/pose` | `jsk_recognition_msgs/PeoplePoseArray` | 各犬の可視キーポイント。`limb_names` にキーポイント名、`scores` にスコア |
| `/dog_pose/rects` | `jsk_recognition_msgs/RectArray` | 各犬の bbox |
| `/dog_pose/class` | `jsk_recognition_msgs/ClassificationResult` | `rects` と同順のラベルと確信度 |
| `/dog_pose/image_annotated` | `sensor_msgs/Image` | 骨格を描画した画像（購読者がいるときだけ生成） |

4 本とも検出 0 件でも毎フレーム空で publish される。配列の i 番目どうしが対応する。

キーポイント名は `dog-pose.yaml` の `kpt_names` そのまま（`nose`, `withers`,
`tail_start`, `front_left_paw`, ... の 24 個）。骨の繋ぎ方は yaml に定義が無いので
`dog_pose_node.py` の `DOG_BONES`（23 本）で定義している。

#### 24 点のうち 4 点は絶対に出ない

dog-pose データセットの全 8476 インスタンスのラベルを集計したところ、
**`left_eye` / `right_eye` / `withers` / `throat` の 4 点はアノテーション率
0.0%** だった。`kpt_names` には名前があるが実際には一度もラベルされていない
ので、このデータセットで学習した**どのモデルでもこの 4 点は出ない**（実測でも
conf ≈ 0.00〜0.03）。

そのため `DOG_BONES` は、背骨を `withers` ではなく**四肢の elbow と
`tail_start` の間に張る**構成にしてある。`withers` を経由させると胴体が丸ごと
切れてしまうため。実写で試すと 23 本中 14〜18 本が残り、頭 → 耳 → 肩 → 脇腹 →
尻尾 → 四肢が 1 本に繋がる。

参考までに実測のアノテーション率（高い順の一部）: `nose` 99.3%, `chin` 89.9%,
`front_left_paw` 89.0%, `left_ear_base` 88.3%, `left_ear_tip` 68.4%,
`rear_left_paw` 55.1%, `tail_end` 47.0%, `tail_start` 45.3%,
`rear_right_elbow` 40.5%。後ろ足と尻尾は半分程度なので、欠けるのは正常。

### `dog_pose.launch` の引数

| Arg | Default | Notes |
|---|---|---|
| `model` | （空） | 空なら公開の学習済み重みを自動 DL。ローカルパスでも URL でも可 |
| `image_topic` | `image` | 入力 `sensor_msgs/Image`。 |
| `device` | `cpu` | 例: `cuda:0`。 |
| `conf` | `0.25` | 検出の確信度しきい値。 |
| `iou` | `0.5` | NMS の IoU しきい値。 |
| `kpt_conf` | `0.3` | これ未満のキーポイントは出力しない。 |
| `track` | `true` | ByteTrack で `human_ids` を安定させる。 |
| `min_valid_keypoints` | `0` | キーポイントがこの数未満の検出を捨てる。0 で無効。 |
| `with_depth` | `false` | depth + `camera_info` を同期して 3D 化。 |
| `depth_topic` | `depth_image` | color に位置合わせ済みの depth。 |
| `camera_info_topic` | `camera_info` | color の内部パラメータ。 |
| `visualize` | `false` | `image_view` を開く。 |

動画ファイルで試すなら `dog_pose_video.launch`（`video_path:=` は絶対パス必須）。

```bash
roslaunch balloon_detection dog_pose_video.launch video_path:=$HOME/dog.mp4
```

### 誤検出について

犬・自転車・トラックが写った写真で試したところ、犬を conf 0.896 で正しく
検出する一方、トラックの領域を conf 0.567 で犬と誤検出した。ただし誤検出側は
キーポイントが 7/24 しか乗らなかったので、`min_valid_keypoints:=10` のような
足切りか `conf:=0.5` 以上で落とせる。犬が写っていないバスや絵画では 0 件だった。

## トラブルシュート

### numpy 2 系がらみの import エラー

ROS 1 の `cv_bridge` は numpy 1.x に対してビルドされているので、パッケージの
virtualenv に numpy 2 が入ると C 拡張の ABI が合わずに import で落ちる。
`requirements.txt` で `numpy<2` に固定してある。既に numpy 2 でビルドしてしまった
環境では virtualenv を作り直す。

```bash
catkin clean balloon_detection
catkin build balloon_detection
```

### `load() got an unexpected keyword argument 'download_root'`

ultralytics は CLIP を `clip.load(size, device=..., download_root=...)` と呼ぶ
（`ultralytics/nn/text_model.py`）。`detect_objects_node.py` 冒頭の shim は
JIT を無効にするためだけに `clip.load` を差し替えているので、`download_root` を
含む他の引数はそのまま素通しする必要がある。現在の shim は `**kwargs` で
渡しているのでこのエラーは出ない。

## License

MIT. See [LICENSE](LICENSE).
