# k_arm_grasp_close

K_ARM の右グリッパーを**いつ閉じるか**を、テレオペで撮った rosbag から学習する．
入力は指先センサ（ToF・近接）と腕の関節速度だけで、カメラは使わない．
出力は「今閉じるべきか」の確率と、それを 0.5 で切った閉じる / 閉じないの判断．

腕の動きまで出す模倣学習の方策（ACT など）ではなく、操作者が閉じたタイミングだけを真似する
行動クローニング（2 値分類）．データ作成・学習・実機での実行をこのパッケージだけで回せる．

```
rosbag ──make_dataset.py──> data/*.npz ──train.py──> runs/v1/model.pt ──grasp_close.launch──> /grasp_close/close
                                                                                          └─(gripper:=true)─> 右グリッパー
```

## TL;DR

学習済みのモデル（`train.py` が出す `model.pt`）で、指先センサが反応したら右グリッパーを閉じたいだけなら:

```bash
source <catkin ワークスペース>/devel/setup.bash
roslaunch k_arm_grasp_close grasp_close.launch model:=/path/to/model.pt gripper:=true
# 開く
rosservice call /grasp_close_gripper/open
```

- 判断だけ見たい（グリッパーは動かさない）なら `gripper:=true` を外す．
- 自分の bag で学習し直すなら:

  ```bash
  rosrun k_arm_grasp_close make_dataset.py --out data bags/*.bag
  rosrun k_arm_grasp_close train.py --data data --out runs/v1 --cv   # -> runs/v1/model.pt
  ```

### 入力（ノードが購読する．どれかが 0.3 秒来ないと判断しない）

| トピック | 型 | 中身 |
| --- | --- | --- |
| `/gripper_tactile/wsb/finger_0/{0,1,2}/range_tof` | `sensor_msgs/Range` | 指先の ToF 距離 ×3 |
| `/gripper_tactile/wsb/finger_0/{0,1,2}/proximity_intensity/raw` | `std_msgs/UInt32` | 指先の近接強度 ×3 |
| `/gripper_tactile/udts/finger_0/proximities_intensity/raw` | `std_msgs/UInt32MultiArray` | udts の近接 37 画素 |
| `/joint_states` | `sensor_msgs/JointState` | 右腕 `RARM_JOINT0`〜`6`（速度にして使う） |

学習（`make_dataset.py`）では、これに加えて正解として `/gripper_right/joint_group_effort_controller/command`
（`std_msgs/Float64MultiArray`、操作者の閉じる指令）が bag に要る．

### 出力

| トピック / サービス | 型 | 中身 |
| --- | --- | --- |
| `/grasp_close/close` | `std_msgs/Bool` | 閉じる判断（確率 0.5 以上が 2 フレーム続いたら True）．20 Hz |
| `/grasp_close/close_probability` | `std_msgs/Float64` | 閉じる確率 0〜1．直近 1 秒の入力がそろっている間 20 Hz |
| `/grasp_close/status` | `std_msgs/String` | JSON: `state`（`ok` / `filling` / `stale`）、`stale`（来ていないトピック）、`probability`、`close` |
| `/gripper_right/joint_group_effort_controller/command` | `std_msgs/Float64MultiArray` | `gripper:=true` のとき: 閉じる -0.4 / 開く 0.0（`k_arm_gripper_bridge` が送る） |
| `/grasp_close_gripper/open` | `std_srvs/Trigger` | `gripper:=true` のとき: 開く |
| `/grasp_close_gripper/state` | `std_msgs/String` | `gripper:=true` のとき: `armed` / `closed` / `closed by hand` / … |

## 1. データ

### 撮り方

テレオペ（`k_arm_teleop`）でりんごに近づけて閉じる．記録は `teleop.launch` か `record_only.launch`．
必要なトピック（`config/right.yaml`）:

| 中身 | トピック |
| --- | --- |
| ToF 距離 ×3 | `/gripper_tactile/wsb/finger_0/{0,1,2}/range_tof` |
| 近接強度 ×3 | `/gripper_tactile/wsb/finger_0/{0,1,2}/proximity_intensity/raw` |
| udts 近接 37 画素 | `/gripper_tactile/udts/finger_0/proximities_intensity/raw` |
| 右腕 7 関節 | `/joint_states`（`RARM_JOINT0`〜`6`） |
| 正解（操作者が閉じた時刻） | `/gripper_right/joint_group_effort_controller/command` |

- 1 本に何回閉じてもよい（閉じて、開いて、また閉じる）．
- **閉じない試行も撮る**: 葉に触る、枝や実の横をかすめる、何もない所で手を動かす．閉じる操作のない bag は全部
  「閉じない」の例になり、誤って閉じるのを減らすのに一番効く．

### 正解ラベル

effort の指令 `data[0]` が -0.2 を下回った時刻を「操作者が閉じた時刻」t_c とする（閉じる指令は -0.4、開くのは 0.0）．

```
            使わない         閉じる(1)
   閉じない(0) │◄─ 0.3 s ─►│◄ 0.3 s ►│◄0.1►│   閉じている間〜開いて 1 s 後は使わない   │ 閉じない(0)
───────────────┴───────────┴─────────t_c─────┴─────────────────────────────────────────┴──────
```

閉じる直前の 0.3 秒を「閉じる」とし、その前 0.3 秒は操作者の反応の遅れとして使わない．

### 変換

```bash
rosrun k_arm_grasp_close make_dataset.py --out data bags/*.bag
```

bag ごとに 20 Hz の特徴量（1 フレーム 53 個）とラベルを `.npz` に書く．入力のどれかが 0.3 秒より古いフレームは使わない．

| 特徴量 | 個数 |
| --- | --- |
| ToF 距離（0.3 m で打ち切り）と測れたかの印 | 3 × 2 |
| 近接強度 log(1+x) | 3 |
| udts 近接 log(1+x) | 37 |
| 関節速度 | 7 |

関節角度とグリッパーの開き具合は使わない．関節角度は学習した bag でりんごがあった場所を覚えてしまう．
グリッパーのサーボ値は起動ごとに基準がずれる（同じ開いた状態で -3.1 のことも -497 前後のこともあった）．

## 2. 学習

```bash
rosrun k_arm_grasp_close train.py --data data --out runs/v1 --cv
```

- モデル: 直近 1 秒（20 フレーム）を見る 1 次元畳み込み（dilation 1/2/4）＋全結合．CPU で数分．
- `--cv`: 1 本ずつ外して学習し、外した bag で判断を出して評価する（leave-one-bag-out）．操作者が閉じるたびに
  - `on time`: 閉じた時刻の 0.6 秒前〜0.1 秒後に判断が出た
  - `early`: それより前に出た（早すぎ）
  - `missed`: 0.1 秒後までに出なかった
  - 閉じる操作のない bag: `no close, fired`（誤って閉じた）/ `no close, quiet`
- 出力: `model.pt`（重み・設定・正規化．実機ではこれだけ使う）と `metrics.json`．

## 3. 実機で使う

```bash
# 判断だけ出す（グリッパーは動かさない）
roslaunch k_arm_grasp_close grasp_close.launch model:=/path/to/runs/v1/model.pt

# 判断でグリッパーを閉じる
roslaunch k_arm_grasp_close grasp_close.launch model:=/path/to/runs/v1/model.pt gripper:=true
```

- `/grasp_close/status` が `ok`（`stale` が空）になっているかを最初に見る．`stale` に出たトピックが来ていない．
- `/grasp_close/close`（Bool）、`/grasp_close/close_probability`（Float64）．
- `gripper:=true` は `k_arm_teleop` の `grasp_close_gripper.launch` を一緒に起動する:
  - 最初に判断が True になったら閉じ、その後は判断によらず閉じたまま（つかんだりんごを落とさない）．
  - 開く: `rosservice call /grasp_close_gripper/open`．開いた後は判断が一度 False に戻るまで閉じない．
  - `open_on_release:=true`: 判断が 0.3 秒 False になったら開く（手をかざして試すとき用）．
  - `rec_button:=true`: AtomS3 の長押しで開閉（録画と一緒なら `record_only.launch gripper_service:=/grasp_close_gripper/toggle_gripper` を使い、こちらは付けない）．
  - launch を止めるとグリッパーの力を抜く（つかんでいるものは落ちる）．
  - テレオペと同時に使わない（同じグリッパーに指令を送る）．

## ファイル

| ファイル | 中身 |
| --- | --- |
| `config/right.yaml` | トピック、20 Hz・1 秒の窓、ラベルの時間、判断のしきい値 |
| `scripts/make_dataset.py` | bag → `.npz` |
| `scripts/train.py` | 学習と leave-one-bag-out 評価 |
| `node_scripts/grasp_close_node.py` | 実機で判断を出すノード |
| `src/k_arm_grasp_close/features.py` | 特徴量・ラベル・判断（3 つで共通） |
| `src/k_arm_grasp_close/model.py` | ネットワークと保存形式 |

## 結果（2026-10-07 に撮った bag）

正解のトピックがある 24 本（閉じる操作 34 回、閉じない bag 1 本）．`train.py --cv` の leave-one-bag-out:

| | on time | early | missed | 閉じない bag で閉じた |
| --- | --- | --- | --- | --- |
| 短い試行 13 本（1 本 1 回閉じる） | 10 | 3（-0.63 / -1.38 / -2.58 s） | 0 | - |
| 長い試行 11 本（1 本に何回か閉じる） | 6 | 8 | 7 | 1 / 1 |
| 計 | 16 / 34 | 11 | 7 | 1 / 1 |

フレーム単位: 閉じるべき所で閉じると言った率 0.559、閉じるべきでない所で閉じると言った率 0.047．

- 長い試行は、何十秒も枝や葉の間を動かしているうちに早く閉じる（-7〜-44 s）か、閉じ損ねる（6 回のうち 5 回を
  逃した bag もある）．閉じない例が少なく、長い試行の本数も少ないため．閉じない試行と、葉の中を動かす試行を増やすのが先．
- ノードは bag を再生して確認した（`status` が `ok` になり、操作者が閉じる 0.39 s 前に `close` が True）．
  その bag は学習にも入っているので、性能ではなく動くことの確認．
