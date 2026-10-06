# k_arm_teleop

FEETECH サーボのリーダーアーム（`teleop_leader_pair_description`）で K_ARM_DUALARM を
動かし、模倣学習用のデータを rosbag で取るためのパッケージ．

## TL;DR（新しい PC で）

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone --single-branch -b k-imitation-demo https://github.com/iory/jsk_demos.git
git clone --single-branch -b uv https://github.com/iory/catkin_virtualenv.git
source /opt/ros/one/setup.bash
cd ~/catkin_ws && catkin config --extend /opt/ros/one && catkin build k_arm_teleop
source ~/catkin_ws/devel/setup.bash
sudo usermod -aG dialout $USER                                          # 初回のみ．再ログインが必要
sudo $(rospack find k_arm_teleop)/scripts/install_leader_udev.py --apply  # アダプタを挿してサーボの電源を入れてから
roslaunch k_arm_teleop teleop.launch                                    # 実機に送るなら real:=true
```

```
feetech_leader_driver.py ──/teleop_leader/servo_joint_states──> teleop_manager.py ──> <ns>/rarm_controller/follow_joint_trajectory
   (feetech-cli, id 1-8)                                          ^  approach → follow     <ns>/rarm_controller/command
                                                                  |                       （arm:=left なら larm_controller）
                                                        rviz TeleopPanel (services + ~status)
```

| target | 送り先 | 表示 |
| --- | --- | --- |
| virtual | `/virtual/<controller>`（`fake_trajectory_controller.py`） | 半透明の K_ARM (virtual) |
| real | `/<controller>`（実機の JointTrajectoryController） | K_ARM (real) |

## 別の環境で使う

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone --single-branch -b k-imitation-demo https://github.com/iory/jsk_demos.git
git clone --single-branch -b uv https://github.com/iory/catkin_virtualenv.git   # uv 対応版（標準版ではない）
source /opt/ros/one/setup.bash
cd ~/catkin_ws && catkin config --extend /opt/ros/one && catkin build k_arm_teleop
source devel/setup.bash
sudo usermod -aG dialout $USER   # シリアルポートの権限（初回のみ．再ログインが必要）
# USB アダプタを挿してサーボの電源を入れ、/dev/k_arm_leader と /dev/k_arm_follower の名前を付ける（PC ごとに初回のみ）
sudo $(rospack find k_arm_teleop)/scripts/install_leader_udev.py --apply
roslaunch k_arm_teleop teleop.launch
```

- ビルド時に uv が feetech-cli を PyPI から取るので、ネットワークが必要．
- 実機（`real:=true`）の rviz で実機モデルをメッシュ付きで出すには、実機の `/robot_description` が参照する
  `k_arm_ros_bridge_tutorials`（`models/K_ARM_DUALARM_LINEAR_meshes`）を同じワークスペースに置く．
  このリポジトリには入れない（メッシュは再配布不可）．無くてもテレオペと録画は動き、実機モデルの表示だけがエラーになる．
- `config/leader_calibration.yaml`（ゼロ点・向き・トリガー）は**このリーダーの個体**の値．同じリーダーなら別の PC でも
  そのまま使える（サーボ側の `homing_offset` は EEPROM に残っている）．別のリーダーを使うときはキャリブレーションと
  `recenter_leader.py` をやり直す．
- シリアルポートは `config/leader_bus.yaml` の `port`（既定 `/dev/k_arm_leader`）．`install_leader_udev.py` が、
  つないでいるアダプタのシリアル番号に一致する udev ルール（`/etc/udev/rules.d/99-k_arm_leader.rules`）を書き、
  どの USB ポートに挿しても同じ名前になる．まだ名前のないアダプタそれぞれにサーボの ping（読み出しのみ）を送り、
  リーダーのサーボ（id 1-8）が全部応答すればリーダー（`/dev/k_arm_leader`）、一部だけならフォロワー
  （`/dev/k_arm_follower`）と判定する．サーボの電源を入れ、teleop.launch は止めておく．判定させずに決めるなら
  `--role leader|follower`．`--apply` なしなら書くルールを表示するだけ．
  `--list` で、つないでいるアダプタと書いたルールの対応を表示する．アダプタが複数あるときは
  `--device /dev/ttyACM1`、別の名前にするなら `--name k_arm_leader_left` のように指定する．
  すでに別の名前が付いたアダプタや、別のアダプタが持っている名前は上書きしない（入れ替えるときは `--replace`）．
  ルールを入れていない環境では `teleop.launch port:=/dev/ttyACM0` などで指定する．

## Build

Python の依存（feetech-cli）は `catkin_virtualenv`（uv）で入る．`catkin_make` ではなく `catkin build`．
ワークスペースに `catkin_virtualenv`（`~/catkin_ws/src/catkin_virtualenv`）が必要．

```bash
source /opt/ros/one/setup.bash
cd ~/catkin_ws
catkin config --extend /opt/ros/one   # 初回のみ
catkin build k_arm_teleop             # catkin_virtualenv, k_arm_descriptions, teleop_leader_pair_description も一緒にビルドされる
source devel/setup.bash
```

fish から使うときは `bash -c 'source ~/catkin_ws/devel/setup.bash && roslaunch ...'` のように bash で source する．

## 起動

リーダー（FEETECH 基板）を USB につなぎ、サーボの電源を入れてから:

```bash
source ~/catkin_ws/devel/setup.bash
# 実機なし: "real" も simulated stand-in（パネルにその旨が出る）
roslaunch k_arm_teleop teleop.launch
# 実機あり: 実機の bringup（/joint_states と <controller>/follow_joint_trajectory）を先に起動しておく
roslaunch k_arm_teleop teleop.launch real:=true
```

主な引数:

| 引数 | 既定値 | 内容 |
| --- | --- | --- |
| `real` | `false` | `true`: 実機に送る．`false`: 実機の代わりに `fake_trajectory_controller.py` を起動 |
| `real_robot_state_publisher` | `not real` | 実機側の bringup が robot_state_publisher を出していないなら `true` |
| `arm` | `right` | リーダー（サーボ id 1-8）でどちらの腕を動かすか．`right` / `left`．サーボの割り当ては `config/leader_servos_<arm>.yaml` |
| `wrist_only` | `false` | `true`: 手首（とグリッパー）だけリーダーに追従．肩・肘は Start 時のロボットの姿勢で固定（virtual でも同じ） |
| `gripper_only` | `false` | `true`: Real robot には腕の指令を送らず、グリッパー（トリガー）だけ動かす |
| `leader` | `feetech` | `feetech`: サーボから読む．`gui`: スライダー（机上テスト）．`external`: 何も起動しない（bag 再生など） |
| `rviz` | `true` | rviz（TeleopPanel 付き）を起動．`real:=true` では `rviz/teleop_real.rviz`（手首・頭カメラの画像付き） |
| `port` | （空） | リーダーのシリアルポート．空なら `leader_bus.yaml` の `port`（`/dev/k_arm_leader`） |
| `config` | `config/teleop.yaml` | 対応・速度・記録トピック |
| `leader_bus` | `config/leader_bus.yaml` | ポート、サーボ id、可動域 |
| `leader_calibration` | `config/leader_calibration.yaml` | ゼロ点・向き・トリガーの保存先（パネルの操作でここが更新される） |

サーボが見えるかの確認は feetech-cli で: `uvx --from feetech-cli feetech --port /dev/k_arm_leader scan`（id 1-8 が見えればよい）．
基板のジャンパが USB 側になっていないと何も応答しない．

## 手順

1. **サーボの確認**: パネルの Leader arm の表で 8/8 servos・電圧・温度を確認．
2. **リーダーのキャリブレーション（初回、または向きを変えたとき）**: rviz の薄いリーダーモデル（Calibration guide）と同じ姿勢、
   つまり重力に任せて腕・肘・手首・ハンドルをまっすぐ真下に垂らし、トリガーを離した姿勢（= K_ARM のゼロ姿勢）で
   `Calibrate leader zero`．`config/leader_calibration.yaml` に保存される（サーボの EEPROM は書かない）．
   未キャリブレーションの間は real に Start できない．
3. **向きの確認**: 1 関節ずつ動かし、rviz のリーダーモデルが実物と同じ向きに動かない関節は表の `flip`．
   上腕ヨー（id 3）は肘を曲げた状態で回すと分かりやすい．角度がオレンジ = 可動域外（向きかキャリブレーションの誤り）．
4. **トリガー**: 離した状態で `Trigger released`、引ききって `Trigger pulled`（2 点で開〜閉に割り当てる．向きも自動）．
5. **virtual で確認**: `Send to: Virtual` → `Start`．ゴーストがゆっくりリーダーの姿勢まで動き（approach）、その後追従する（following）．
6. **real**: `Send to: Real robot`（確認ダイアログあり）→ `Start`．同じく approach してから追従．
7. `Robot -> zero pose` は選択中の target をゆっくりゼロ姿勢へ．`STOP` はいつでも．
8. **記録**: ディレクトリを選んで `Start recording`．パネルにディレクトリ内の bag 数・合計サイズ・空き容量、
   記録中は経過時間とサイズが出る．記録トピックは `config/teleop.yaml` の `recording.topics`．

### リーダーを動かす（サーボに通電）

パネルの Leader arm 欄:

- `Leader -> init pose`: `teleop.yaml` の `leader_move.init_pose`（K_ARM の関節角で指定．既定は全 0 = 腕を垂らした姿勢）に対応する
  リーダーの姿勢へ動かす．
- `Leader -> robot pose`: 選択中の target（real / virtual）の今の関節角に対応する姿勢へ動かす（K_ARM → リーダーの逆変換．
  手首は手先の回転が一致する角度を数値で解く）．これで Start 時の approach がほぼ不要になる．
- 移動後はその姿勢で保持する．ハンドルを持ってから `Release leader`（トルク OFF）か `Start`（保持中ならトルクを切ってから追従）．
- 安全: トルク上限は `move_torque_limit`（既定 500/1000）、速度は `leader_move.velocity`（0.4 rad/s）．
  移動中にサーボが指令から 400 カウント（約 35°）以上遅れたら（ぶつかった・押さえられた）即トルク OFF．

**初回に 1 度だけ: エンコーダの境目をずらす**．サーボの位置指令は 0〜4095 の 1 回転なので、関節の可動域がカウントの
0/4095 の境目をまたぐと、サーボが逆回りしてストッパーにぶつかる．ドライバはそういう移動を拒否する
（「would cross the encoder 0/4095 seam」）．`recenter_leader.py` で各サーボの `homing_offset`（EEPROM）を書き換え、
可動域の中央を 2048 にする．キャリブレーションファイルのカウントも同じ量だけ換算するので、キャリブレーションはそのまま使える．

```bash
# teleop.launch を止めてから（バスを占有するため）
rosrun k_arm_teleop recenter_leader.py --arm right           # 確認のみ（dry run）
rosrun k_arm_teleop recenter_leader.py --arm right --apply   # EEPROM と config/leader_calibration.yaml を更新
```

- リーダーのデータが `leader_timeout` 以上途切れると `leader_lost` になり送信を止める．再開は `Start`（再び approach から）．
- 追従中のステータスに `at joint limit: ...`（K_ARM の可動域の端）、`speed-limited: ...`（速度制限）が出る．
- 学習側の実行ノードなど、他のノードが同じコントローラに指令を送るときはテレオペを STOP しておく．

### 実機につないだときの表示（`real:=true`）

- 実機の TF（`world` → 直動部 → 腕）をそのまま使う．こちらからは `world -> ARM_BODY` を出さず、`/robot_description` も上書きしない．
- rviz は `ARM_BODY`（腕の付け根）基準．直動部と、リーダーで動かさない側の腕（`arm:=right` なら左腕）は表示しない
  （`display_description.py` が実機の `/robot_description` から取り除いた表示用モデルを `/k_arm_teleop/robot_description` に書く）．
- 仮想ロボット（半透明）とリーダー・ガイドは実機の `ARM_BODY` に重ねて表示する．
- カメラ画像は圧縮トピック（`compressed`）で受ける．
- 実機側の PC の時計がずれていると TF とカメラのタイムスタンプが合わず表示・録画が壊れる．全 PC を NTP / chrony で同期しておく．

### 記録される bag

1 エピソード 1 ファイル（ROS1 bag v2）．既定のトピック:

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | K_ARM（実機または代用品）の関節角 |
| `/k_arm_teleop_manager/command` | `sensor_msgs/JointState` | 送った関節目標（50 Hz、追従中のみ）．行動ラベルに使う |
| `/teleop_leader/servo_joint_states` | `sensor_msgs/JointState` | リーダーの生値（100 Hz） |
| `/virtual/joint_states` | `sensor_msgs/JointState` | 仮想ロボット |
| `/k_arm_teleop_manager/status` | `k_arm_teleop/TeleopStatus` | フェーズ・メッセージ |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | |

カメラ（RGB・深度）は `recording.topics` に追加する（容量のため `.../compressed`、`.../compressedDepth` を推奨）．

## 設定

- `config/leader_bus.yaml`: ポート、ボーレート、`joint_ranges_deg`（キャリブレーション姿勢からの可動域．出典はユーザー提供の表）
- `config/leader_servos_right.yaml` / `leader_servos_left.yaml`: サーボ id → リーダー関節、`direction`（既定値．パネルの flip が優先）、
  `clamp_to_range`（ストッパーのない id 3・5・7．何回転しても、ロボットへ渡す値は可動域の端で止める）
- `config/teleop.yaml`: リーダー → K_ARM の対応、コントローラ名、approach / follow の速度、記録トピック．
  - 腕 (J0-J3): `follower = sign * (leader - leader_zero)`
  - 手首 (J4-J6): リーダーは「ピッチ＋ロール」、K_ARM は z-y-x 軸で構成が違うので、関節角ではなく前腕に対する
    手先の回転を合わせる（`k_arm_teleop/kinematics.py` の `WristMap`）．J5 は ±90° で頭打ち、特異姿勢付近では J4 を保持．
  - `leader_zero` はキャリブレーション姿勢のリーダー URDF 上の関節角．ドライバ、ガイド表示、マネージャがここを読む．
  - `leader_base`: リーダーのベース → K_ARM の向き（対応計算と rviz 表示の両方で使う）と表示位置．
  - `follow`: 50 Hz、到達時間 0.04 s（遅れ約 50〜70 ms）．`max_velocity` は送り先・関節ごと（virtual 10 rad/s、
    real は腕 1.5、手首 J4 5 / J5・J6 3 rad/s）．
- `config/leader_calibration.yaml`: `zero_raw`（ゼロ点）、`direction`（flip の結果）、`trigger`（2 点）．リーダーのハードウェア固有の値．
  左右で 1 ファイルを共有し、保存時は今つないでいない腕の値を残す（右でキャリブレーションしても左の値は消えない）

### 未確認の点（実機で要確認）

- `leader_zero`・`sign`・`leader_base` は 2 つの URDF から解いた（腕の関節軸が一致、上腕・前腕が下、肘が前に曲がる、
  ハンドルが真下でトリガーが前）．ランダムな姿勢で腕の関節軸と手先の回転が一致することは数値で確認済み（誤差 1e-5）．
  残る仮定は「リーダーのベースのどちらが上・前か」と「トリガーを前に向けるか」なので、ガイドと実物が同じ見た目か確認する．
- **サーボの向き**: パネルの flip で合わせる（手順 3）．上腕ヨー（id 3）と肩ピッチ（id 1）は未確認．
- **グリッパー**: 実機は Dynamixel のトルク制御（`teleop.yaml` の `gripper.real`）．追従中、トリガーの引き量を
  `/gripper_<side>/joint_group_effort_controller/command` のトルク（開 0 〜 閉 -0.4、`mode: proportional` は比例、
  `binary` はしきい値でヒステリシス付き開閉）にして 50 Hz で送り、Start 時に `/gripper_<side>/dynamixel_general_control/servo`
  を ON にする．STOP やリーダーが途切れたときは最後のトルクを保持する（つかんだ物を落とさない）．
  行動ラベル `/k_arm_teleop_manager/command` にはグリッパーの閉じ具合（スライダー関節の値）と送ったトルク（effort）が入る．
  virtual と `real:=false` の代用品はスライダー関節の軌道で動かす．`send_to_real: false` で実機のグリッパーを止められる．
- 右腕の `leader_zero`・`sign` も左と同じ方法で URDF から解いた値．右で使うときもガイドと実物の見た目、向き（flip）を確認する．

## Test

```bash
cd ~/catkin_ws/src/jsk_demos/k_arm_teleop && python3 -m pytest test/
```
