# k_arm_teleop

FEETECH サーボのリーダーアーム（`teleop_leader_pair_description`）で K_ARM_DUALARM を
動かし、模倣学習用のデータを rosbag で取るためのパッケージ．

```
feetech_leader_driver.py ──/teleop_leader/servo_joint_states──> teleop_manager.py ──> <ns>/larm_controller/follow_joint_trajectory
   (feetech-cli, id 1-8)                                          ^  approach → follow     <ns>/larm_controller/command
                                                                  |
                                                        rviz TeleopPanel (services + ~status)
```

| target | 送り先 | 表示 |
| --- | --- | --- |
| virtual | `/virtual/<controller>`（`fake_trajectory_controller.py`） | 半透明の K_ARM (virtual) |
| real | `/<controller>`（実機の JointTrajectoryController） | K_ARM (real) |

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
| `leader` | `feetech` | `feetech`: サーボから読む．`gui`: スライダー（机上テスト）．`external`: 何も起動しない（bag 再生など） |
| `rviz` | `true` | rviz（TeleopPanel 付き）を起動 |
| `config` | `config/teleop.yaml` | 対応・速度・記録トピック |
| `leader_bus` | `config/leader_bus.yaml` | ポート、サーボ id、可動域 |
| `leader_calibration` | `config/leader_calibration.yaml` | ゼロ点・向き・トリガーの保存先（パネルの操作でここが更新される） |

サーボが見えるかの確認は feetech-cli で: `cd ~/src/github.com/iory/feetech-cli && uv run feetech scan`（id 1-8 が見えればよい）．
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

- リーダーのデータが `leader_timeout` 以上途切れると `leader_lost` になり送信を止める．再開は `Start`（再び approach から）．
- 追従中のステータスに `at joint limit: ...`（K_ARM の可動域の端）、`speed-limited: ...`（速度制限）が出る．
- 学習側の実行ノードなど、他のノードが同じコントローラに指令を送るときはテレオペを STOP しておく．

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
- **グリッパー**: 実機のグリッパーのインターフェースが不明なので `send_to_real: false`（real ではグリッパーを送らない）．
  コントローラ名を合わせて `true` にする．
- 右腕の `leader_zero`・`sign` も左と同じ方法で URDF から解いた値．右で使うときもガイドと実物の見た目、向き（flip）を確認する．

## Test

```bash
cd ~/catkin_ws/src/jsk_demos/k_arm_teleop && python3 -m pytest test/
```
