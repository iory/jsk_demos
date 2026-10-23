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
sudo $(rospack find k_arm_teleop)/scripts/install_leader_udev.py --apply  # USB アダプタを挿してから
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
# USB アダプタを挿して、/dev/k_arm_leader、/dev/k_arm_follower、/dev/k_arm_rec_button（AtomS3）の名前を付ける（PC ごとに初回のみ）
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
  どの USB ポートに挿しても同じ名前になる．どのアダプタがリーダー（`/dev/k_arm_leader`）かフォロワー
  （`/dev/k_arm_follower`）かは `config/usb_adapters.yaml`（アダプタのシリアル番号 → 役割）で決まる
  （両方とも同じサーボ id 1-8 なのでバス上では見分けられない）．載っていない新しいアダプタは一度だけ
  `--role leader|follower` を付けて実行すると、`--apply` でこのファイルにも追記されるのでコミットする．
  `--apply` なしなら書くルールを表示するだけ．
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
| `allow_remote_sim` | `false` | `real:=false` はマスターがこの PC（localhost、127.0.0.1、`$ROS_IP`、`$ROS_HOSTNAME`）でないと起動を断る（実機のマスターで代わりのコントローラが `/robot_description` を上書きし、腕のコントローラと同じ名前で出てしまうため）．`true` でそれでも起動する |
| `real_robot_state_publisher` | `not real` | 実機側の bringup が robot_state_publisher を出していないなら `true` |
| `arm` | `right` | リーダー（サーボ id 1-8）でどちらの腕を動かすか．`right` / `left`．サーボの割り当ては `config/leader_servos_<arm>.yaml` |
| `wrist_only` | `false` | `true`: 手首（とグリッパー）だけリーダーに追従．肩・肘は Start 時のロボットの姿勢で固定（virtual でも同じ） |
| `gripper_only` | `false` | `true`: Real robot には腕の指令を送らず、グリッパー（トリガー）だけ動かす |
| `leader` | `feetech` | `feetech`: サーボから読む．`gui`: スライダー（机上テスト）．`external`: 何も起動しない（bag 再生など） |
| `rviz` | `true` | rviz（TeleopPanel 付き）を起動．`real:=true` では `rviz/teleop_real.rviz`（手首・頭カメラの画像付き） |
| `rec_button` | `true` | AtomS3 の記録ボタン（`rec_button_bridge.py`）を起動．AtomS3 がなくてもポートを 1 秒ごとに開き直すだけで、他は止めない |
| `rec_button_port` | `/dev/k_arm_rec_button` | AtomS3 のシリアルポート |
| `port` | （空） | リーダーのシリアルポート．空なら `leader_bus.yaml` の `port`（`/dev/k_arm_leader`） |
| `config` | `config/teleop.yaml` | 対応・速度・記録トピック |
| `leader_bus` | `config/leader_bus.yaml` | ポート、サーボ id、可動域 |
| `leader_device` | `leader` | リーダーに使う腕．`leader`: `/dev/k_arm_leader` と `config/leader_calibration.yaml`．`follower`: フォロワーの腕を `/dev/k_arm_follower` と `config/leader_calibration_follower.yaml` で使う |
| `leader_calibration` | `config/leader_calibration.yaml`（`leader_device:=follower` なら `_follower` 付き） | ゼロ点・向き・トリガーの保存先（パネルの操作でここが更新される） |

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
   リーダーを持ったままなら、AtomS3 の画面を押しても記録の開始・停止ができる（下記）．

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
# 書き込んだら必ずサーボの電源を入れ直す（入れ直すまで位置制御は古いオフセットのままで、通電した移動が逆向きに暴れる）
rosrun k_arm_teleop recenter_leader.py --device follower      # フォロワーの腕をリーダーに使うとき
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

### AtomS3 の記録ボタン

AtomS3（画面付き．AtomS3 Lite・AtomS3R ではない）を USB でこの PC につなぐと、記録の状態を画面に出し、
画面（ボタン）を押すと記録を開始・停止する．`teleop.launch` が `rec_button_bridge.py` を起動し、
`/k_arm_teleop_manager/status` を AtomS3 に送り、押されたら `start_recording` / `stop_recording` を呼ぶ
（パネルの `Start recording` と同じサービス．パネルの表示とずれない）．

| 画面 | 意味 |
| --- | --- |
| 赤地に `REC`、経過時間とサイズ | 記録中 |
| `REC` の下に黄色の `NO LABEL` | 記録中だが following でない．`/k_arm_teleop_manager/command`（行動ラベル）が出ていない |
| `REC` の下に `VIRTUAL` | target が virtual．実機は動いていない |
| オレンジの点滅で `NOT REC` | following なのに記録していない |
| 暗い地に `STANDBY`、bag の本数とフェーズ | 記録していない |
| 灰色の `NO TELEOP` | ブリッジは動いているが、`/k_arm_teleop_manager/status` が 2 秒以上来ない |
| 灰色の `NO LINK` | PC から 2 秒以上何も来ない（ブリッジが動いていない・ポートが開けない） |
| 右上の `CLOSE` / `OPEN` の札 | 長押しでグリッパーを閉じた / 開いた状態（トリガーは無視中） |
| 白帯の `...` → `OK` / `ERR <理由>` | 押した結果．結果が返るまで次の押下は無視する．5 秒返らなければ `no reply from the PC` |

following でないときに押しても、確認なしでそのまま記録を始める（画面に `NO LABEL` が出る）．パネルのボタンは
確認ダイアログを出す．

**長押し（0.8 秒）でグリッパーの開閉**: 実機のグリッパーを開いていれば閉じ、閉じていれば開く（`/k_arm_teleop_manager/toggle_gripper`）．
以後はリーダーのトリガーを無視してその状態を保つ（following 中も．記録する行動ラベル `RARM_GRIPPER` もボタンの開閉に
なる）．画面右上に `CLOSE` / `OPEN` の札が出ている間がこの状態．トリガーに戻すには STOP → Start．
target が real のときだけ効く（virtual では `ERR` で断る）．短く押した場合は今までどおり記録の開始・停止．

**テレオペなしで記録だけ**: リーダーも rviz も使わず、AtomS3 で rosbag の記録開始・停止だけをする．
ロボットへは何も送らない（ロボットを別の方法で動かしながら記録する用）．

```bash
roslaunch k_arm_teleop record_only.launch     # ROS_MASTER_URI はロボットのマスター
```

記録するトピック・保存先・ファイル名の頭は `teleop.yaml` の `recording` と同じ（`~/k_arm_bags`）．
AtomS3 の画面は同じだが、行動ラベルも送り先もないので `NO LABEL` / `VIRTUAL` は出ない（フェーズ `record_only`）．
長押し（グリッパー）は `ERR` になる．`rosservice call /k_arm_recorder/start_recording`（`stop_recording`）でも操作できる．

初回（PC ごと）: `sudo $(rospack find k_arm_teleop)/scripts/install_leader_udev.py --apply` で
`/dev/k_arm_rec_button` の名前を付ける（`config/usb_adapters.yaml` に AtomS3 のシリアル番号が載っている．別の
AtomS3 なら `--role rec_button` を付けて一度実行する）．

ファームウェアは `firmware/atoms3_rec_button`（PlatformIO、M5Unified）．書き込み:

```bash
pio run -d $(rospack find k_arm_teleop)/firmware/atoms3_rec_button -t upload --upload-port /dev/k_arm_rec_button
```

PC との通信は 115200 bps の 1 行テキスト（`rec_button_bridge.py` と `src/main.cpp` の先頭に仕様）．
`pio device monitor` で開いて `S rec=1 dur=12 size=3000000 bags=2 phase=following target=real` と打てば、
ROS なしで表示を確かめられる．

## 学習した方策で動かす（右腕）

テレオペで撮った bag を学習サーバー（supernova の tree-vla-il）で追加学習し、できた方策を
`k_arm_pick_policy` の実行ノードで右腕に動かす．方策自体は左腕用なので、右腕のデータと指令は
tree-vla-il 側（ブランチ field-pipeline、1e4f6a2 以降）で ARM_BODY の xz 平面について鏡映する．
右手首カメラの取り付けが左の鏡映になっていることが前提（未確認）．

```bash
rosrun k_arm_teleop kpick.sh check ~/k_arm_bags/xxx.bag    # トピックが足りているか（サーバー側で確認）
rosrun k_arm_teleop kpick.sh train                         # ~/k_arm_bags の bag を全部送って学習 -> ~/kpick_results/<job>/result
rosrun k_arm_teleop kpick.sh train -e 20 ~/k_arm_bags/today # シミュレーションの確認を 20 本に減らす
rosrun k_arm_teleop kpick.sh run                           # 最新の結果を確認モードで（ロボットには送らない）
rosrun k_arm_teleop kpick.sh run execute                   # 実際に動かす（テレオペは STOP にしておく）
```

- rviz パネルの「Learned policy (right arm)」からも同じことができる（`kpick_manager.py`、teleop.launch で起動）．
  `Train on the recorded bags` で録画ディレクトリの bag を全部送って学習（進捗を表示、結果は `~/kpick_results`）、
  `Result` で結果を選んで `Report` でレポートを開き、`Preview`（ロボットに送らない）/ `Execute`（実機）で起動、
  `Start picking` / `Stop picking` で 1 回の収穫を開始・停止、`Shut down` で終了．Execute はテレオペが実機に
  追従している間は押せない．学習の Cancel はこの PC の待ち受けをやめるだけで、サーバーに渡ったジョブは続く．
- `Step mode`（既定で ON）: 起動した方策は Start picking で次の 8 ステップ分の予測を rviz（`Policy prediction`。
  太線が次に実行する分で、グリッパーが閉じるところは赤、細い灰色はその先）に出すだけで止まり、`Step` を押すと
  その 8 ステップだけ実行して次の予測を出す．表示は方策が出す手先の目標で、腕が実際にたどる軌道ではない
  （シミュレーションでは 8 ステップ後の実際の手先と中央値 48 mm ずれた）．tree-vla-il 6011254 以降の結果で使える．
- rviz（`real:=true`）の `Policy detections` に、方策ノードの検出結果（`/kpick_policy/debug/image`）が出る．
  対象のりんごは赤（TARGET）、ほかは緑．待機中も出るので、Start picking でどのりんごを狙うかを先に見られる．
  パネルには `/kpick_policy/status` を 1 行にして出す（状態、ステップ、検出、1 tick の時間）．
  どちらも tree-vla-il f9e8933 以降のノード、つまりそれ以降に学習したジョブの結果で出る（結果ごとに `deploy/` のコードで動くため）．
- 設定は `config/kpick_right.yaml`．tree-vla-il の `k_arm_right`（右腕、右手首カメラ、`/rarm_controller/command`）に
  グリッパーの送り先を足しただけ．学習ジョブと実行ノードの両方に渡る．`k_arm_right` を持たない古い学習結果では動かない．
- グリッパー: 方策は 0（開）〜1（閉）を出す．`gripper_command_bridge.py` がテレオペのトリガーと同じ開閉の
  ロジック（`teleop.yaml` の `arms/rarm/gripper/real`）で実機の effort に変える．`execute` のときだけ起動し、
  最初の指令が来るまで何も送らない．指令が止まっても最後の状態を保つ（つかんだものを落とさない）．
  開くときは `rosservice call /k_arm_gripper_bridge/release`．
- 指先センサで閉じる: `roslaunch k_arm_grasp_close grasp_close.launch model:=<model.pt> gripper:=true`
  （このパッケージの `grasp_close_gripper.launch` を含む）．k_arm_grasp_close の判断（`/grasp_close/close`）が最初に True になったら右グリッパーを閉じ、
  その後はモデルの出力によらず閉じたまま保つ．開くのは `rosservice call /grasp_close_gripper/open`．開いた後は判断が
  一度 False に戻るまで閉じない．`/grasp_close_gripper/enable`（SetBool）で閉じる動作を止められる．
  手で試すときは `open_on_release:=true` で、判断が `open_delay`（既定 0.3 s）続けて False になったら開く
  （掴んでいる間のモデルの出力は当てにならないので、収穫では使わない）．
  テレオペのトリガー / AtomS3 も同じグリッパーに送るので、テレオペと同時に起動しないこと．
  止める（Ctrl-C）と、一度でも指令を送っていればグリッパーの力を抜く（effort 0 をコントローラに直接送る）．
  掴んでいるものは落ちる．
  AtomS3 の長押しで開閉（`/grasp_close_gripper/toggle_gripper`．閉じていれば開き、開いていれば手で閉じる．
  手で閉じたものは `open_on_release` でも開かない）: 録画と一緒なら
  `roslaunch k_arm_teleop record_only.launch gripper_service:=/grasp_close_gripper/toggle_gripper`、
  録画なしなら `grasp_close_gripper.launch rec_button:=true`（同じポートを開くので両方は起動しない）．
- 学習の流れ・レポートの見方・困ったときは tree-vla-il の `docs/trickstar_guide.md`．

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
  - `follow`: virtual は 50 Hz、到達時間 0.04 s（遅れ約 50〜70 ms）．**real は `ri.angle_vector(pose, 5000)` 式**：
    5 秒ごとにリーダーのその時の姿勢を「5 秒かけて到達する」action goal で送る（`send_rate.real` 0.2、
    `lookahead.real` 5.0、`interface.real` action）．速い動きで実機が落ちるため．ロボットはリーダーから最大 10 秒ほど
    遅れる．STOP / リーダー切断で goal を取り消すので、動作中の goal はその場で止まる．N を変えるときは
    `lookahead.real` を N、`send_rate.real` を 1/N にする（以前の追従は 10.0 / 0.1）．
    `max_velocity` は送り先・関節ごと（virtual 10 rad/s、real は腕 1.5、手首 J4 5 / J5・J6 3 rad/s）．
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
