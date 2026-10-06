# K-ARM 7DOF and Linear Robot Descriptions

このパッケージは、K-ARMロボット（7自由度アーム + リニア機構）のURDF記述とGazeboシミュレーション設定を含んでいます．

同じリンクを使ったこのなる構造の4つのロボットをurdfとして記述しています．

- **k_linear_arm.urdf**
  - ベース直動アーム+7DoFアームの手首3軸
- **k-arm_7dof-and-linear.urdf**
  - ベース直動アーム+7DoFアーム
- **k-arm_7dof-single.urdf**
  - 7DoFアーム単体
- **k-arm_linear.urdf**
  - 直動アーム単体

## k-arm_7dof-and-linear.urdf

### リンク一覧

| No. | リンク名                       | 分類         | 説明                   |
|-----|-------------------------------|--------------|------------------------|
|  1  | `world`                       | 基準         | ワールド座標系         |
|  2  | `linear_yaw_link`             | リニア部     | リニアアームのヨー軸ベース |
|  3  | `linear_base_pitch_link`      | リニア部     | リニアアームのベースピッチ |
|  4  | `linear_base_boom_link`       | リニア部     | リニアアームのベースブーム |
|  5  | `linear_top_pitch_link`       | リニア部     | リニアアームのトップピッチ |
|  6  | `linear_top_boom_link`        | リニア部     | リニアアームのトップブーム |
|  7  | `arm_base_link`               | 7DOFアーム   | アームベース           |
|  8  | `shoulder_pitch_link`         | 7DOFアーム   | 肩ピッチ               |
|  9  | `shoulder_roll_link`          | 7DOFアーム   | 肩ロール               |
| 10  | `shoulder_yaw_link`           | 7DOFアーム   | 肩ヨー                 |
| 11  | `elbow_pitch_link`            | 7DOFアーム   | 肘ピッチ               |
| 12  | `wrist_yaw_link`              | 手首         | 手首ヨー               |
| 13  | `wrist_pitch_link`            | 手首         | 手首ピッチ             |
| 14  | `wrist_roll_link`             | 手首         | 手首ロール             |
| 15  | `k_linear_arm_ee_link`   | エンドエフェクタ | ハンド部           |
| 16  | `camera_link`                 | センサー     | カメラ本体             |
| 17  | `camera_color_frame`          | センサー     | カラーフレーム         |
| 18  | `camera_color_optical_frame`  | センサー     | カラー光学フレーム     |
| 19  | `camera_depth_frame`          | センサー     | 深度フレーム           |
| 20  | `camera_depth_optical_frame`  | センサー     | 深度光学フレーム       |

### 関節一覧

#### 制御可能関節（11関節）

| idx | name                      | Type | 可動範囲 (rad)   | 可動範囲 (deg) | 軸  | 説明                |
| --- | ------------------------- | ---- | ---------------- | -------------- | --- | ------------------- |
| 1   | `linear_base_yaw_joint`   | R    | -3.14 ~ 3.14     | -180° ~ 180°   | Z   | Linear Base Yaw回転 |
| 2   | `linear_base_pitch_joint` | R    | -1.39 ~ 0        | -79.6° ~ 0°    | Y   | Linear Base Pitch   |
| 3   | `linear_top_pitch_joint`  | R    | 0 ~ 1.5708       | 0° ~ 90°       | Y   | Linear Top Pitch    |
| 4   | `linear_top_linear_joint` | P    | 0 ~ 0.680m       | 0 ~ 680mm      | X   | Linear Top 伸縮     |
| -   | -                         | -    | -                | -              | -   | -                   |
| 5   | `shoulder_roll1_joint`    | R    | -2.8798 ~ 2.8798 | -165° ~ 165°   | X   | 肩Roll-1            |
| 6   | `shoulder_pitch_joint`    | R    | -1.7453 ~ 2.0944 | -100° ~ 120°   | Y   | 肩Pitch             |
| 7   | `shoulder_roll2_joint`    | R    | -2.8798 ~ 2.8798 | -165° ~ 165°   | X   | 肩Roll-2            |
| 8   | `elbow_pitch_joint`       | R    | 0 ~ 2.4435       | 0° ~ 140°      | Y   | 肘Pitch             |
| 9   | `wrist_roll_joint`        | R    | -2.8798 ~ 2.8798 | -165° ~ 165°   | X   | 手首Roll            |
| 10  | `wrist_pitch_joint`       | R    | -1.57 ~ 1.57         | -90° ~ 90°     | Y   | 手首Pitch           |
| 11  | `wrist_yaw_joint`         | R    | -1.57 ~ 1.57     | -90° ~ 90°     | Z   | 手首Yaw             |

#### 固定関節

| 関節名                        | タイプ   | 親リンク                     | 子リンク                       | 説明                       |
|-------------------------------|----------|------------------------------|-------------------------------|----------------------------|
| `world_joint`                 | fixed    | `world`                      | `linear_yaw_link`             | ワールド固定               |
| `linear_top_fixed_joint`      | fixed    | `linear_top_boom_link`       | `arm_base_link`               | リニア部-アーム部接続      |
| `k_linear_arm_ee_joint`  | fixed    | `wrist_roll_link`            | `k_linear_arm_ee_link`   | 手首-エンドエフェクタ接続  |
| `camera_joint`                | fixed    | `k_linear_arm_ee_link`  | `camera_link`                 | エンドエフェクタ-カメラ接続|

## K_ARM_DUALARM_primitive.urdf

アームのメッシュは再配布できないので、`K_ARM_DUALARM.urdf` のアームリンクを scikit-robot で
box / cylinder に置き換えたもの．グリッパー（HARVEST_GRIPPER_meshes）だけ元のメッシュのまま．
元メッシュのディレクトリは `.gitignore` 済み．

```bash
# 再生成（元メッシュが手元にあるとき）
ROS_PACKAGE_PATH=$(dirname $PWD) uv run scripts/generate_primitive_urdf.py
roslaunch k_arm_descriptions display_dualarm.launch
```
