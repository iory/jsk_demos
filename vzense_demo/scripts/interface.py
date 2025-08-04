import time as time_lib
import numpy as np
import rospy
from skrobot.coordinates import Coordinates
from skrobot.coordinates import CascadedCoords

from vzense_demo.k_arm import KARM
from vzense_demo.k_arm_interface import KARMROSRobotInterface
from skrobot.viewers import TrimeshSceneViewer
from skrobot.viewers import PyrenderViewer
from skrobot.model import Axis
from skrobot.model import Link
from skrobot.model import FixedJoint
from skrobot.model import RotationalJoint
from vzense_demo.subscribers.bounding_box_array_subscriber import BoundingBoxArraySubscriber


robot_model = KARM()

# 関節速度を制限する設定。早く動かしたい場合は上限を緩和する
for j in robot_model.joint_list:
    if j.max_joint_velocity == 0.0:
        j.max_joint_velocity = np.deg2rad(5)

robot_model.init_pose()
robot_model.rotate(- np.pi / 2.0, 'y')
viewer = PyrenderViewer()  # viewer
viewer.add(robot_model)
viewer.show()

rarm_end_coords_axis = Axis.from_coords(robot_model.rarm_end_coords)
viewer.add(rarm_end_coords_axis)

rarm_elbow_end_coords_axis = Axis.from_coords(robot_model.rarm_elbow_end_coords)
viewer.add(rarm_elbow_end_coords_axis)

larm_end_coords_axis = Axis.from_coords(robot_model.larm_end_coords)
viewer.add(larm_end_coords_axis)

larm_elbow_end_coords_axis = Axis.from_coords(robot_model.larm_elbow_end_coords)
viewer.add(larm_elbow_end_coords_axis)

rarm_target_coords_axis = Axis()
viewer.add(rarm_target_coords_axis)

larm_target_coords_axis = Axis()
viewer.add(larm_target_coords_axis)

use_ri = True
use_hand = True
send_time = 3
if use_ri:
    rospy.init_node('k_arm_demo')
    rospy.sleep(3.0)
    ri = KARMROSRobotInterface(robot_model, use_hand=use_hand)  # 実機との接続処理
    boxes_sub = BoundingBoxArraySubscriber('/box_fitting_node/output/boxes',
                                           start=False)
    left_boxes_sub = BoundingBoxArraySubscriber('/camera/left_vzense_camera/object_detection/output/boxes',
                                                start=False)  # ぶどうの認識器
    right_boxes_sub = BoundingBoxArraySubscriber('/camera/right_vzense_camera/object_detection/output/boxes',
                                                 start=False)  # ぶどうの認識器
    robot_model.angle_vector(ri.angle_vector())  # 実機の関節角度をrobot_model (viewerにうつっているもの)に反映させる。
    viewer.redraw()


def ri2ir():
    robot_model.angle_vector(ri.angle_vector())


def calibrated_angle_vector(target_angles, time=10.0, move=False):
    rospy.loginfo('calibrated_angle_vector called')
    target_angles = np.array(target_angles)
    robot_model.angle_vector(target_angles)
    if move is True:
        rospy.loginfo('send first angle vector')
        ri.angle_vector(robot_model.angle_vector(), time)
        ri.wait_interpolation()
        current_angles = ri.angle_vector()
        target_angles += target_angles - current_angles
        rospy.loginfo('send second angle vector')
        ri.angle_vector(target_angles, time)  # robot_modelの姿勢を関節角度指令として実機に送る。
        ri.wait_interpolation()  # ロボットの関節指令の補間が終わるまで待つ。
    else:
        time_lib.sleep(1.0)


def send_robot(send_time=10.0, move=False, arm=None):
    viewer.redraw()
    if move is True:
        controller_type = None
        if arm == 'left' or arm == 'larm':
            controller_type = 'larm_controller'
        elif arm == 'right' or arm == 'rarm':
            controller_type = 'rarm_controller'
        ri.angle_vector(robot_model.angle_vector(), send_time,
                        controller_type=controller_type)
        ri.wait_interpolation(controller_type=controller_type)
    else:
        time_lib.sleep(1.0)


def servo_off_pose(send_time=10.0, move=False, arm=None):
    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(40))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(5))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(40))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(-5))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-90))
    send_robot(send_time=send_time, move=move, arm=arm)


def servo_off_pose_to_init_pose(send_time=10.0, move=False, arm=None):
    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(40))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-90))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(40))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(90))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-90))
    send_robot(send_time=send_time, move=move, arm=arm)


def move_box(send_time=10.0, move=False, step_by_step=True):
    # robot_model.init_pose()
    # robot_model.RARM_JOINT0.joint_angle(np.deg2rad(30))
    # robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-70))
    # robot_model.LARM_JOINT0.joint_angle(np.deg2rad(30))
    # robot_model.LARM_JOINT1.joint_angle(np.deg2rad(70))
    # robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))

    # recognition_pose()
    # calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)

    # robot_model.RARM_JOINT0.joint_angle(np.deg2rad(30))
    # robot_model.LARM_JOINT0.joint_angle(np.deg2rad(30))

    # calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)

    servo_off_pose(move=False)

    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    if move is True:
        ri.rhand.move_hand([0, 0, -np.deg2rad(30), 0], wait=False)
        ri.lhand.move_hand([0, 0, -np.deg2rad(30), 0], wait=True)
        ri.rhand.move_hand([0, np.deg2rad(140), np.deg2rad(-60), np.deg2rad(120)], wait=False)
        ri.lhand.move_hand([0, np.deg2rad(140), np.deg2rad(-60), np.deg2rad(120)], wait=False)
    # send_robot(send_time=send_time, move=move)

    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(30))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(0))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(30))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(0))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-90))
    pre_hook_pose = robot_model.angle_vector()
    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move)

    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(0))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(0))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(0))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(0))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-90))
    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move)

    hook_pose = robot_model.angle_vector()
    robot_model.rarm.inverse_kinematics(
        robot_model.rarm_elbow_end_coords.copy_worldcoords().translate((0, -0.0, 0.1), wrt='world'),
        move_target=robot_model.rarm_elbow_end_coords,
        stop=100,
        rotation_axis='x',
        revert_if_fail=False)
    robot_model.larm.inverse_kinematics(
        robot_model.larm_elbow_end_coords.copy_worldcoords().translate((0, 0, 0.1), wrt='world'),
        move_target=robot_model.larm_elbow_end_coords,
        stop=100,
        rotation_axis='x',
        revert_if_fail=False)
    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move)

    robot_model.angle_vector(hook_pose)
    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move)

    robot_model.angle_vector(pre_hook_pose)
    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move)


def recognition_pose(send_time=10.0, move=False, step_by_step=True):
    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(-20))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-90))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-150))

    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(-20))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(90))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-150))
    send_robot(send_time=send_time, move=move)


def left_place(send_time=10.0, move=False, step_by_step=True):
    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(-30))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-80))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-150))

    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(0))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(80))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-150))

    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move, arm='left')

    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(-30))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-80))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-150))

    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(-30))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(80))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-150))

    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move, arm='left')
    if move is True:
        ri.lhand.stop_grasp(wait=False)


def right_place(send_time=10.0, move=False, step_by_step=True):
    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(0))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-80))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-150))

    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(-30))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(80))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-150))

    if move and step_by_step:
        input('Send Angle vector? [Enter]')
    send_robot(send_time=send_time, move=move, arm='rarm')

    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(-30))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-80))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-150))

    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(-30))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(80))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-150))

    if move and step_by_step:
        input('Send Angle vector? [Enter]')
        send_robot(send_time=send_time, move=move, arm='rarm')
    if move is True:
        ri.lhand.stop_grasp(wait=False)



# viewer.redraw()  # viewerを更新する（viewerをクリックしても更新される)
# robot_model.rarm.angle_vector(np.array(
#     [0.6, -0.0316071, 1.4665161, -1.9041522], dtype=np.float32))  # 右腕のrobot_modelの関節値を4つ分更新
# # robot_model.RARM_JOINT0.joint_angle(0.6)  # 個別に設定する場合はこのように関節Jointクラス(RARM_JOINT0)にjoint_angleというメソッドがあるのでこれで送る。
# # 今の値を確認する場合は引数を与えずに関数を呼ぶrobot_model.RARM_JOINT0.joint_angle()
# robot_model.larm.angle_vector(np.array(
#     [0.6, -0.0316071, -1.4665161, -1.9041522], dtype=np.float32))
# viewer.redraw()


def left_pick_grape(send_time=10.0, move=False, step_by_step=True):
    recognition_pose()
    calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        left_boxes = left_boxes_sub.to_coords()
        rate.sleep()
        if left_boxes is not None and len(left_boxes) > 0:
            rospy.loginfo('Detected boxes from {}'.format(left_boxes_sub.topic_name))
            break
        rospy.logwarn('Waiting detected boxes from {}'.format(left_boxes_sub.topic_name))

    target_pose = left_boxes[0].copy_worldcoords().translate((0.0, 0, -0.06), wrt='world')
    ret = robot_model.larm.inverse_kinematics(
        target_pose,
        move_target=robot_model.larm_end_coords,
        stop=100,
        rotation_axis=False)
    if ret is None:
        rospy.logwarn('Could not solve ik. try force ik if failed.')
        ret = robot_model.larm.inverse_kinematics(
            target_pose,
            move_target=robot_model.larm_end_coords,
            stop=100,
            rotation_axis=False,
            revert_if_fail=False)
    pick_pose = robot_model.angle_vector()
    robot_model.larm.inverse_kinematics(
        robot_model.larm_end_coords.copy_worldcoords().translate((0.01, 0, -0.02), wrt='world'),
        move_target=robot_model.larm_end_coords,
        stop=100,
        rotation_axis=False,
        revert_if_fail=False)
    pick_pose = robot_model.angle_vector()
    robot_model.larm.inverse_kinematics(
        robot_model.larm_end_coords.copy_worldcoords().translate((-0.1, 0, 0)),
        move_target=robot_model.larm_end_coords,
        stop=100,
        rotation_axis=False,
        revert_if_fail=False)
    pre_pick_pose = robot_model.angle_vector()

    robot_model.angle_vector(pre_pick_pose)
    # ri.lhand.stop_grasp(wait=False)
    if move is True:
        ri.lhand.move_hand([np.deg2rad(-50), 0, 0, 0], wait=False)
    calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)
    robot_model.angle_vector(pick_pose)
    calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)

    # octomapのために止める処理
    if move is True:
        ri.lhand.move_hand([np.deg2rad(-50), np.deg2rad(60), 0, np.deg2rad(60)], wait=False)
        rospy.sleep(5.0)
        ri.lhand.init_octomap()
        rospy.sleep(5.0)
        ri.lhand.stop_octomap()
        # ri.lhand.start_grasp(angle=130, wait=True)
        # ri.lhand.move_hand([np.deg2rad(-50), np.deg2rad(130), 0, np.deg2rad(130)], wait=False)
        ri.lhand.move_hand([np.deg2rad(-50), np.deg2rad(160), 0, np.deg2rad(160)], wait=False)
        rospy.sleep(5.0)
    robot_model.angle_vector(pre_pick_pose)
    send_robot(send_time=5.0, move=move)


def demo(send_time=10.0, move=False, step_by_step=True):
    ri.lhand.reset_octomap()
    ri.rhand.reset_octomap()
    left_pick_grape(send_time=send_time, move=move, step_by_step=step_by_step)
    left_place(send_time=send_time, move=move, step_by_step=step_by_step)



def box_demo(send_time=10.0, move=False, step_by_step=False):
    servo_off_pose(send_time=send_time, move=move)
    servo_off_pose_to_init_pose(send_time=send_time, move=move)
    recognition_pose(send_time=send_time, move=move)
    servo_off_pose_to_init_pose(send_time=send_time, move=move)
    move_box(send_time=send_time, move=move, step_by_step=step_by_step)
