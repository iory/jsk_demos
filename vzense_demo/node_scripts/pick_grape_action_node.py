#!/usr/bin/env python3

import rospy
import actionlib
from vzense_demo.msg import PickGrapeAction, PickGrapeFeedback, PickGrapeResult
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


def send_robot(ri, robot_model, viewer, send_time=10.0, move=False, arm=None):
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


def recognition_pose(robot_model):
    robot_model.RARM_JOINT0.joint_angle(np.deg2rad(-20))
    robot_model.RARM_JOINT1.joint_angle(np.deg2rad(-80))
    robot_model.RARM_JOINT2.joint_angle(np.deg2rad(90))
    robot_model.RARM_JOINT3.joint_angle(np.deg2rad(-150))

    robot_model.LARM_JOINT0.joint_angle(np.deg2rad(-20))
    robot_model.LARM_JOINT1.joint_angle(np.deg2rad(80))
    robot_model.LARM_JOINT2.joint_angle(np.deg2rad(-90))
    robot_model.LARM_JOINT3.joint_angle(np.deg2rad(-150))


def calibrated_angle_vector(ri, robot_model, target_angles, time=10.0, move=False):
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


def pick_grape(ri, robot_model, send_time=10.0, move=False, step_by_step=True,
               topic_name='/camera/left_vzense_camera/object_detection/output/boxes'):
    if 'left' in topic_name:
        hand = ri.lhand
        arm = robot_model.larm
    else:
        hand = ri.rhand
        arm = robot_model.rarm

    boxes_sub = BoundingBoxArraySubscriber(topic_name, start=True)  # ぶどうの認識器
    recognition_pose(robot_model)
    calibrated_angle_vector(robot_model.angle_vector().copy(), time=send_time / 2.0, move=move)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        left_boxes = boxes_sub.to_coords()
        rate.sleep()
        if left_boxes is not None and len(left_boxes) > 0:
            rospy.loginfo('Detected boxes from {}'.format(boxes_sub.topic_name))
            break
        rospy.logwarn('Waiting detected boxes from {}'.format(boxes_sub.topic_name))

    # offset量(この値を調整する)
    target_pose = left_boxes[0].copy_worldcoords().translate((0.0, 0, -0.06), wrt='world')

    ret = arm.inverse_kinematics(
        target_pose,
        move_target=arm.end_coords,
        stop=100,
        rotation_axis=False)
    if ret is None:
        rospy.logwarn('Could not solve ik. try force ik if failed.')
        ret = arm.inverse_kinematics(
            target_pose,
            move_target=arm.end_coords,
            stop=100,
            rotation_axis=False,
            revert_if_fail=False)
    pick_pose = robot_model.angle_vector()
    arm.inverse_kinematics(
        arm.end_coords.copy_worldcoords().translate((0.01, 0, -0.02), wrt='world'),
        move_target=arm.end_coords,
        stop=100,
        rotation_axis=False,
        revert_if_fail=False)
    pick_pose = robot_model.angle_vector()
    arm.inverse_kinematics(
        arm.end_coords.copy_worldcoords().translate((-0.1, 0, 0)),
        move_target=arm.end_coords,
        stop=100,
        rotation_axis=False,
        revert_if_fail=False)
    pre_pick_pose = robot_model.angle_vector()

    robot_model.angle_vector(pre_pick_pose)
    if move is True:
        hand.move_hand([np.deg2rad(-50), 0, 0, 0], wait=False)
    calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)
    robot_model.angle_vector(pick_pose)
    calibrated_angle_vector(robot_model.angle_vector().copy(), time=5.0, move=move)

    # octomapのために止める処理
    if move is True:
        hand.move_hand([np.deg2rad(-50), np.deg2rad(60), 0, np.deg2rad(60)], wait=False)
        rospy.sleep(5.0)
        hand.init_octomap()
        rospy.sleep(5.0)
        hand.stop_octomap()
        # hand.start_grasp(angle=130, wait=True)
        # hand.move_hand([np.deg2rad(-50), np.deg2rad(130), 0, np.deg2rad(130)], wait=False)
        hand.move_hand([np.deg2rad(-50), np.deg2rad(160), 0, np.deg2rad(160)], wait=False)
        rospy.sleep(5.0)
    robot_model.angle_vector(pre_pick_pose)
    send_robot(send_time=5.0, move=move)


class PickGrapeActionServer(object):

    def __init__(self, name):
        self.robot_model = KARM()

        # 関節速度を制限する設定。早く動かしたい場合は上限を緩和する
        for j in self.robot_model.joint_list:
            if j.max_joint_velocity == 0.0:
                j.max_joint_velocity = np.deg2rad(5)

        self.topic_name = rospy.get_param('~topic_name',
                                          '/camera/left_vzense_camera/object_detection/output/boxes')

        self.robot_model.init_pose()
        self.robot_model.rotate(- np.pi / 2.0, 'y')
        self.viewer = PyrenderViewer()  # viewer
        if 'left' in self.topic_name:
            self.viewer.viewer_flags['window_title'] = 'pick left'
        else:
            self.viewer.viewer_flags['window_title'] = 'pick right'
        self.viewer.add(self.robot_model)
        self.viewer.show()

        rarm_end_coords_axis = Axis.from_coords(self.robot_model.rarm_end_coords)
        self.viewer.add(rarm_end_coords_axis)
        larm_end_coords_axis = Axis.from_coords(self.robot_model.larm_end_coords)
        self.viewer.add(larm_end_coords_axis)

        self.ri = KARMROSRobotInterface(self.robot_model, use_hand=False)  # 実機との接続処理

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            PickGrapeAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        feedback = PickGrapeFeedback()
        result = PickGrapeResult()

        rospy.loginfo('%s: Executing, picking grape with send_time = %f, move = %s' %
                      (self._action_name, goal.send_time, goal.move))

        ########
        ######## 以下でロボットを動かすコードを実行。
        ######## 右手のみ左手のみのを動かすようにcontroller_typeをlarm_controllerもしくはrarm_contlrolerを指定すると指定した方の腕のみ動く

        # 実機を反映してRARM_JOINT3だけ５度動かすサンプル
        # self.robot_model.angle_vector(self.ri.angle_vector())
        # if 'left' in self.topic_name:
        #     self.robot_model.LARM_JOINT3.joint_angle(np.deg2rad(5), relative=True)
        #     self.viewer.redraw()
        #     self.ri.angle_vector(self.robot_model.angle_vector(), 10,
        #                          controller_type='larm_controller')
        # else:
        #     self.robot_model.RARM_JOINT3.joint_angle(np.deg2rad(5), relative=True)
        #     self.viewer.redraw()
        #     self.ri.angle_vector(self.robot_model.angle_vector(), 10,
        #                          controller_type='rarm_controller')

        # 以下は本番環境で調整する想定なのでサンプルを試す場合には上の部分だけコメントイン
        # try:
        #     pick_grape(self.ri, self.robot_model,
        #                send_time=goal.send_time, move=goal.move,
        #                step_by_step=False, topic_name=self.topic_name)
        #     result.success = True
        # except Exception as e:
        #     rospy.logerr("Error during left_pick_grape execution: %s" % str(e))
        #     result.success = False

        if result.success:
            feedback.feedback = "Grape picked successfully"
        else:
            feedback.feedback = "Failed to pick grape"

        self._as.publish_feedback(feedback)
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('pick_grape_action_server')
    server = PickGrapeActionServer(rospy.get_name())
    rospy.spin()
