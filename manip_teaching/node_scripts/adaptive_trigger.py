#!/usr/bin/env python

from __future__ import print_function

import argparse
import numpy as np

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
import rospy
from skrobot.model import RobotModel

from one_button import Button

pump_state = True


def main():
    global pump_state
    parser = argparse.ArgumentParser(
        description='Run KXRROSRobotInterface')
    parser.add_argument(
        '--namespace', type=str, help='Specify the ROS namespace', default='')
    args = parser.parse_args()

    rospy.init_node('kxr_interface', anonymous=True)

    download_urdf_mesh_files(args.namespace)

    robot_model = RobotModel()
    robot_model.load_urdf_from_robot_description(
        args.namespace + '/robot_description_viz')
    ri = KXRROSRobotInterface(  # NOQA
        robot_model, namespace=args.namespace)

    ri.send_stretch(5)
    ri.servo_on()
    target_angle = -1.7482963
    ri.angle_vector(len(ri.angle_vector()) * [target_angle])

    board_idx = 40
    ri.control_air_board(board_idx, 'open_work_valve')
    ri.control_air_board(board_idx, 'close_work_valve')
    rate = rospy.Rate(30)
    rospy.loginfo('ready')

    def on_click():
        rospy.loginfo("Single click detected")

    def on_double_click():
        rospy.loginfo("Double click detected")

    def on_triple_click():
        rospy.loginfo("Triple click detected")

    def on_long_press():
        global pump_state
        rospy.loginfo("Long press detected")
        pump_state = not pump_state
        if pump_state:
            ri.control_air_board(board_idx, 'open_work_valve')
            ri.control_air_board(board_idx, 'close_relay_valve')
        else:
            ri.control_air_board(board_idx, 'close_work_valve')
            ri.control_air_board(board_idx, 'open_relay_valve')

    def on_long_press_stop():
        rospy.loginfo("Long press stopped")

    button = Button()
    button.set_click_callback(on_click)
    button.set_double_click_callback(on_double_click)
    button.set_triple_click_callback(on_triple_click)
    button.set_long_press_callback(on_long_press)
    button.set_long_press_stop_callback(on_long_press_stop)

    while not rospy.is_shutdown():
        if np.any(ri.angle_vector() < -1.90):
            state = True
        else:
            state = False
        button.tick(state)
        rate.sleep()


if __name__ == '__main__':
    main()
