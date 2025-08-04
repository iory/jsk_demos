import actionlib
import control_msgs.msg
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import Empty

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
from vzense_demo.k_arm import KARM


class HandInterface(object):

    def __init__(self, hand='rhand', use_hand=True):
        self.hand = hand
        self.use_hand = use_hand
        if self.use_hand is False:
            rospy.logwarn('{} interface is disabled.'.format(self.hand))
        else:
            rospy.loginfo('Waiting {} startup'.format(hand))
            self.init_hand(hand)
            rospy.loginfo('Waiting {} initialization end.'.format(hand))

    def init_hand(self, hand):
        self.hand = hand
        self.jnt_names = []
        for finger in ['thumb', 'index']:
            self.jnt_names.append(
                '{}_{}_base_joint'.format(self.hand, finger))
            self.jnt_names.append(
                '{}_{}_joint'.format(self.hand, finger))
        self.jnt_pos = None
        self.rarm_jnt_traj_act = actionlib.SimpleActionClient(
            ('{}/position_joint_trajectory_controller/'
                'follow_joint_trajectory').format(self.hand),
            control_msgs.msg.FollowJointTrajectoryAction,
        )
        self.rarm_jnt_traj_act.wait_for_server()
        self.enable_int_srvs = []
        self.enable_tof_srvs = []
        for finger in ['thumb', 'index']:
            for pos in ['tip', 'root']:
                int_name = '{}/{}/distal/{}/enable_intensity'.format(
                    self.hand, finger, pos)
                rospy.wait_for_service(int_name)
                self.enable_int_srvs.append(
                    rospy.ServiceProxy(int_name, SetBool)
                )
                tof_name = '{}/{}/distal/{}/enable_tof'.format(
                    self.hand, finger, pos)
                rospy.wait_for_service(tof_name)
                self.enable_tof_srvs.append(
                    rospy.ServiceProxy(tof_name, SetBool)
                )
        set_init_i_srv_names = []
        for finger in ['thumb', 'index']:
            for pos in ['tip', 'root']:
                set_init_i_srv_names.append(
                    '{}/{}/distal/{}/intensity_model_acquisition/set_init_intensity'.format(
                        self.hand, finger, pos)
                )
        self.set_init_i_srvs = []
        for srv_name in set_init_i_srv_names:
            rospy.wait_for_service(srv_name)
            self.set_init_i_srvs.append(
                rospy.ServiceProxy(srv_name, Trigger))

        cloud_req_srv_name = '{}/wrpps_pointcloud_passthrough/request'.format(
            self.hand)
        rospy.wait_for_service(cloud_req_srv_name)
        self.cloud_req_srv = rospy.ServiceProxy(cloud_req_srv_name, Empty)
        cloud_stop_srv_name = '{}/wrpps_pointcloud_passthrough/stop'.format(
            self.hand)
        rospy.wait_for_service(cloud_stop_srv_name)
        self.cloud_stop_srv = rospy.ServiceProxy(cloud_stop_srv_name, Empty)

    def move_hand(self, target_pos, time=1, wait=True):
        if self.use_hand is False:
            rospy.logwarn('{} interface is disabled.'.format(self.hand))
            return
        rospy.loginfo('Current joint position: {}'.format(self.jnt_pos))
        rospy.loginfo('Target joint position: {}'.format(target_pos))
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time(0)  # Immediate execution
        traj_msg.joint_names = self.jnt_names
        traj_msg.points.append(
            JointTrajectoryPoint(
                positions=target_pos,
                time_from_start=rospy.Duration(time),
            )
        )
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = traj_msg
        self.rarm_jnt_traj_act.send_goal(goal)
        if wait:
            self.rarm_jnt_traj_act.wait_for_result()

    def init_octomap(self):
        # handの間にのがない状態を確認して実行
        self.enable_int_srvs[1](False)
        self.enable_int_srvs[3](False)
        self.enable_tof_srvs[1](False)
        self.enable_tof_srvs[3](False)
        rospy.sleep(0.1)

        # Initialize intensity_model_acquisition
        for srv in self.set_init_i_srvs:
            srv()

        self.cloud_req_srv()

    def stop_octomap(self):
        self.cloud_stop_srv()

        # Re-enable all sensors
        self.enable_int_srvs[1](True)
        self.enable_int_srvs[3](True)
        self.enable_tof_srvs[1](True)
        self.enable_tof_srvs[3](True)

    def start_grasp(self, time=1, wait=True,
                    angle=90):
        angle = max(min(angle, 130), 0)
        self.move_hand([-np.pi/2, np.deg2rad(angle), 0, np.deg2rad(angle)],
                       time=time, wait=wait)

    def stop_grasp(self, time=1, wait=True):
        self.move_hand([-np.pi/2, 0, 0, 0],
                       time=time, wait=wait)

    def grasp_box_pose(self):
        self.move_hand([0, 0, -np.deg2rad(30), 0], wait=True)
        # self.move_hand([0, np.deg2rad(150), -np.deg2rad(30), np.deg2rad(0)], wait=True)
        self.move_hand([0, np.deg2rad(150), -np.deg2rad(30), np.deg2rad(130)], waite=False)

    def reset_octomap(self):
        if self.use_hand is False:
            rospy.logwarn('{} interface is disabled.'.format(self.hand))
            return
        service_name = "{}/octomap_server/reset".format(self.hand)
        rospy.loginfo('{} reset_octomap called. waiting service {}'.format(self.hand, service_name))
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, Empty)
        ret = service()
        rospy.loginfo('{} octomap successfully called'.format(self.hand))
        return ret

    def servo_on(self):
        if self.use_hand is False:
            rospy.logwarn('{} interface is disabled.'.format(self.hand))
            return
        service_name = "{}/servo_state_manager/servo".format(self.hand)
        rospy.loginfo('{} servo on called. waiting service {}'.format(self.hand, service_name))
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, SetBool)
        ret = service(SetBoolRequest(data=True))
        rospy.loginfo('{} servo on successfully called'.format(self.hand))
        return ret

    def servo_off(self):
        if self.use_hand is False:
            rospy.logwarn('{} interface is disabled.'.format(self.hand))
            return
        service_name = "{}/servo_state_manager/servo".format(self.hand)
        rospy.loginfo('{} servo off called. waiting service {}'.format(self.hand, service_name))
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, SetBool)
        ret = service(SetBoolRequest(data=False))
        rospy.loginfo('{} servo off successfully called'.format(self.hand))
        return ret


class KARMROSRobotInterface(ROSRobotInterfaceBase):

    def __init__(self, *args, **kwargs):
        use_hand = kwargs.pop('use_hand', True)
        super(KARMROSRobotInterface, self).__init__(*args, **kwargs)
        self.rhand = HandInterface('rhand', use_hand)
        self.lhand = HandInterface('lhand', use_hand)

    @property
    def rarm_controller(self):
        return dict(
            controller_type='rarm_controller',
            controller_action='rarm_controller/follow_joint_trajectory_action',  # NOQA
            controller_state='rarm_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[
                "RARM_JOINT0",
                "RARM_JOINT1",
                "RARM_JOINT2",
                "RARM_JOINT3",
                "RARM_JOINT4",
                "RARM_JOINT5",
                "RARM_JOINT6",
            ],
        )


    @property
    def larm_controller(self):
        return dict(
            controller_type='larm_controller',
            controller_action='larm_controller/follow_joint_trajectory_action',  # NOQA
            controller_state='larm_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[
                "LARM_JOINT0",
                "LARM_JOINT1",
                "LARM_JOINT2",
                "LARM_JOINT3",
                "LARM_JOINT4",
                "LARM_JOINT5",
                "LARM_JOINT6",
            ],
        )

    def default_controller(self):
        return [self.rarm_controller,
                self.larm_controller]

    def start_grasp(self):
        self.move_hand([-np.pi/2, np.pi/2, 0, np.pi/2], 1, wait=True)

    def stop_grasp(self):
        self.move_hand([-np.pi/2, 0, 0, 0], 1, wait=True)


if __name__ == '__main__':
    from skrobot.viewers import TrimeshSceneViewer
    from skrobot.viewers import PyrenderViewer
    from skrobot.model import Axis
    robot_model = KARM()

    for j in robot_model.joint_list:
        if j.max_joint_velocity == 0.0:
            j.max_joint_velocity = np.deg2rad(5)

    robot_model.init_pose()
    robot_model.rotate(- np.pi / 2.0, 'y')
    viewer = PyrenderViewer()
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

    rarm_target_coords = Axis()
    viewer.add(rarm_target_coords)

    ri = KARMROSRobotInterface(robot_model)
    robot_model.angle_vector(ri.angle_vector())
    viewer.redraw()
