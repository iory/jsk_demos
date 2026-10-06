#!/usr/bin/env python3
"""Copy a robot_description for rviz without some links.

Waits for ``~source`` (the robot's /robot_description, set by its bringup),
removes the links matching ``~drop`` (regex) and their joints, and writes the
result to ``~target``. teleop.launch real:=true uses it to show the robot
from the arm base, without the linear stage and the arm that is not
teleoperated.
"""

import rospy

from k_arm_teleop.urdf_utils import drop_links


def main():
    rospy.init_node('display_description')
    source = rospy.get_param('~source', '/robot_description')
    target = rospy.get_param('~target')
    pattern = rospy.get_param('~drop', '')
    deadline = rospy.get_time() + rospy.get_param('~timeout', 30.0)
    while not rospy.is_shutdown() and not rospy.has_param(source):
        if rospy.get_time() > deadline:
            rospy.logerr('%s is not set: the robot bringup has to be running', source)
            return
        rospy.sleep(0.5)
    try:
        urdf = drop_links(rospy.get_param(source), pattern)
    except ValueError as error:
        rospy.logerr('cannot filter %s: %s', source, error)
        return
    rospy.set_param(target, urdf)
    rospy.loginfo('%s: %s without links matching %r', target, source, pattern)


if __name__ == '__main__':
    main()
