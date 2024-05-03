#!/usr/bin/env python3

import sys
import os

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32


pub = None
suctioned = False


def callback(msg):
    global pub
    global suctioned

    if msg.data == 1:
        if suctioned:
            pressure = Float32(data=1000)
        else:
            pressure = Float32(data=0)
        suctioned = not suctioned
        pub.publish(pressure)


if __name__ == '__main__':
    rospy.init_node('pseudo_vacuum_pressure', anonymous=True)
    pub = rospy.Publisher('/vacuum_pressure', Float32, queue_size=10)
    sub = rospy.Subscriber('/atom_s3_button_state',
                           Int32, queue_size=1,
                           callback=callback)
    rospy.spin()
