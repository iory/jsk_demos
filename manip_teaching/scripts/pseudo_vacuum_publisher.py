#!/usr/bin/env python3

import sys
import os

import rospy
from std_msgs.msg import Float32


if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios


def getKey():
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def talker():
    pub = rospy.Publisher('/vacuum_pressure', Float32, queue_size=10)
    rate = rospy.Rate(10)
    suctioned = False
    while not rospy.is_shutdown():
        key = getKey()
        if key == '\x0d':
            print("Enter pressed, publishing...")
            if suctioned:
                pressure = Float32(data=1000)
            else:
                pressure = Float32(data=0)
            suctioned = not suctioned
            pub.publish(pressure)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pseudo_vacuum_pressure', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
