#!/usr/bin/env python

import socket
import time

import rospy
import std_msgs.msg


def send_pisugar_command(command_str):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect(('localhost', 8423))
        s.sendall(command_str.encode())
        time.sleep(0.1)  # Wait for pisugar to response
        pisugar_str = s.recv(1024).decode()
    except Exception as e:
        print('{}: {}'.format(type(e), e))
    finally:
        s.close()
    return pisugar_str

def get_battery():
    battery_str = send_pisugar_command('get battery')
    try:
        # battery: [Battery Level]\n -> [Battery Level]
        tmp = battery_str.split(':')
        if len(tmp) > 1:
            battery_str = tmp[1].replace('\n', '')
            battery_level = float(battery_str)
        else:
            battery_level = 0.0
    except Exception as e:
        print('{}: {}'.format(type(e), e))
        print('get battery result: {}'.format(battery_str))
        battery_level = 0.0
    return(battery_level)


def publiship_battery():
    rate = rospy.Rate(1)
    pub = rospy.Publisher('/visualization/battery/value',
                          std_msgs.msg.Float32,
                          queue_size=1)
    while not rospy.is_shutdown():
        battery_value = get_battery()
        pub.publish(std_msgs.msg.Float32(battery_value))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('battery_publisher')
    publiship_battery()
