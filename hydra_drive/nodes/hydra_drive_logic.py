#!/usr/bin/env python

import os
import sys
import rospy

from hydra_drive.msg import PowerLevels

TIMEOUT = 1.0                   # one second


class Data(object):
    def __init__(self):
        self.message = PowerLevels()
        self.timeout = None

rospy.init_node('hydra_drive_logic')

data = Data()
pub = rospy.Publisher('power', PowerLevels)

def halt(event):
    data.message.left = 0.0
    data.message.right = 0.0
    pub.publish(data.message)

def handle_joystick_received(msg):
    if data.timeout:
        data.timeout.shutdown()
    data.timeout = rospy.Timer(rospy.Duration(TIMEOUT), halt, True)
    data.message = msg

sub = rospy.Subscriber('joystick', PowerLevels, handle_joystick_received)


def run():
    rospy.loginfo('hydra_drive_logic initialized.')

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        pub.publish(data.message)
        r.sleep()

if __name__ == '__main__':
    run()
