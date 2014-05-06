#!/usr/bin/env python

"""
Listens on "input" (type sensor_msgs/Joy) and writes individual axis values on
{rotation,joint0,joint1,joint2}/power (all type std_msgs/Float32).

"""

import rospy
import std_msgs.msg
import sensor_msgs.msg

DEFAULT_RATE=50
JOINT_NAMES=['rotation', 'joint0', 'joint1', 'joint2']

class Publisher(object):
    def __init__(self):
        self.values = {}
        self.publishers = {}

        self.subscriber = rospy.Subscriber("input", sensor_msgs.msg.Joy, self.update)

        for name in JOINT_NAMES:
            self.values[name] = 0.0
            self.publishers[name] = rospy.Publisher('%s/power' % name, std_msgs.msg.Float32)

    def publish(self):
        for name in self.publishers:
            self.publishers[name].publish(data=self.values[name])

    def update(self, msg):
        self.values['rotation'] = -msg.buttons[4] + msg.buttons[5]
        self.values['joint0'] = -msg.buttons[6] + msg.buttons[7]
        self.values['joint1'] = msg.axes[1]
        self.values['joint2'] = msg.axes[3]

def main():
    rospy.init_node("hydra_arm_teleop")
    rospy.logout('Hydra Arm teleop initialized')
    rate = rospy.get_param('~rate') if rospy.has_param('~rate') else DEFAULT_RATE
    pub = Publisher()
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        pub.publish()
        r.sleep()

if __name__ == '__main__':
    main()
