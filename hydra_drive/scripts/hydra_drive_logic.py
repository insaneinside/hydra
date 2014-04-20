import os
import sys
import rospy

from hydra_drive.msg import PowerLevels

TIMEOUT = 1.0                   # one second

last_receive_time = None
message = None

def handle_power_received(msg):
    last_receive_time = rospy.get_time()
    message = msg

def run():
    rospy.init_node('hydra_drive_logic')

    pub = rospy.Publisher('power', PowerLevels)
    sub = rospy.Subscriber('joystick', PowerLevels, handle_power_received)
    message = PowerLevels()

    rospy.loginfo('hydra_drive_logic initialized.')

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        if last_receive_time is None or rospy.get_time() - last_receive_time >= TIMEOUT:
            message.left = 0
            message.right = 0
        pub.publish(message)
            
        r.sleep()

if __name__ == '__main__':
    run()
