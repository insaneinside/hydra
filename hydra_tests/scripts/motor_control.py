#!/usr/bin/env python

import os
import sys
import rospy
from threading import Lock

from std_msgs.msg import UInt8
from hydra_crisp.msg import AxisState

AXIS_IDS = (1,5)
AXIS_NUMBERS = (1,3)
DEADZONE = .016

publishers = {}

last_data = {}

def _map(stick_y):
    """
    Map a stick input to a "motor" (servo) output.
    
    """
    if abs(stick_y) > DEADZONE:
        return 90 * (1.0 + stick_y)
    else:
        return 90;

def received_axis_data(state):
    global lock
    lock.acquire()
    last_data[state.id] = state.mapped
    lock.release()
    
def run():
    rospy.init_node('control')
    sys.stderr.write('Goodbye, Moon!\n')

    for i in xrange(len(AXIS_IDS)):
        n = AXIS_IDS[i]
        publishers[n] = rospy.Publisher('motor%d'%(i+1), UInt8)

    for n in AXIS_NUMBERS:
        rospy.Subscriber('axis%d'%n, AxisState, received_axis_data)

    r = rospy.Rate(50)
    global lock
    lock = Lock()

    while not rospy.is_shutdown():
        lock.acquire()
        for n in AXIS_IDS:
            try:
                publishers[n].publish(_map(last_data[n]))
            except Exception as err:
                print(str(err))
        lock.release()
        r.sleep()

# if __name__ == '__main__':
run()
