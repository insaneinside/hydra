#!/usr/bin/env python

"""
Listens for power-level values on {rotation,joint0,joint1,joint2}/power (all of
type std_msgs/Float32) and publishes corresponding PWM signal widths on
{...}/signal (type std_msgs/UInt16).

"""

import rospy
import std_msgs.msg

class Mapper(object):
    def __init__(self, inputs, outputs):
        """
        inputs: [min, neutral, max] arbitrary values
        outputs: [min, neutral, max] in milliseconds for PWM signal pulse widths
        """
        assert(len(inputs) == 3)
        assert(len(outputs) == 3)
        self.inputs = sorted(inputs)
        self.outputs = sorted(outputs)

    def map(self, value):
        if value < self.inputs[1]:
            return ((value - self.inputs[0])
                    * (self.outputs[1] - self.outputs[0])
                    / (self.inputs[1] - self.inputs[0])) + self.outputs[0]
        elif value > self.inputs[1]:
            return ((value - self.inputs[1])
                    * (self.outputs[2] - self.outputs[1])
                    / (self.inputs[2] - self.inputs[1])) + self.outputs[1]
        else:
            return self.outputs[1]

TALON_MAPPING = Mapper([-1.0, 0.0, 1.0],
                        [1001, 1500, 2041])
SPIKE_MAPPING = Mapper([-1.0, 0.0, 1.0],
                        [1000, 1500, 2000])

class JointPowerSignalMapper(object):
    def __init__(self, name, mapping):
        self.mapper = mapping
        self.subscriber = rospy.Subscriber('%s/power' % name, std_msgs.msg.Float32, self.translate)
        self.publisher = rospy.Publisher('%s/signal' % name, std_msgs.msg.UInt16)

    def translate(self, power_level):
        """
        Translate and republish a received power-level message for the joint.
        """
        self.publisher.publish(std_msgs.msg.UInt16(self.mapper.map(power_level.data)))


def main():
    rospy.init_node("hydra_arm_motor_control")
    rospy.logout('Hydra Arm motor-control initialized')

    mappers = {}
    mapper_names_mappings = {'rotation' : TALON_MAPPING,
                             'joint0' : SPIKE_MAPPING,
                             'joint1' : SPIKE_MAPPING,
                             'joint2' : SPIKE_MAPPING}
    for name in mapper_names_mappings:
        mappers[name] = JointPowerSignalMapper(name, mapper_names_mappings[name])
    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
