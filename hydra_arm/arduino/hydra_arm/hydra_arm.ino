#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include "JointSignalSubscriber.h"

const JointPinMapping
joints_mappings[] =
  { { "rotation", JointPinMapping::TALON, 2 },
    { "joint0",   JointPinMapping::SPIKE, 3, 4 },
    { "joint1",   JointPinMapping::SPIKE, 5, 6 },
    { "joint2",   JointPinMapping::SPIKE, 6, 7 } };

ros::NodeHandle nh;

JointSignalSubscriber subscribers[] =
  { JointSignalSubscriber(joints_mappings[0]),
    JointSignalSubscriber(joints_mappings[1]),
    JointSignalSubscriber(joints_mappings[2]),
    JointSignalSubscriber(joints_mappings[3]) };

void
setup()
{
  nh.initNode();
  for ( uint8_t i = 0; i < sizeof(subscribers) / sizeof(JointSignalSubscriber); ++i )
    {
      nh.subscribe(subscribers[i]);
      subscribers[i].attach();
    }
}


void
loop()
{
  nh.spinOnce();
}
