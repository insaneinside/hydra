// -*- C++ -*-
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include <Arduino.h>
#include <Servo.h> 

#include <ros.h>
#include <hydra_drive/PowerLevels.h>

#define PIN_LEFT 9
#define PIN_RIGHT 10

#define MIN_PULSE_WIDTH 1001
#define NEUTRAL_PULSE_WIDTH 1500
#define MAX_PULSE_WIDTH 2041

ros::NodeHandle  nh;
ros::Subscriber<hydra_drive::PowerLevels> subscription ( "power", power_cb );
Servo left, right;

void power_cb(const hydra_drive::PowerLevels& msg) {
  left.write_ms(map(data.left, -1.0, 1.0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  right.write_ms(map(data.right, -1.0, 1.0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));

}

void setup(){
  nh.initNode();
  
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);


  left.attach(PIN_LEFT);
  left.write_ms(NEUTRAL_PULSE_WIDTH);

  right.attach(PIN_RIGHT);
  right.write_ms(NEUTRAL_PULSE_WIDTH);
}

void loop(){
  nh.spinOnce();
}
