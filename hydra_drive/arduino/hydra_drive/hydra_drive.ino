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

static void power_cb(const hydra_drive::PowerLevels& msg);

static ros::NodeHandle  nh;
static ros::Subscriber<hydra_drive::PowerLevels> subscription ( "power", power_cb );
static Servo left, right;

static void power_cb(const hydra_drive::PowerLevels& msg) {
  left.writeMicroseconds(map(msg.left, -1.0, 1.0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  right.writeMicroseconds(map(msg.right, -1.0, 1.0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
}

void setup(){
  nh.initNode();
  
  nh.subscribe(subscription);

  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);


  left.attach(PIN_LEFT);
  left.writeMicroseconds(NEUTRAL_PULSE_WIDTH);

  right.attach(PIN_RIGHT);
  right.writeMicroseconds(NEUTRAL_PULSE_WIDTH);
}

void loop(){
  nh.spinOnce();
}
