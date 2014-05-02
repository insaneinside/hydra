#include <Arduino.h>
#include <ros.h>
#include <LSM303.h>
#include <L3G.h>
#include <geometry_msgs/Vector3.h>
#include <hydra_tests/RawIMUData.h>
#include <Wire.h>
#include "ArduinoIDESucks.h"

#define MAX_STORED_SAMPLES 4
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

struct Vector3Publisher
{
  hydra_tests::RawIMUData msg;
  ros::Publisher publisher;

  inline
  Vector3Publisher(const char* name)
    : msg (),
      publisher ( name, &msg )
  {
  }

  inline void
  publish()
  {
    publisher.publish(&msg);
  }
};

static LSM303 accel_mag;               /* accelerometer/magnetometer chip handle */
static L3G gyro;

static ros::NodeHandle nh;

static Vector3Publisher
  paccel ( "accel" ),
  pmag ( "mag" ),
  pgyro ( "gyro" );

static unsigned char
  publish_interval = 0,
  update_interval = 0,
  samples_per_message = 0;

static int16_t
  accel_data[MAX_STORED_SAMPLES][3],
  mag_data[MAX_STORED_SAMPLES][3],
  gyro_data[MAX_STORED_SAMPLES][3];

static unsigned char n;

static void
fetch(const uint16_t** src, hydra_tests::RawIMUData& dest)
{
  dest.x = 0; dest.y = 0; dest.z = 0;

  for ( unsigned char i = 0; i < samples_per_message; ++i )
    {
      unsigned char m = (n - i) % MAX_STORED_SAMPLES;
      dest.x += src[m][0];
      dest.y += src[m][1];
      dest.z += src[m][2];
    }

  dest.x /= samples_per_message;
  dest.y /= samples_per_message;
  dest.z /= samples_per_message;
}


void
setup()
{
  nh.initNode();

  nh.spinOnce();
  while ( ! nh.connected() )
    nh.spinOnce();

  nh.advertise(paccel.publisher);
  nh.advertise(pmag.publisher);
  nh.advertise(pgyro.publisher);

  memset(accel_data, 0, sizeof(int16_t) * 3 * MAX_STORED_SAMPLES);
  memset(mag_data, 0, sizeof(int16_t) * 3 * MAX_STORED_SAMPLES);
  memset(gyro_data, 0, sizeof(int16_t) * 3 * MAX_STORED_SAMPLES);

  int val = 50;
  nh.getParam("/imu/publish_rate", &val, 1);
  publish_interval = 1000 / val;
  
  val = 100;
  nh.getParam("/imu/update_rate", &val, 1);
  update_interval = 1000 / val;
  
  val = 2;
  nh.getParam("/imu/samples_per_message", &val, 1);
  samples_per_message = constrain(val, 1, 64);
  
  Wire.begin();


  accel_mag.init();
  accel_mag.enableDefault();

  gyro.init();
  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyro.enableDefault();

  char buf[32];
  snprintf(buf, sizeof(buf), "free mem: %d", freeRam());
  nh.logerror(buf);
}

 long next_update = 0;
unsigned long next_publish = 0;

void
loop()
{
  unsigned long now = millis();
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "free mem: %d", freeRam());
    nh.logerror(buf);
  }
  if ( now >= next_update )
    {
      accel_mag.read();
      store((uint16_t**)accel_data, accel_mag.a, n);
      store((uint16_t**)mag_data, accel_mag.m, n);

      gyro.read();
      store((uint16_t**)gyro_data, gyro.g, n);

      n = (n+1) % MAX_STORED_SAMPLES;

      next_update = now + update_interval;
    }

  if ( now >= next_publish )
    {
      fetch((const uint16_t**)accel_data, paccel.msg);
      fetch((const uint16_t**)mag_data, pmag.msg);
      fetch((const uint16_t**)gyro_data, pgyro.msg);

      paccel.publish();
      pmag.publish();
      pgyro.publish();

      next_publish = now + publish_interval;
    }

  nh.spinOnce();
}
