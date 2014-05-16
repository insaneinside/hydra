/* -*- c++ -*- */
#include <Arduino.h>
#include <ros.h>
#include <LSM303.h>
#include <L3G.h>
#include <hydra_imu/RawIMUData.h>
#include <hydra_imu/IMUConfiguration.h>
#include <Wire.h>
#include <limits.h>
#include "ArduinoIDESucks.h"

static int freeRam () {
  extern int __heap_start, *__brkval; 
  char v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

static ros::NodeHandle_<ArduinoHardware,4,4,172,172> nh;

static hydra_imu::RawIMUData msg;
static ros::Publisher publisher ( "imu/raw", &msg );

static LSM303 accel_mag;               /* accelerometer/magnetometer chip handle */
static L3G gyro_chip;

static unsigned char
  publish_interval = 0;


#define DOWNGRADE_INTERVAL (10*publish_interval)

enum
  {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G = 1,
    ACCEL_SCALE_6G = 2,
    ACCEL_SCALE_8G = 3,
    ACCEL_SCALE_16G = 4
  };
#define ACCEL_MAX_SCALE 4
const uint16_t
accel_downgrade_thresholds[] =
  {
    0,
    SHRT_MAX/2 - 1,
    2*(SHRT_MAX/3) - 1,
    3*(SHRT_MAX/4) - 1,
    SHRT_MAX/2 - 1
  };

enum
  {
    MAG_SCALE_2GAUSS = 0,
    MAG_SCALE_4GAUSS = 1,
    MAG_SCALE_8GAUSS = 2,
    MAG_SCALE_12GAUSS = 3
  };
#define MAG_MAX_SCALE 3
const uint16_t
mag_downgrade_thresholds[] =
  {
    0,
    SHRT_MAX/2 - 1,
    SHRT_MAX/2 - 1,
    2*(SHRT_MAX/3) - 1
  };

enum
  {
    GYRO_SCALE_245DPS = 0,
    GYRO_SCALE_500DPS = 1,
    GYRO_SCALE_2000DPS = 2
    /* note that a value of 3 also corresponds to a full-scale value of 2000
       dps. */
  };
#define GYRO_MAX_SCALE 2
const uint16_t
gyro_downgrade_thresholds[] =
  {
    0,
    SHRT_MAX/2 - 1,
    SHRT_MAX/4 - 1
  };

float
get_sensor_rate(uint8_t shift, uint8_t index)
{
  float o = power<int8_t, float>(2, index - shift, 25.0f);
  return o < 3
    ? 0
    : o;
}

#define GYRO_RATE_SHIFT 1
#define MAG_RATE_SHIFT 3
#define ACCEL_RATE_SHIFT 4

Sensor<LSM303, LSM303::regAddr, LSM303::vector<int16_t> >
  accel(accel_mag,
        SensorParameter<LSM303::regAddr>(LSM303::CTRL1, 0xF0, 4), /* rate: bits 7:4 in CTRL1 (mask 11110000b) */
        SensorParameter<LSM303::regAddr>(LSM303::CTRL2, 0x38, 3), /* scale: bits 5:3 in CTRL2 (mask 00111000b) */
        ACCEL_MAX_SCALE, accel_downgrade_thresholds,
        &LSM303::readAcc, &LSM303::a),
  mag(accel_mag,
      SensorParameter<LSM303::regAddr>(LSM303::CTRL5, 0x1C, 2), /* rate: bits  4:2 in CTRL5 (mask 00011100b) */
      SensorParameter<LSM303::regAddr>(LSM303::CTRL7, 0x60, 5), /* scale: bits 5:6 in CTRL6 (mask 01100000b) */
      MAG_MAX_SCALE, mag_downgrade_thresholds,
      &LSM303::readMag, &LSM303::m);

Sensor<L3G>
  gyro(gyro_chip,
       SensorParameter<>(L3G_CTRL_REG1, 0xC0, 6), /* bits 7:6 in CTRL1 (mask 11000000b) */
       SensorParameter<>(L3G_CTRL_REG4, 0x30, 4), /* bits 4:5 in CTRL4 (mask 00110000b) */
       GYRO_MAX_SCALE, gyro_downgrade_thresholds,
       &L3G::read, &L3G::g);
  
unsigned long
  next_publish = 0,
  next_downgrade_check = 0;



/* void
 * handle_get_configuration(const hydra_imu::GetIMUConfigurationRequest&,
 *                               hydra_imu::GetIMUConfigurationResponse& response)
 * {
 *   response.configuration.accelerometer_rate = accel.rate.get(accel.chip);
 *   response.configuration.accelerometer_scale = accel.scale.get(accel.chip);
 * 
 *   response.configuration.magnetometer_rate = mag.rate.get(mag.chip);
 *   response.configuration.magnetometer_scale = mag.scale.get(mag.chip);
 * 
 *   response.configuration.gyroscope_rate = gyro.rate.get(gyro.chip);
 *   response.configuration.gyroscope_scale = gyro.scale.get(gyro.chip);
 * }
 * 
 * void
 * handle_set_configuration(const hydra_imu::SetIMUConfigurationRequest& request,
 *                               hydra_imu::SetIMUConfigurationResponse&)
 * {
 *   accel.rate.set(accel.chip, request.configuration.accelerometer_rate,);
 *   accel.scale.set(accel.chip, response.configuration.accelerometer_scale);
 * 
 *   response.configuration.magnetometer_rate = mag.rate.get(mag.chip);
 *   response.configuration.magnetometer_scale = mag.scale.get(mag.chip);
 * 
 *   response.configuration.gyroscope_rate = gyro.rate.get(gyro.chip);
 *   response.configuration.gyroscope_scale = gyro.scale.get(gyro.chip);
 * }
 * 
 * static ros::ServiceServer<hydra_imu::GetIMUConfigurationRequest,
 *                           hydra_imu::GetIMUConfigurationResponse>
 *   configuration_server("get_configuration", handle_get_configuration); */



void
setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  nh.getHardware()->setBaud(500000);

  nh.initNode();
  digitalWrite(13, HIGH);

  nh.spinOnce();

  digitalWrite(13, LOW);  

  nh.advertise(publisher);
  /* nh.advertiseService(configuration_server); */

  int val = 200;
  nh.getParam("/imu/publish_rate", &val, 1);
  publish_interval = 1000 / val;
  

  Wire.begin();

  accel_mag.init();
  accel_mag.enableDefault();
  gyro_chip.init();
  gyro_chip.enableDefault();


  /* LSM303D CTRL1: set accelerometer data-rate to 400Hz, enable all
     accelerometer axes, and set the chip to "block data-update" mode, in which
     the output registers are not updated until the previous values have been
     read. */
  accel_mag.writeReg(LSM303::CTRL1, 0x8f);

  /* LSM303D CTRL2: set accelerometer anti-alias filter bandwidth (<7:6>) to
     194 Hz Hz (01b), full scale (<5:3>) to +/- 2g (000b), disable (0b)
     self-test (<1>), and set SPI Serial Interface mode (<0>) to the default
     value (0b).  Bit <2> _must_ be zero.
     Value: 01000000b == 0x40.
  */
  accel_mag.writeReg(LSM303::CTRL2, 0x40);

  /* Put the gyro chip into BDU mode as well. */
  gyro_chip.writeReg(L3G_CTRL_REG4, gyro_chip.readReg(L3G_CTRL_REG4) | 0x80);

  /* Set magnetometer to high resolution and data-rate to 100Hz.
     (no higher rates supported) */
  byte reg = accel_mag.readReg(LSM303::CTRL5);
  accel_mag.writeReg(LSM303::CTRL5, (reg & 0x0C) | 0x74);
                     
  /* Set gyro data-rate to 200Hz */
  /* disable low output-data-rate flag */
  gyro_chip.writeReg(L3G_LOW_ODR, gyro_chip.readReg(L3G_LOW_ODR) & ~0x01);

  /* set data-rate DR<1:0>=10b and bandwidth-selection BW<1:0>=00b,
     PD=1 (normal) and all axes enabled. */
  gyro_chip.writeReg(L3G_CTRL_REG1, 0x8f);

  accel.fetch_scale();
  mag.fetch_scale();
  gyro.fetch_scale();

  accel.rate.set(accel.chip, hydra_imu::IMUConfiguration::ACCELEROMETER_RATE_200);
  mag.rate.set(mag.chip, hydra_imu::IMUConfiguration::MAGNETOMETER_RATE_100);
  /* gyro.rate.set(gyro.chip, hydra_imu::IMUConfiguration::GYROSCOPE_RATE_200); */

  char buf[16];
  snprintf(buf, sizeof(buf), "free ram: %d", freeRam());
  nh.logerror(buf);

  digitalWrite(13, HIGH);
}

void
loop()
{
  nh.spinOnce();
  unsigned long now = millis();

  if ( now >= next_publish )
    {
      PINB |= _BV(PINB5);

      accel.next(msg.linear_acceleration, msg.linear_acceleration_full_scale);
      mag.next(msg.magnetic_field, msg.magnetic_field_full_scale);
      gyro.next(msg.angular_velocity, msg.angular_velocity_full_scale);
      publisher.publish(&msg);


      /* If we've recently upgraded any sensor's scale,
         reset the downgrade-check timer.
       */
      if ( accel.current_scale != accel.selected_scale ||
           mag.current_scale != mag.selected_scale ||
           gyro.current_scale != gyro.selected_scale )
        next_downgrade_check = now + DOWNGRADE_INTERVAL;

      if ( now >= next_downgrade_check )
        {
          accel.check_downgrade();
          mag.check_downgrade();
          gyro.check_downgrade();

          next_downgrade_check = now + DOWNGRADE_INTERVAL;
        }

      next_publish = now + publish_interval;
    }
}
