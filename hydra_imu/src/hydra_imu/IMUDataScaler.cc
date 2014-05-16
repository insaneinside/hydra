/* DUBotics Hydra: IMU raw-data processor.
 * Copyright (C) 2014 DUBotics at the University of Washington
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * The GNU General Public License version 2 may be found at
 * <http://www.gnu.org/licenses/gpl-2.0.html>.
 */

#include <hydra_imu/IMUDataScaler.hh>
#include <hydra_imu/IMUData.h>
#include <hydra_imu/IMUDataLimits.h>
#include <pluginlib/class_list_macros.h>
#include <utility>
#include <limits.h>

#define SUBSCRIBER_QUEUE_SIZE 128
#define PUBLISHER_QUEUE_SIZE 128

PLUGINLIB_EXPORT_CLASS(hydra_imu::IMUDataScaler, nodelet::Nodelet)

namespace MinIMU9v3
{
  /** Acceleration full-scale values, in Gs. */
  uint8_t
  accel_scales[] =
    { 2, 4, 6, 8, 16 };

  /** Magnetometer full-scale values, in Gauss. */
  uint8_t
  mag_scales[] =
    { 2, 4, 8, 12 };

  /** Gyroscope full-scale values, in degrees/sec. */
  uint16_t
  gyro_scales[] =
    { 245, 500, 2000 };
}

template <typename _TpA, typename _TpB>
void
scale_and_store(_TpA& dest, const _TpB& src, double full_scale_value)
{
  double a = full_scale_value / SHRT_MAX;
  dest.x = a * src.x;
  dest.y = a * src.y;
  dest.z = a * src.z;
}


namespace hydra_imu
{
  IMUDataScaler::IMUDataScaler()
    : m_subscriber ( ),
      m_data_publisher ( ),
      m_limits_publisher ( )
  {
  }

  IMUDataScaler::~IMUDataScaler()
  {
  }

  void
  IMUDataScaler::onInit()
  {
    ros::NodeHandle& nh ( getMTNodeHandle() );
    m_subscriber = nh.subscribe<hydra_imu::RawIMUData>("imu/raw", SUBSCRIBER_QUEUE_SIZE,
                                                         &IMUDataScaler::handleRawData, this);
    m_data_publisher = nh.advertise<hydra_imu::IMUData>("imu/data", PUBLISHER_QUEUE_SIZE);
    m_limits_publisher = nh.advertise<hydra_imu::IMUDataLimits>("imu/limits", PUBLISHER_QUEUE_SIZE);

    NODELET_INFO("IMUDataScaler initialized");
  }

  void
  IMUDataScaler::handleRawData(const hydra_imu::RawIMUData::ConstPtr& raw)
  {
    IMUData::Ptr out ( new IMUData );
    uint8_t accel_limit ( MinIMU9v3::accel_scales[raw->linear_acceleration_full_scale] );
    uint8_t mag_limit ( MinIMU9v3::mag_scales[raw->magnetic_field_full_scale] );
    uint16_t gyro_limit ( MinIMU9v3::gyro_scales[raw->angular_velocity_full_scale] );

    scale_and_store(out->linear_acceleration,
                    raw->linear_acceleration,
                    accel_limit);
    scale_and_store(out->angular_velocity,
                    raw->angular_velocity,
                    gyro_limit);
    scale_and_store(out->magnetic_field,
                    raw->magnetic_field,
                    mag_limit);
    m_data_publisher.publish(out);

    IMUDataLimits::Ptr limits ( new IMUDataLimits );
    limits->linear_acceleration = accel_limit;
    limits->angular_velocity = gyro_limit;
    limits->magnetic_field = mag_limit;
    m_limits_publisher.publish(limits);
  }
}
