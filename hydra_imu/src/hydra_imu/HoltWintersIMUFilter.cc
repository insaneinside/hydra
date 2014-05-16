/* DUBotics Hydra: Holt-Winters smoothing filter, vector implementation.
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

#include <hydra_imu/HoltWintersIMUFilter.hh>
#include <pluginlib/class_list_macros.h>
#include <utility>
#include <limits.h>
#include <cassert>
#include <functional>

#define ORDER 1
#define ALPHA 0.5
#define BETA 0.25

PLUGINLIB_EXPORT_CLASS(hydra_imu::HoltWintersIMUFilter, nodelet::Nodelet)

namespace hydra_imu
{
  static inline bool
  update_smoothed_value(size_t n,
                        const HoltWintersParameters& params,
                        HoltWintersData& data,
                        double input, double& output)
  {
    if ( n == 0 )
      {
        data.smoothed_value = input;
        data.trend_estimate = input;
      }
    else if ( n == 1 )
      {
        data.smoothed_value = input;
        data.trend_estimate = input - data.trend_estimate;
      }
    else
      {
        /* Handle infinity and NaN values somewhat gracefully. */
        if ( isinf(data.smoothed_value) || isnan(data.smoothed_value ) )
          data.smoothed_value = input;
        if ( isinf(data.trend_estimate) || isnan(data.trend_estimate) )
          data.trend_estimate = input;

        double sv0 = data.smoothed_value;
        data.smoothed_value = params.alpha*input + params.oma*(sv0+data.trend_estimate);
        data.trend_estimate = params.beta*(data.smoothed_value-sv0) + params.omb*data.trend_estimate;

        output = data.smoothed_value + data.trend_estimate;
      }

    return n > 1;
  }

  HoltWintersVector3Filter::HoltWintersVector3Filter()
    : HoltWintersVector3Filter::ParentType({ALPHA, BETA, 1-ALPHA, 1-BETA},
                                           &update_smoothed_value)
  {
  }

  HoltWintersVector3Filter::~HoltWintersVector3Filter()
  {
  }


  HoltWintersIMUFilter::HoltWintersIMUFilter()
    : HoltWintersIMUFilter::ParentType( ),
      m_dynamic_configuration_server ( )
  {
  }

  HoltWintersIMUFilter::~HoltWintersIMUFilter()
  {
  }

  void
  HoltWintersIMUFilter::onInit()
  {
    ParentType::onInit();

    ros::NodeHandle& nh ( getMTNodeHandle() );

    m_dynamic_configuration_server.init(getName(), nh);
    m_dynamic_configuration_server.setCallback(std::bind(&HoltWintersIMUFilter::reconfigure, this,
                                                         std::placeholders::_1,
                                                         std::placeholders::_2));

    double a, b; /* int o; */
    /* nh.param("linear_acceleration/order", o, ORDER); */
    nh.param("linear_acceleration/alpha", a, ALPHA);
    nh.param("linear_acceleration/beta", b, BETA);
    m_linear_acceleration.parameters = { a, b, 1-a, 1-b };
  
    /* nh.param("angular_velocity/order", o, ORDER); */
    nh.param("angular_velocity/alpha", a, BETA);
    nh.param("angular_velocity/beta", b, ALPHA);
    m_angular_velocity.parameters = { a, b, 1-a, 1-b };
  
    /* nh.param("magnetic_field/order", o, ORDER); */
    nh.param("magnetic_field/alpha", a, ALPHA);
    nh.param("magnetic_field/beta", b, BETA);
    m_magnetic_field.parameters = { a, b, 1-a, 1-b };
  

    NODELET_INFO("HoltWintersIMUFilter initialized");


    /* Make sure the configuration server knows the initial configuration. */
    hydra_imu::HoltWintersIMUFilterConfig cfg;
    cfg.linear_acceleration_alpha = m_linear_acceleration.parameters.alpha;
    cfg.linear_acceleration_beta = m_linear_acceleration.parameters.beta;
    cfg.angular_velocity_alpha = m_angular_velocity.parameters.alpha;
    cfg.angular_velocity_beta = m_angular_velocity.parameters.beta;
    cfg.magnetic_field_alpha = m_magnetic_field.parameters.alpha;
    cfg.magnetic_field_beta = m_magnetic_field.parameters.beta;
    m_dynamic_configuration_server.sync(cfg);

  }
  
  void
  HoltWintersIMUFilter::reconfigure(HoltWintersIMUFilterConfig& config, uint32_t level)
  {
    NODELET_DEBUG("HoltWintersIMUFilter: reconfigure request, level %u", level);

    double a, b; /* int o; */
    if ( level & 0x03 )
      {
        /* o = level & 0x01 ? config.linear_acceleration_order : m_linear_acceleration.parameters.order; */
        a = level & 0x01 ? config.linear_acceleration_alpha : m_linear_acceleration.parameters.alpha;
        b = level & 0x02 ? config.linear_acceleration_beta : m_linear_acceleration.parameters.beta;
        m_linear_acceleration.parameters = { a, b, 1-a, 1-b };
        NODELET_INFO("linear_acceleration: alpha = %f, beta = %f",
                     m_linear_acceleration.parameters.alpha, m_linear_acceleration.parameters.beta);
      }
  
    if ( level & 0x0C )
      {
        /* o = level & 0x01 ? config.angular_velocity_order : m_angular_velocity.parameters.order; */
        a = level & 0x04 ? config.angular_velocity_alpha : m_angular_velocity.parameters.alpha;
        b = level & 0x08 ? config.angular_velocity_beta : m_angular_velocity.parameters.beta;
        m_angular_velocity.parameters = { a, b, 1-a, 1-b };
        NODELET_INFO("angular_velocity: alpha = %f, beta = %f",
                     m_angular_velocity.parameters.alpha, m_angular_velocity.parameters.beta);
      }

    if ( level & 0x30 )
      {
        /* o = level & 0x01 ? config.magnetic_field_order : m_magnetic_field.parameters.order; */
        a = level & 0x10 ? config.magnetic_field_alpha : m_magnetic_field.parameters.alpha;
        b = level & 0x20 ? config.magnetic_field_beta : m_magnetic_field.parameters.beta;
        m_magnetic_field.parameters = { a, b, 1-a, 1-b };
        NODELET_INFO("magnetic_field: alpha = %f, beta = %f",
                     m_magnetic_field.parameters.alpha, m_magnetic_field.parameters.beta);
      }
  }

}
