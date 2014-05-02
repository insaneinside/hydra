/* DUBotics Hydra: drive-control low-level hardware outputs mapper.
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

#include "TwistSpeedMapper.hh"
#include <pluginlib/class_list_macros.h>
#include <hydra_drive/TreadSpeeds.h>
#include <utility>

#define SUBSCRIBER_QUEUE_SIZE 128
#define PUBLISHER_QUEUE_SIZE 128

PLUGINLIB_DECLARE_CLASS(hydra_drive, TwistSpeedMapper, hydra_drive::TwistSpeedMapper, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(hydra_drive::TwistSpeedMapper, nodelet::Nodelet)

namespace hydra_drive
{

  TwistSpeedMapper::TwistSpeedMapper()
    : m_subscriber ( ),
      m_publisher ( ),
      m_wheel_base ( 1.0 )
  {
  }

  TwistSpeedMapper::~TwistSpeedMapper()
  {
  }


  void TwistSpeedMapper::onInit()
  {
    ros::NodeHandle& nh ( getMTPrivateNodeHandle() );
    nh.getParam("wheel_base", m_wheel_base);
    m_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", SUBSCRIBER_QUEUE_SIZE, 
                                                      &TwistSpeedMapper::handleCmdVel, this);
    m_publisher = nh.advertise<hydra_drive::TreadSpeeds>("tread_speeds_target", PUBLISHER_QUEUE_SIZE);;
    NODELET_INFO("TwistSpeedMapper initialized.");
  }

  /** Update the target tread speeds from a geometry_msgs/Twist message.
   * Because Hydra is a tank-drive design, we can only (voluntarily!) move
   * linearly in the X (forward) direction and angularly in the Z (left/right)
   * direction.
   */
  void
  TwistSpeedMapper::handleCmdVel(const geometry_msgs::Twist::ConstPtr& twist)
  {
    TreadSpeedsPtr speeds ( new TreadSpeeds );
    speeds->left = twist->linear.x - twist->angular.z * m_wheel_base / 2;
    speeds->right = twist->linear.x + twist->angular.z * m_wheel_base / 2;
    m_publisher.publish(speeds);
  }
}
