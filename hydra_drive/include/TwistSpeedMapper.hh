#ifndef hydra_drive_TwistSpeedMapper_hh
#define hydra_drive_TwistSpeedMapper_hh 1

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace hydra_drive
{
  class TwistSpeedMapper : public nodelet::Nodelet
  {
  private:
    ros::Subscriber m_subscriber;
    ros::Publisher m_publisher;

    /** @begin Parameters
     *@{
     */
    double m_wheel_base;
    /**@}*/
  public:
    TwistSpeedMapper();
    virtual ~TwistSpeedMapper();

    virtual void onInit();
    virtual void handleCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel);
  };
}

#endif  /* hydra_drive_TwistSpeedMapper_hh */
