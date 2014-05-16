#ifndef hydra_imu_AHRS_hh
#define hydra_imu_AHRS_hh 1

#include <ros/ros.h>
#include <hydra_imu/IMUData.h>
#include <nodelet/nodelet.h>
#include <hydra_common/dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include <hydra_imu/AHRSConfig.h>
#include <hydra_imu/MadgwickAHRS.hh>

namespace hydra_imu
{
  class AHRS : public nodelet::Nodelet
  {
  private:
    using Vector3 = tf::Vector3;
    using Quaternion = tf::Quaternion;

    struct CalibrationData
    {
      std::list<hydra_imu::IMUData::ConstPtr>
        samples;
      size_t sample_count;

      Vector3
        linear_acceleration,
        angular_velocity,
        magnetic_field;
    } m_calibration_data;
    bool m_calibrated;

    MadgwickAHRS m_ahrs;
    ros::Time m_last_update;

    tf::TransformBroadcaster m_broadcaster;
    ros::Subscriber m_subscriber;
    ros::Publisher m_publisher;    
    hydra_common::dynamic_reconfigure::Server<hydra_imu::AHRSConfig>
      m_dynamic_configuration_server;

  public:
    AHRS();
    virtual ~AHRS();

    virtual void
    onInit();

    void
    update(const hydra_imu::IMUData::ConstPtr& data);

    void
    reconfigure(hydra_imu::AHRSConfig& config, uint32_t level);
  };
  
}

#endif  /* hydra_imu_AHRS_hh */
