#ifndef hydra_imu_IMUDataScaler_hh
#define hydra_imu_IMUDataScaler_hh 1

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <hydra_imu/RawIMUData.h>

namespace hydra_imu
{
  class IMUDataScaler : public nodelet::Nodelet
  {
  private:
    ros::Subscriber m_subscriber; /**< Subscription to raw IMU data stream */
    ros::Publisher
      m_data_publisher,   /**< Publishes scaled data stream. */
      m_limits_publisher; /**< Publishes limits data. */

  public:
    IMUDataScaler();
    virtual ~IMUDataScaler();

    virtual void
    onInit();

    virtual void
    handleRawData(const hydra_imu::RawIMUData::ConstPtr& raw_data);
  };
}

#endif  /* hydra_imu_IMUDataScaler_hh */
