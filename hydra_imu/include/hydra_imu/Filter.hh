#ifndef hydra_imu_Filter_hh
#define hydra_imu_Filter_hh 1

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <hydra_imu/IMUData.h>

namespace hydra_imu
{
  using Vector3 = geometry_msgs::Vector3;

  /** Interface for a vector-type filter channel. */
  template < typename _TpA, typename _TpB >
  struct FilterInterface
  {
    virtual bool
    update(size_t n, const _TpA& input, _TpB& output) = 0;
  };

  /** Basic Vector3 filter implemented using callbacks.
   *
   * @tparam _Parameters Data structure type used to hold per-filter parameter
   *     values.
   *
   * @tparam _Data Data structure type used to hold per-*channel* data values.
   *
   * @tparam _ChannelUpdateFunction Any function or functor type for which an
   *     instance can be invoked with the signature
   *
   *         bool (size_t, const _Parameters&, _Data&, double, double&)
   */
  template < typename _Parameters, typename _Data,
             typename _ChannelUpdateFunction =
             bool(*)(size_t, const _Parameters&, _Data&, double, double&) >
  struct Vector3Filter : public FilterInterface<Vector3, Vector3>
  {
    typedef _ChannelUpdateFunction ChannelUpdateFunction;
    typedef _Parameters Parameters;
    typedef _Data Data;

    Vector3Filter(const Parameters& _parameters,
                  ChannelUpdateFunction _update_channel);
    virtual ~Vector3Filter();

    Parameters parameters;
    ChannelUpdateFunction update_channel;
    Data x, y, z;

    virtual bool
    update(size_t n, const Vector3& input, Vector3& output);
  };

  /** Smoothing filter for 9-DOF IMUs. */
  template < typename _Filter >
  class IMUFilter : public nodelet::Nodelet
  {
  public:
    typedef _Filter SensorFilter;
    SensorFilter
      m_linear_acceleration,
      m_angular_velocity,
      m_magnetic_field;

    size_t m_num_points_processed;

    ros::Subscriber m_subscriber; /**< subscription to the input data stream */
    ros::Publisher m_publisher;   /**< publisher of the transformed data */

  public:
    IMUFilter();
    virtual ~IMUFilter();

    virtual void
    onInit();

    virtual void
    update(const hydra_imu::IMUData::ConstPtr& data);
  };
};

#include <hydra_imu/bits/Filter.tcc>

#endif  /* hydra_imu_Filter_hh */
