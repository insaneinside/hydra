/* -*- c++ -*- */
#ifndef hydra_imu_bits_Filter_tcc
#define hydra_imu_bits_Filter_tcc 1

#ifndef hydra_imu_Filter_hh
# error Never include hydra_imu/bits/Filter.tcc directly; use hydra_imu/Filter.hh instead.
#endif

namespace hydra_imu
{
  template < typename _Parameters, typename _Data,
             typename _ChannelUpdateFunction>
  Vector3Filter<_Parameters,_Data,_ChannelUpdateFunction>::
    Vector3Filter(const _Parameters& _parameters,
                  _ChannelUpdateFunction _update_channel)
    : parameters ( _parameters ),
      update_channel ( _update_channel ),
      x ( ), y ( ), z ( )
  {
  }
  template < typename _Parameters, typename _Data,
             typename _ChannelUpdateFunction>
  Vector3Filter<_Parameters,_Data,_ChannelUpdateFunction>::
    ~Vector3Filter()
  {
  }
                   

  template < typename _Parameters, typename _Data,
             typename _ChannelUpdateFunction>
  bool
  Vector3Filter<_Parameters,_Data,_ChannelUpdateFunction>::
    update(size_t n, const Vector3& input, Vector3& output)
  {
    return
      update_channel(n, parameters, x, input.x, output.x) &&
      update_channel(n, parameters, y, input.y, output.y) &&
      update_channel(n, parameters, z, input.z, output.z);
  }

  
  template < typename _Filter >
  IMUFilter<_Filter>::IMUFilter()
    : m_linear_acceleration ( ),
      m_angular_velocity ( ),
      m_magnetic_field ( ),
      m_num_points_processed ( 0 ),
      m_subscriber ( ),
      m_publisher ( )
  {
  }

  template < typename _Filter >
  IMUFilter<_Filter>::~IMUFilter()
  {
  }


  template < typename _Filter >
  void
  IMUFilter<_Filter>::onInit()
  {
    ros::NodeHandle& nh ( getMTNodeHandle() );
    m_subscriber = nh.subscribe<hydra_imu::IMUData>("input", 256,
                                                    &IMUFilter::update, this);
    m_publisher = nh.advertise<hydra_imu::IMUData>("output", 256);

    NODELET_INFO("IMUFilter initialized");
  }

  template < typename _Filter >
  void
  IMUFilter<_Filter>::update(const hydra_imu::IMUData::ConstPtr& data)
  {
    IMUData::Ptr out ( new IMUData );
    m_linear_acceleration.update(m_num_points_processed,
                                 data->linear_acceleration,
                                 out->linear_acceleration);
    m_angular_velocity.update(m_num_points_processed,
                              data->angular_velocity,
                              out->angular_velocity);
    m_magnetic_field.update(m_num_points_processed,
                            data->magnetic_field,
                            out->magnetic_field);
    m_publisher.publish(out);

    ++m_num_points_processed;
  }

}

#endif  /* hydra_imu_bits_Filter_tcc */
