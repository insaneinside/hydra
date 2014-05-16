#ifndef hydra_imu_HoltWintersIMUFilter_hh
#define hydra_imu_HoltWintersIMUFilter_hh 1

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <hydra_common/dynamic_reconfigure/server.h>
#include <hydra_imu/IMUData.h>
#include <hydra_imu/HoltWintersIMUFilterConfig.h>
#include <hydra_imu/Filter.hh>


namespace hydra_imu
{
  struct HoltWintersData
  {
    double trend_estimate;
    double smoothed_value;
  };
  struct HoltWintersParameters
  {
    double alpha;
    double beta;
    double oma;
    double omb;
  };

  /** Implements a modified Holt-Winters exponential-smoothing filter. */
  class HoltWintersVector3Filter
    : public Vector3Filter<HoltWintersParameters,
                           HoltWintersData>
  {
  public:
    typedef Vector3Filter<HoltWintersParameters,HoltWintersData> ParentType;
    HoltWintersVector3Filter();
    virtual ~HoltWintersVector3Filter();
  };

  class HoltWintersIMUFilter
    : public IMUFilter<HoltWintersVector3Filter>
  {
  private:
    typedef IMUFilter<HoltWintersVector3Filter> ParentType;
    hydra_common::dynamic_reconfigure::Server<hydra_imu::HoltWintersIMUFilterConfig>
      m_dynamic_configuration_server;

    void
    reconfigure(HoltWintersIMUFilterConfig& config, uint32_t level);

  public:
    HoltWintersIMUFilter();
    virtual ~HoltWintersIMUFilter();

    virtual void
    onInit();
  };
}
#endif  /* hydra_imu_HoltWintersIMUFilter_hh */
