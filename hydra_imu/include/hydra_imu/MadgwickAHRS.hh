//=====================================================================================================
// MadgwickAHRS.hh
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 15/05/2014   C.J. Sutton	Adapted for integration with DUBotics Hydra code
//
//=====================================================================================================
#ifndef MadgwickAHRS_hh
#define MadgwickAHRS_hh

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

namespace hydra_imu
{
  struct MadgwickAHRS
  {
    double beta;                // algorithm gain
    tf::Quaternion orientation; // quaternion of sensor frame relative to auxiliary frame

    MadgwickAHRS(double _beta = 0.1);
    ~MadgwickAHRS();

    void update(double dt, const tf::Vector3& linear_acceleration, const tf::Vector3& angular_velocity, const tf::Vector3& magnetic_field);
    void update(double dt, const tf::Vector3& linear_acceleration, const tf::Vector3& angular_velocity);
  };
}

#endif  /* MadgwickAHRS_hh */
