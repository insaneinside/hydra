#include <hydra_imu/AHRS.hh>
#include <pluginlib/class_list_macros.h>
#include <tf/LinearMath/Transform.h>
#include <cassert>
#include <cstdio>

#define DEFAULT_GAIN 0.1
#define CALIBRATION_SAMPLES 5000
#define SUBSCRIBER_QUEUE_SIZE 128
#define PUBLISHER_QUEUE_SIZE 128

PLUGINLIB_EXPORT_CLASS(hydra_imu::AHRS, nodelet::Nodelet)

typedef double scalar_t;
typedef tf::Vector3 Vector3;

/* some conversion and operator-conversion functions */
static inline tf::Vector3
gmV3totfV3(const geometry_msgs::Vector3& gm)
{
  return tf::Vector3(gm.x, gm.y, gm.z);
}
static inline geometry_msgs::Vector3
tfV3togmV3(const tf::Vector3& tfv)
{
  geometry_msgs::Vector3 out;
  out.x = tfv[0];
  out.y = tfv[1];
  out.z = tfv[0];
  return out;
}


static inline tf::Vector3
operator - (const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b)
{
  return gmV3totfV3(a) - gmV3totfV3(b);
}

static inline tf::Vector3
operator - (const geometry_msgs::Vector3& a, const tf::Vector3& b)
{
  return gmV3totfV3(a) - b;
}

static inline tf::Vector3
operator - (const tf::Vector3& a, const geometry_msgs::Vector3& b)
{
  return a - gmV3totfV3(b);
}

template < typename _Tp >
typename std::enable_if<!std::is_arithmetic<_Tp>::value,_Tp>::type
operator * (scalar_t a, const _Tp& b)
{
  return b * a;
}


#if 0
/** Runge-Kutta fourth-order normalized numerical integration algorithm, based
 * on the explanation in [1].
 */
static inline tf::Quaternion
integrate_RK4n(const tf::Quaternion& q, const tf::Vector3& omega,
               scalar_t dt)
{
  static const scalar_t
    a[] = { 0.5, 0.5, 1 };
  scalar_t t ( 0 ), w, x, y, z;
  tf::Quaternion qi ( q ), o[4];

  for ( uint8_t i = 0; i < 3; ++i )
    {
      w = qi.getW();
      x = qi.getX();
      y = qi.getY();
      z = qi.getZ();

      if ( std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
          fprintf(stderr, "NaN found in sub-step quaternion; i = %u", i);
          abort();
        }
      /* FIXME: currently have no way to update omega to correspond to current
         time value (since we're integrating once per IMU update).  Need to fix
         this -- one possibility is using the Holt-Winters smoothing filter to
         extrapolate.  */
      o[i].setValue(
                    0.5 * (omega[0] * w + omega[2] * y - omega[1] * z), /* x */
                    0.5 * (omega[1] * w - omega[2] * x + omega[0] * z), /* y */
                    0.5 * (omega[2] * w + omega[1] * x - omega[0] * y), /* z */
                    0.5 * (-omega[0] * x - omega[1] * y - omega[2] * z) /* w */
                    );
      
      /* set up q^{(i)} and t for next iteration */
      qi += dt * a[i] * o[i];
      t += a[i] * dt;
    }

  return (q + (dt/6)*(o[1] + o[2] * 2 + o[3] * 2 + o[4])).normalized();
}

static inline void
fuse(tf::Quaternion& q, scalar_t dt,
     const tf::Vector3& a, const tf::Vector3& omega,
     const tf::Vector3& m)
{
  q = integrate_RK4n(q, omega, dt);
}
#endif  /* 0 */

namespace hydra_imu
{
  AHRS::AHRS()
    : m_calibration_data ( ),
      m_calibrated ( false ),
      m_ahrs ( 200 ),
      m_last_update ( ),
      m_broadcaster ( ),
      m_subscriber ( ),
      m_publisher ( ),
      m_dynamic_configuration_server ( )
  {
  }

  AHRS::~AHRS()
  {
  }

  void
  AHRS::onInit()
  {
    ros::NodeHandle& nh ( getMTNodeHandle() );
    m_subscriber = nh.subscribe<hydra_imu::IMUData>("input", SUBSCRIBER_QUEUE_SIZE,
                                                    &AHRS::update, this);
    m_publisher = nh.advertise<hydra_imu::IMUData>("output", PUBLISHER_QUEUE_SIZE);

    m_dynamic_configuration_server.init(getName());
    m_dynamic_configuration_server.setCallback(std::bind(&AHRS::reconfigure, this,
                                                         std::placeholders::_1,
                                                         std::placeholders::_2));
    nh.param("gain", m_ahrs.beta, DEFAULT_GAIN);

    NODELET_INFO("hydra_imu/AHRS initialized.");

    /* Make sure the configuration server knows the initial configuration. */
    hydra_imu::AHRSConfig cfg; cfg.gain = m_ahrs.beta;
    m_dynamic_configuration_server.sync(cfg);
  }

  void
  AHRS::update(const hydra_imu::IMUData::ConstPtr& data)
  {
    if ( ! m_calibrated )
      {
        if ( m_calibration_data.sample_count < CALIBRATION_SAMPLES )
          {
            m_calibration_data.samples.push_front(data);
            ++m_calibration_data.sample_count;
          }
        if ( m_calibration_data.sample_count >= CALIBRATION_SAMPLES )
          {
            m_calibration_data.linear_acceleration.setZero();
            m_calibration_data.angular_velocity.setZero();
            m_calibration_data.magnetic_field.setZero();
            for ( const hydra_imu::IMUData::ConstPtr& s : m_calibration_data.samples )
              {
                m_calibration_data.linear_acceleration +=
                  gmV3totfV3(s->linear_acceleration) / m_calibration_data.sample_count;
                m_calibration_data.angular_velocity +=
                  gmV3totfV3(s->angular_velocity) / m_calibration_data.sample_count;
                m_calibration_data.magnetic_field += 
                  gmV3totfV3(s->magnetic_field) / m_calibration_data.sample_count;
              }
            m_calibrated = true;
            m_calibration_data.samples.clear();
            m_ahrs.orientation = tf::Quaternion::getIdentity();
            m_last_update = ros::Time::now();

            NODELET_INFO("hydra_imu/AHRS calibration complete");
          }
      }
    else
      {
        tf::Transform transform;

        transform.setOrigin(tf::Vector3(0, 0, 0)); /* only tracking rotation for now. */

        ros::Time now ( ros::Time::now() );
        ros::Duration dt ( now - m_last_update );

        hydra_imu::IMUData::Ptr out ( new IMUData );

        out->linear_acceleration = data->linear_acceleration;

        out->angular_velocity = data->angular_velocity;
        out->angular_velocity.x -= m_calibration_data.angular_velocity[0];
        out->angular_velocity.y -= m_calibration_data.angular_velocity[1];
        out->angular_velocity.z -= m_calibration_data.angular_velocity[2];
        out->magnetic_field = data->magnetic_field;

        m_ahrs.update(dt.toSec(),
                      gmV3totfV3(out->linear_acceleration),
                      gmV3totfV3(out->angular_velocity) * (M_PI/180),
                      gmV3totfV3(out->magnetic_field));
        m_last_update = now;

        m_publisher.publish(out);


        transform.setRotation(m_ahrs.orientation);
        m_broadcaster.sendTransform(tf::StampedTransform(transform, now, "base", "imu"));
      }
  }

  void
  AHRS::reconfigure(hydra_imu::AHRSConfig& config, uint32_t level)
  {
    if ( level & 0x01 )
      {
        NODELET_INFO("AHRS: reconfigured gain = %f", config.gain);
        m_ahrs.beta = config.gain;
      }
  }

}


/* References
 *
 * [1] D. Tedaldi, A. Pretto and E. Menegatti, "A Robust and Easy to Implement
 *     Method for IMU Calibration without External Equipments".  IEEE
 *     International Conference on Robotics and Automation (ICRA2014), May
 *     2014.  Retrieved 2014 May 10
 *     <http://robotics.dei.unipd.it/~pretto/papers/tpm_icra2014.pdf>.
 */

