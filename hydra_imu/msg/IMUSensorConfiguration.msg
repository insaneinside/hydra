# This message type is used primarily to provide encapsulation within the
# `hydra_imu/IMUConfiguration` message.  The `rate` and `scale` values are
# indices of sensor-reporting rates and full-scale settings, respectively, that
# are specific to each sensor on the IMU board.

uint8 rate
uint8 scale
