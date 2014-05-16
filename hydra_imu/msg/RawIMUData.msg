# Message for transfer of raw data from a 9DOF (accelerometer, magnetometer,
# gyroscope) IMU.  Specifically, this structure is designed to hold raw,
# unprocessed -- and unscaled -- data from the Pololu MinIMU9v3.
#
# Because the Arduino-based driver implementation for Hydra changes the scale
# dynamically to enable both high precision and a high dynamic range, the
# full-scale index for each sensor at the time the reading was taken is
# duplicated in this message to ensure correct scaling by the receiver.

vector3_int16 linear_acceleration
uint8 linear_acceleration_full_scale

vector3_int16 angular_velocity
uint8 angular_velocity_full_scale

vector3_int16 magnetic_field
uint8 magnetic_field_full_scale
