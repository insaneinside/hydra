# Contains information about how a particular MinIMU9v3 is configured.

# Accelerometer data-rate selection
uint8 ACCELEROMETER_RATE_0     = 0
uint8 ACCELEROMETER_RATE_3_125 = 1
uint8 ACCELEROMETER_RATE_6_25  = 2
uint8 ACCELEROMETER_RATE_12_5  = 3
uint8 ACCELEROMETER_RATE_25    = 4
uint8 ACCELEROMETER_RATE_50    = 5
uint8 ACCELEROMETER_RATE_100   = 6
uint8 ACCELEROMETER_RATE_200   = 7
uint8 ACCELEROMETER_RATE_400   = 8
uint8 ACCELEROMETER_RATE_800   = 9
uint8 ACCELEROMETER_RATE_1600  = 10

# Accelerometer scale selection
uint8 ACCELEROMETER_SCALE_2G  = 0
uint8 ACCELEROMETER_SCALE_4G  = 1
uint8 ACCELEROMETER_SCALE_6G  = 2
uint8 ACCELEROMETER_SCALE_8G  = 3
uint8 ACCELEROMETER_SCALE_16G = 4

IMUSensorConfiguration accelerometer


# Magnetometer data-rate selection
uint8 MAGNETOMETER_RATE_3_125 = 0
uint8 MAGNETOMETER_RATE_6_25  = 1
uint8 MAGNETOMETER_RATE_12_5  = 2
uint8 MAGNETOMETER_RATE_25    = 3
uint8 MAGNETOMETER_RATE_50    = 4
uint8 MAGNETOMETER_RATE_100   = 5

# Magnetometer scale selection
uint8 MAGNETOMETER_SCALE_2_GAUSS  = 0
uint8 MAGNETOMETER_SCALE_4_GAUSS  = 1
uint8 MAGNETOMETER_SCALE_8_GAUSS  = 2
uint8 MAGNETOMETER_SCALE_12_GAUSS = 3

IMUSensorConfiguration magnetometer


# Gyroscope data-rate selection
uint8 GYROSCOPE_RATE_12_5 = 0
uint8 GYROSCOPE_RATE_25   = 1
uint8 GYROSCOPE_RATE_50   = 2
uint8 GYROSCOPE_RATE_100  = 3
uint8 GYROSCOPE_RATE_200  = 4
uint8 GYROSCOPE_RATE_400  = 5
uint8 GYROSCOPE_RATE_800  = 6

# Gyroscope scale selection
uint8 GYROSCOPE_SCALE_245_DPS  = 0
uint8 GYROSCOPE_SCALE_500_DPS  = 1
uint8 GYROSCOPE_SCALE_2000_DPS = 2

IMUSensorConfiguration gyroscope