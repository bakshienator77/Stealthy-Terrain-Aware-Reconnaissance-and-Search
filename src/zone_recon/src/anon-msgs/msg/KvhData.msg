# KVH IMU Message

Header header

# Sensor Timestamp
uint64 sensor_timestamp

# Status
uint8 status

####
# Angular Velocity in radians/sec
####
float32 gx
float32 gy
float32 gz

####
# Linear Acceleration in m/s^2
####
float32 ax
float32 ay
float32 az

####
# Magnetic Data in Tesla
####
float32 magnetic_data_x
float32 magnetic_data_y
float32 magnetic_data_z

# Temperature in C
int16 temperature

# Encoder ticks
uint32 encoder_left
uint32 encoder_right
