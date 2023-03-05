# Advanced Navigation Spatial Dual Inertial Navigation System Data

Header header

# Indicates INS filter is initialized and is ready to send valid data
# An AND of the Orientation Filter Initialised, INS Filter Initialised, and Heading Initialised flags
bool ready

####
# State Packet
####
# GNSS status
string gnss_fix_type_string
uint8 GNSS_NO_FIX_TYPE = 0
uint8 GNSS_2D_FIX_TYPE = 1
uint8 GNSS_3D_FIX_TYPE = 2
uint8 GNSS_SBAS_FIX_TYPE = 3
uint8 GNSS_DIFFERENTIAL_FIX_TYPE = 4
uint8 GNSS_OMNISTAR_FIX_TYPE = 5
uint8 GNSS_RTK_FLOAT_FIX_TYPE = 6
uint8 GNSS_RTK_FIXED_FIX_TYPE = 7
uint8 gnss_fix_type_int

# Filter status messages
bool orientation_filter_initialised
bool ins_filter_initialised
bool heading_initialised
bool utc_time_initialised
bool internal_gnss_enabled
bool magnetic_heading_enabled       # Spatial only
bool dual_antenna_heading_active    # Spatial Dual only
bool velocity_heading_enabled
bool atmospheric_altitude_enabled
bool external_position_active
bool external_velocity_active
bool external_heading_active

# System status messages
bool system_failure
bool accelerometer_sensor_failure
bool gyroscope_sensor_failure
bool magnetometer_sensor_failure
bool pressure_sensor_failure
bool gnss_failure
bool accelerometer_over_range
bool gyroscope_over_range
bool magnetometer_over_range
bool pressure_over_range
bool minimum_temperature_alarm
bool maximum_temperature_alarm
bool low_voltage_alarm
bool high_voltage_alarm
bool gnss_antenna_disconnected
bool serial_portOverflow_alarm

# Trigger input set
bool event1
bool event2

# Geometry information
float64 latitude
float64 longitude
float64 height

float64 velocity_north
float64 velocity_east
float64 velocity_down

float64 body_acceleration_x
float64 body_acceleration_y
float64 body_acceleration_z

float64 g_force

float64 roll
float64 pitch
float64 heading

float64 angular_velocity_x
float64 angular_velocity_y
float64 angular_velocity_z

float64 latitude_standard_deviation
float64 longitude_standard_deviation
float64 altitude_standard_deviation


# UTM Position Packet
float64 utm_northing
float64 utm_easting
float64 utm_height

uint8 zone_number
int8 zone_character

####
# Body Velocity Packet
####
float64 velocity_x
float64 velocity_y
float64 velocity_z

####
# Euler Orientation Standard Deviation Packet
####
float64 roll_standard_deviation
float64 pitch_standard_deviation
float64 heading_standard_deviation

####
# Raw GNSS Packet
####
# Position information
float64 gnss_latitude
float64 gnss_longitude
float64 gnss_height

float64 gnss_velocity_north
float64 gnss_velocity_east
float64 gnss_velocity_down

float64 gnss_latitude_standard_deviation
float64 gnss_longitude_standard_deviation
float64 gnss_height_standard_deviation

float64 gnss_tilt
float64 gnss_tilt_standard_deviation
float64 gnss_heading
float64 gnss_heading_standard_deviation

# Status
bool gnss_doppler_velocity_valid
bool gnss_external_gnss_valid
bool gnss_time_valid
bool gnss_tilt_valid
bool gnss_heading_valid
bool gnss_floating_ambiguity_heading # Not provided by SDK or documented in manual. Unused

####
# Raw Sensors Packet
####
# In m/s/s
float32 accelerometer_x_raw
float32 accelerometer_y_raw
float32 accelerometer_z_raw

# In rad/s
float32 gyroscope_x_raw
float32 gyroscope_y_raw
float32 gyroscope_z_raw

# In mG
float32 magnetometer_x_raw
float32 magnetometer_y_raw
float32 magnetometer_z_raw

# In deg C
float32 imu_temperature_raw

# In Pascals
float32 pressure_raw

# In deg C
float32 pressure_temperature_raw

####
# Geoid Height Packet
####
# In meters
float32 geoid_height
