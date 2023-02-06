Header header
# The track uid.
uint64 track_id
# The timestamp when the last track predict/update has been performed.
uint64 latest_update_timestamp
# The timestamp when the latest observation is obtained.
uint64 latest_observation_timestamp

# Estimated track position in UTM coordinate
geometry_msgs/Point position
# 3x3 Position covariance in a row-major order.
float64[] covariance

# Sum of average confidence
float64 weighted_confidence

# The class name associated with this track.
string object_class
# The robot name that made the latest observation
string latest_robot
# The camera name that made the latest observation
string latest_detector_camera