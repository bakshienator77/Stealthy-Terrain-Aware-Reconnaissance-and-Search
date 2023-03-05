# Local Track Message

string track_class # eg person, pickup, etc
string uid # Locally unique tract id
geometry_msgs/PoseStamped local_pose
float64[] covariance
string maturity_level # verified vs unverified
LastDetection last_detection
float64 velocity
uint64 track_timestamp