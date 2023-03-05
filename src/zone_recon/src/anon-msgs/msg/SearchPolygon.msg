# Informs WayPoint Planner node which coordinate polygon to limit itself too

# For timestamp info to determine on planner side which message was received later
Header header

# Contains ordered list of x,y,z coordinates with first and last assumed to be connected
geometry_msgs/Polygon polygon

# Exact use of following variables to be fleshed out
# Flag to indicate where zone clearing behaviour is desired for the search
bool zone_clearing

# Expected range 0-100, default is be zero, the higher value the more the robots value remaining hidden during the search
int64 stealth

# Indicates which robot this message is indicated for assuming they read from the same topic for user specific search zone
string robot_id