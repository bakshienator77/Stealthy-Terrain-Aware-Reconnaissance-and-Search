source ../zone_recon/devel/setup.bash
VARIABLE=500
mode=rsi
visibility=0
distance=0
map_type=aerial_thesis_v2
timeout $1 roslaunch aidtr_bringup autonomy.launch  launch_unity:=true ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true robot_name:=grizzly0 location:=ntc robot_class:=grizzly map_type:=$map_type random_seed:=$VARIABLE validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=true distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=stealth &
timeout $1 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=stealth