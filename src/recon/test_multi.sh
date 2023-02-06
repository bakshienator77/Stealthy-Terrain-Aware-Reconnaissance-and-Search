source ../zone_recon/devel/setup.bash
VARIABLE=500
mode=nats
visibility=1
distance=0
threat_breakdown=true
simulate_crosspose=false
timeout $1 roslaunch  aidtr_bringup simulate_2_robots.launch ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true location:=ntc map_type:=aerial_thesis_v1 validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=true distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=stealth simulate_crosspose:=$simulate_crosspose &
timeout $1 roslaunch  --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=aerial_thesis_v1 robot1_name:=grizzly0 robot2_name:=ripsaw0 random_seed:=$VARIABLE threat_sampling:=stealth threat_breakdown:=$threat_breakdown
