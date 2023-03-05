source ../recon/devel/setup.bash
for VARIABLE in 7 8 9 10 11 12 13 14 15 16
do
    for mode in nats rsi coverage random
    do
        if [[ $mode == nats ]]
        then
            for visibility in 1 0
            do
                for distance in 0
                do
                    timeout 4200 roslaunch  aidtr_bringup simulate_2_robots.launch ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true location:=ntc map_type:=aerial_thesis_v1 validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=stealth &
                    timeout 4200 roslaunch  --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=aerial_thesis_v1 robot1_name:=grizzly0 robot2_name:=ripsaw0 random_seed:=$VARIABLE threat_sampling:=stealth
                    wait
                done
            done
        else
            timeout 4200 roslaunch  aidtr_bringup simulate_2_robots.launch ompl_state_cost_weight:=0.0 enable_zone_recon:=true location:=ntc map_type:=aerial_thesis_v1 validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=stealth &
            timeout 4200 roslaunch  --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=aerial_thesis_v1 robot1_name:=grizzly0 robot2_name:=ripsaw0 random_seed:=$VARIABLE threat_sampling:=stealth
            wait
        fi
    done
done