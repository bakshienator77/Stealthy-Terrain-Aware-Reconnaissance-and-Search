source ../zone_recon/devel/setup.bash
threat_breakdown=true
simulate_crosspose=false
map_type=aerial_thesis_v1
for VARIABLE in 7 8 9 10 11 12 13 14 15 16 17 18 19 20
do
    for robot_num in single double
    do
        for sampling_type in uniform stealth
        do
            for mode in rsi nats coverage random
            do
                if [[ $mode == rsi ]]
                then
                    for visibility in 0
                    do
                        for distance in 0
                        do
                            if [[ $robot_num == single ]]
                            then
                                timeout 6000 roslaunch aidtr_bringup autonomy.launch  ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true robot_name:=grizzly0 location:=ntc robot_class:=grizzly map_type:=$map_type random_seed:=$VARIABLE validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type &
                                timeout 6000 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type
                                wait
                            else
                                timeout 7000 roslaunch  aidtr_bringup simulate_2_robots.launch simulate_crosspose:=$simulate_crosspose ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true location:=ntc map_type:=$map_type validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=$sampling_type &
                                timeout 7000 roslaunch  --wait waypoint_planner simulate_threats2.launch threat_breakdown:=$threat_breakdown location:=ntc map_type:=$map_type robot1_name:=grizzly0 robot2_name:=ripsaw0 random_seed:=$VARIABLE threat_sampling:=$sampling_type
                                wait
                            fi
                        done
                    done
                elif [[ $mode == nats ]]
                then
                    for visibility in 1 0
                    do
                        for distance in 0
                        do
                            if [[ $robot_num == single ]]
                            then
                                timeout 6000 roslaunch aidtr_bringup autonomy.launch  ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true robot_name:=grizzly0 location:=ntc robot_class:=grizzly map_type:=$map_type random_seed:=$VARIABLE validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type &
                                timeout 6000 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type
                                wait
                            else
                                timeout 7000 roslaunch  aidtr_bringup simulate_2_robots.launch simulate_crosspose:=$simulate_crosspose ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true location:=ntc map_type:=$map_type validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=$sampling_type &
                                timeout 7000 roslaunch  --wait waypoint_planner simulate_threats2.launch threat_breakdown:=$threat_breakdown location:=ntc map_type:=$map_type robot1_name:=grizzly0 robot2_name:=ripsaw0 random_seed:=$VARIABLE threat_sampling:=$sampling_type
                                wait
                            fi
                        done
                    done
                else
                    if [[ $robot_num == single ]]
                    then
                        timeout 5000 roslaunch aidtr_bringup autonomy.launch ompl_state_cost_weight:=0.0 enable_zone_recon:=true robot_name:=grizzly0 location:=ntc robot_class:=grizzly map_type:=$map_type validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=$sampling_type &
                        timeout 5000 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type
                        wait
                    else
                        timeout 6000 roslaunch  aidtr_bringup simulate_2_robots.launch simulate_crosspose:=$simulate_crosspose ompl_state_cost_weight:=$((10 * $visibility)) enable_zone_recon:=true location:=ntc map_type:=$map_type validate_waypoints:=false start_nodelets:=false enable_obstacle_avoidance:=false show_viz:=false distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode random_seed:=$VARIABLE threat_sampling:=$sampling_type &
                        timeout 6000 roslaunch  --wait waypoint_planner simulate_threats2.launch threat_breakdown:=$threat_breakdown location:=ntc map_type:=$map_type robot1_name:=grizzly0 robot2_name:=ripsaw0 random_seed:=$VARIABLE threat_sampling:=$sampling_type
                        wait
                    fi
                fi
            done
        done
    done
done