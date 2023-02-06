source ../zone_recon/devel/setup.bash

# communication breakdown
threat_breakdown=true
simulate_crosspose=false

# fake ntc map
map_type=aerial_thesis_v1

# k=5
num_threats=5

for robot_num in octuple quadruple
do
    for VARIABLE in 7 8 9 10 11 12 13 14 15 16
    do
        for sampling_type in uniform 
        do
            for mode in nats coverage
            do
                if [[ $mode == rsi ]]
                then
                    for visibility in 0
                    do
                        for distance in 0
                        do
                            if [[ $robot_num == single ]]
                            then
                                timeout 600 roslaunch waypoint_planner waypoint_planner.launch robot_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num &
                                timeout 600 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            elif [[ $robot_num == double ]]
                            then
                                timeout 600 roslaunch waypoint_planner waypoint_planner2.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num simulate_crosspose:=$simulate_crosspose &
                                timeout 600 roslaunch --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            elif [[ $robot_num == quadruple ]]
                            then
                                timeout 700 roslaunch waypoint_planner waypoint_planner4.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num simulate_crosspose:=$simulate_crosspose &
                                timeout 700 roslaunch --wait waypoint_planner simulate_threats4.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            else
                                timeout 800 roslaunch waypoint_planner waypoint_planner8.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num simulate_crosspose:=$simulate_crosspose &
                                timeout 800 roslaunch --wait waypoint_planner simulate_threats8.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
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
                                timeout 600 roslaunch waypoint_planner waypoint_planner.launch robot_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   &
                                timeout 600 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            elif [[ $robot_num == double ]]
                            then
                                timeout 600 roslaunch waypoint_planner waypoint_planner2.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   simulate_crosspose:=$simulate_crosspose &
                                timeout 600 roslaunch --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            elif [[ $robot_num == quadruple ]]
                            then
                                timeout 700 roslaunch waypoint_planner waypoint_planner4.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   simulate_crosspose:=$simulate_crosspose &
                                timeout 700 roslaunch --wait waypoint_planner simulate_threats4.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            else
                                timeout 800 roslaunch waypoint_planner waypoint_planner8.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   simulate_crosspose:=$simulate_crosspose &
                                timeout 800 roslaunch --wait waypoint_planner simulate_threats8.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                                wait
                            fi
                        done
                    done
                else
                    if [[ $robot_num == single ]]
                    then
                        timeout 500 roslaunch waypoint_planner waypoint_planner.launch robot_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=0.0 visibility_cost_weightage:=0.0 search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   &
                        timeout 500 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                        wait
                    elif [[ $robot_num == double ]]
                    then
                        timeout 500 roslaunch waypoint_planner waypoint_planner2.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=0.0 visibility_cost_weightage:=0.0 search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   simulate_crosspose:=$simulate_crosspose &
                        timeout 500 roslaunch --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                        wait
                    elif [[ $robot_num == quadruple ]]
                    then
                        timeout 500 roslaunch waypoint_planner waypoint_planner4.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=0.0 visibility_cost_weightage:=0.0 search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num   simulate_crosspose:=$simulate_crosspose &
                        timeout 500 roslaunch --wait waypoint_planner simulate_threats4.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                        wait
                    else
                        timeout 600 roslaunch waypoint_planner waypoint_planner8.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=0.0 visibility_cost_weightage:=0.0 search_mode:=$mode threat_sampling:=$sampling_type  autonomy_faker:=true num_targets:=$num_threats log_config:=$robot_num simulate_crosspose:=$simulate_crosspose &
                        timeout 600 roslaunch --wait waypoint_planner simulate_threats8.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 random_seed:=$VARIABLE threat_sampling:=$sampling_type  num_targets:=$num_threats log_config:=$robot_num threat_breakdown:=$threat_breakdown
                        wait
                    fi
                fi
            done
        done
    done
done