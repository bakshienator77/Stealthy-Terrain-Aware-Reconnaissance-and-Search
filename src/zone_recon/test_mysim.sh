source ../zone_recon/devel/setup.bash
VARIABLE=502
mode=$2
if [[ $mode == star ]]
then
    mode=nats
    visibility=1
elif [[ $mode == guts ]]
then
    mode=nats
    visibility=0
fi
distance=0
map_type=aerial_thesis_v1
num_threats=$3
num_agents=$4
if [[ $num_agents == 1 ]]
then
    timeout $1 roslaunch waypoint_planner waypoint_planner.launch robot_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=stealth autonomy_faker:=true simulate_crosspose:=false num_targets:=$num_threats &
    timeout $1 roslaunch --wait waypoint_planner simulate_threats.launch location:=ntc map_type:=$map_type robot_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=stealth  num_targets:=$num_threats
elif [[ $num_agents == 2 ]]
then
    timeout $1 roslaunch waypoint_planner waypoint_planner2.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=stealth autonomy_faker:=true simulate_crosspose:=true  num_targets:=$num_threats &
    timeout $1 roslaunch --wait waypoint_planner simulate_threats2.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 random_seed:=$VARIABLE threat_sampling:=stealth  num_targets:=$num_threats
elif [[ $num_agents == 4 ]]
then
    timeout $1 roslaunch waypoint_planner waypoint_planner4.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=stealth autonomy_faker:=true simulate_crosspose:=true  num_targets:=$num_threats &
    timeout $1 roslaunch --wait waypoint_planner simulate_threats4.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 random_seed:=$VARIABLE threat_sampling:=stealth num_targets:=$num_threats
elif [[ $num_agents == 8 ]]
then
    timeout $1 roslaunch waypoint_planner waypoint_planner8.launch robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 location:=ntc map_type:=$map_type random_seed:=$VARIABLE distance_cost_weightage:=$distance visibility_cost_weightage:=$visibility search_mode:=$mode threat_sampling:=stealth autonomy_faker:=true simulate_crosspose:=true num_targets:=$num_threats  &
    timeout $1 roslaunch --wait waypoint_planner simulate_threats8.launch location:=ntc map_type:=$map_type robot1_name:=ripsaw0 robot2_name:=grizzly0 robot3_name:=recbot0 robot4_name:=bae0 robot5_name:=ripsaw1 robot6_name:=grizzly1 robot7_name:=recbot1 robot8_name:=bae1 random_seed:=$VARIABLE threat_sampling:=stealth num_targets:=$num_threats
fi