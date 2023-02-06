## Overview
This module would conceptually sit between the perception and navigation manager modules with the aim of providing high-level waypoints to the navigation manager in order to optimise search efficiency.

It would operate in parallel to the ROS-COT bridge in [this diagram](https://cmu-aihub.atlassian.net/wiki/spaces/AIDTR/pages/236781575/Autonomy+Software+Details) in an approximate sense within autonomy. Waypoints can be given by the user via ATAK but they may also be specified by providing a search zone to this module.

## Current exploration paradigm
![NATS search waypoint planner](./gascola_diag_icra_22%20copy.jpg)

As depicted in the image above the zone_recon module is provided with a search region. It then uses the Generalised Noise Aware Thompson Sampling algorithm to select a suitable waypoint based on the information it has already collected.

Broadly speaking, only reachable locations within the search region on the basis of the cost map are considered when selecting the next waypoint.

The navigation manager provides feedback whether the goal has been reached via the action server and hence if an unattainable waypoint is provided, it is dealt with by redoing the selection.

Additionally, once a waypoint has been accepted, the next waypoint is selected en route in an effort to avoid having the robot stop for long periods whenever it completes a waypoint.

[comment]: <> (The robot is currently not equipped to move in reverse and hence providing points behind it will mean greater delays as unattainable waypoints will be more frequent or situations where the navigation manager is forced to compute very long paths that would involve the robot taking a circuitous path in order reach only a few meters behind its current location.)

## How to use this module?
In order to see a demo of the module working in simulation the steps are as follows:

- run `roslaunch aidtr_bringup autonomy.launch enable_zone_recon:=true`
- Use the second "Publish Point" button to start creating the polygon, once the polygon is closed the points are published to waypoint planner

[comment]: <> (- [Optional] In the RVIZ monitor that opens on the LHS tray add a Marker and set the topic that it listens on to `/search_polygon/visualisation`)
- You will see the robot start exploring the environment while remaining within the search polygon. Make sure to draw the search polygon containing the robot's current location as the behviour to move toward the search polygon is as yet unimplemented.
- To pause the waypoint planner run: `rosservice call /recbot0/pause True`
- To restart it run: `rosservice call /recbot0/pause False`  
- [Optional] If the functionality of the search polygon wants to be tested via rostopic publish, run the following command for GASCOLA: 
    
```
rostopic pub -1 /search_polygon/specification aidtr_ros_msgs/SearchPolygon "header:
   seq: 0
   stamp: {secs: 0, nsecs: 0}
   frame_id: 'earth_zero'
polygon:
   points:
   - {x: 602668, y: 4479527, z: 0.0}
   - {x: 602732, y: 4479363, z: 0.0}
   - {x: 602635, y: 4479115, z: 0.0}
   - {x: 602400, y: 4479215, z: 0.0}
   - {x: 602603, y: 4479552, z: 0.0}
   - {x: 602668, y: 4479527, z: 0.0}
   
zone_id: 0
robot_id: lhex2"

```

```
rostopic pub -1 /clicked_point geometry_msgs/PointStamped "header:
    seq: 2
    stamp: 
      secs: 225
      nsecs: 196000000
    frame_id: "earth"
point: 
  x: 602571.94
  y: 4479453.79
  z: 0"
```
    
```
rostopic pub -1 /search_polygon/specification aidtr_ros_msgs/SearchPolygon "header: 
  seq: 1
  stamp: 
    secs: 43
    nsecs: 471000000
  frame_id: "recbot0/map_zero"
polygon: 
  points: 
    - 
      x: 1315.73596191
      y: 759.985351562
      z: -0.00924682617188
    - 
      x: 1317.23840332
      y: 186.894821167
      z: -0.00534057617188
    - 
      x: 5.95076370239
      y: 185.94871521
      z: -0.00534057617188
    - 
      x: 6.94760894775
      y: 756.554199219
      z: -0.0013427734375
    - 
      x: 1315.72949219
      y: 762.477111816
      z: -0.00924682617188
zone_id: 0
robot_id: "recbot0"
"

```
- Search polygon YPG_KOFA 4-1 for drone (change robot_id)

```
rostopic pub -1 /nrec/input_polygon aidtr_ros_msgs/SearchPolygon "header: 
  seq: 1
  stamp: 
    secs: 43
    nsecs: 471000000
  frame_id: "earth"
polygon: 
  points: 
    - 
      x: 760552.014
      y: 3641606.999
      z: -0.00924682617188
    - 
      x: 761867.015
      y: 3641606.999
      z: -0.00534057617188
    - 
      x: 761867.015
      y: 3641036.999
      z: -0.00534057617188
    - 
      x: 760552.014
      y: 3641036.999
      z: -0.0013427734375
    - 
      x: 760552.014
      y: 3641606.999
      z: -0.00924682617188
zone_id: 0
robot_id: "bhex8"
"


```
- Search polygon YPG_KOFA 4-3 for drone (change robot_id)

```
rostopic pub -1 /nrec/input_polygon aidtr_ros_msgs/SearchPolygon "header: 
  seq: 1
  stamp: 
    secs: 43
    nsecs: 471000000
  frame_id: "earth"
polygon: 
  points: 
    - 
      x: 761091.015
      y: 3642126.999
      z: -0.00924682617188
    - 
      x: 762010.015
      y: 3642126.999
      z: -0.00534057617188
    - 
      x: 762010.015
      y: 3641770.999
      z: -0.00534057617188
    - 
      x: 761091.015
      y: 3641770.999
      z: -0.0013427734375
    - 
      x: 761091.015
      y: 3642126.999
      z: -0.00924682617188
zone_id: 0
robot_id: "bhex8"
"
```

- [Optional] The command above creates a regular square of side 100m, feel free to change or add coordinates to the above polygon. 

- [Optional] The following command sends a message that contains the track data for three fictitious observations.
```
rostopic pub /recbot0/tracks ros_track_msgs/TrackDataVector "
  data:  
  - header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: 'global_world'
    track_id: 77
    latest_update_timestamp: 1593210486400212
    latest_observation_timestamp: 1593210486400212
    position: 
      x: 4479366.54917
      y: 602670.979411
      z: -330.494134198
    covariance: [21.532487374275913, -9.412886389110671, -0.9019071100095701, -9.412886389110655, 4.866897070725197, 0.4061655477972556, -0.9019071100095726, 0.40616554779725567, 0.6665859831564867]
    weighted_confidence: 0.66227221489
    object_class: 'person'
    latest_robot: 'recbot0'
    latest_detector_camera: 'camera_module1_lens2_rect'
  - header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: 'global_world'
    track_id: 78
    latest_update_timestamp: 1593210486400212
    latest_observation_timestamp: 1593210486400212
    position: 
      x: 4479386.54917
      y: 602670.979411
      z: -330.494134198
    covariance: [21.532487374275913, -9.412886389110671, -0.9019071100095701, -9.412886389110655, 4.866897070725197, 0.4061655477972556, -0.9019071100095726, 0.40616554779725567, 0.6665859831564867]
    weighted_confidence: 0.36227221489
    object_class: 'gator'
    latest_robot: 'recbot0'
    latest_detector_camera: 'camera_module1_lens2_rect'
  - header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: 'global_world'
    track_id: 79
    latest_update_timestamp: 1593210486400212
    latest_observation_timestamp: 1593210486400212
    position: 
      x: 4479346.54917
      y: 602670.979411
      z: -330.494134198
    covariance: [21.532487374275913, -9.412886389110671, -0.9019071100095701, -9.412886389110655, 4.866897070725197, 0.4061655477972556, -0.9019071100095726, 0.40616554779725567, 0.6665859831564867]
    weighted_confidence: 1
    object_class: 'cone'
    latest_robot: 'recbot0'
    latest_detector_camera: 'camera_module1_lens2_rect'
"


```
## Multirobot Simulation

In order to develop more sophisticated algorithms this module supports simulating more than 1 agent (2 for now). In order to run a simulation with two robots do the following:

1. Ensure you are in the branch `nikhil/multirobot` in `aidtr-autonomy` and in the branch `nikhil/multirobot_sim` in this repo and in the `dev` branch in the `aidtr-urdf` repo.
  
2. a. run `roslaunch aidtr_bringup simulate_2_robots.launch zone_recon:=true` for two robot simulation

2. b. run `roslaunch aidtr_bringup simulate_3_robots.launch enable_zone_recon:=true map_type:=empty validate_waypoints:=false` for a three robot simulation [under development]

4. Specify the search polygon by running:

```
rostopic pub -1 /search_polygon/specification aidtr_ros_msgs/SearchPolygon "header: 
  seq: 1
  stamp: 
    secs: 63
    nsecs: 980000000
  frame_id: 'recbot0/map_zero'
polygon: 
  points: 
    - 
      x: 13.517332077
      y: 236.514343262
      z: -0.00143432617188
    - 
      x: 105.299087524
      y: 505.115875244
      z: -0.00143432617188
    - 
      x: 214.554672241
      y: 515.033813477
      z: -0.0013427734375
    - 
      x: 243.194885254
      y: 126.405548096
      z: -0.00143432617188
    - 
      x: 38.0705871582
      y: 89.589515686
      z: -0.00143432617188
    - 
      x: 13.5217981339
      y: 234.085083008
      z: -0.00143432617188

zone_id: 0
robot_id: 'recbot0'
"

```


## TODOs

- Incorporate viewshed information in GNATS
- Develop Adversarial active search algorithm
- Incorporate a slider that allows the user to control tactical movement behvaiour varying from "Stealthily and Deliberately" and "Rapidly and Forcefully"
- Explore creating a more efficient object class in the client code so that information is shared between methods
