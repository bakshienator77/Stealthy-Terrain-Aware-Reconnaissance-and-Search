#!/bin/bash

for i in src/zone_recon/src/waypoint_planner/src/waypoint_planner/*.py # or whatever other pattern...
do
  if ! grep -q Copyright $i
  then
    cat copyright.txt $i >$i.new && mv $i.new $i
  fi
done