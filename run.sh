#!/bin/bash

# Run the stack
# roslaunch realm_ros disasterprobe_reco_ros.launch 

# when done detect vehicles
LATEST_PLY_DIR="$(ls -td realm_ros/output/*/mosaicing/elevation/ply | head -1)"
VEHICLES="disasterprobe_bridge/test/vehicles.txt"

rosrun disasterprobe_bridge plan_observations "$LATEST_PLY_DIR/elevation.ply" "$VEHICLES" out.txt

