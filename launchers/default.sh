#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/devel/setup.bash --extend

exec roslaunch visual_lane_servoing visual_lane_servoing_node.launch veh:=$VEHICLE_NAME