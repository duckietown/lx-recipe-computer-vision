#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# ROS 2 runs over zenoh: connect this container to the robot's zenoh router.
# Works both on the robot (router on localhost) and from a laptop (router reached over the network).
if [ -z "${ZENOH_CONFIG_OVERRIDE:-}" ] && [ -n "${VEHICLE_NAME:-}" ]; then
    export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/${VEHICLE_NAME}.local:7447\"]"
fi

# launching app (ROS 2)
dt-exec ros2 launch visual_lane_servoing visual_lane_servoing_node.launch.py veh:="${VEHICLE_NAME}" car_cmd_topic:="${CAR_CMD_TOPIC:-car_cmd_switch_node/cmd}"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
