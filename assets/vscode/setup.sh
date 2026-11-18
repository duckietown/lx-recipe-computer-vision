#!/bin/bash
set -euo pipefail

# open README.md at startup and enable Jupyter widget sources (ipywidgets / ipympl panels)
VSCODE_USER_SETTINGS=/home/duckie/.local/share/code-server/User/settings.json
OUT=$(jq '. + {"workbench.startupEditor": "readme", "jupyter.widgetScriptSources": ["jsdelivr.com", "unpkg.com"]}' "${VSCODE_USER_SETTINGS}")
echo "${OUT}" > "${VSCODE_USER_SETTINGS}"

# ROS 2 in every interactive terminal of the editor
cat >> /etc/bash.bashrc <<'EOF'

source "/opt/ros/${ROS2_DISTRO}/setup.bash"
if [[ -f "/opt/duckietown/ros2/duckietown_msgs/share/duckietown_msgs/local_setup.bash" ]]; then
    source "/opt/duckietown/ros2/duckietown_msgs/share/duckietown_msgs/local_setup.bash"
fi
if [[ -f "${COLCON_WS}/install/setup.bash" ]]; then
    source "${COLCON_WS}/install/setup.bash"
fi

# `dt-ros2-connect ROBOT` points the `ros2` tools of this terminal at the robot's zenoh router (client mode);
# `dt-ros2-local` goes back to a local graph (start a local router with `dt-ros2-local-router`).
dt-ros2-connect() {
    if [[ -z "${1:-}" ]]; then echo "usage: dt-ros2-connect ROBOT_NAME"; return 1; fi
    export VEHICLE_NAME="${1%.local}"
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${VEHICLE_NAME}.local:7447\"]"
    ros2 daemon stop >/dev/null 2>&1 || true
    echo "ros2 tools in this terminal now talk to '${VEHICLE_NAME}' (tcp/${VEHICLE_NAME}.local:7447)"
}
dt-ros2-local() {
    unset ZENOH_CONFIG_OVERRIDE
    ros2 daemon stop >/dev/null 2>&1 || true
    echo "ros2 tools in this terminal now use the local graph (run dt-ros2-local-router in another terminal)"
}
dt-ros2-local-router() {
    ZENOH_CONFIG_OVERRIDE= ros2 run rmw_zenoh_cpp rmw_zenohd
}
# auto-connect when a real robot name was given (the entrypoint sets VEHICLE_NAME to the container hostname otherwise)
if [[ -z "${ZENOH_CONFIG_OVERRIDE:-}" && -n "${VEHICLE_NAME:-}" && "${VEHICLE_NAME}" != "$(hostname)" ]]; then
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${VEHICLE_NAME}.local:7447\"]"
fi
EOF
