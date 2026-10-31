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
# connect `ros2 ...` CLI tools to the robot's zenoh router when a robot name is known
if [[ -z "${ZENOH_CONFIG_OVERRIDE:-}" && -n "${VEHICLE_NAME:-}" ]]; then
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${VEHICLE_NAME}.local:7447\"]"
fi
EOF
