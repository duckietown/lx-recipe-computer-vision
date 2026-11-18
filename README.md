# LX Recipe: Computer Vision

This is the recipe for the [`lx-computer-vision`](https://github.com/duckietown/lx-computer-vision)
Learning Experience (ROS 2 Jazzy, `rmw_zenoh_cpp`, `ente` distro). It holds everything the learner
never sees: the Docker images, the dependencies, the launcher and the instructor-provided nodes.

Learners do not clone this repository. The Duckietown Shell pulls it automatically because
`lx-computer-vision/.dtproject` points at it:

    RECIPE_REPOSITORY=duckietown/lx-recipe-computer-vision
    RECIPE_BRANCH=ente-ros2

## What is here

| Path | Purpose |
| --- | --- |
| `Dockerfile` | workbench image (`dts code build` / `dts code workbench`), based on `dt-ros2-commons`; builds `packages/` with colcon |
| `Dockerfile.vscode` | image behind `dts code editor` (code-server + ROS 2 Jazzy + `duckietown_msgs`) |
| `dependencies-apt.txt`, `dependencies-py3.txt` | runtime APT / pip packages of the workbench image (pip packages are installed in the editor too) |
| `dependencies-apt.vscode.txt` | ROS 2 APT packages for the editor image |
| `launchers/default.sh` | what runs when the workbench container starts: one `ros2 launch` |
| `settings.yaml` | Duckiematrix map and vehicle used by `dts code start_matrix` |
| `assets/vscode/setup.sh`, `assets/vscode/launch.sh` | code-server settings, ROS-enabled terminals, writable colcon directories |
| `assets/duckiematrix/` | the `loop` map |
| `packages/lx_ros2_tools/` | helpers shared by every ROS 2 LX: image conversion, `LiveParams`, robot parameters, notebook panels (`RosSession`, `ActionButtons`, `ParamPanel`, `ImageView`, `TrajectoryPlot`) |
| `packages/encoder_pose/` | instructor node `encoder_pose_node` (dead reckoning from the wheel encoders, for the trajectory plot) |

## What gets launched

`launchers/default.sh` runs

    ros2 launch visual_lane_servoing visual_lane_servoing_node.launch.py veh:=ROBOT

which starts the learner's `vls_node` (package `visual_lane_servoing` in the LX repository) and includes
`encoder_pose/launch/encoder_pose_node.launch.py`. Both nodes live in the robot namespace `/ROBOT`.

`encoder_pose_node`:

- subscribes `left_wheel_encoder_driver_node/tick`, `right_wheel_encoder_driver_node/tick` (`duckietown_msgs/WheelEncoderStamped`, sensor-data QoS)
- publishes `encoder_pose_node/pose` (`nav_msgs/Odometry`, frame `map`)
- service `encoder_pose_node/reset` (`std_srvs/Trigger`)
- parameter `publish_rate` (Hz, default 2.0); wheel radius / baseline come from `lx_ros2_tools.robot.get_wheel_params`

`vls_node` (learner node, see the LX repository): subscribes the camera, publishes `car_cmd_switch_node/cmd`
(`duckietown_msgs/Twist2DStamped`) and the debug masks `vls_node/debug/{left,right}_mask/compressed`,
exposes the `std_srvs/Trigger` services `vls_node/{init,calibrate,start,stop}` and the parameters `v_0`,
`omega_max`.

## Robot side

The ROS 2 stack must be running on the robot (real or virtual):

    dts stack up -H ROBOT ros2/duckiebot
    dts stack up -H ROBOT ros2-core/default

The workbench container connects to the robot's zenoh router (`tcp/ROBOT.local:7447`); the launcher sets
`ZENOH_CONFIG_OVERRIDE` from `VEHICLE_NAME`.

The launcher passes `car_cmd_topic:="${CAR_CMD_TOPIC:-car_cmd_switch_node/cmd}"` to the launch file, which remaps the
node's `car_cmd_switch_node/cmd` publisher. Robot images of `dt-ros2-core` built before 2026-10-20 still have
`kinematics_node` subscribing `/ROBOT/car_cmd`: if the node publishes but the robot does not move, check
`ros2 node info /ROBOT/kinematics_node` and relaunch with `car_cmd_topic:=car_cmd` (`CAR_CMD_TOPIC=car_cmd` for the launcher).
Right after connecting, the graph takes a few seconds to sync through the zenoh router: use
`ros2 topic list --no-daemon --spin-time 5` when a list comes back empty.

## Testing local recipe changes

Check out this repository beside the LX repository and, from the LX directory:

    dts code build -R ROBOT --recipe ../lx-recipe-computer-vision
    dts code workbench -R ROBOT --recipe ../lx-recipe-computer-vision
    dts code editor --recipe ../lx-recipe-computer-vision

Without a robot, the images can be built and smoke-tested with plain docker:

    docker buildx build --load -f Dockerfile --build-context recipe=. --build-context meat=../lx-computer-vision \
      --build-arg ARCH=arm64v8 --build-arg DISTRO=ente --build-arg DOCKER_REGISTRY=docker.io --build-arg BASE_TAG=ente-arm64v8 \
      -t lx-computer-vision:test .
    docker run --rm -e VEHICLE_NAME=testbot lx-computer-vision:test bash -c \
      "source /environment.sh >/dev/null; timeout 15 ros2 launch visual_lane_servoing visual_lane_servoing_node.launch.py veh:=testbot car_cmd_topic:=car_cmd_switch_node/cmd; true"
