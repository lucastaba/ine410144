#!/bin/bash

xhost +local:docker

mkdir -p ~/Desktop/ros_logs

: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"

docker run -d --rm --init \
  --network=host \
  --env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  --env RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  --env DISPLAY=$DISPLAY \
  --name vehicle_model_double_track_cpp \
  -v "$(pwd)/config:/dev_ws/config" \
  -v "$(pwd)/vehicle_simulation_logs:/dev_ws/logs" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  open_car_dynamics:local \
  bash -c "source /dev_ws/install/setup.bash && ros2 run vehicle_model_nodes vehicle_model_double_track_cpp_node --ros-args --params-file /dev_ws/config/example_config.yml"
