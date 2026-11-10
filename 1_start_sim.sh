#!/bin/bash
xhost +local:root >/dev/null 2>&1
xhost + >/dev/null 2>&1
docker start robotics_essentials_ros2 >/dev/null 2>&1 || true
docker exec -it robotics_essentials_ros2 /ros_entrypoint.sh ros2 launch andino_gz andino_gz.launch.py
