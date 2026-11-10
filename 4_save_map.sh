#!/bin/bash
docker exec -it robotics_essentials_ros2 /ros_entrypoint.sh ros2 run nav2_map_server map_saver_cli --free 0.15 --fmt png -f /home/user/andino_map
docker cp robotics_essentials_ros2:/home/user/andino_map.yaml /home/alireza/Slam/ 2>/dev/null || true
docker cp robotics_essentials_ros2:/home/user/andino_map.png /home/alireza/Slam/ 2>/dev/null || true
echo "نقشه با موفقیت در /home/alireza/Slam/ و داخل کانتینر ذخیره شد."
