#!/bin/bash
echo "در حال اجرای نود اکتشاف و حرکت خودکار ربات..."
echo "برای توقف حرکت، کافیست Ctrl+C را در این پنجره بزنید."
echo "-------------------------------------------------------------"
docker exec -it robotics_essentials_ros2 /ros_entrypoint.sh python3 /home/user/exercises_ws/src/auto_explore.py
