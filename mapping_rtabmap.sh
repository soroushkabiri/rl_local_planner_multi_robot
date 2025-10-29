#!/bin/bash
set -e

# Kill any old tmux sessions
tmux kill-server || true
sleep 3

dir_rtabmap=~/ros2_ws_build_rtabmap/install/setup.bash

# First tmux session
tmux new-session -d -s ros_sim1 "./1_launch.sh" \; \
split-window -v "sleep 30; ./2_launch.sh" \; \
split-window -h "sleep 50; ./3_launch.sh" \; \
split-window -v "sleep 50 ; rm /home/soroush/.ros/rtabmap.db" \; \
split-window -h "sleep 53; source $dir_rtabmap; ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/rect_obj/camera/image_raw depth_topic:=/rect_obj/camera/depth/image_raw camera_info_topic:=/rect_obj/camera/camera_info  publish_tf:=true  use_sim_time:=true  approx_sync:=true  approx_sync_max_interval:=0.6  topic_queue_size:=30  sync_queue_size:=30" \; \
select-layout tiled


# Second tmux session
tmux new-session -d -s ros_sim2 "sleep 60; ./8_launch.sh" \; \
split-window -v "sleep 65; ./9_launch.sh" \; \
select-layout tiled


# Open both sessions in separate terminal windows
gnome-terminal -- bash -c "tmux attach -t ros_sim1" &
gnome-terminal -- bash -c "tmux attach -t ros_sim2" &


