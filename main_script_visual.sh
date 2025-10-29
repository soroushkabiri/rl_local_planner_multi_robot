#!/bin/bash
set -e

# Kill any old tmux sessions
tmux kill-server || true
sleep 3

# First tmux session
tmux new-session -d -s ros_sim1 "./1_launch.sh" \; \
split-window -v "sleep 30; ./2_launch.sh" \; \
split-window -h "sleep 50; ./3_launch.sh" \; \
split-window -v "sleep 50; ./4_launch.sh" \; \
split-window -h "sleep 52; ./10_launch.sh" \; \
split-window -v "sleep 60; ./5_launch.sh" \; \
split-window -h "sleep 70; ./6_launch.sh" \; \
select-layout tiled

# Second tmux session
tmux new-session -d -s ros_sim2 "sleep 80; ./7_launch.sh" \; \
split-window -v "sleep 85; ./8_launch.sh" \; \
split-window -h "sleep 95; ./9_launch.sh" \; \
split-window -v "sleep 105; ./11_launch.sh" \; \
split-window -h "sleep 107; ./12_launch.sh" \; \
split-window -v "sleep 107; ./13_launch.sh" \; \
select-layout tiled


# Open both sessions in separate terminal windows
gnome-terminal -- bash -c "tmux attach -t ros_sim1" &
gnome-terminal -- bash -c "tmux attach -t ros_sim2" &


#ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{start: {},goal: {header: {frame_id: 'map'}, pose: {position: {x: -14.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}},use_start: false}"
