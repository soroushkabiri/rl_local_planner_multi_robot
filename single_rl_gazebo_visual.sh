#!/bin/bash
set -e

# Kill any old tmux sessions
tmux kill-server || true
sleep 3

# First tmux session
tmux new-session -d -s ros_sim1 "./1_single_launch.sh" \; \
split-window -h "sleep 12; ./2_single_launch.sh" \; \
split-window -v "sleep 13; ./3_single_launch_rl.sh" \; \
split-window -h "sleep 14; ./4_single_launch_rl.sh" \; \
split-window -v "sleep 15; ./5_single_rl_launch.sh" \; \
split-window -h "sleep 16; ./6_single_launch.sh" \; \
select-layout tiled \; attach


# # Second tmux session
# tmux new-session -d -s ros_sim2 "sleep 15; ./5_single_rl_launch.sh" \; \
# split-window -v "sleep 16; ./6_single_launch.sh" \; \
# select-layout tiled



# # Open both sessions in separate terminal windows
# gnome-terminal -- bash -c "tmux attach -t ros_sim1" &
# gnome-terminal -- bash -c "tmux attach -t ros_sim2" &


#ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{start: {},goal: {header: {frame_id: 'map'}, pose: {position: {x: -14.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}},use_start: false}"
