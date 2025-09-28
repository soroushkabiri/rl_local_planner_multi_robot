#!/bin/bash
set -e

# Kill any old tmux sessions
tmux kill-server || true
sleep 3

# First tmux session
tmux new-session -d -s ros_sim1 "./1_launch.sh" \; \
split-window -v "sleep 30; ./2_launch.sh" \; \
split-window -h "sleep 50; ./3_launch.sh" \; \
split-window -v "sleep 52; ./4_launch.sh" \; \
select-layout tiled

#sleep 84

# Second tmux session
tmux new-session -d -s ros_sim2 "sleep 53; " \; \
split-window -v "sleep 54; ./9_launch.sh" \; \
split-window -v "sleep 65; ./13_launch.sh" \; \
select-layout tiled
#split-window -v "sleep 65; ./11_launch_consensus_tester.sh" \; \


# Open both sessions in separate terminal windows
gnome-terminal -- bash -c "tmux attach -t ros_sim1" &
gnome-terminal -- bash -c "tmux attach -t ros_sim2" &


#ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{start: {},goal: {header: {frame_id: 'map'}, pose: {position: {x: -14.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}},use_start: false}"
