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
split-window -h "sleep 60; ./5_mapping_launch.sh" \; \
select-layout tiled
#split-window -v "sleep 70; ./6_launch.sh" \; \
#split-window -h "sleep 80; ./7_launch.sh" \; \
#sleep 84

# Second tmux session
tmux new-session -d -s ros_sim2 "sleep 75; ./8_launch.sh" \; \
split-window -v "sleep 80; ./9_launch.sh" \; \
select-layout tiled
#split-window -h "sleep 100; ./10_launch.sh" \; \
#split-window -h "sleep 105; ./11_launch.sh" \; \

# Open both sessions in separate terminal windows
gnome-terminal -- bash -c "tmux attach -t ros_sim1" &
gnome-terminal -- bash -c "tmux attach -t ros_sim2" &


