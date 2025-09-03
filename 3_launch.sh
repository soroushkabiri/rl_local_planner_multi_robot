#!/bin/bash
set -e

source install/setup.bash
ros2 launch comp_pkg imu_yaw.launch.py


# Terminal 1: make gazebo model
#gnome-terminal -- bash -c "
#  source install/setup.bash;
#  ros2 launch comp_pkg imu_yaw.launch.py;
#  exec bash
#"
#sleep 6

#source install/setup.bash

#ros2 launch comp_pkg nodes_before_nav2_slamtoolbox.launch.py


# Terminal 2: connect robots to rect_obj
#gnome-terminal -- bash -c "
#  source install/setup.bash;
##  ros2 launch comp_pkg nodes_before_nav2_slamtoolbox.launch.py;
#  exec bash
#"
#sleep 2