#!/bin/bash
set -e

source install/setup.bash
ros2 launch comp_pkg imu_yaw_single.launch.py

