#!/bin/bash
set -e

source install/setup.bash

ros2 launch comp_pkg nodes_before_nav2_slamtoolbox.launch.py