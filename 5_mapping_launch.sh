#!/bin/bash
set -e


source install/setup.bash

ros2 launch my_bot online_async_launch.py use_sim_time:=true



