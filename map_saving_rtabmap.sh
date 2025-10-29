#!/bin/bash
set -e

dir_rtabmap=~/ros2_ws_build_rtabmap/install/setup.bash
map_dir=~/ros_for_project_1/articulate_robot/map3

tmux new-session -d -s actual_robot_map_save \
"source $dir_rtabmap; ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty '{}' " \; \
split-window -v "sleep 6; source $dir_rtabmap; ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty '{}' " \; \
split-window -h "sleep 10; source $dir_rtabmap; ros2 run nav2_map_server map_saver_cli -f $map_dir --ros-args -r /map:=/rtabmap/map" \; \
split-window -v "sleep 14; source $dir_rtabmap; ros2 run nav2_map_server map_saver_cli -f $map_dir --ros-args -r /map:=/rtabmap/map" \; \
split-window -h "sleep 18; source $dir_rtabmap; mv ~/.ros/rtabmap.db ~/ros_for_project_1/articulate_robot/rtabmap_map3.db" \; \
select-layout tiled \; attach



# so important: i have to run this in terminal to save the 3d map and move it to another place

# ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}"

# so important: for saving 2d map:

#ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r /map:=/rtabmap/map

#mv ~/.ros/rtabmap.db ~/rtabmap_mymap.db