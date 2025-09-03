#!/bin/bash
set -e

# Terminal 1: Run localization
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 launch my_bot localization_launch.py map:=./my_map_save.yaml use_sim_time:=True;
  exec bash
"

# Give AMCL some time to start
#sleep 15
sleep 10

# Terminal 2: Publish initial pose
gnome-terminal -- bash -c "
  source install/setup.bash;
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
    header: {frame_id: \"map\"},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
      },
      covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
    }
  }';

  sleep 1;
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
    header: {frame_id: \"map\"},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
      },
      covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
    }
  }';

  exec bash
"


sleep 10

rviz2