#!/bin/bash
set -e

# Publish initial pose

source install/setup.bash
sleep 1

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]"


sleep 1


ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]"
