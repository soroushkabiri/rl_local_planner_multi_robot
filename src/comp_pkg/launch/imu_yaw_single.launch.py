# this launch file 1) catch yaw data from imu 
# 2) initialize consensus observer 
# 3) initialize desired cmd_vel node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    ld = LaunchDescription()

    name='single_robot'
    yaw_from_imu=Node(package='comp_pkg',
    executable='imu_yaw_node',
    name='imu_yaw_node',
    parameters=[{'imu_topic': name}],
    output='screen',)

    ld.add_action(yaw_from_imu)


    return ld

