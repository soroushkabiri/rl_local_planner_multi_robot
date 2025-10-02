from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Find nav2_bringup package
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')
    # Your package
    pkg_my_bot = FindPackageShare('my_bot').find('my_bot')

    # Path to your params file
    nav2_params = os.path.join(pkg_my_bot, 'config', 'nav2_params_actual_robot.yaml')

    # Declare use_sim_time argument (default false for real robot)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true (false for real robot)'
    )

    # Include nav2 bringup launch file
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        nav2
    ])
