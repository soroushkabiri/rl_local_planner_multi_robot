import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file_rect = os.path.join(pkg_path,'description','actual_robot_rect_obj_corr.urdf.xacro')




    # now inserting rectangular object
    robot_description_rect = Command(['xacro ', xacro_file_rect, ' prefix:=rect_obj_'])
    rect_obj_state_publisher = Node(package="robot_state_publisher",
                namespace='rect_obj',executable="robot_state_publisher",output="screen",
                parameters=[{'robot_description': ParameterValue(robot_description_rect, value_type=str),
                             #"use_sim_time": True,
                             'prefix': 'rect_obj_'}],)

    # Create spawn call
    spawn_entity_rect_obj = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", 'rect_obj' + "/robot_description","-entity", 'rect_obj', "-robot_namespace", 'rect_obj',
                    "-x", '0.0', "-y", '0.0', "-z", "0.1", "-Y", "0.0",
                      "-unpause", 

                     ] , output="screen")

    ld.add_action(rect_obj_state_publisher)
    #ld.add_action(spawn_entity_rect_obj)

    



    return ld


    #colcon build --packages-select my_bot
