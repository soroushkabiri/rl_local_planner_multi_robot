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



    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),

        )
        
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")),
                   launch_arguments={'gui-client-plugin': ''}.items() )

    #ld.add_action(gzserver_cmd)
    #ld.add_action(gzclient_cmd)
    


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

    

    joystick_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('my_bot'),'launch','joystick.launch.py')]), 
        #launch_arguments={'cmd_vel_topic': '/'+leader_robot+'/cmd_vel'}.items())
        launch_arguments={'cmd_vel_topic': '/cmd_vel_joy'}.items())

    #ld.add_action(joystick_launcher)


    twis_mux_params=os.path.join(get_package_share_directory('my_bot'),'config','twist_mux.yaml')
    twist_mux=Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twis_mux_params,
                    #{'use_sim_time': True}
                    ],
        remappings=[('/cmd_vel_out','/robot0_0/cmd_vel')]
    )
    #ld.add_action(twist_mux)


    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config/ekf.yaml'), {'use_sim_time': False}]
    )
    #ld.add_action(robot_localization_node)


    return ld


    #colcon build --packages-select my_bot
