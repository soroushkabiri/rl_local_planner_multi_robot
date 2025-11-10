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
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file_single_robot = os.path.join(pkg_path,'description','single_robot.urdf.xacro')

    #leader_robot='robot0_0'

    world = os.path.join(pkg_path, "worlds", "world3.world")
    base_world= os.path.join(pkg_path, "worlds", "base.world")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        #launch_arguments={"world": base_world}.items(),
        launch_arguments={"world": world,}.items(),)
        
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")),
                   launch_arguments={'gui-client-plugin': ''}.items() )

    ld.add_action(gzserver_cmd)
    #ld.add_action(gzclient_cmd)
    
    single_robot_description = Command(['xacro ', xacro_file_single_robot, ' prefix:=', 'single_robot' + '_'])

    single_robot_state_publisher = Node(package="robot_state_publisher",
                namespace='single_robot',executable="robot_state_publisher",output="screen",
                parameters=[{'robot_description': single_robot_description,"use_sim_time": True,'prefix': 'single_robot'+'_'}],)

    # Create spawn call
    spawn_entity_single_robot = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", 'single_robot' + "/robot_description","-entity", 'single_robot', "-robot_namespace", 'single_robot',
                    "-x", '0.0', "-y", '0.0', "-z", "0.0", "-Y", "3.14159",
                  #    "-unpause", 
                     ] , output="screen")

    ld.add_action(single_robot_state_publisher)
    ld.add_action(spawn_entity_single_robot)

    joystick_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('my_bot'),'launch','joystick_single.launch.py')]), 
        launch_arguments={'cmd_vel_topic': '/cmd_vel_joy'}.items())
    
    ld.add_action(joystick_launcher)

    twis_mux_params=os.path.join(get_package_share_directory('my_bot'),'config','twist_mux.yaml')
    twist_mux=Node(package='twist_mux', executable='twist_mux', parameters=[twis_mux_params,{'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/single_robot/cmd_vel')])
    ld.add_action(twist_mux)

    return ld

    #colcon build --packages-select my_bot