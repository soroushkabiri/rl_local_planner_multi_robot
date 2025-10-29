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
    xacro_file_robot = os.path.join(pkg_path,'description','robot_corr.urdf.xacro')
    xacro_file_rect = os.path.join(pkg_path,'description','rect_obj_corr.urdf.xacro')
    leader_robot='robot0_0'

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
    ld.add_action(gzclient_cmd)
    
    # now inserting rectangular object
    robot_description_rect = Command(['xacro ', xacro_file_rect, ' prefix:=', 'rect_obj' + '_'])
    rect_obj_state_publisher = Node(package="robot_state_publisher",
                namespace='rect_obj',executable="robot_state_publisher",output="screen",
                parameters=[{'robot_description': robot_description_rect,"use_sim_time": True,'prefix': 'rect_obj'+'_'}],)

    # Create spawn call
    spawn_entity_rect_obj = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", 'rect_obj' + "/robot_description","-entity", 'rect_obj', "-robot_namespace", 'rect_obj',
                    "-x", '0.0', "-y", '0.0', "-z", "0.0", "-Y", "3.14159",
                  #    "-unpause", 
                     ] , output="screen")

    ld.add_action(rect_obj_state_publisher)
    ld.add_action(spawn_entity_rect_obj)

    ROWS, COLS = 2, 2

    # Define initial poses for the robots (skip the 4th one)
    robot_configs = [
        {"name": "robot0_0", "x": 0,   "y": -0.9, "yaw": 4.71238},
        {"name": "robot0_1", "x": 0.9, "y": 0,   "yaw": 0},
        {"name": "robot1_0", "x": 0,   "y": 0.9, "yaw": 1.570795},
    ]

    last_action = None

    for cfg in robot_configs:
        name = cfg["name"]

        # Robot description
        robot_description = ParameterValue(
            Command(['xacro ', xacro_file_robot, ' prefix:=', name + '_']),
            value_type=str)

        # Robot state publisher
        robot_state_publisher = Node(
            package="robot_state_publisher",
            namespace=name,
            executable="robot_state_publisher",
            output="screen",
            parameters=[{'robot_description': robot_description, "use_sim_time": True, 'prefix': name+'_'}],
        )

        # Spawn entity
        spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", f"{name}/robot_description",
                "-entity", name,
                "-robot_namespace", name,
                "-x", str(cfg["x"]),
                "-y", str(cfg["y"]),
                "-z", "0.0",
                "-Y", str(cfg["yaw"])
            ],
            output="screen",)

        if last_action is None:
            # First robot: add actions directly
            ld.add_action(robot_state_publisher)
            ld.add_action(spawn_entity)
        else:
            # Subsequent robots: register to run after previous spawn
            ld.add_action(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[robot_state_publisher, spawn_entity])))

        last_action = spawn_entity

    joystick_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('my_bot'),'launch','joystick.launch.py')]), 
        launch_arguments={'cmd_vel_topic': '/cmd_vel_joy'}.items())
    
    ld.add_action(joystick_launcher)

    twis_mux_params=os.path.join(get_package_share_directory('my_bot'),'config','twist_mux.yaml')
    twist_mux=Node(package='twist_mux', executable='twist_mux', parameters=[twis_mux_params,{'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/robot0_0/cmd_vel')])
    ld.add_action(twist_mux)

    return ld

    #colcon build --packages-select my_bot