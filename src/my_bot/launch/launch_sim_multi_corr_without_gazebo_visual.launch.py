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

def generate_launch_description():
    ld = LaunchDescription()
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file_robot = os.path.join(pkg_path,'description','robot_corr.urdf.xacro')
    xacro_file_rect = os.path.join(pkg_path,'description','rect_obj_corr.urdf.xacro')
    leader_robot='robot0_0'
    base_world= os.path.join(pkg_path, "worlds", "base.world")
    world = os.path.join(pkg_path, "worlds", "world2.world")


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        #launch_arguments={"world": base_world}.items(),

        launch_arguments={"world": base_world,}.items(),
        )
        
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")),
                   launch_arguments={'gui-client-plugin': ''}.items() )

    ld.add_action(gzserver_cmd)
    ##########
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

    ROWS = 2
    COLS = 2

    x = -ROWS
    y = -COLS
    last_action = None

    # Spawn turtlebot3 instances in gazebo
    for i in range(COLS):
        for j in range(ROWS):
            # Construct a unique name and namespace
            name = "robot" + str(i) + "_" + str(j)
            robot_description = Command(['xacro ', xacro_file_robot, ' prefix:=', name + '_'])

            turtlebot_state_publisher = Node(package="robot_state_publisher",
                namespace=name,executable="robot_state_publisher",output="screen",
                parameters=[{'robot_description': robot_description,"use_sim_time": True,'prefix': name+'_'}],)

            #defining initial pose of robots
            #f
            if i==0 and j==0:
                x='-0.9'    
                y='0'
                yaw='3.14159'
               # yaw='2.8'

            #l
            elif i==0 and j==1:
                x='0'
                y='-0.9'
                yaw='4.71238'
            #b
            elif i==1 and j==0:
                x='0.9'
                y='0'
                yaw='0'
            #r
            elif i==1 and j==1:
                x='0'
                y='0.9'
                yaw='1.570795'
               
            # Create spawn call
            spawn_entity = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", name + "/robot_description",
                    "-entity",
                    name,
                    "-robot_namespace",
                    name,
                    "-x", str(x), "-y", str(y), "-z", "0.0", "-Y", yaw, 
                    #"-unpause",
                    
                ],
                output="screen",
            )
            # Add after the spawn_entity creation:
            #print(f"Spawning robot: {name} at position ({x}, {y})")

            if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(turtlebot_state_publisher)
                ld.add_action(spawn_entity)

            else:
                # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
                # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
                spawn_turtlebot3_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[
                                 turtlebot_state_publisher,
                                 spawn_entity,
                        ]
                    )
                )
                ld.add_action(spawn_turtlebot3_event)

            # Save last instance for next RegisterEventHandler
            last_action = spawn_entity

    joystick_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('my_bot'),'launch','joystick.launch.py')]), 
        #launch_arguments={'cmd_vel_topic': '/'+leader_robot+'/cmd_vel'}.items())
        launch_arguments={'cmd_vel_topic': '/cmd_vel_joy'}.items())

    ld.add_action(joystick_launcher)


    twis_mux_params=os.path.join(get_package_share_directory('my_bot'),'config','twist_mux.yaml')
    twist_mux=Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twis_mux_params,{'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/robot0_0/cmd_vel')]
    )
    ld.add_action(twist_mux)



    return ld


    #colcon build --packages-select my_bot
