from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    joy_params=os.path.join(get_package_share_directory('my_bot'),'config','joystick.yaml')
    # Declare the remapping destination as a launch argument
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/single_robot/cmd_vel',
        description='Remapped cmd_vel topic'
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    joy_node=Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node=Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        #remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
        remappings=[('/cmd_vel',cmd_vel_topic)]

    )



    return LaunchDescription([
        cmd_vel_topic_arg,
        joy_node,
        teleop_node
    ])