from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # First service (no delay)
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/DETACHLINK', 'linkattacher_msgs/srv/DetachLink',
                "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot0_0', link2_name: 'robot0_0_fixed_link'}"
            ],
            output='screen'
        ),


        # Third service after 7 seconds
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/DETACHLINK', 'linkattacher_msgs/srv/DetachLink',
                        "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot1_0', link2_name: 'robot1_0_fixed_link'}"
                    ],
                    output='screen'
                )
            ]
        ),

        # Fourth service after 9 seconds
        TimerAction(
            period=9.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/DETACHLINK', 'linkattacher_msgs/srv/DetachLink',
                        "{model1_name: 'rect_obj', link1_name: 'base_link', model2_name: 'robot0_1', link2_name: 'robot0_1_fixed_link'}"
                    ],
                    output='screen'
                )
            ]
        ),
    ])
