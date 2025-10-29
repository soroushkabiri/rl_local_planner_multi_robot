#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

class GazeboResetNode(Node):
    def __init__(self):
        super().__init__('gazebo_reset_node')

        # Create clients for Gazebo control services
        self.cli_pause = self.create_client(Empty, '/pause_physics')
        self.cli_unpause = self.create_client(Empty, '/unpause_physics')
        self.cli_reset_sim = self.create_client(Empty, '/reset_simulation')

        # Wait for services to be available
        for cli, name in [
            (self.cli_pause, '/pause_physics'),
            (self.cli_unpause, '/unpause_physics'),
            (self.cli_reset_sim, '/reset_simulation')
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

        self.req = Empty.Request()

    def pause(self):
        self.get_logger().info('Pausing physics...')
        future = self.cli_pause.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Physics paused.')

    def unpause(self):
        self.get_logger().info('Unpausing physics...')
        future = self.cli_unpause.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Physics unpaused.')

    def reset_simulation(self):
        self.get_logger().info('Resetting full simulation (time + world)...')
        future = self.cli_reset_sim.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Simulation reset done!')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboResetNode()

    # Pause
    node.pause()
    node.get_logger().info('Waiting 5 seconds before reset...')
    time.sleep(10)

    # Reset
    node.reset_simulation()
    node.get_logger().info('Waiting 5 seconds before unpause...')
    time.sleep(10)

    # Unpause
    node.unpause()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Empty
# from geometry_msgs.msg import Twist
# import time

# class GazeboResetWorldNode(Node):
#     def __init__(self):
#         super().__init__('gazebo_reset_world_node')

#         # Gazebo control services
#         self.cli_pause = self.create_client(Empty, '/pause_physics')
#         self.cli_unpause = self.create_client(Empty, '/unpause_physics')
#         self.cli_reset_world = self.create_client(Empty, '/reset_world')

#         # Wait for services
#         for cli, name in [
#             (self.cli_pause, '/pause_physics'),
#             (self.cli_unpause, '/unpause_physics'),
#             (self.cli_reset_world, '/reset_world')
#         ]:
#             while not cli.wait_for_service(timeout_sec=1.0):
#                 self.get_logger().info(f'Waiting for {name} service...')

#         # Publishers to all robot cmd_vel topics
#         self.robot_topics = [
#             '/robot0_0/cmd_vel',
#             '/robot0_1/cmd_vel',
#             '/robot1_0/cmd_vel'
#         ]
#         self.vel_publishers = [self.create_publisher(Twist, topic, 10) for topic in self.robot_topics]
#         self.req = Empty.Request()

#     def pause(self):
#         self.get_logger().info('Pausing physics...')
#         future = self.cli_pause.call_async(self.req)
#         rclpy.spin_until_future_complete(self, future)
#         self.get_logger().info('Physics paused.')

#     def unpause(self):
#         self.get_logger().info('Unpausing physics...')
#         future = self.cli_unpause.call_async(self.req)
#         rclpy.spin_until_future_complete(self, future)
#         self.get_logger().info('Physics unpaused.')

#     def reset_world(self):
#         self.get_logger().info('Resetting world (positions and velocities)...')
#         future = self.cli_reset_world.call_async(self.req)
#         rclpy.spin_until_future_complete(self, future)
#         self.get_logger().info('World reset done!')

#     def stop_all_robots(self):
#         self.get_logger().info('Publishing zero velocity to all robots...')
#         twist = Twist()  # all zeros
#         for pub in self.vel_publishers:
#             pub.publish(twist)
#         time.sleep(0.5)  # short delay to ensure message delivery
#         self.get_logger().info('All robots stopped.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GazeboResetWorldNode()

#     node.pause()
#     node.get_logger().info('Waiting 5 seconds before world reset...')
#     time.sleep(5)

#     node.reset_world()

#     # Stop all robots right after reset
#     node.stop_all_robots()

#     node.get_logger().info('Waiting 5 seconds before unpause...')
#     time.sleep(5)

#     node.unpause()

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
