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
        self.cli_reset_world = self.create_client(Empty, '/reset_world')

        # Wait for services to be available
        for cli, name in [
            (self.cli_pause, '/pause_physics'),
            (self.cli_unpause, '/unpause_physics'),
            (self.cli_reset_world, '/reset_world')
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

    def reset_world(self):
        self.get_logger().info('Resetting world (models + physics, time unchanged)...')
        future = self.cli_reset_world.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('World reset done!')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboResetNode()

    node.pause()
    node.get_logger().info('Waiting 2 seconds before world reset...')
    time.sleep(2)

    node.reset_world()
    node.get_logger().info('Waiting 2 seconds before unpause...')
    time.sleep(2)

    node.unpause()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
