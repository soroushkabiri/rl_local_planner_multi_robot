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
    time.sleep(4)

    # Reset
    node.reset_simulation()
    node.get_logger().info('Waiting 5 seconds before unpause...')
    time.sleep(2)

    # Unpause
    node.unpause()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


