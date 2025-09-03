import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class WaypointCatcher(Node):
    def __init__(self, num_waypoints=100):
        super().__init__('waypoint_catcher')

        self.num_waypoints = num_waypoints
        self.waypoints = []  # Downsampled waypoints

        self.create_subscription(Path, '/plan', self.plan_callback, 10)

    def plan_callback(self, msg: Path):
        if self.waypoints:  # Already saved
            return
        else:
            """Catch waypoints when Nav2 publishes a new plan."""
            total = len(msg.poses)
            if total == 0:
                self.get_logger().warn("Received empty path!")
                return

            # Downsample: pick evenly spaced waypoints
            step = max(1, total // self.num_waypoints)
            self.waypoints = [
                (msg.poses[i].pose.position.x, msg.poses[i].pose.position.y)
                for i in range(0, total, step)
            ][:self.num_waypoints]  # Ensure exactly N waypoints

            self.get_logger().info(
                f"Received {total} poses → reduced to {len(self.waypoints)} waypoints."
            )
            for i, (x, y) in enumerate(self.waypoints):
                self.get_logger().info(f"  WP {i}: ({x:.2f}, {y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCatcher(num_waypoints=10)  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -14.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
