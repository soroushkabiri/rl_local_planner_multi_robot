import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool


class GlobalPathManager(Node):
    def __init__(self):
        super().__init__('global_path_manager')

        self.global_path = None
        self.latched = False  # whether we already stored a path

        # Subscriptions
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)

        # Publisher for static global path
        self.publisher_ = self.create_publisher(Path, '/global_path_nav2', 10)
    
    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal reached → reset path latch.")
            #self.latched = False
            #self.global_path = None
    
    def plan_callback(self, msg: Path):
        """Latch the first non-empty global path from Nav2 and publish it."""
        if self.latched:
            return

        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path from /plan, waiting...")
            return

        # Store and latch the path
        self.global_path = msg
        self.latched = True

        self.get_logger().info(
            f"Latched global path with {len(self.global_path.poses)} poses."
        )

        # Immediately publish once
        self.publisher_.publish(self.global_path)






def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
