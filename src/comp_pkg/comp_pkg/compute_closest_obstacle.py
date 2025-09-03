import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
import math

class ClosestObstacleFinder(Node):
    def __init__(self):
        super().__init__('closest_obstacle_finder')

        self.map_data = None
        self.occupied_cells = []  # List of (i, j)
        self.odom_pose = None

        # Publisher for closest obstacle
        self.closest_obstacle_pub = self.create_publisher(Point, 'closest_obstacle_in_range', 10)

        # Subscribe once to the global costmap
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        # Subscribe continuously to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer to check closest obstacle every second
        self.timer = self.create_timer(0.1, self.check_closest_obstacle)

    def map_callback(self, msg):
        """Store occupied cells only once."""
        if self.map_data is not None:
            return  # Already processed

        self.map_data = msg
        width = msg.info.width
        height = msg.info.height

        for i in range(height):
            for j in range(width):
                idx = i * width + j
                if msg.data[idx] >= 99:  # occupied/lethal
                    self.occupied_cells.append((i, j))

        self.get_logger().info(f"Stored {len(self.occupied_cells)} occupied cells from costmap.")

    def odom_callback(self, msg):
        """Update robot position."""
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def check_closest_obstacle(self):
        """Check closest obstacle within 2 meters radius and publish it."""
        if self.map_data is None or self.odom_pose is None or not self.occupied_cells:
            return

        width = self.map_data.info.width
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        robot_x, robot_y = self.odom_pose
        min_dist = float('inf')
        closest_cell = None

        for i, j in self.occupied_cells:
            # Convert cell indices to world coordinates
            x = origin_x + (j + 0.5) * resolution
            y = origin_y + (i + 0.5) * resolution
            dist = math.hypot(x - robot_x, y - robot_y)

            if dist <= 2.0 and dist < min_dist:  # Only consider obstacles within 2 meters
                min_dist = dist
                closest_cell = (x, y)

        if closest_cell:
            self.get_logger().info(
                f"Closest obstacle within 2m: {closest_cell}, distance: {min_dist:.2f} m"
            )

            # Publish as Point
            point_msg = Point()
            point_msg.x = closest_cell[0]
            point_msg.y = closest_cell[1]
            point_msg.z = 0.0
            self.closest_obstacle_pub.publish(point_msg)
        else:
            self.get_logger().info("No obstacle within 2 meters.")

def main(args=None):
    rclpy.init(args=args)
    node = ClosestObstacleFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
