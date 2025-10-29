# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Odometry
# from geometry_msgs.msg import Point
# import math

# class ClosestObstacleFinder(Node):
#     def __init__(self):
#         super().__init__('closest_obstacle_finder')

#         self.map_data = None
#         self.occupied_cells = []  # List of (i, j)
#         self.odom_pose = None

#         # Publisher for closest obstacle
#         self.closest_obstacle_pub = self.create_publisher(Point, 'closest_obstacle_in_range', 10)

#         # Subscribe to map or costmap
#         # self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
#         self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

#         # Subscribe continuously to odometry
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

#         # Timer to check closest obstacle periodically
#         self.timer = self.create_timer(0.1, self.check_closest_obstacle)

#     def map_callback(self, msg):
#         """Store occupied cells only once."""
#         if self.map_data is not None:
#             return  # Already processed

#         self.map_data = msg
#         width = msg.info.width
#         height = msg.info.height

#         # 🔹 Added: Debug statistics
#         occupied = sum(1 for v in msg.data if v >= 99)
#         unknown = sum(1 for v in msg.data if v < 0)
#         self.get_logger().info(
#             f"Map stats → Occupied: {occupied}, Unknown: {unknown}, Total: {len(msg.data)}"
#         )

#         # 🔹 Adaptive threshold — works for both map (100) and costmap (99+)
#         for i in range(height):
#             for j in range(width):
#                 idx = i * width + j
#                 if msg.data[idx] >= 50:  # Treat anything >=50 as occupied
#                     self.occupied_cells.append((i, j))

#         self.get_logger().info(
#             f"Stored {len(self.occupied_cells)} occupied cells from map/costmap."
#         )

#     def odom_callback(self, msg):
#         """Update robot position."""
#         self.odom_pose = (
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y
#         )

#     def check_closest_obstacle(self):
#         """Check closest obstacle within 2 meters radius and publish it."""
#         if self.map_data is None or self.odom_pose is None or not self.occupied_cells:
#             return

#         resolution = self.map_data.info.resolution
#         origin_x = self.map_data.info.origin.position.x
#         origin_y = self.map_data.info.origin.position.y

#         robot_x, robot_y = self.odom_pose
#         min_dist = float('inf')
#         closest_cell = None

#         for i, j in self.occupied_cells:
#             # Convert cell indices to world coordinates
#             x = origin_x + (j + 0.5) * resolution
#             y = origin_y + (i + 0.5) * resolution
#             dist = math.hypot(x - robot_x, y - robot_y)

#             if dist <= 5.0 and dist < min_dist:  # Only consider obstacles within 2 meters
#                 min_dist = dist
#                 closest_cell = (x, y)

#         if closest_cell:
#             self.get_logger().info(
#                 f"Closest obstacle within 2m: {closest_cell}, distance: {min_dist:.2f} m"
#             )
#             # Publish as Point
#             point_msg = Point()
#             point_msg.x = closest_cell[0]
#             point_msg.y = closest_cell[1]
#             point_msg.z = 0.0
#             self.closest_obstacle_pub.publish(point_msg)
#         else:
#             self.get_logger().info("No obstacle within 5 meters.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ClosestObstacleFinder()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


















#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
import math
import os
import yaml

class ClosestObstacleFinder(Node):
    def __init__(self):
        super().__init__('closest_obstacle_finder')

        self.map_data = None
        self.occupied_cells = []  # List of (i, j)
        self.odom_pose = None

        # File paths for offline storage
        self.map_yaml_file = os.path.expanduser('~/ros_for_project_1/articulate_robot/saved_map_closest_map_3.yaml')
        self.map_data_file = os.path.expanduser('~/ros_for_project_1/articulate_robot/saved_map_data_closest_3.txt')

        # Publisher for closest obstacle
        self.closest_obstacle_pub = self.create_publisher(Point, 'closest_obstacle_in_range', 10)

        # Try to load map from file first (offline mode)
        self.load_saved_map()

        # Subscribe to /map (if Nav2 is active)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for checking closest obstacle
        self.timer = self.create_timer(0.1, self.check_closest_obstacle)

    # -----------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        """Store and save occupied cells only once."""
        if self.map_data is not None:
            return  # Already loaded or saved

        self.map_data = msg
        width, height = msg.info.width, msg.info.height

        # Save map to file for offline reuse
        self.save_map_to_file(msg)

        # Process occupied cells
        self.extract_occupied_cells(msg)

    def odom_callback(self, msg: Odometry):
        """Update robot position."""
        self.odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def check_closest_obstacle(self):
        """Find closest obstacle within 5 meters."""
        if self.map_data is None or self.odom_pose is None or not self.occupied_cells:
            return

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        robot_x, robot_y = self.odom_pose
        min_dist = float('inf')
        closest_cell = None

        for i, j in self.occupied_cells:
            x = origin_x + (j + 0.5) * resolution
            y = origin_y + (i + 0.5) * resolution
            dist = math.hypot(x - robot_x, y - robot_y)
            if dist <= 5.0 and dist < min_dist:
                min_dist = dist
                closest_cell = (x, y)

        if closest_cell:
            self.get_logger().info(
                f"Closest obstacle within 5m: {closest_cell}, distance: {min_dist:.2f} m"
            )
            point_msg = Point()
            point_msg.x, point_msg.y, point_msg.z = closest_cell[0], closest_cell[1], 0.0
            self.closest_obstacle_pub.publish(point_msg)
        else:
            self.get_logger().info("No obstacle within 5 meters.")

    # -----------------------------------------------------------
    # Saving and Loading Map Files
    # -----------------------------------------------------------
    def save_map_to_file(self, msg: OccupancyGrid):
        """Save map metadata (YAML) and grid data (TXT)."""
        metadata = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'z': msg.info.origin.position.z
            }
        }

        with open(self.map_yaml_file, 'w') as f:
            yaml.dump(metadata, f)

        with open(self.map_data_file, 'w') as f:
            f.write(' '.join(map(str, msg.data)))

        self.get_logger().info(f"Saved map to {self.map_yaml_file} and {self.map_data_file}")

    def load_saved_map(self):
        """Load previously saved map from file if available."""
        if not os.path.exists(self.map_yaml_file) or not os.path.exists(self.map_data_file):
            self.get_logger().warn("No saved map found. Waiting for /map topic.")
            return

        with open(self.map_yaml_file, 'r') as f:
            metadata = yaml.safe_load(f)

        with open(self.map_data_file, 'r') as f:
            flat_data = list(map(int, f.read().split()))

        # Construct OccupancyGrid message
        msg = OccupancyGrid()
        msg.info.width = metadata['width']
        msg.info.height = metadata['height']
        msg.info.resolution = metadata['resolution']
        msg.info.origin.position.x = metadata['origin']['x']
        msg.info.origin.position.y = metadata['origin']['y']
        msg.info.origin.position.z = metadata['origin']['z']
        msg.data = flat_data

        self.map_data = msg
        self.extract_occupied_cells(msg)

        self.get_logger().info(
            f"Loaded saved map ({msg.info.width}x{msg.info.height}) with {len(self.occupied_cells)} occupied cells."
        )

    # -----------------------------------------------------------
    # Helper
    # -----------------------------------------------------------
    def extract_occupied_cells(self, msg: OccupancyGrid):
        """Extract and store occupied cell coordinates."""
        self.occupied_cells.clear()
        width, height = msg.info.width, msg.info.height

        occupied = sum(1 for v in msg.data if v >= 99)
        unknown = sum(1 for v in msg.data if v < 0)
        self.get_logger().info(f"Map stats → Occupied: {occupied}, Unknown: {unknown}, Total: {len(msg.data)}")

        for i in range(height):
            for j in range(width):
                idx = i * width + j
                if msg.data[idx] >= 50:  # Consider >=50 as occupied
                    self.occupied_cells.append((i, j))

        self.get_logger().info(f"Stored {len(self.occupied_cells)} occupied cells.")

def main(args=None):
    rclpy.init(args=args)
    node = ClosestObstacleFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

