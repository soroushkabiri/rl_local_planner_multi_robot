# this is uses for the situation that we can use nav2 easily now that we have to train rl
# we cant use it and have to save waypoints for later usage


# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from std_msgs.msg import Bool

# class GlobalPathManager(Node):
#     def __init__(self):
#         super().__init__('global_path_manager')

#         self.global_path = None
#         self.latched = False  # whether we already stored a path

#         # Subscriptions
#         self.create_subscription(Path, '/plan', self.plan_callback, 10)
#         self.create_subscription(Bool, "/goal_reached", self.goal_reached_callback, 10)

#         # Publisher for static global path
#         self.publisher_ = self.create_publisher(Path, '/global_path_nav2', 10)
    
#     def goal_reached_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Goal reached → reset path latch.")
#             #self.latched = False
#             #self.global_path = None
    
#     def plan_callback(self, msg: Path):
#         """Latch the first non-empty global path from Nav2 and publish it."""
#         if self.latched:
#             return

#         if len(msg.poses) == 0:
#             self.get_logger().warn("Received empty path from /plan, waiting...")
#             return

#         # Store and latch the path
#         self.global_path = msg
#         self.latched = True

#         self.get_logger().info(
#             f"Latched global path with {len(self.global_path.poses)} poses.")

#         # Immediately publish once
#         self.publisher_.publish(self.global_path)

# def main(args=None):
#     rclpy.init(args=args)
#     node = GlobalPathManager()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# with this code we gate waypoints immediatley after starting


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from std_msgs.msg import Bool
# import yaml
# import os

# class GlobalPathManager(Node):
#     def __init__(self):
#         super().__init__('global_path_manager')

#         # File path to save the global path
#         self.path_file = os.path.expanduser('~/ros_for_project_1/articulate_robot/my_map_3_global_path.yaml')

#         self.global_path = None
#         self.latched = False

#         # Publishers and subscribers
#         self.publisher_ = self.create_publisher(Path, '/global_path_nav2', 10)         
#         self.create_subscription(Path, '/plan', self.plan_callback, 10)
#         self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)

#         # Try loading a previously saved path
#         self.load_saved_path()

#     # -------------------------------
#     # Callbacks
#     # -------------------------------
#     def goal_reached_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Goal reached → keeping saved path.")
#             # You can uncomment below to reset if you prefer
#             # self.latched = False
#             # self.global_path = None

#     def plan_callback(self, msg: Path):
#         """Latch the first valid path from Nav2 and save it."""
#         if self.latched:
#             return

#         if len(msg.poses) == 0:
#             self.get_logger().warn("Received empty path from /plan.")
#             return

#         # Latch and save
#         self.global_path = msg
#         self.latched = True
#         self.save_path_to_file(msg)

#         self.get_logger().info(f"Latched and saved global path with {len(msg.poses)} poses.")
#         self.publisher_.publish(msg)

#     # -------------------------------
#     # File handling
#     # -------------------------------
#     def save_path_to_file(self, path_msg: Path):
#         """Save Path message to YAML file."""
#         data = {
#             'frame_id': path_msg.header.frame_id,
#             'poses': [
#                 {
#                     'position': {
#                         'x': p.pose.position.x,
#                         'y': p.pose.position.y,
#                         'z': p.pose.position.z
#                     },
#                     'orientation': {
#                         'x': p.pose.orientation.x,
#                         'y': p.pose.orientation.y,
#                         'z': p.pose.orientation.z,
#                         'w': p.pose.orientation.w
#                     }
#                 }
#                 for p in path_msg.poses
#             ]
#         }

#         with open(self.path_file, 'w') as f:
#             yaml.dump(data, f)

#         self.get_logger().info(f"Global path saved to {self.path_file}")

#     def load_saved_path(self):
#         """Load Path message from YAML file if available."""
#         if not os.path.exists(self.path_file):
#             self.get_logger().warn("No saved path found. Waiting for Nav2 to publish one.")
#             return

#         with open(self.path_file, 'r') as f:
#             data = yaml.safe_load(f)

#         # Construct a Path message
#         path_msg = Path()
#         path_msg.header.frame_id = data.get('frame_id', 'map')

#         from geometry_msgs.msg import PoseStamped
#         for pose_data in data['poses']:
#             pose_stamped = PoseStamped()
#             pose_stamped.pose.position.x = pose_data['position']['x']
#             pose_stamped.pose.position.y = pose_data['position']['y']
#             pose_stamped.pose.position.z = pose_data['position']['z']
#             pose_stamped.pose.orientation.x = pose_data['orientation']['x']
#             pose_stamped.pose.orientation.y = pose_data['orientation']['y']
#             pose_stamped.pose.orientation.z = pose_data['orientation']['z']
#             pose_stamped.pose.orientation.w = pose_data['orientation']['w']
#             path_msg.poses.append(pose_stamped)

#         self.global_path = path_msg
#         self.latched = True
#         self.get_logger().info(f"Loaded saved global path with {len(path_msg.poses)} poses.")
#         self.publisher_.publish(path_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = GlobalPathManager()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
import yaml
import os
import math


class GlobalPathManager(Node):
    def __init__(self):
        super().__init__('global_path_manager')

        # File path to save the global path
        self.path_file = os.path.expanduser('~/ros_for_project_1/articulate_robot/my_map_3_global_path.yaml')
        self.global_path = None
        self.latched = False
        self.number_of_waypoints = 10  # <-- configurable number of waypoints to use

        # Publishers
        self.publisher_ = self.create_publisher(Path, '/global_path_nav2', 10)
        self.current_wp_pub = self.create_publisher(PoseStamped, '/current_waypoint', 10)
        self.current_wp_index_pub = self.create_publisher(Int32, '/current_waypoint_index', 10)

        # Subscribers
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Try loading a previously saved path
        self.load_saved_path()

    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal reached → keeping saved path.")

    def plan_callback(self, msg: Path):
        """Latch the first valid path from Nav2 and save it."""
        if self.latched:
            return

        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path from /plan.")
            return

        # Latch and save
        self.global_path = msg
        self.latched = True
        self.save_path_to_file(msg)
        self.get_logger().info(f"Latched and saved global path with {len(msg.poses)} poses.")
        self.publisher_.publish(msg)

    def odom_callback(self, msg: Odometry):
        """Find the closest waypoint and publish it."""
        if not self.global_path or len(self.global_path.poses) == 0:
            return
        
        # Downsample the path to self.number_of_waypoints (include last point)
        path = self.downsample_path(self.global_path, self.number_of_waypoints)

        # Current robot position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Find closest waypoint
        min_dist = float('inf')
        closest_idx = 0
        for i, pose_stamped in enumerate(path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Pick the NEXT waypoint (unless we’re already at the end)
        next_idx = min(closest_idx + 1, len(path.poses) - 1)

        # Publish current waypoint and index
        current_wp = path.poses[next_idx]
        index_msg = Int32()
        index_msg.data = next_idx

        self.current_wp_pub.publish(current_wp)
        self.current_wp_index_pub.publish(index_msg)

        self.get_logger().info(f"Current waypoint index: {closest_idx}/{len(path.poses)-1}, "
            f"position=({current_wp.pose.position.x:.2f}, {current_wp.pose.position.y:.2f})")

    def save_path_to_file(self, path_msg: Path):
        """Save Path message to YAML file."""
        data = {
            'frame_id': path_msg.header.frame_id,
            'poses': [{'position': { 'x': p.pose.position.x,'y': p.pose.position.y,'z': p.pose.position.z},
                    'orientation': {
                        'x': p.pose.orientation.x,'y': p.pose.orientation.y,'z': p.pose.orientation.z,'w': p.pose.orientation.w}}
                for p in path_msg.poses]}

        with open(self.path_file, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"Global path saved to {self.path_file}")

    def load_saved_path(self):
        """Load Path message from YAML file if available."""
        if not os.path.exists(self.path_file):
            self.get_logger().warn("No saved path found. Waiting for Nav2 to publish one.")
            return

        with open(self.path_file, 'r') as f:
            data = yaml.safe_load(f)

        path_msg = Path()
        path_msg.header.frame_id = data.get('frame_id', 'map')

        for pose_data in data['poses']:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose_data['position']['x']
            pose_stamped.pose.position.y = pose_data['position']['y']
            pose_stamped.pose.position.z = pose_data['position']['z']
            pose_stamped.pose.orientation.x = pose_data['orientation']['x']
            pose_stamped.pose.orientation.y = pose_data['orientation']['y']
            pose_stamped.pose.orientation.z = pose_data['orientation']['z']
            pose_stamped.pose.orientation.w = pose_data['orientation']['w']
            path_msg.poses.append(pose_stamped)

        self.global_path = path_msg
        self.latched = True
        self.get_logger().info(f"Loaded saved global path with {len(path_msg.poses)} poses.")
        self.publisher_.publish(path_msg)


    def downsample_path(self, path_msg: Path, num_points: int) -> Path:
        """Downsample path to exactly `num_points` waypoints including the actual last one, and print them."""
        total = len(path_msg.poses)
        if total <= num_points:
            self.get_logger().info(f"Path already has {total} poses — no downsampling needed.")
            return path_msg

        downsampled = Path()
        downsampled.header = path_msg.header

        # Compute evenly spaced indices between 0 and total - 1
        indices = [int(round(i * (total - 1) / (num_points - 1))) for i in range(num_points)]

        for idx in indices:
            downsampled.poses.append(path_msg.poses[idx])

        #  Ensure last waypoint is the *true* last pose (actual goal)
        downsampled.poses[-1] = path_msg.poses[-1]

        # Log waypoints
        #self.get_logger().info(f"Downsampled path to {len(downsampled.poses)} waypoints (last one is actual goal):")
        #for i, pose_stamped in enumerate(downsampled.poses):
            #x = pose_stamped.pose.position.x
            #y = pose_stamped.pose.position.y
            #self.get_logger().info(f"  Waypoint {i+1}/{len(downsampled.poses)}: (x={x:.2f}, y={y:.2f})")

        return downsampled




def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
