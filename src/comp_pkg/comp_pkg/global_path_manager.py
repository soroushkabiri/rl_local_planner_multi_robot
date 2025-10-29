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


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import yaml
import os

class GlobalPathManager(Node):
    def __init__(self):
        super().__init__('global_path_manager')

        # File path to save the global path
        self.path_file = os.path.expanduser('~/ros_for_project_1/articulate_robot/my_map_3_global_path.yaml')

        self.global_path = None
        self.latched = False

        # Publishers and subscribers
        self.publisher_ = self.create_publisher(Path, '/global_path_nav2', 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)

        # Try loading a previously saved path
        self.load_saved_path()

    # -------------------------------
    # Callbacks
    # -------------------------------
    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal reached → keeping saved path.")
            # You can uncomment below to reset if you prefer
            # self.latched = False
            # self.global_path = None

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

    # -------------------------------
    # File handling
    # -------------------------------
    def save_path_to_file(self, path_msg: Path):
        """Save Path message to YAML file."""
        data = {
            'frame_id': path_msg.header.frame_id,
            'poses': [
                {
                    'position': {
                        'x': p.pose.position.x,
                        'y': p.pose.position.y,
                        'z': p.pose.position.z
                    },
                    'orientation': {
                        'x': p.pose.orientation.x,
                        'y': p.pose.orientation.y,
                        'z': p.pose.orientation.z,
                        'w': p.pose.orientation.w
                    }
                }
                for p in path_msg.poses
            ]
        }

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

        # Construct a Path message
        path_msg = Path()
        path_msg.header.frame_id = data.get('frame_id', 'map')

        from geometry_msgs.msg import PoseStamped
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


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
