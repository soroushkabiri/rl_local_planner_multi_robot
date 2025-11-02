#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class ObservationState(Node):
    def __init__(self):
        super().__init__('observation_state')

        # Initialize storage variables as NumPy arrays
        self.rect_obj_pos = np.zeros(2, dtype=np.float32)  # x, y from odom
        self.leader_teta = np.array([0.0], dtype=np.float32)  # yaw in radians
        self.obstacles = np.zeros((6, 2), dtype=np.float32)  # 6 obstacles, each [x, y]
        self.current_wp = np.zeros(2, dtype=np.float32)        # current waypoint [x, y]
        self.last_wp = np.zeros(2, dtype=np.float32)        # last waypoint [x, y]


        # Publisher for aggregated observation
        self.obs_pub = self.create_publisher(Float32MultiArray, '/observation_state', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/robot0_0/yaw_rad', self.yaw_callback, 10)
        self.create_subscription(PoseStamped, '/current_waypoint', self.current_wp_callback, 10)
        self.create_subscription(PoseStamped, '/last_waypoint', self.last_wp_callback, 10)


        # Subscribe to 6 obstacle topics
        for i in range(6):
            topic_name = f'/closest_obstacle_{i+1}_in_range' if i < 5 else '/closest_obstacle_in_range'
            self.create_subscription(Point, topic_name, self.make_obstacle_callback(i), 10)

        # Timer to periodically publish aggregated observation
        self.create_timer(0.05, self.publish_observation)  # 20 Hz
        self.get_logger().info("ObservationState node started. Publishing /observation_state at 20 Hz.")

    def odom_callback(self, msg: Odometry):
        self.rect_obj_pos[:] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=np.float32)

    def yaw_callback(self, msg: Float32):
        self.leader_teta[0] = msg.data

    def make_obstacle_callback(self, index):
        # Return a callback function that saves the obstacle at a fixed index
        def callback(msg: Point):
            self.obstacles[index, :] = np.array([msg.x, msg.y], dtype=np.float32)
        return callback

    def current_wp_callback(self, msg: PoseStamped):
        """Store the current waypoint x, y."""
        self.current_wp[:] = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ], dtype=np.float32)


    def last_wp_callback(self, msg: PoseStamped):
        """Store the last waypoint x, y."""
        self.last_wp[:] = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ], dtype=np.float32)


    def publish_observation(self):
        """Publish all observations as a single flattened Float32MultiArray."""
        obs_vector = np.concatenate([
            #self.current_wp,          # 2
            self.rect_obj_pos,          # 2
            self.leader_teta,           # 1
            self.obstacles.flatten(),    # 6*2 = 12
            self.last_wp,          # 2
        ]).astype(np.float32)           # Total length = 17 (without current waypoint)
        msg = Float32MultiArray()
        msg.data = obs_vector.tolist()
        self.obs_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObservationState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
###############probably have to use rect obj teta as observation and previous action as observation too###############
