#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import math


class InitialAlignmentNode(Node):
    def __init__(self):
        super().__init__("initial_alignment_node")
        
        # subscribers
        self.init_align_sub = self.create_subscription(
            Bool, "/rl_initial_alignment", self.init_align_callback, 10)
        
        self.yaw_sub = self.create_subscription(
            Float32, "/robot0_0/yaw_deg", self.yaw_callback, 10)

        # publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_fuzzy", 10)

        # store state
        self.align_enabled = False
        self.current_yaw_deg = None
        
        # fixed target yaw (-180 deg = -π rad)
        self.target_yaw_rad = -math.pi
        self.kp = 10.5
        self.yaw_tolerance_rad = math.radians(10)   # 3° tolerance

        # timer to run control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info("Initial Alignment Node Running")


    # --- Callbacks ---
    def init_align_callback(self, msg: Bool):
        self.align_enabled = msg.data
        self.get_logger().info(f"RL alignment triggered: {self.align_enabled}")


    def yaw_callback(self, msg: Float32):
        self.current_yaw_deg = msg.data


    # --- Controller ---
    def calculate_angular_z(self, desired_orientation, current_orientation):
        current_orientation = (current_orientation / 180.0) * math.pi

        # shortest rotation
        diff = ((current_orientation - desired_orientation + math.pi) % (2 * math.pi)) - math.pi

        angular_z = self.kp * diff

        # limit max speed
        max_speed = 2.0
        angular_z = max(-max_speed, min(max_speed, angular_z))

        return -angular_z  # invert per your logic


    def control_loop(self):
        if not self.align_enabled or self.current_yaw_deg is None:
            return
        
        # compute angular velocity
        angular_z = self.calculate_angular_z(self.target_yaw_rad, self.current_yaw_deg)

        # if yaw already close enough, stop
        if abs(((self.current_yaw_deg/180)*math.pi) - self.target_yaw_rad) < self.yaw_tolerance_rad:
            cmd = Twist()
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("✅ Alignment complete. Holding position.")
            return
        
        # publish command
        cmd = Twist()
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = InitialAlignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
