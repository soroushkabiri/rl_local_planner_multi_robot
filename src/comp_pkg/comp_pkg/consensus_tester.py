#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
from std_msgs.msg import Float32


class ConsensusTester(Node):
    def __init__(self):
        super().__init__('consensus_tester')
        self.signal_type = "step"   # "sin" or "step"
        self.step_period = 25

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'robot0_0/cmd_vel', 10)
        self.yaw_pub = self.create_publisher(Float32, '/robot0_0/yaw_deg', 10)

        # Parameters
        self.max_v_leader = 0.4
        self.max_yaw_leader = 180.0   # yaw amplitude (deg)
        self.start_time = time.time()

        # Timer to publish at 50 Hz
        timer_period = 0.02  # [s]
        self.timer = self.create_timer(timer_period, self.publish_cmd)

    def publish_cmd(self):
        t = time.time() - self.start_time

        msg = Twist()
        yaw_msg = Float32()

        if self.signal_type == "sin":
            # linear.x = sinusoidal
            msg.linear.x = self.max_v_leader * math.sin(0.25 * t)
            msg.angular.z = 0.0  # not used

            # yaw = sinusoidal (shifted to 0–360 range)
            yaw_val = self.max_yaw_leader+self.max_yaw_leader * math.sin(0.25 * t)
            yaw_msg.data = (yaw_val + 360.0) % 360.0

        elif self.signal_type == "step":
            # linear.x = square wave
            phase = (t % self.step_period) < (self.step_period / 2.0)
            msg.linear.x = self.max_v_leader if phase else -self.max_v_leader
            msg.angular.z = 0.0

            # yaw = square wave (0° or 180°)
            yaw_msg.data = 4.0 if phase else 180.0

        # Publish both
        self.cmd_pub.publish(msg)
        self.yaw_pub.publish(yaw_msg)

        self.get_logger().info(
            f"Publishing: v={msg.linear.x:.3f}, yaw={yaw_msg.data:.2f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
