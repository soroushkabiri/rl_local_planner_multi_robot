#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class ConsensusTester(Node):
    def __init__(self):
        super().__init__('consensus_tester')
        self.signal_type="step"
        self.step_period=50
        #self.step_period=5000

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_fuzzy', 10)

        # Parameters
        self.max_v_leader = 0.4
        self.max_omega_leader = 1.0
        self.start_time = time.time()

        # Timer to publish at 50 Hz
        timer_period = 0.02  # [s]
        self.timer = self.create_timer(timer_period, self.publish_cmd)

    def publish_cmd(self):
        t = time.time() - self.start_time

        msg = Twist()

        if self.signal_type == "sin":
            msg.linear.x = 0*self.max_v_leader * math.sin(0.1*t)
            msg.angular.z =1* self.max_omega_leader * math.sin(0.1*t)

        elif self.signal_type == "step":
            # Periodic step signal (square wave)
            phase = (t % self.step_period) < (self.step_period / 2.0)
            msg.linear.x = 1*(self.max_v_leader if phase else -self.max_v_leader)
            msg.angular.z = 0*(self.max_omega_leader if phase else -self.max_omega_leader)


        self.cmd_pub.publish(msg)
        
        self.get_logger().info(
            f"Publishing: v={msg.linear.x:.3f}, omega={msg.angular.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ConsensusTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
