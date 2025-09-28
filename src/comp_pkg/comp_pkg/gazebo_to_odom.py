#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
from geometry_msgs.msg import Vector3


# this node get the rect_obj from gazebo and use it to make rect_pobj/odom topic

class CartOdomPublisher(Node):
    def __init__(self):
        super().__init__('cart_odom_pub')
        self.last_stamp = None
        self.robot0_odom_sub = self.create_subscription(
            Odometry,
            '/robot0_0/odom',
            self.robot0_odom_cb,
            10
        )

        self.use_sim_time = self.get_parameter('use_sim_time').value

        self.sub = self.create_subscription(
            LinkStates,
            '/link_states',
            self.link_states_cb,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cart_link_name = 'rect_obj::base_link'  # adjust if needed

    def robot0_odom_cb(self, msg: Odometry):
        self.last_stamp = msg.header.stamp


    def link_states_cb(self, msg: LinkStates):
        if self.cart_link_name in msg.name:
            idx = msg.name.index(self.cart_link_name)
            pose = msg.pose[idx]
            twist = msg.twist[idx]


            if self.last_stamp is None:
                return  # wait until we have a timestamp
            
            # Use current simulation time from ROS clock
            stamp = self.get_clock().now().to_msg()

            # Publish Odometry
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose = pose
            odom.twist.twist = twist
            self.odom_pub.publish(odom)

            # Broadcast TF
            t = TransformStamped()
            t.header.stamp = self.last_stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            # Convert Point to Vector3
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z

            t.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CartOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
