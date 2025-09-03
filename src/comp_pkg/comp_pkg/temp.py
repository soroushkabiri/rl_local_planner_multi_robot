import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import math

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (rotation around Z)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class CostmapOriginChecker(Node):
    def __init__(self):
        super().__init__('costmap_origin_checker')
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.callback, 10)

    def callback(self, msg):
        origin = msg.info.origin.position
        orientation = msg.info.origin.orientation
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height

        yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

        self.get_logger().info(f"Origin position: x={origin.x:.2f}, y={origin.y:.2f}")
        self.get_logger().info(f"Origin orientation yaw: {math.degrees(yaw):.2f} deg")
        self.get_logger().info(f"Map resolution: {resolution:.3f} m/cell")
        self.get_logger().info(f"Map size: width={width}, height={height}")

        # World coordinates of corners
        top_left_x = origin.x
        top_left_y = origin.y
        bottom_right_x = origin.x + width * resolution
        bottom_right_y = origin.y + height * resolution
        self.get_logger().info(f"Top-left corner: ({top_left_x:.2f}, {top_left_y:.2f})")
        self.get_logger().info(f"Bottom-right corner: ({bottom_right_x:.2f}, {bottom_right_y:.2f})")

def main():
    rclpy.init()
    node = CostmapOriginChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
