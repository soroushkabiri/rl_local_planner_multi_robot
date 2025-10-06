# mag_calib_wifi_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import requests
import pandas as pd
import os

class MAGCALIBWIFINode(Node):
    def __init__(self):
        super().__init__('mag_calib_wifi_node')
        self.esp32_url = 'http://192.168.4.1/magnetometer'  # New URL endpoint
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.data = []
        self.output_file = os.path.join(os.path.expanduser("~"), "Desktop/magnetometer_data.csv")
        self.get_logger().info(f"Magnetometer data will be saved to: {self.output_file}")
        self.create_timer(0.1, self.read_data)  # 10Hz polling rate

    def read_data(self):
        try:
            response = requests.get(self.esp32_url, timeout=1.0)
            if response.status_code == 200:
                data = response.json()
                mx = data['mx']
                my = data['my']
                mz = data['mz']

                self.data.append({'mx': mx, 'my': my, 'mz': mz})

                mag_msg = MagneticField()
                mag_msg.magnetic_field.x = mx
                mag_msg.magnetic_field.y = my
                mag_msg.magnetic_field.z = mz
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                mag_msg.header.frame_id = "mag_link"
                self.pub_mag.publish(mag_msg)
            else:
                self.get_logger().warn(f"ESP32 returned status code: {response.status_code}")
        except Exception as e:
            self.get_logger().warn(f"Failed to get data from ESP32: {e}")

    def destroy_node(self):
        if self.data:
            df = pd.DataFrame(self.data)
            df.to_csv(self.output_file, index=False)
            self.get_logger().info(f"Saved {len(self.data)} samples to {self.output_file}")
        super().destroy_node()

def main():
    rclpy.init()
    node = MAGCALIBWIFINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
