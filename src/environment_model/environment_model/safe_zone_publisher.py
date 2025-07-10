#safezone publisher

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class SafeZonePublisher(Node):
    def __init__(self):
        super().__init__('safe_zone_publisher')

        self.publisher = self.create_publisher(Float32MultiArray, '/uav/safe_zone_map', 10)

        # Timer to publish periodically
        self.timer = self.create_timer(5.0, self.publish_safe_zones)  # Publish every 5 seconds

        self.get_logger().info('SafeZonePublisher started.')

    def publish_safe_zones(self):
        # For now, generate synthetic safe zone map (real one would call estimator)
        gx, gy = 50, 50
        safe_zone_map = np.random.choice([0.0, 1.0], size=(gx, gy), p=[0.7, 0.3])

        msg = Float32MultiArray()
        msg.data = safe_zone_map.flatten().tolist()

        self.publisher.publish(msg)
        self.get_logger().info('Published safe zone map.')

def main(args=None):
    rclpy.init(args=args)
    node = SafeZonePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
