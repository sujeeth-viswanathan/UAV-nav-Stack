import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class SafeZoneEstimator:
    def __init__(self, max_risk=1.0):
        self.max_risk = max_risk

    def estimate_safe_zones(self, fused_risk):
        print("Estimating soft safe zones based on environment risks...")
        safe_zone_map = np.clip((1.0 - fused_risk) ** 2, 0.0, 1.0)
        return safe_zone_map

    def has_viable_safe_zone(self, safe_zone_map, threshold=0.5):
        return np.any(safe_zone_map > threshold)

class SafeZoneEstimatorNode(Node):
    def __init__(self):
        super().__init__('safe_zone_estimator')

        self.estimator = SafeZoneEstimator()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/risk_map/fused',
            self.risk_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/safe_zone/soft_map',
            10
        )

    def risk_callback(self, msg):
        fused_risk = np.array(msg.data).reshape((10, 10))  # assuming 10x10
        safe_zone_map = self.estimator.estimate_safe_zones(fused_risk)

        out_msg = Float32MultiArray()
        out_msg.data = safe_zone_map.flatten().tolist()
        self.publisher.publish(out_msg)

        if self.estimator.has_viable_safe_zone(safe_zone_map):
            self.get_logger().info("✅ Viable safe zone detected.")
        else:
            self.get_logger().warn("⚠️ No safe zones available!")

def main(args=None):
    rclpy.init(args=args)
    node = SafeZoneEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
