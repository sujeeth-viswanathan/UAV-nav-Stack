import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class CostFieldGenerator:
    def __init__(self, fused_risk_map, wind_field=None, risk_weight=0.7, energy_weight=0.3):
        super().__init__('cost_field_generator')
        self.fused_risk_map = fused_risk_map
        self.wind_field = wind_field
        self.risk_weight = risk_weight
        self.energy_weight = energy_weight

    def compute_energy_cost(self):
        if self.wind_field is None:
            return np.ones(self.fused_risk_map.shape)
        wind_magnitude = np.linalg.norm(self.wind_field, axis=-1)
        energy_cost = 1.0 + 0.5 * wind_magnitude
        return energy_cost

    def generate_cost_field(self):
        energy_cost = self.compute_energy_cost()
        energy_cost = (energy_cost - np.min(energy_cost)) / (np.ptp(energy_cost) + 1e-6)
        total_cost = self.risk_weight * self.fused_risk_map + self.energy_weight * energy_cost
        return np.clip(total_cost, 0.0, 1.0)

class CostFieldGeneratorNode(Node):

    def __init__(self):
        super().__init__('cost_field_generator_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/risk_map/fused',
            self.risk_callback,
            10
        )
        self.wind_subscription = self.create_subscription(
            Float32MultiArray,
            '/weather/wind_field',
            self.wind_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/planning/cost_field',
            10
        )

        self.wind_field = None

    def wind_callback(self, msg):
        # Expecting a 10x10x2 flattened vector
        wind_array = np.array(msg.data).reshape((10, 10, 2))
        self.wind_field = wind_array

    def risk_callback(self, msg):
        fused_risk_map = np.array(msg.data).reshape((10, 10))

        generator = CostFieldGenerator(fused_risk_map, self.wind_field)
        cost_field = generator.generate_cost_field()

        out_msg = Float32MultiArray()
        out_msg.data = cost_field.flatten().tolist()
        self.publisher.publish(out_msg)

        self.get_logger().info("Published cost field.")

def main(args=None):
    rclpy.init(args=args)
    node = CostFieldGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
