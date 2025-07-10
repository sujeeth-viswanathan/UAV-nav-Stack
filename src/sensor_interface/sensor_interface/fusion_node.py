import rclpy
from rclpy.node import Node
from custom_msgs.msg import SensorPacket, MissionGoals
import numpy as np

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Subscribers
        self.sensor_sub = self.create_subscription(
            SensorPacket,
            '/uav/sensors/raw',
            self.sensor_callback,
            10
        )

        self.mission_sub = self.create_subscription(
            MissionGoals,
            '/uav/mission/goals',
            self.mission_callback,
            10
        )

        # Internal storage
        self.latest_sensor = None
        self.latest_mission = None

        # Timer to fuse data every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def sensor_callback(self, msg):
        self.latest_sensor = msg

    def mission_callback(self, msg):
        self.latest_mission = msg

    def timer_callback(self):
        if self.latest_sensor is None or self.latest_mission is None:
            self.get_logger().info('Waiting for both sensor and mission data...')
            return

        # Build input vector
        input_vector = self.build_input_vector(self.latest_sensor, self.latest_mission)

        self.get_logger().info(f'Input Vector: {input_vector}')

    def build_input_vector(self, sensor_msg, mission_msg):
        # Example vector construction
        vector = np.array([
            sensor_msg.position.x,
            sensor_msg.position.y,
            sensor_msg.position.z,
            sensor_msg.velocity.x,
            sensor_msg.velocity.y,
            sensor_msg.velocity.z,
            sensor_msg.wind.x,
            sensor_msg.wind.y,
            sensor_msg.rain_rate,
            mission_msg.current_waypoint.x,
            mission_msg.current_waypoint.y,
            mission_msg.current_waypoint.z,
            mission_msg.deadline.sec,
            mission_msg.payload_mass,
        ], dtype=np.float32)

        return vector

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
