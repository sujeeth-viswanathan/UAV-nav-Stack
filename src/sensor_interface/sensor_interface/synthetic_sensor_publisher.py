import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, Vector3, Polygon, Point32
from custom_msgs.msg import MissionGoals, SensorPacket
import random
import time

class SyntheticSensorPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_sensor_publisher')
        self.sensor_pub = self.create_publisher(SensorPacket, '/uav/sensors/raw', 10)
        self.publisher_ = self.create_publisher(MissionGoals, '/uav/mission/goals', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)
        self.get_logger().info("Synthetic sensor publisher started")

    def publish_sensor_data(self):
        sensor_msg = SensorPacket()
        msg = MissionGoals()
        sensor_msg.position = Point(x=random.uniform(0, 10), y=random.uniform(0, 10), z=2.0)
        sensor_msg.attitude = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        sensor_msg.velocity = Vector3(x=random.uniform(-5, 5), y=random.uniform(-5, 5), z=random.uniform(-1, 1))
        sensor_msg.wind = Vector3(x=random.uniform(0, 5), y=random.uniform(0, 5), z=0.0)
        sensor_msg.rain_rate = random.uniform(0.0, 10.0)
         
        msg.current_waypoint = Point(x=5.0, y=5.0, z=2.0)
        msg.deadline = self.get_clock().now().to_msg()
        msg.payload_mass = 1.5
        msg.payload_type = "medical_supplies"

        geofence = Polygon()
        geofence.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=10.0, y=0.0, z=0.0),
            Point32(x=10.0, y=10.0, z=0.0),
            Point32(x=0.0, y=10.0, z=0.0)
        ]
        msg.geofences = [geofence]

        self.sensor_pub.publish(sensor_msg)
        self.publisher_.publish(msg)
        self.get_logger().info("Published synthetic SensorPacket and MIssionGoals")

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
