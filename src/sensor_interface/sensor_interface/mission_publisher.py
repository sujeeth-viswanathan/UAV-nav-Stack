# mission_publisher.py

import rclpy
from rclpy.node import Node
from custom_msgs.msg import MissionGoals
from geometry_msgs.msg import Point32, Polygon, Point

class MissionPublisher(Node):
    def __init__(self):
        super().__init__('mission_publisher')
        self.publisher = self.create_publisher(MissionGoals, '/uav/mission/goals', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MissionGoals()

    # Fill waypoint
        msg.current_waypoint.x = 100.0
        msg.current_waypoint.y = 50.0
        msg.current_waypoint.z = 20.0

    # Fill deadline
        msg.deadline.sec = 3600  # 1 hour
        msg.deadline.nanosec = 0

    # Payload
        msg.payload_mass = 3.0  # kg
        msg.payload_type = 'medical_supplies'

    # Initialize geofences list
        msg.geofences = []

    # Create a geofence polygon
        geofence_polygon = Polygon()

    # Construct Point32s properly
        p1 = Point32()
        p1.x, p1.y, p1.z = 0.0, 0.0, 0.0

        p2 = Point32()
        p2.x, p2.y, p2.z = 100.0, 0.0, 0.0

        p3 = Point32()
        p3.x, p3.y, p3.z = 100.0, 100.0, 0.0

        p4 = Point32()
        p4.x, p4.y, p4.z = 0.0, 100.0, 0.0

    # Add manually constructed points
        geofence_polygon.points.append(p1)
        geofence_polygon.points.append(p2)
        geofence_polygon.points.append(p3)
        geofence_polygon.points.append(p4)

    # Append the polygon
        msg.geofences.append(geofence_polygon)

    # Publish
        self.publisher.publish(msg)
        self.get_logger().info('Published mission goals.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

