#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, FluidPressure
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from custom_msgs.msg import SensorPacket  # Custom message aggregating all sensor data
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SensorInterfaceNode(Node):
    """
    ROS 2 Humble node that aggregates multiple sensor streams into a single SensorPacket.
    Subscriptions: IMU, GPS, Barometer, Wind, Rain.
    Publishes: /uav/sensors/raw at 50 Hz.
    """
    def __init__(self):
        # Initialize the node with name 'sensor_interface'
        super().__init__('sensor_interface')

        # QoS profile for sensor data: best-effort delivery, small buffer
        qos_profile = rclpy.qos.QoSProfile(depth=10)

        # Initialize storage for the latest sensor messages
        self.imu_data = None
        self.gps_data = None
        self.baro_data = None
        self.wind_vector = None
        self.rain_rate = None
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create subscriptions for each sensor topic
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, qos_profile)
        self.create_subscription(FluidPressure, '/barometer', self.baro_callback, qos_profile)
        self.create_subscription(Vector3Stamped, '/weather/wind', self.wind_callback, qos_profile)
        self.create_subscription(Float32MultiArray, '/weather/rain_rate', self.rain_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.latest_odom = None
        # Publisher for the combined sensor packet message
        self.publisher = self.create_publisher(SensorPacket, '/uav/sensors/raw', qos_profile)

        # Timer to call publish_packet() at 50 Hz
        timer_period = 1.0 / 50.0  # seconds
        self.create_timer(timer_period, self.publish_packet)
        self.get_logger().info("Sensor Interface Node is running and aggregating data...")
        
    def imu_callback(self, msg: Imu):
        # Store the latest IMU message (orientation, angular rates, linear acc.)
        self.imu_data = msg

    def gps_callback(self, msg: NavSatFix):
        # Store the latest GPS fix (latitude, longitude, altitude)
        self.gps_data = msg

    def baro_callback(self, msg: FluidPressure):
        # Store the latest barometer reading (pressure -> altitude conversion externally)
        self.baro_data = msg

    def wind_callback(self, msg: Vector3Stamped):
        # Extract 3D wind vector (m/s) from the message
        self.wind_vector = msg.vector

    def rain_callback(self, msg: Float32MultiArray):
        # Store scalar rain rate (mm/s)
        self.rain_rate = msg.data[0]
          
    # Add this method to your class
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg.twist.twist

    def publish_packet(self):
        # Only publish when all sensor data streams have produced at least one message
        if None in (self.imu_data, self.gps_data, self.baro_data, self.wind_vector, self.rain_rate, self.latest_odom):
            return  # skip until fully initialized

        # Create and populate the SensorPacket
        packet = SensorPacket()
        packet.header.stamp = self.get_clock().now().to_msg()

        # --- UAV State ---
        # Map GPS and barometer into a position vector
        packet.position.x = self.gps_data.latitude
        packet.position.y = self.gps_data.longitude
        # Using raw fluid pressure as a placeholder for altitude
        packet.position.z = self.gps_data.altitude

        # Orientation quaternion from IMU data
        packet.attitude = self.imu_data.orientation

        # Velocity placeholder: populate if velocity topic is available
        if self.latest_odom:
            packet.velocity.vector.x = self.latest_odom.linear.x
            packet.velocity.vector.y = self.latest_odom.linear.y
            packet.velocity.vector.z = self.latest_odom.linear.z
        else:
            packet.velocity.vector.x = 0.0
            packet.velocity.vector.y = 0.0
            packet.velocity.vector.z = 0.0

        # --- Environmental Data ---
        packet.wind = self.wind_vector
        packet.rain_rate = self.rain_rate

        sensor_interface_node.py        # Publish the aggregated sensor packet
        self.publisher.publish(packet)
        
        # Broadcast transform for TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"             # Global frame
        t.child_frame_id = "base_link"        # UAV's local frame

        t.transform.translation.x = packet.position.x
        t.transform.translation.y = packet.position.y
        t.transform.translation.z = packet.position.z

# Reuse the IMU quaternion for orientation
        t.transform.rotation = packet.attitude

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    # Initialize ROS 2 and spin the node until shutdown
    rclpy.init(args=args)
    node = SensorInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
