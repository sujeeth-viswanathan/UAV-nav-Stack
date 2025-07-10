import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import SensorPacket  # For current position
from geometry_msgs.msg import Point  # Waypoint
from geometry_msgs.msg import Vector3
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.get_logger().info('Publishing velocity commands to: /cmd_vel')
        self.pose_sub = self.create_subscription(SensorPacket, '/uav/sensors/raw', self.pose_callback, 10)
        self.waypoint_sub = self.create_subscription(Point, '/uav/next_waypoint', self.waypoint_callback, 10)
        self.current_position = None
        self.wind_vector = Vector3()
        self.target_waypoint = None
        self.max_speed = 5.0  # m/s, your normal cruise speed
        self.min_speed = 0.5  # m/s, minimum safe speed near waypoint
        self.braking_distance = 10.0  # meters to start slowing down

        self.battery_level = 100.0  # percentage
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
           '/uav/battery',
            self.battery_callback,
            10
         )
        self.path_sub = self.create_subscription(
                           Path,
                               '/uav/smoothed_path',
                                self.path_callback,
                                10
                                )
        self.received_path = []
        self.path_index = 0
        self.kp = 0.5  # Proportional gain for velocity control
        self.low_battery_triggered = False  # Safety flag

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.home_position = None  # Dynamic home position
        self.returning_home = False
        self.landing = False
        self.landed = False
        self.status_pub = self.create_publisher(String, '/uav/status', 10)

    def path_callback(self, msg):
        self.received_path = msg.poses  # Store incoming smoothed waypoints
        self.path_index = 0  # Reset to start following from the beginning
        self.get_logger().info(f"[Controller] Received new path with {len(msg.poses)} waypoints.")

    def pose_callback(self, msg):
        self.current_position = msg.position
        self.wind_vector = msg.wind
        
        if self.home_position is None:
            self.home_position = Point(
                x=self.current_position.x,
                y=self.current_position.y,
        
         z=5.0  # Fixed safe hover height for return
        )
            self.get_logger().info(f'Home position set at: {self.home_position.x:.2f}, {self.home_position.y:.2f}, {self.home_position.z:.2f}')

    def waypoint_callback(self, msg):
        self.target_waypoint = msg
 
    def battery_callback(self, msg):
        battery_percentage = msg.data

        if battery_percentage < 30.0 and not self.low_battery_triggered:
            self.get_logger().warn(f'Battery LOW ({self.battery_level:.1f}%)')
            self.returning_home = True
            self.low_battery_triggered = True

    def timer_callback(self):
        if self.current_position is None :
            self.get_logger().info('Waiting for position and waypoint...')
            return

        if self.battery_level <= 5.0:
            self.get_logger().warn('Battery critically low. Hovering...')
            self.cmd_pub.publish(Twist())  # Zero velocity
            return
        if self.battery_level <= 20.0 and not self.returning_home:
            self.get_logger().warn('Battery low! Initiating Return-To-Home...')
            self.returning_home = True
        
        
        # Landing complete check
        if self.landing and self.current_position.z <= 0.6:
            if not self.landed:
                self.get_logger().warn('Landing Complete. Motors Stopping...')
                self.landed = True
                self.cmd_pub.publish(Twist())  # Send zero velocity

                # Publish Landed Status
                status_msg = String()
                # Decide what to publish
                if self.returning_home:
                    status_msg.data = "LANDED at HOME"
                elif self.target_waypoint is None:
                    status_msg.data = "LANDED - MISSION ABORTED"
                else:
                    status_msg.data = "LANDED"

                self.status_pub.publish(status_msg)
            return  # Do not continue commanding after landing
         # --- Target selection ---
        # === Path Following ===
        if not self.received_path or self.path_index >= len(self.received_path):
            self.get_logger().info("No more path points. Hovering...")
            self.cmd_pub.publish(Twist())
            return

        target_pose = self.received_path[self.path_index].pose
        target = target_pose.position
        
      
        # --- Vector to target ---
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        dz = target.z - self.current_position.z
        distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)

        cmd = Twist()
        # --- Predictive Braking Logic ---
        if distance_to_target > 0.0:
            if distance_to_target < self.braking_distance:
                desired_speed = (distance_to_target / self.braking_distance) * self.max_speed
                desired_speed = max(desired_speed, self.min_speed)
            else:
                desired_speed = self.max_speed

    # --- Build command ---
            cmd.linear.x = (dx / distance_to_target) * desired_speed
            cmd.linear.y = (dy / distance_to_target) * desired_speed
            cmd.linear.z = (dz / distance_to_target) * desired_speed
            
            # Wind Compensation
            compensate_gain = 0.5
            cmd.linear.x -= compensate_gain * self.wind_vector.x
            cmd.linear.y -= compensate_gain * self.wind_vector.y
            cmd.linear.z -= compensate_gain * self.wind_vector.z 
        # === Clamp velocities to reasonable limits ===
            cmd.linear.x = max(min(cmd.linear.x, self.max_speed), -self.max_speed)
            cmd.linear.y = max(min(cmd.linear.y, self.max_speed), -self.max_speed)
            cmd.linear.z = max(min(cmd.linear.z, self.max_speed), -self.max_speed)

            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Commanded Velocity: x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}, z={cmd.linear.z:.2f}')
        else:
        # At Target
            self.cmd_pub.publish(Twist())
            self.get_logger().info('At Target Position. Hovering...')
 

        energy_used = math.sqrt(cmd.linear.x**2 + cmd.linear.y**2 + cmd.linear.z**2) * 0.1
        self.battery_level -= energy_used
        self.battery_level = max(self.battery_level, 0.0)
        self.get_logger().info(f'Battery Level: {self.battery_level:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        
