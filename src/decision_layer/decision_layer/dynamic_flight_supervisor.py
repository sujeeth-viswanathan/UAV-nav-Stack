import rclpy
from rclpy.node import Node
from custom_msgs.msg import SensorPacket
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
import numpy as np

class DynamicFlightSupervisor(Node):
    def __init__(self):
        super().__init__('dynamic_flight_supervisor')

        # Publishers
        self.waypoint_pub = self.create_publisher(Point, '/uav/next_waypoint', 10)
        self.status_pub = self.create_publisher(String, '/uav/status', 10)

        # Subscribers
        self.battery_sub = self.create_subscription(Float32MultiArray,'/uav/battery', self.battery_callback, 10)
        self.position_sub = self.create_subscription(SensorPacket, '/uav/sensors/raw', self.position_callback, 10)
        self.environment_sub = self.create_subscription(Float32MultiArray,'/uav/environment_risk', self.environment_callback ,10)
        # === New: Safe Zone Map Subscription ===
        self.safezone_sub = self.create_subscription(Float32MultiArray, '/uav/safe_zone_map', self.safezone_callback, 10)
        # Internal State
        self.current_position = None
        self.battery_level = 100.0
        self.environment_risk = 0.0
        self.safe_zone_map = None
        self.home_position = None
        self.mission_goal = None
        self.mission_priority = 'normal'
        self.priority_pub = self.create_publisher(String, '/uav/mission_priority', 10)
        self.low_battery_triggered = False
        self.risk_triggered = False

        self.timer = self.create_timer(1.0, self.decision_loop)

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def position_callback(self, msg):
        self.current_position = msg.position

        if self.home_position is None:
            self.home_position = Point(
                x=self.current_position.x,
                y=self.current_position.y,
                z=5.0
            )
            self.get_logger().info(f'[Supervisor] Home position set.')

    def environment_callback(self, msg):
        self.environment_risk = msg.data
        self.get_logger().info(f'[Supervisor] Risk received: {self.environment_risk:.2f}')

    # Trigger adaptive behavior based on mission context
        if self.environment_risk > 0.6:
            if self.mission_priority == "high":
                self.get_logger().warn('[Supervisor] High risk detected, but mission is HIGH PRIORITY. Proceeding...')
            else:
                self.get_logger().warn('[Supervisor] Environmental risk high. Preparing to reroute...')
                self.risk_triggered = True

    def safezone_callback(self, msg):
        # Now properly decode the 1D flattened array
        size = int(np.sqrt(len(msg.data)))  # Assuming square safezone map
        self.safe_zone_map = np.array(msg.data).reshape((size, size))

    def decision_loop(self):
        if self.current_position is None:
            self.get_logger().info('[Supervisor] Waiting for position...')
            return
        # === Publish mission priority every cycle ===
        priority_msg = String()
        priority_msg.data = self.mission_priority
        self.priority_pub.publish(priority_msg)
         # If already landed due to emergency, do not continue flying
        if self.low_battery_triggered or self.risk_triggered:
            self.get_logger().info('[Supervisor] Emergency landing completed. Awaiting manual reset.')
            return

        # Check waypoint timeout logic
        if hasattr(self, 'current_waypoint_start_time') and self.mission_goal is not None:
            elapsed = (self.get_clock().now() - self.current_waypoint_start_time).nanoseconds / 1e9  # seconds
            if elapsed > 120.0:  # 2 minutes timeout
                self.get_logger().warn('[Supervisor] Failed to reach waypoint in time! Attempting replan...')
              # Attempt to replan or fallback to home
                if hasattr(self, 'replan_attempted') and self.replan_attempted:
                    self.get_logger().error('[Supervisor] Replan already attempted. Returning Home.')
                    self.publish_waypoint(self.home_position)
                    self.low_battery_triggered = True  # Force landing flow
                else:
                    safe_spot = self.find_nearest_safe_zone()
                    self.publish_waypoint(safe_spot)
                    self.replan_attempted = True
                return

        if self.battery_level <= 20.0 and not self.low_battery_triggered:
            self.get_logger().warn('[Supervisor] Battery Low! Returning Home...')
            self.publish_waypoint(self.home_position)
            self.low_battery_triggered = True
            return

        if self.environment_risk >= 0.7 and not self.risk_triggered:
            if self.mission_priority == "high":
                self.get_logger().info('[Supervisor] Risk high, but priority is HIGH — not rerouting yet.')
            else:
                self.get_logger().warn('[Supervisor] Environmental risk high! Finding safe zone...')
                safe_spot = self.find_nearest_safe_zone()
                self.publish_waypoint(safe_spot)
                self.risk_triggered = True
                return

        if self.mission_goal is not None and not self.low_battery_triggered and not self.risk_triggered:
            self.publish_waypoint(self.mission_goal)

    def publish_waypoint(self, point):
        self.waypoint_pub.publish(point)
        self.get_logger().info(f'[Supervisor] Publishing Target: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})')

    def set_mission_goal(self, goal_point):
        self.mission_goal = goal_point
        self.get_logger().info('[Supervisor] New mission goal set.')

    def find_nearest_safe_zone(self):
        if self.safe_zone_map is None:
            self.get_logger().warn('[Supervisor] Safe zone map missing! Hovering...')
            return self.home_position  # fallback

        gx, gy = self.safe_zone_map.shape

        # Current UAV grid index
        i = int(np.clip(self.current_position.x, 0, gx-1))
        j = int(np.clip(self.current_position.y, 0, gy-1))

        # Search nearest 1.0 cell (safe zone)
        safe_points = np.argwhere(self.safe_zone_map == 1.0)
        if safe_points.size == 0:
            self.get_logger().error('[Supervisor] No safe zones detected!')
            return self.home_position

        distances = np.linalg.norm(safe_points - np.array([i, j]), axis=1)
        nearest_idx = np.argmin(distances)
        safe_cell = safe_points[nearest_idx]

        # Map back to world coordinates (assuming 1:1 grid for simplicity)
        safe_point = Point(
            x=float(safe_cell[0]),
            y=float(safe_cell[1]),
            z=5.0
        )

        self.get_logger().info(f'[Supervisor] Safe zone selected at ({safe_point.x:.2f}, {safe_point.y:.2f})')

        return safe_point

def main(args=None):
    rclpy.init(args=args)
    node = DynamicFlightSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
