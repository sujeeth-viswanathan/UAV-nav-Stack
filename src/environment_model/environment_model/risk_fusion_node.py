#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
import numpy as np

from environment_model.risk_fusion import (
    WindInfluenceModel, RainInfluenceModel,
    FogInfluenceModel, LightningInfluenceModel,
    DynamicRiskFusionModule
)

class RiskFusionNode(Node):
    def __init__(self):
        super().__init__('risk_fusion_node')

        self.wind = None
        self.rain = None
        self.fog = None
        self.lightning = None
        
        self.create_subscription(Vector3Stamped, '/weather/wind', self.wind_callback, 10)
        self.create_subscription(Float32MultiArray,'/weather/rain_rate', self.rain_callback, 10)
        self.create_subscription(Float32MultiArray, '/weather/fog_density', self.fog_callback, 10)
        self.create_subscription(Float32MultiArray, '/weather/lightning_density', self.lightning_callback, 10)
        self.mission_priority = 'normal'  # default value
        self.create_subscription(String, '/uav/mission_priority', self.priority_callback, 10)

        self.risk_pub = self.create_publisher(Float32MultiArray,'/uav/environment_risk', 10)
        self.timer = self.create_timer(1.0, self.fuse_risks)
        self.fusion_module = DynamicRiskFusionModule()

    def wind_callback(self, msg):
        self.wind = np.array([msg.vector.x, msg.vector.y])

    def rain_callback(self, msg):
        self.rain = msg.data

    def fog_callback(self, msg):
        self.fog = msg.data

    def lightning_callback(self, msg):
        self.lightning = msg.data

    def fuse_risks(self):
        if None in (self.wind, self.rain, self.fog, self.lightning):
            self.get_logger().info("Waiting for all weather inputs...")
            return

        wind_field = np.zeros((10, 10, 2))
        wind_field[..., 0] = self.wind[0]
        wind_field[..., 1] = self.wind[1]

        rain_map = np.ones((10, 10)) * self.rain
        fog_map = np.ones((10, 10)) * self.fog
        lightning_map = np.ones((10, 10)) * self.lightning

        wind_risk = WindInfluenceModel(wind_field).compute_risk()
        rain_risk = RainInfluenceModel(rain_map).compute_risk()
        fog_risk = FogInfluenceModel(fog_map).compute_risk()
        lightning_risk = LightningInfluenceModel(lightning_map).compute_risk()

        risks = {
            'wind': wind_risk,
            'rain': rain_risk,
            'fog': fog_risk,
            'lightning': lightning_risk
        }

        uav_status = {
            'battery_level': 1.0,
            'mission_priority': 'self.mission_priority',
            'distance_to_goal': 100.0
        }

        fused_grid = self.fusion_module.fuse(risks, uav_status)

        avg_risk = float(np.mean(fused_grid))
        self.risk_pub.publish(Float32(data=avg_risk))
        self.get_logger().info(f'Published Environment Risk: {avg_risk:.2f}')

    def priority_callback(self, msg):
        self.mission_priority = msg.data
        self.get_logger().info(f'Received mission priority: {self.mission_priority}')

def main(args=None):
    rclpy.init(args=args)
    node = RiskFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
