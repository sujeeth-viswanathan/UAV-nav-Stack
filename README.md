# 🌦️ Robust UAV Navigation Framework for Adverse Weather Conditions

This repository contains the ROS 2–based implementation of a modular, deterministic UAV navigation framework designed to maintain safe, energy-efficient, and mission-critical drone deliveries under challenging environmental conditions like wind, fog, and rain.

## 📌 Project Overview

The system is designed around a **layered ROS 2 architecture** and includes the following capabilities:
- Real-time sensor fusion from simulated GPS, IMU, and environmental sensors
- Environmental risk modeling using fused weather data
- Cost field generation combining energy cost and weather risk
- Adaptive gradient-based path planning with wind compensation
- Rule-based mission decision logic (return-to-home, rerouting, emergency landing)
- Smoothed trajectory execution with velocity control and wind drift mitigation

## 🏗️ System Architecture

The system is divided into four modular layers:
1. **Sensor Interface** – Fusion of positional, environmental, and mission data
2. **Environment Modeling** – Risk mapping and cost field generation
3. **Decision Layer** – Goal selection, safe zone detection, rerouting logic
4. **Control Layer** – Motion planning and actuator commands

All nodes communicate over ROS 2 topics using custom message types for high interoperability.

## 🧪 Simulation and Testing

Although full end-to-end Gazebo simulations are pending, the entire framework is:
- Implemented using ROS 2 Humble (Python & `rclpy`)
- Tested at the module level with synthetic weather inputs
- Structured for seamless launch via `full_system_launch.py`
- Ready for scenario-based evaluation in Gazebo (with scripted weather dynamics)

## 📦 Key ROS 2 Packages

| Package Name         | Description |
|----------------------|-------------|
| `sensor_interface`   | Fuses GPS, IMU, wind, and rain data |
| `environment_model`  | Computes risk maps and energy-based cost fields |
| `decision_layer`     | Applies mission logic and adaptive path planning |
| `control_layer`      | Executes smoothed trajectory with wind compensation |
| `custom_msgs`        | Defines `SensorPacket` and `MissionGoal` messages |

## ⚙️ Launch Instructions

```bash
source install/setup.bash
ros2 launch full_system_launch.py

Author
Sujeeth Viswanthan
Capstone Project – EG 6011
