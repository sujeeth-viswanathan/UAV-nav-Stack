from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensor Interface
        Node(
            package='sensor_interface',
            executable='sensor_interface_node',
            name='sensor_interface',
            output='screen'
        ),
        Node(
            package='sensor_interface',
            executable='mission_publisher',
            name='mission_publisher',
            output='screen'
        ),
        Node(
            package='sensor_interface',
            executable='fusion_node',
            name='fusion_node',
            output='screen'
        ),

        # Environment Model
        Node(
            package='environment_model',
            executable='risk_fusion_node',
            name='risk_fusion_node',
            output='screen'
           
        ),
        Node(
            package='environment_model',
            executable='cost_field_generator',
            name='cost_field_generator',
            output='screen'
        ),
        Node(
            package='environment_model',
            executable='safe_zone_estimator',
            name='safe_zone_estimator',
            output='screen' 
        ),
        Node(
            package='environment_model',
            executable='safe_zone_publisher',
            name='safe_zone_publisher',
            output='screen'
        ),

        # Decision Layer
        Node(
            package='decision_layer',
            executable='dynamic_flight_supervisor',
            name='supervisor',
            output='screen'
        ),
        Node(
            package='decision_layer',
            executable='gradient_planner_node',
            name='planner',
            output='screen'
        ),

        # Control Layer
        Node(
            package='control_layer',
            executable='motion_controller_node',
            name='motion_controller',
            output='screen'
        ),
    ])
