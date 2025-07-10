from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    uav_description_pkg = get_package_share_directory('uav_description')
    sensor_interface_pkg = get_package_share_directory('sensor_interface')

    # Launch Gazebo + robot spawn
    spawn_uav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uav_description_pkg, 'launch', 'spawn_uav_launch.py')
        )
    )

    # Launch system nodes
    uav_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_interface_pkg, 'launch', 'uav_system_launch.py')
        )
    )

    return LaunchDescription([
        spawn_uav,
        uav_system
    ])
