from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Use substitution to construct URDF path dynamically
    urdf_file = PathJoinSubstitution([
        FindPackageShare('uav_description'),
        'urdf',
        'uav.urdf.xacro'
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare('uav_description'),
        'worlds',
        'uav_navigation.world'
    ])
    print("[DEBUG] Xacro command:", ['xacro', urdf_file])
    return LaunchDescription([
        # Run robot_state_publisher with URDF from xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                  TextSubstitution(text='xacro '),
                   urdf_file
               ])
            }]
        ),

        # Start Gazebo with a custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the UAV into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'uav',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '1'
            ],
            output='screen'
        )
    ])

