from setuptools import find_packages, setup

package_name = 'sensor_interface'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uav_system_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujeeth',
    maintainer_email='sujeeth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               'sensor_interface_node = sensor_interface.sensor_interface_node:main',
               'mission_publisher = sensor_interface.mission_publisher:main',
               'fusion_node = sensor_interface.fusion_node:main',
                'simple_tf_publisher = sensor_interface.simple_tf_publisher:main',
               'synthetic_sensor_publisher = sensor_interface.synthetic_sensor_publisher:main',
        ],
    },
)
