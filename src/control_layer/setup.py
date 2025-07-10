from setuptools import find_packages, setup

package_name = 'control_layer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujeeth',
    maintainer_email='sujeeth@todo.todo',
    description='control layer for UAV motion',
    license='MIT',
    license_files=['LICENSE'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
                 'motion_controller_node = control_layer.motion_controller_node:main',
        ],
    },
)
