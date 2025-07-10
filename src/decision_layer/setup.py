from setuptools import find_packages, setup

package_name = 'decision_layer'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dynamic_flight_supervisor = decision_layer.dynamic_flight_supervisor:main',
                'gradient_planner_node = decision_layer.gradient_planner_node:main',

        ],
    },
)
