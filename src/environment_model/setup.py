from setuptools import find_packages, setup

package_name = 'environment_model'

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
   
                          'risk_fusion_node = environment_model.risk_fusion_node:main',
                          'safe_zone_estimator = environment_model.safe_zone_estimator:main',
                          'cost_field_generator = environment_model.cost_field_generator:main',
                          'safe_zone_publisher = environment_model.safe_zone_publisher:main',
        ],
    },
)
