from setuptools import setup
import os
from glob import glob

package_name = 'uav_description'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

     ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujeeth',
    maintainer_email='sujeeth@todo.todo',
    description='UAV robot description package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
