from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ROS2HLA_Bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('examples/turtlesim/launch/*.launch.py')),
        (os.path.join('share', package_name, 'examples/turtlesim/config'), glob('examples/turtlesim/config/*.yaml')),
        (os.path.join('share', package_name, 'examples/turtlesim/fom'), glob('examples/turtlesim/fom/*.xml')),
        ('lib/' + package_name, ['examples/turtlesim/pose_listener.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vicente Barreiro',
    maintainer_email='vbars@unileon.es',
    description='ROS2 to HLA Bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ROS2HLA_Bridge.bridge_node:main',
        ],
    },
)
