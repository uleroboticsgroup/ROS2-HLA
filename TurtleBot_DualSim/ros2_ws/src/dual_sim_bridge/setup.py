from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_sim_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'fom'),
            glob('fom/*.xml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Vicente Barreiro',
    maintainer_email='vbars@unileon.es',
    description='Bridge HLA para sincronizar TurtleBot entre Gazebo y Unity',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = dual_sim_bridge.bridge_node:main',
        ],
    },
)
