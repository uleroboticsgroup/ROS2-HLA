from setuptools import setup
import os
from glob import glob

package_name = 'bridge_hla2ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={package_name: 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'generated'), glob('generated/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vicen',
    maintainer_email='vicen@todo.todo',
    description='ROS 2 Bridge for HLA',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'universal_bridge = bridge_hla2ros2.universal_bridge:main',
        ],
    },
)
