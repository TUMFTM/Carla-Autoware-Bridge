from glob import glob
import os

from setuptools import setup

package_name = 'carla_autoware_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/objects.json']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics010',
    maintainer_email='kirill.mitkovskii@gmail.com',
    description='Convert and transfer messages between carla_ros_bridge and autoware universe',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_autoware_bridge = carla_autoware_bridge.carla_autoware_bridge:main'
        ],
    },
)
