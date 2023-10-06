from setuptools import setup
import os
from glob import glob

package_name = 'asv_loyola_us'
submodule = 'asv_loyola_us/submodulos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alexcpesp@gmail.com',
    description='Working with Navio2 helping the project ASV_Loyola_US merging it in ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone = asv_loyola_us.dronekit_node:main',
            'mqtt = asv_loyola_us.mqtt_node:main',
            'mission = asv_loyola_us.mission_node:main',
            'sensors = asv_loyola_us.sensor_module_node:main',
            'planner = asv_loyola_us.planner_node:main',
            'watchdog = asv_loyola_us.watchdog_node:main',
            'camera = asv_loyola_us.camera_node:main',
            'database= asv_loyola_us.database_node:main',
            'sonar = asv_loyola_us.sonar_module:main',
            'asv = asv_loyola_us.asv_node:main',
            'communications = asv_loyola_us.server_communication_node:main',
        ],
    },
)
