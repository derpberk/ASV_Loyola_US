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
    maintainer='Samuel Yanes',
    maintainer_email='syanes@us.es',
    description='Autonomous Surface Vehicles for Environmental Monitoring - US/Loyola',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wqp_sensor = asv_loyola_us.wqp_sensor_node:main',
            'sonar = asv_loyola_us.sonar_node:main',
            'asv = asv_loyola_us.asv_node:main',
            'communications = asv_loyola_us.server_communication_node:main',
            'path_planner = asv_loyola_us.path_planner_node:main',
        ],
    },
)
