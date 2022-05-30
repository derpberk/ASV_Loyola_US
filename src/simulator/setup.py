from setuptools import setup
import os
from glob import glob

package_name = 'simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*.yaml')),
        (os.path.join('share', package_name, "config"), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    maintainer='alejandro',
    maintainer_email='alexcpesp@gmail.com',
    description='visualization of ardupilot SITL via Rviz',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = simulator.test:main',
            'dummy_publisher = simulator.dummy_publisher:main',
            'radar_ploter = simulator.radar_ploter:main',
        ],
    },
)
