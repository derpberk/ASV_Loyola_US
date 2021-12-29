from setuptools import setup
import os
from glob import glob

package_name = 'asv_loyola_us'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
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
            'dronekit = asv_loyola_us.dronekit_node:main',
        ],
    },
)
