from setuptools import setup
import os
from glob import glob

package_name = 'python_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aloepacci',
    maintainer_email='aloepacci@gmail.com',
    description='package to learn how to use Python programs in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['talker = python_example.publisher_member_function:main',
                            'listener = python_example.subscriber_member_function:main',
                            'server = python_example.Add2Ints_server:main',
                            'client = python_example.Add2Ints_client:main',
                            'param_talker = python_example.parameters:main',
        ],
    },
)
