from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('zed2i_camera'),
        'config',
        'config.yaml')

    return LaunchDescription([
        Node(
            package='zed2i_camera',
            namespace='zed2i_camera',
            executable='custom_object_detection',
            name='custom_object_detection_node',
            parameters = [config]
        )
    ])