from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime
def generate_launch_description():
    today_str=datetime.now().strftime('%H:%M:%S')

    return LaunchDescription([
        Node(
            package='rosbag',
            namespace='zed2i_camera',
            executable='play',
            arguments= f"--loop ~/.ros/bags/{today_str}.db3",
            output='screen'
        )
    ])