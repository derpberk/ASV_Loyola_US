from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition



def generate_launch_description():
    create_bag = LaunchConfiguration('create_bag')
    today_str=datetime.now().strftime('%H:%M:%S')
    create_bag_arg = DeclareLaunchArgument('create_bag',  default_value='True')
    bag_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('zed2i_camera'), 'launch'),
            '/rosbag.launch.py']),condition=IfCondition(create_bag))
    return LaunchDescription([
        create_bag,
        create_bag_arg,
        bag_launch,
    ])