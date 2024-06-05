from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from datetime import datetime

def launch_setup(context, *args, **kwargs):
    create_bag = LaunchConfiguration('create_bag')
    today_str=datetime.now().strftime('%H:%M:%S')
    print(f"started rosbag at {today_str} ")
    rosbag_node=Node(
            package='rosbag',
            executable='play',
            arguments= f"--loop ~/.ros/bags/{today_str}.db3",
            output='screen'
        ),
    set_createbag_arg = SetParameter(name="create_bag", value=create_bag)
    return [
        set_createbag_arg,
        rosbag_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('create_bag',  default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])