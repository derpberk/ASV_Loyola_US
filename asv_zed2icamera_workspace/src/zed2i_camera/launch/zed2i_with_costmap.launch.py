from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter
from launch.actions import DeclareLaunchArgument,ExecuteProcess, OpaqueFunction, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from datetime import datetime

def launch_setup(context, *args, **kwargs):
    create_bag = LaunchConfiguration('create_bag')
    today_str=datetime.now().strftime('%H:%M:%S')
    result=context.perform_substitution(create_bag)in "True"
    print(f"started rosbag at {today_str} with result{context.perform_substitution(create_bag)} ")
    rosbag_node=LaunchDescription([
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a'],
                    output='screen'
                )
    ])
    set_createbag_arg = SetParameter(name="create_bag", value=create_bag)
    if result:
        return [
            set_createbag_arg,
            rosbag_node,
        ]
    else:
        return[
            set_createbag_arg,
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('create_bag',  default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])