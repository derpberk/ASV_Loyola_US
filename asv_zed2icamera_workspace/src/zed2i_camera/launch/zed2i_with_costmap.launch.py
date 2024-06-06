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
    #as launch argument whether to create a bag
    create_bag = LaunchConfiguration('create_bag')
    set_createbag_arg = SetParameter(name="create_bag", value=create_bag)


    bag_path=os.path.join(os.path.expanduser("~"),"save_bags") #path to save bag file
    
    rosbag_node=LaunchDescription([ #ros2 bag launch
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a'],
                    output='screen',
                    cwd=bag_path,
                    condition=IfCondition(create_bag),
                )
    ])

    default_config_common = os.path.join( #path of common yaml file for camera node but with gnss enabled
        get_package_share_directory('zed2i_camera'),'config','gnss_with_fusion.yaml')

    zed2_launch=GroupAction( #launch of camera wrapper with configuration
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),'/zed_camera.launch.py']),
            launch_arguments = {
                'camera_model' : "zed2i",
                'config_path'  : default_config_common
             }.items(),
            )
        ]
    )


    slam_launch=GroupAction( #launch of camera wrapper with configuration
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch'),'/online_async_launch.py']),
            launch_arguments = {
                'slam_params_file' : "",
             }.items(),
            )
        ]
    )


    nav2_with_costmap=GroupAction( #launch of camera wrapper with configuration
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch'),'/navigation_launch.launch.py']),
            launch_arguments = {
                'params_file' : "",
             }.items(),
            )
        ]
    )

    return [
        set_createbag_arg,
        rosbag_node,
        # zed2_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('create_bag',  default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])




    
#get todays date, unused
# today_str=datetime.now().strftime('%H:%M:%S')
# result=context.perform_substitution(create_bag)in "True" #boolean variable to create or not a bag file
# print(f"started rosbag at {today_str} with result{context.perform_substitution(create_bag)} ") #this is just debug, erasable

