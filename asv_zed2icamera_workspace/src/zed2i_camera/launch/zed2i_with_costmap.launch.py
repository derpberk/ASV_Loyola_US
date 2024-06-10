from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter
from launch.actions import TimerAction, DeclareLaunchArgument,ExecuteProcess, OpaqueFunction, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
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
                    cmd=['ros2', 'bag', 'record', '-a', '--compression-mode', 'file', '--compression-format', 'zstd'],
                    output='screen',
                    cwd=bag_path,
                    condition=IfCondition(create_bag),
                )
    ])

    default_config_common = os.path.join( #path of common yaml file for camera node but with gnss enabled
        get_package_share_directory('zed2i_camera'),'config','gnss_with_fusion.yaml')
    default_xacro_common = os.path.join( #path of common yaml file for camera node but with gnss enabled
        get_package_share_directory('zed2i_camera'),'urdf','ship_descr.urdf.xacro')

    zed2_launch=GroupAction( #launch of camera wrapper with configuration
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),'/zed_camera.launch.py']),
            launch_arguments = {
                'camera_model' : "zed2i",
                'config_path'  : default_config_common,
                'xacro_path' : default_xacro_common
             }.items(),
            )
        ]
    )


    default_config_common_slam = os.path.join( #path of common yaml file for camera node but with gnss enabled
        get_package_share_directory('zed2i_camera'),'config','slam.yaml')
    
    slam_launch=GroupAction( #launch of camera wrapper with configuration
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch'),'/online_async_launch.py']),
            launch_arguments = {
                'slam_params_file' : default_config_common_slam,
             }.items(),
            )
        ]
    )


    default_config_common_nav2 = os.path.join( #path of common yaml file for camera node but with gnss enabled
        get_package_share_directory('zed2i_camera'),'config','nav2.yaml')

    nav2_with_costmap=TimerAction(period=5.0,
        actions=[GroupAction( #launch of camera wrapper with configuration
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch'),'/navigation_launch.py']),
                launch_arguments = {
                    'params_file' : default_config_common_nav2,
                }.items(),
                )
            ]
        )]
    )


    pintcloud_to_laserscan= LaunchDescription([Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'zed/zed_node/point_cloud/cloud_registered'),
                        ('scan', 'scan')],
            parameters=[{
                'use_sim_time' : False,
                'target_frame': 'zed_left_camera_frame',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.0,  # -55º
                'angle_max': 1.0,  # 55º
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.2,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
            )
    ])

    return [
        set_createbag_arg,
        rosbag_node,
        pintcloud_to_laserscan,
        zed2_launch,
        # slam_launch,
        nav2_with_costmap,
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

