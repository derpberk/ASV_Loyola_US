import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.substitutions import TextSubstitution, LaunchConfiguration, LocalSubstitution
from launch_ros.actions import Node, PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():

    namespace = DeclareLaunchArgument(
        "namespace", default_value=TextSubstitution(text="ASV")
    )
    node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('asv_loyola_us'), 'launch'),
            '/nodes.launch.py'])
    )

    launch_file = GroupAction(
        actions=[
            # push_ros_namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('namespace')),
            node_launch
        ]
    )



    return launch.LaunchDescription([
        namespace,
        launch_file,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                         LocalSubstitution('event.reason')]
                    ),
                ]
            )
        ),
    ])

