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
            get_package_share_directory('simulator'), 'launch'),
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
                )]
            )
        ),
    ])

"""
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
        
        
        
    RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[
                LogInfo(msg='Turtlesim started, spawning turtle'),
                spawn_turtle
            ]
        )
    ),
    RegisterEventHandler(
        OnProcessIO(
            target_action=spawn_turtle,
            on_stdout=lambda event: LogInfo(
                msg='Spawn request says "{}"'.format(
                    event.text.decode().strip())
            )
        )
    ),
    RegisterEventHandler(
        OnExecutionComplete(
            target_action=spawn_turtle,
            on_completion=[
                LogInfo(msg='Spawn finished'),
                change_background_r,
                TimerAction(
                    period=2.0,
                    actions=[change_background_r_conditioned],
                )
            ]
        )
    ),
    RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_node,
            on_exit=[
                LogInfo(msg=(EnvironmentVariable(name='USER'),
                        ' closed the turtlesim window')),
                EmitEvent(event=Shutdown(
                    reason='Window closed'))
            ]
        )
    ),
"""