import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import time

# este launch carga la configuración de la carpeta {simulator}/config y lanza ardupilot SITL, MavROS y RVIZ


def generate_launch_description():
    # open ardupilot SITL in a new terminal
    os.system("gnome-terminal --command=\"sim_vehicle.py -v Rover --console --map\"")

    #load the configuration of mavros
    mavros_config = os.path.join(
        get_package_share_directory('simulator'),
        'config',
        'mavros_param.yaml'
    )

    # load the configuration of Rviz
    rviz_config = os.path.join(
        get_package_share_directory('simulator'),
        'config',
        'rviz_param.rviz'
    )

    #start mavros to create a translation between mavlink and ros
    time.sleep(2)
    Mavros =launch_ros.actions.Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        #name="mavros_node",
        parameters=[mavros_config]
        )

    #start Rviz as a visualizer of the robot
    Rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        #name="mavros_node",
        #parameters=[mavros_config],
        arguments=['-d'+rviz_config]
        )


    return launch.LaunchDescription([
        Mavros,
        #Rviz,
    ])
