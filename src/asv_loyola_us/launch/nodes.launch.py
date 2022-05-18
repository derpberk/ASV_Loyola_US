import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import time

# este launch carga la configuración de la carpeta {simulator}/config y lanza ardupilot SITL, MavROS y RVIZ


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('asv_loyola_us'),
        'config',
        'config.yaml')

    comunication = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='drone',
        name='drone_node',
        parameters = [config]
    )

    planner = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='planner',
        name='planner_node',
        parameters = [config]
    )

    watchdog = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='watchdog',
        name='watchdog_node',
        parameters = [config]
    )

    mission = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='mission',
        name='mission_node',
        parameters = [config]
    )

    mqtt = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='mqtt',
        name='mqtt_node',
        parameters = [config]
    )

    sensors = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='sensors',
        name='sensors_node',
        parameters = [config]
    )

    camera = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='camera',
        name='camera_node',
        parameters = [config]
    )



    return launch.LaunchDescription([
        mission,
        watchdog,
        comunication,
        mqtt,
        sensors,
        planner,
        camera,
    ])
