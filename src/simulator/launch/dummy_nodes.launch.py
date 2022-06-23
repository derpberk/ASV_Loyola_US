import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import time

# este launch carga la configuración de la carpeta {simulator}/config y lanza ardupilot SITL, MavROS y RVIZ


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('simulator'),
        'config',
        'simulation_config.yaml')

    drone = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='drone',
        name='drone_node',
        emulate_tty=True,
        output='screen',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    planner = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='planner',
        emulate_tty=True,
        output='screen',
        name='mission_node',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    watchdog = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='watchdog',
        emulate_tty=True,
        output='screen',
        name='watchdog_node',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    mission = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='mission',
        name='mission_node',
        emulate_tty=True,
        output='screen',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    mqtt = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='mqtt',
        emulate_tty=True,
        name='mqtt_node',
        output='screen',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    sensors = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='sensors',
        emulate_tty=True,
        output='screen',
        name='sensors_node',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    dummy_publisher = launch_ros.actions.Node(
        package='simulator',
        executable='dummy_publisher',
        name='dummy_drone_node',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )

    camera = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='camera',
        emulate_tty=True,
        output='screen',
        name='camera_node',
        prefix=['stdbuf -o L'],
        parameters = [config]
    )


    return launch.LaunchDescription([
        mission,
        watchdog,
        dummy_publisher,
        mqtt,
        sensors,
        planner,
        #camera,
    ])
