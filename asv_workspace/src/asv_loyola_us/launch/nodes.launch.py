import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import time
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

def launch_setup(context, *args, **kwargs):
    use_sensors_arg = LaunchConfiguration('use_sensors')
    use_sensors_bool=context.perform_substitution(use_sensors_arg).lower() in "true"
    set_use_sensors_arg = SetParameter(name="use_sensors", value=use_sensors_arg)

    config = os.path.join(
        get_package_share_directory('asv_loyola_us'),
        'config',
        'config.yaml')
    
    sonar_node = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='sonar',
        name='sonar_node',
        parameters = [config]
    )

    wqp_sensor_node = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='wqp_sensor',
        name='wqp_sensor_node',
        parameters = [config]
    )

    if use_sensors_bool:
        return [
            set_use_sensors_arg,
            sonar_node,
            wqp_sensor_node
        ]
    else:
        return[
            set_use_sensors_arg,
        ]

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('asv_loyola_us'),
        'config',
        'config.yaml')

    use_sensors_argument = DeclareLaunchArgument(
        "use_sensors", default_value="True"    )

    
    
    asv_node = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='asv',
        name='asv_node',
        parameters = [config]
    )

    server_comms_node = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='communications',
        name='server_communication_node',
        parameters = [config]
    )

    path_planner_node = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='path_planner',
        name='path_planner_node',
        parameters = [config]
    )



    return launch.LaunchDescription([
        use_sensors_argument,
        asv_node,
        server_comms_node,
        path_planner_node,
        OpaqueFunction(function=launch_setup),
    ])
