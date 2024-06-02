import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import time



def generate_launch_description():

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
        sonar_node,
        wqp_sensor_node,
        asv_node,
        server_comms_node,
        path_planner_node
    ])
