import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    background_b_launch_arg = DeclareLaunchArgument(
        'background_b', default_value=TextSubstitution(text='122')
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/turtlesim_rviz.launch.py'])
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])

    return LaunchDescription([
          background_r_launch_arg,
          background_g_launch_arg,
          background_b_launch_arg,
          Node(
             package='turtlesim',
             executable='turtlesim_node',
             name='sim',
             parameters=[{
                'background_r': LaunchConfiguration('background_r'),
                'background_g': LaunchConfiguration('background_g'),
                'background_b': LaunchConfiguration('background_b'),
             }]
          ),
       ])
