from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node

def launch_amcl(context, *args, **kwargs):

    if LaunchConfiguration('use_localization').perform(context) == 'false':
        return []

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(args[0]),
        launch_arguments={
            'use_sim_time': args[1],
            'map': LaunchConfiguration('map_file'),
            'params_file': LaunchConfiguration('nav2_params')   
        }.items()
    )

    return [localization_node]

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch_path       = PathJoinSubstitution([get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'])
    nav2_localization_path = PathJoinSubstitution([get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py'])
    map_file_path          = PathJoinSubstitution([get_package_share_directory('jackal_navigation'), 'maps', 'raw', 'playground.yaml'])
    nav2_params_path       = PathJoinSubstitution([get_package_share_directory('jackal_navigation'), 'config', 'amcl_params.yaml'])

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    twist_remap_node = Node(
                    package='jackal_navigation', executable='nav_vel_remap', output='screen', name='twist_remap_nav',
                    parameters=[{'use_sim_time': use_sim_time}]
    )

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('use_localization', default_value='false', choices=['true', 'false'], description='Use amcl for localization'),
        DeclareLaunchArgument('map_file', default_value=map_file_path, description='Map file to use for navigation'),
        DeclareLaunchArgument('nav2_params', default_value=nav2_params_path, description='Path to the amcl config file')
    ]

    return LaunchDescription([*args, 
                             nav2_node,
                             OpaqueFunction(function=launch_amcl, args=[nav2_localization_path, use_sim_time]),
                            #  twist_remap_node
                            ])