from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def modify_ekf_yaml(context, *args, **kwargs):
    ekf_file = LaunchConfiguration('ekf_params_file').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params = {}

    with open(ekf_file, 'r') as file:
        yaml_content = yaml.safe_load(file)
        ekf_params = yaml_content['ekf_filter_node']['ros__parameters']
        if namespace != '':
            ekf_params['odom_frame'] = f'{namespace}/{ekf_params['odom_frame']}'
            ekf_params['base_link_frame'] = f'{namespace}/{ekf_params['base_link_frame']}'
            ekf_params['world_frame'] = f'{namespace}/{ekf_params['world_frame']}'

    print(ekf_params)

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time, **ekf_params}],
    )

    return [ekf_node]

def generate_launch_description():
    
    args = [
        DeclareLaunchArgument('ekf_params_file', default_value=os.path.join(get_package_share_directory("jackal_navigation"),
                                                        'config', 'slam_parameters.yaml'),
                                description='Full path to the ROS2 parameters file to use for the robot_localization (ekf) node'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation/gazebo time'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
    ]

    # LAUNCH
    return LaunchDescription([*args,
                             OpaqueFunction(function=modify_ekf_yaml)])
