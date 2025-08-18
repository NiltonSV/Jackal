from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml

def modify_yaml_bridge(context, *args, **kwargs):
    packageName  = "jackal_description"
    bridgeparams = "bridge_parameters.yaml"
    packagePath = get_package_share_directory(packageName)

    namespace = args[0].perform(context)

    config_file = PathJoinSubstitution([packagePath, 'config', bridgeparams])
    
    new_config = []
    if namespace != '':
        with open(os.path.join(packagePath, 'config', bridgeparams), 'r') as f:
            bridges = yaml.safe_load(f)

        for bridge in bridges:
            bridge['ros_topic_name'] = f"{namespace}/{bridge['ros_topic_name']}"
            bridge['gz_topic_name'] = f"{namespace}/{bridge['gz_topic_name']}"
            new_config.append(bridge)

        config_file = f"/tmp/{namespace}_bridge.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(new_config, f)
        
    # print(new_config)

    gazebo_bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        name=f'{namespace}_gz_bridge',
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', config_file]
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return [gazebo_bridge_node]

def generate_launch_description():
    namespace    = LaunchConfiguration('namespace')

    # Spawn the robot gotten from the robot_description topic
    gazebo_spawn_node = Node(
        package='ros_gz_sim', executable='create', output='screen', namespace=namespace,
        arguments=['-topic', 'robot_description',
                   '-name', namespace,
                   '-allow_renaming', 'true',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('Y'),],
    )

    # Declare launch arguments
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot initial pose x'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot initial pose y'),
        DeclareLaunchArgument('z', default_value='0.06', description='Robot initial pose z: must be 0.06'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Robot initial orientation yaw'),
    ]

    return LaunchDescription([
        *args,
        gazebo_spawn_node,
        OpaqueFunction(function=modify_yaml_bridge, args=[namespace]),
    ])