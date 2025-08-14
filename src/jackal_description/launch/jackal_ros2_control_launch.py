from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
import yaml
import copy
import tempfile

def generate_launch_description():
    packageName     = "jackal_description"
    ros2ControlName = "jackal_control.yaml"
    packagePath     = get_package_share_directory(packageName)

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace    = LaunchConfiguration('namespace')
    
    robot_controllers = PathJoinSubstitution([packagePath, 'config', ros2ControlName])
    ros2_control_params = LaunchConfiguration('ros2_control_params')

    joint_state_broadcaster = Node(
                    package='controller_manager', executable='spawner', output='screen', name='joint_state_broadcaster', namespace=namespace,
                    arguments=['joint_state_broadcaster'],
                    parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_base_controller = Node(
                package='controller_manager', executable='spawner', output='screen', name='diff_drive_controller', namespace=namespace,
                arguments=['diff_drive_base_controller', '--param-file', ros2_control_params],
                parameters=[{'use_sim_time': use_sim_time}],
    )

    # joint_trajectory_controller = Node(
    #                 package='controller_manager', executable='spawner', output='screen', name='joint_trajectory_controller',
    #                 condition=IfCondition(PythonExpression(["'", LaunchConfiguration('robot_model'), "' == 'jackal_kinova'"])),
    #                 arguments=['kinova_trajectory_controller', '--param-file', ros2_control_params],
    #                 parameters=[{'use_sim_time': use_sim_time}],
    # )

    # Declare launch arguments
    args = [
        DeclareLaunchArgument('ros2_control_params', default_value=robot_controllers, description='Path to the ros2 control parameters file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use sim time'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
    ]

    return LaunchDescription([
        *args,
        joint_state_broadcaster,
        diff_drive_base_controller,
        # joint_trajectory_controller
    ])