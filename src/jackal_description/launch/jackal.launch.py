from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def print_robot_description(context, *args, **kwargs):
    print(args[0].perform(context))

def generate_launch_description():
    
    namePackage   = "jackal_description"
    nameRviz2File = "rviz_jackal.rviz"
    packagePath   = get_package_share_directory(namePackage)

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace    = LaunchConfiguration('namespace')

    # Robot State Node
    robot_description = Command(['xacro ', 
                                 PathJoinSubstitution([packagePath, 'urdf', LaunchConfiguration('robot_model')]),
                                 '.urdf.xacro', ' namespace:=', namespace])

    robot_state_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        name='robot_state_publisher' ,namespace=namespace,
        parameters=[{
                    #  'frame_prefix': [namespace, '/'],
                     'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
    )

    # Joint State Node
    joint_state_publisher = Node(
        condition=UnlessCondition(use_sim_time),
        package='joint_state_publisher', executable='joint_state_publisher', output='screen',
        name='joint_state_publisher', namespace=namespace,
        parameters=[{
                    #  'frame_prefix': [namespace, '/'],
                     'use_sim_time': use_sim_time}],
    )

    # RViz2 Node
    rviz2_path = PathJoinSubstitution([packagePath, 'config', nameRviz2File])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2", executable="rviz2", name="jackal_rviz2", output="screen",
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo Node
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([packagePath, 'launch', 'jackal_gazebo_launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace, '-allow_renaming': 'true',
                          'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
                          'z': LaunchConfiguration('z') ,'Y': LaunchConfiguration('Y')}.items()
    )

    # ROS2 Control Node
    robot_controllers = PathJoinSubstitution([ packagePath, 'config',
                                              PythonExpression(['"', LaunchConfiguration('robot_model'), '_control.yaml"']) ])

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([packagePath, 'launch', 'jackal_ros2_control_launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={'use_sim_time': use_sim_time, 'ros2_control_params': robot_controllers, 'namespace': namespace}.items()
    )

    args = [
        DeclareLaunchArgument('robot_model', default_value='jackal', choices=['jackal', 'jackal_kinova'], description='Robot model to use'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation/gazebo time'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'], description='Rviz visualization'),
        DeclareLaunchArgument('rviz2_config', default_value=rviz2_path, description='RViz2 configuration file'),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot initial pose x'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot initial pose y'),
        DeclareLaunchArgument('z', default_value='0.06', description='Robot initial pose z: must be 0.06'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Robot initial orientation yaw'),
    ]

    # LAUNCH
    return LaunchDescription([*args,
                             robot_state_node,
                             joint_state_publisher,
                             rviz2_node,
                             gazebo_launch,
                             TimerAction(period=5.0, actions=[ros2_control_launch]),
    ])
