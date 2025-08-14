import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml

def modify_slam_yaml(context, *args, **kwargs):
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    slam_params = {}

    with open(slam_params_file, 'r') as file:
        yaml_content = yaml.safe_load(file)
        slam_params = yaml_content['slam_toolbox']['ros__parameters']
        if namespace != '':
            slam_params['odom_frame'] = f'{namespace}/{slam_params['odom_frame']}'
            slam_params['base_frame'] = f'{namespace}/{slam_params['base_frame']}'
            # map_frame must be the same (map) for each robot
    
    additional_params = {
        'use_lifecycle_manager': use_lifecycle_manager,
        'use_sim_time': use_sim_time
    }
        
    # based on https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[{**additional_params, **slam_params}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=[('/map', f'/{namespace}/map'),
                    ('/map_metadata', f'/{namespace}/map_metadata'),
                    ('/map_updates', f'/{namespace}/map_updates')]
    )
    
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )
    
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )
    
    return [start_async_slam_toolbox_node, configure_event, activate_event]

def generate_launch_description():
    args = [
        DeclareLaunchArgument('autostart', default_value='true', 
                              description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),
        DeclareLaunchArgument('use_lifecycle_manager', default_value='false', 
                              description='Enable bond connection during node activation'),
        DeclareLaunchArgument('use_sim_time', default_value='true', 
                              description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument('slam_params_file', 
                              default_value=os.path.join(get_package_share_directory("jackal_navigation"),
                                                        'config', 'slam_parameters.yaml'),
                              description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
        DeclareLaunchArgument('namespace', default_value='jackal1', 
                              description='Top-level namespace'),
    ]
    
    ld = LaunchDescription([*args, OpaqueFunction(function=modify_slam_yaml)])
    return ld