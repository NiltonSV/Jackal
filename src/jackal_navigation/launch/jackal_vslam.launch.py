# based on https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/husky/husky_slam3d.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    use_camera   = LaunchConfiguration('use_camera')

    rtabmap_parameters={
        'map_frame_id':'map',
        'odom_frame_id':'odom',
        'odom_tf_linear_variance':0.001,
        'odom_tf_angular_variance':0.001,
        'subscribe_rgb':False,
        'subscribe_depth':False,
        'subscribe_rgbd': use_camera,
        'subscribe_scan_cloud':True,
        'use_action_for_goal':True,
        'publish_tf': True,
        # 'odom_sensor_sync': True,
        # RTAB-Map's parameters should be strings:
        'Mem/NotLinkedNodesKept':'false',
        'Grid/RangeMin':'0.5', # ignore laser scan points on the robot itself
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'1',  # All points over 1 meter are ignored
        'Grid/RayTracing':'true', # Fill empty space
        'Grid/3D':'false', # Use 2D occupancy
        'RGBD/OptimizeMaxError':'0.3', # There are a lot of repetitive patterns, be more strict in accepting loop closures
    }

    # Shared parameters between different nodes
    shared_parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true', # we are moving on a 2D flat floor
          'Mem/NotLinkedNodesKept':'false',
          'Icp/VoxelSize': '0.3',
          'Icp/MaxCorrespondenceDistance': '3', # roughly 10x voxel size
          'Icp/PointToPlaneGroundNormalsUp': '0.9',
          'Icp/RangeMin': '0.5',
          'Icp/MaxTranslation': '1'
    }

    config_rviz = PathJoinSubstitution([
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    ])

    remappings=[
        ('rgb/image',       '/rgb/image'),
        ('depth/image',     '/depth/image'),
        ('rgb/camera_info', '/rgb/camera_info'),
        ('scan',            '/scan'),
    ]

    rtabmap_rgbd_sync = Node(
        condition=IfCondition(use_camera),
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
        remappings=remappings,
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
        arguments=['-d']
    )

    rtabmap_localization = Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings,
    )

    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
    )
    
    rtabmap_rviz = Node(
        package='rviz2', executable='rviz2', name="rviz2", output='screen',
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('use_camera', default_value='false', choices=['false', 'true'], description='Use camera for global loop closure / re-localization.'),
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),
        DeclareLaunchArgument('slam_params', default_value='', description='Path to the SLAM parameters file'),
        
        rtabmap_rgbd_sync,
        rtabmap_node,
        rtabmap_localization,
        rtabmap_viz,
        rtabmap_rviz,
    ])
    return ld
