from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os

def modify_rtabmap_params(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace    = LaunchConfiguration('namespace').perform(context)

    frames_parameters = {}

    if namespace != '':
        frames_parameters={
            'odom_frame_id':f'{namespace}/odom',
            'frame_id':f'{namespace}/base_link',
        }
    else:
        frames_parameters={
            'odom_frame_id':'odom',
            'frame_id':'base_link',
        }
        

    rtabmap_parameters={
        'map_frame_id':'map',
        # 'odom_frame_id':f'{namespace}/odom',
        # 'frame_id':f'{namespace}/base_link',
        'use_sim_time':use_sim_time,
        'odom_tf_linear_variance':0.001,
        'odom_tf_angular_variance':0.001,
        'subscribe_rgb':False,
        'subscribe_depth':False,
        'subscribe_rgbd': False,
        'subscribe_scan_cloud':True,
        'use_action_for_goal':True,
        'publish_tf': True,
        'Mem/NotLinkedNodesKept':'false',
        'Grid/RangeMin':'0.5',
        'Grid/NormalsSegmentation':'false',
        'Grid/MaxGroundHeight':'0.05',
        'Grid/MaxObstacleHeight':'1',
        'Grid/RayTracing':'true',
        'Grid/3D':'false',
        'RGBD/OptimizeMaxError':'0.3',
    }

    shared_parameters={
          
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'Mem/NotLinkedNodesKept':'false',
          'Icp/VoxelSize': '0.3',
          'Icp/MaxCorrespondenceDistance': '3',
          'Icp/PointToPlaneGroundNormalsUp': '0.9',
          'Icp/RangeMin': '0.5',
          'Icp/MaxTranslation': '1',
    }

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=namespace,
        output='screen',
        parameters=[frames_parameters, rtabmap_parameters, shared_parameters],
        arguments=['-d']
    )

    return [rtabmap_node]

def generate_launch_description():

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),      
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),  
    ]

    return LaunchDescription([
        *args,
        OpaqueFunction(function=modify_rtabmap_params),
    ])