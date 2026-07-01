from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
        'use_sim_time': True,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom': True,
        'subscribe_global_descriptor': True,
        'Mem/GlobalDescriptorStrategy': '0',
        'RGBD/LoopClosureReextractFeatures': 'false',
        'frame_id': 'base_link',
        'approx_sync': True,
        'queue_size': 100,
        'sync_queue_size': 100,
        'qos_image': 2,
        'qos_depth': 2,
        'qos_camera_info': 2,
        'qos_odom': 2
    }]

    remappings=[
        ('rgb/image', '/oakd/rgb/preview/image_raw'),
        ('depth/image', '/oakd/rgb/preview/depth'),
        ('rgb/camera_info', '/oakd/rgb/preview/camera_info'),
        ('odom', '/odom'),
        ('global_descriptor', '/global_descriptor')
    ]

    return LaunchDescription([
        Node(
            package='holistic_loop_closure',
            executable='extractor_node',
            name='holistic_feature_extractor',
            output='screen'
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        )
    ])
