"""
RViz2でRGB-Dデータを可視化するlaunchファイル

使用方法:
    ros2 launch rgbd_visualizer rviz_visualize.launch.py \
        dataset_path:=/path/to/rgbd_dataset_freiburg1_desk
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.conditions


def generate_launch_description():
    # Launch arguments
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        description='Path to TUM RGB-D dataset'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='30.0',
        description='Publish rate in Hz'
    )

    # Bag publisher
    bag_publisher_node = Node(
        package='rgbd_visualizer',
        executable='bag_publisher',
        name='bag_publisher',
        output='screen',
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'rate': LaunchConfiguration('rate'),
            'loop': True,
        }]
    )

    # Point cloud generator
    pointcloud_generator_node = Node(
        package='rgbd_visualizer',
        executable='pointcloud_generator',
        name='pointcloud_generator',
        output='screen',
        parameters=[{
            'depth_scale': 5000.0,  # TUM
            'max_depth': 10.0,
            'min_depth': 0.1,
            'downsample': 2,
        }]
    )

    # Static transform: map -> camera
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_depth_optical_frame']
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        dataset_path_arg,
        rate_arg,
        bag_publisher_node,
        pointcloud_generator_node,
        static_tf_node,
        rviz_node,
    ])
