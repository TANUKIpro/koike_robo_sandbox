"""
TUM RGB-Dデータセットを可視化するlaunchファイル

使用方法:
    ros2 launch rgbd_visualizer visualize_tum.launch.py \
        dataset_path:=/path/to/rgbd_dataset_freiburg1_desk

オプション:
    - dataset_path: データセットのパス（必須）
    - rate: パブリッシュレート（デフォルト: 30.0）
    - enable_pointcloud: 点群生成を有効化（デフォルト: true）
    - enable_viewer: OpenCVビューアーを有効化（デフォルト: true）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable point cloud generation'
    )

    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='true',
        description='Enable OpenCV viewer'
    )

    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='5000.0',
        description='Depth scale factor (5000 for TUM)'
    )

    # Nodes
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

    rgbd_viewer_node = Node(
        package='rgbd_visualizer',
        executable='rgbd_viewer',
        name='rgbd_viewer',
        output='screen',
        parameters=[{
            'depth_scale': LaunchConfiguration('depth_scale'),
            'max_depth': 5.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_viewer'))
    )

    pointcloud_generator_node = Node(
        package='rgbd_visualizer',
        executable='pointcloud_generator',
        name='pointcloud_generator',
        output='screen',
        parameters=[{
            'depth_scale': LaunchConfiguration('depth_scale'),
            'max_depth': 10.0,
            'min_depth': 0.1,
            'downsample': 2,
        }],
        condition=IfCondition(LaunchConfiguration('enable_pointcloud'))
    )

    return LaunchDescription([
        dataset_path_arg,
        rate_arg,
        enable_pointcloud_arg,
        enable_viewer_arg,
        depth_scale_arg,
        bag_publisher_node,
        rgbd_viewer_node,
        pointcloud_generator_node,
    ])
