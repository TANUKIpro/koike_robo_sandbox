#!/usr/bin/env python3
"""
平面検出 Launch ファイル

ROSbagの再生と平面検出ノードを同時に起動します。
OpenCVウィンドウで検出結果をリアルタイム表示します。

使用例:
    ros2 launch plane_detector plane_detection.launch.py \
        bag_path:=/path/to/datasets/robocup/storing_try_2

    # ループ再生
    ros2 launch plane_detector plane_detection.launch.py \
        bag_path:=/path/to/datasets/robocup/storing_try_2 \
        loop:=true

    # 0.5倍速で再生
    ros2 launch plane_detector plane_detection.launch.py \
        bag_path:=/path/to/datasets/robocup/storing_try_2 \
        rate:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Launch引数の宣言
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the ROS2 bag directory'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop playback'
    )

    start_offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Start offset in seconds'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # パラメータ設定
    process_rate_arg = DeclareLaunchArgument(
        'process_rate',
        default_value='5.0',
        description='Plane detection processing rate (Hz)'
    )

    normal_threshold_arg = DeclareLaunchArgument(
        'normal_threshold_deg',
        default_value='10.0',
        description='Normal angle threshold for horizontal plane detection (degrees)'
    )

    floor_height_arg = DeclareLaunchArgument(
        'floor_height',
        default_value='0.0',
        description='Floor height in base_footprint frame (meters)'
    )

    min_detection_height_arg = DeclareLaunchArgument(
        'min_detection_height',
        default_value='0.15',
        description='Minimum height to detect planes (meters, above floor)'
    )

    max_detection_height_arg = DeclareLaunchArgument(
        'max_detection_height',
        default_value='2.0',
        description='Maximum height to detect planes (meters)'
    )

    # ROSbag再生コマンドの構築
    bag_play_cmd = [
        'ros2', 'bag', 'play',
        LaunchConfiguration('bag_path'),
        '--rate', LaunchConfiguration('rate'),
        '--clock',
        '--start-offset', LaunchConfiguration('start_offset'),
    ]

    # ループオプション（条件付き）
    bag_play_with_loop = ExecuteProcess(
        cmd=bag_play_cmd + ['--loop'],
        name='rosbag_play',
        output='screen',
        condition=IfCondition(LaunchConfiguration('loop'))
    )

    bag_play_without_loop = ExecuteProcess(
        cmd=bag_play_cmd,
        name='rosbag_play',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('loop'), "' == 'false'"])
        )
    )

    # 平面検出ノード
    plane_detector_node = Node(
        package='plane_detector',
        executable='plane_detector_node',
        name='plane_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rgb_topic': '/head_front_camera/rgb/image_raw',
            'depth_topic': '/head_front_camera/depth/image_raw',
            'camera_info_topic': '/head_front_camera/rgb/camera_info',
            'camera_frame': 'head_front_camera_rgb_optical_frame',
            'base_frame': 'base_footprint',
            'depth_scale': 1000.0,  # mm -> m for TIAGo
            'min_depth': 0.3,
            'max_depth': 5.0,
            'downsample_factor': 4,
            'distance_threshold': 0.02,
            'min_plane_points': 500,
            'normal_threshold_deg': LaunchConfiguration('normal_threshold_deg'),
            'process_rate': LaunchConfiguration('process_rate'),
            # 床面の設定
            'floor_height': LaunchConfiguration('floor_height'),
            'floor_tolerance': 0.10,
            # 検知対象の高さ範囲
            'min_detection_height': LaunchConfiguration('min_detection_height'),
            'max_detection_height': LaunchConfiguration('max_detection_height'),
            # 高さによる分類
            'floor_height_max': 0.15,
            'table_height_min': 0.40,
            'table_height_max': 1.20,
            # OpenCVウィンドウで表示
            'show_window': True,
        }]
    )

    return LaunchDescription([
        # 引数
        bag_path_arg,
        rate_arg,
        loop_arg,
        start_offset_arg,
        use_sim_time_arg,
        process_rate_arg,
        normal_threshold_arg,
        floor_height_arg,
        min_detection_height_arg,
        max_detection_height_arg,

        # プロセス
        bag_play_with_loop,
        bag_play_without_loop,
        plane_detector_node,
    ])
