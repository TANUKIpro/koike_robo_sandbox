#!/usr/bin/env python3
"""
平面検出 Launch ファイル

ROSbagの再生、平面検出ノードを同時に起動します。
OpenCVウィンドウで可視化を表示します。

使用例:
    # 1コマンドで起動
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

    # RViz2も起動
    ros2 launch plane_detector plane_detection.launch.py \
        bag_path:=/path/to/datasets/robocup/storing_try_2 \
        use_rviz:=true

    # 面積閾値を変更（小さい平面も検出）
    ros2 launch plane_detector plane_detection.launch.py \
        bag_path:=/path/to/rosbag \
        min_plane_area:=0.01

    # デプス範囲を変更
    ros2 launch plane_detector plane_detection.launch.py \
        bag_path:=/path/to/rosbag \
        min_depth:=0.2 max_depth:=8.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージディレクトリ
    pkg_dir = get_package_share_directory('plane_detector')
    default_rviz_config = os.path.join(pkg_dir, 'config', 'plane_detection.rviz')

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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 (default: false, OpenCV window is used)'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )

    # パラメータ設定
    process_rate_arg = DeclareLaunchArgument(
        'process_rate',
        default_value='5.0',
        description='Plane detection processing rate (Hz)'
    )

    normal_threshold_arg = DeclareLaunchArgument(
        'normal_threshold_deg',
        default_value='15.0',
        description='Normal angle threshold for horizontal plane detection (degrees)'
    )

    min_plane_area_arg = DeclareLaunchArgument(
        'min_plane_area',
        default_value='0.05',
        description='Minimum plane area in m^2 (default: 0.05 = 22cm x 22cm)'
    )

    min_depth_arg = DeclareLaunchArgument(
        'min_depth',
        default_value='0.1',
        description='Minimum depth in meters (default: 0.1m)'
    )

    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='10.0',
        description='Maximum depth in meters (default: 10.0m)'
    )

    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Show OpenCV visualization window'
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

    # 平面検出ノード（TIAGo rosbag対応）
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
            # TIAGo rosbag対応: 正しいTFフレーム名
            'camera_frame': 'head_front_camera_depth_optical_frame',
            'base_frame': 'base_footprint',
            'depth_scale': 1000.0,  # mm -> m for TIAGo
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth'),
            'downsample_factor': 4,
            'distance_threshold': 0.02,
            'min_plane_points': 500,
            'normal_threshold_deg': LaunchConfiguration('normal_threshold_deg'),
            'process_rate': LaunchConfiguration('process_rate'),
            # 面積ベースのフィルタリング（高さ分類なし）
            'min_plane_area': LaunchConfiguration('min_plane_area'),
            'max_planes': 8,
            'show_window': LaunchConfiguration('show_window'),
        }]
    )

    # RViz2を遅延起動（オプション）
    rviz_node = TimerAction(
        period=2.0,  # 2秒待機
        actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', LaunchConfiguration('rviz_config')],
                name='rviz2',
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # 引数
        bag_path_arg,
        rate_arg,
        loop_arg,
        start_offset_arg,
        use_sim_time_arg,
        use_rviz_arg,
        rviz_config_arg,
        process_rate_arg,
        normal_threshold_arg,
        min_plane_area_arg,
        min_depth_arg,
        max_depth_arg,
        show_window_arg,

        # プロセス
        bag_play_with_loop,
        bag_play_without_loop,
        plane_detector_node,
        rviz_node,
    ])
