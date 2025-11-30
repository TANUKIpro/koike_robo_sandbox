"""
ROS2 rosbagを再生しながらRViz2でRGBDデータを可視化するlaunchファイル

使用方法:
    ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
        bag_path:=/path/to/rosbag/directory

オプション:
    - bag_path: rosbagディレクトリのパス（必須）
    - rate: 再生速度の倍率（デフォルト: 1.0）
    - loop: ループ再生するかどうか（デフォルト: false）
    - start_offset: 再生開始位置（秒）（デフォルト: 0.0）
    - use_sim_time: シミュレーション時間を使用（デフォルト: true）

例:
    # 基本的な使用方法
    ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
        bag_path:=/home/ryo/workspace/koike_robo_sandbox/datasets/robocup/storing_try_2

    # 0.5倍速でループ再生
    ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
        bag_path:=/home/ryo/workspace/koike_robo_sandbox/datasets/robocup/storing_try_2 \
        rate:=0.5 loop:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Package share directory
    pkg_share = get_package_share_directory('rgbd_visualizer')

    # Default RViz config path
    default_rviz_config = os.path.join(pkg_share, 'config', 'rosbag_rgbd.rviz')

    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the rosbag directory'
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
        description='Use simulation time from bag'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )

    # Set use_sim_time for all nodes
    set_use_sim_time = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='0'
    )

    # ROS2 bag play command
    # Build the command dynamically based on loop argument
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--rate', LaunchConfiguration('rate'),
            '--start-offset', LaunchConfiguration('start_offset'),
            '--clock',  # Publish /clock for sim time
        ],
        output='screen',
        name='rosbag_play'
    )

    # ROS2 bag play with loop
    rosbag_play_loop = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--rate', LaunchConfiguration('rate'),
            '--start-offset', LaunchConfiguration('start_offset'),
            '--clock',
            '--loop',
        ],
        output='screen',
        name='rosbag_play_loop',
        condition=IfCondition(LaunchConfiguration('loop'))
    )

    # ROS2 bag play without loop
    rosbag_play_no_loop = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--rate', LaunchConfiguration('rate'),
            '--start-offset', LaunchConfiguration('start_offset'),
            '--clock',
        ],
        output='screen',
        name='rosbag_play_no_loop',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('loop'), "' == 'false'"])
        )
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Arguments
        bag_path_arg,
        rate_arg,
        loop_arg,
        start_offset_arg,
        use_sim_time_arg,
        rviz_config_arg,
        # Environment
        set_use_sim_time,
        # Processes
        rosbag_play_loop,
        rosbag_play_no_loop,
        rviz_node,
    ])
