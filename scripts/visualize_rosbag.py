#!/usr/bin/env python3
"""
ROSbag からのRGB-D可視化スクリプト

ROS1/ROS2 bag形式のデータセットを読み込み、ロボットの視点を可視化します。
対応データセット: ETH3D, VECtor, RoboCup, Voxblox

必要なパッケージ:
    pip install rosbags numpy opencv-python matplotlib open3d

使用方法:
    # ROS1 bag（ETH3D, VECtor, Voxblox）
    python visualize_rosbag.py --bag ../datasets/voxblox/data.bag --version ros1

    # ROS2 bag（RoboCup）
    python visualize_rosbag.py --bag ../datasets/robocup/rosbag2_mapping/ --version ros2

    # トピック一覧の表示
    python visualize_rosbag.py --bag ../datasets/voxblox/data.bag --version ros1 --list-topics
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Generator

import cv2
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    from rosbags.rosbag1 import Reader as Reader1
    from rosbags.rosbag2 import Reader as Reader2
    from rosbags.serde import deserialize_cdr, ros1_to_cdr
    HAS_ROSBAGS = True
except ImportError:
    HAS_ROSBAGS = False
    print("Error: rosbags package not installed.")
    print("Install with: pip install rosbags")


# カメラパラメータ
CAMERA_PARAMS = {
    'eth3d': {'fx': 527.0, 'fy': 527.0, 'cx': 320.0, 'cy': 240.0, 'depth_scale': 1000.0},
    'vector': {'fx': 386.738, 'fy': 386.738, 'cx': 321.281, 'cy': 238.221, 'depth_scale': 1000.0},
    'robocup': {'fx': 570.3, 'fy': 570.3, 'cx': 319.5, 'cy': 239.5, 'depth_scale': 1000.0},
    'voxblox': {'fx': 525.0, 'fy': 525.0, 'cx': 319.5, 'cy': 239.5, 'depth_scale': 1000.0},
}

# 各データセットの典型的なトピック名
TOPIC_PATTERNS = {
    'rgb': ['/camera/color/image_raw', '/d435i/color/image_raw',
            '/camera/rgb/image_raw', '/rgb/image_raw'],
    'depth': ['/camera/depth/image_raw', '/d435i/depth/image_rect_raw',
              '/camera/depth_registered/image_raw', '/depth/image_raw'],
    'pointcloud': ['/camera/depth_registered/points', '/points',
                   '/camera/depth/points', '/d435i/depth/points'],
    'imu': ['/imu', '/imu/data', '/d435i/imu', '/ouster/imu'],
}


def list_topics(bag_path: str, version: str) -> None:
    """ROSbag内のトピック一覧を表示"""
    if not HAS_ROSBAGS:
        return

    Reader = Reader1 if version == 'ros1' else Reader2

    print(f"\nTopics in {bag_path}:")
    print("-" * 60)

    with Reader(bag_path) as reader:
        for connection in reader.connections:
            print(f"  {connection.topic}")
            print(f"    Type: {connection.msgtype}")


def find_topic(connections: list, patterns: List[str]) -> Optional[str]:
    """パターンに一致するトピックを探す"""
    topic_names = [c.topic for c in connections]
    for pattern in patterns:
        if pattern in topic_names:
            return pattern
    # 部分一致も試す
    for pattern in patterns:
        for topic in topic_names:
            if pattern.split('/')[-1] in topic:
                return topic
    return None


def read_image_msg(msg, msgtype: str) -> np.ndarray:
    """sensor_msgs/Image メッセージをNumPy配列に変換"""
    h, w = msg.height, msg.width
    encoding = msg.encoding

    if encoding in ['rgb8', 'bgr8']:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if encoding == 'bgr8':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif encoding in ['16UC1', 'mono16']:
        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    elif encoding == '32FC1':
        img = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    elif encoding in ['8UC1', 'mono8']:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
    else:
        print(f"Warning: Unknown encoding {encoding}")
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)

    return img


def read_ros1_bag(bag_path: str, max_frames: int = 100,
                  rgb_topic: Optional[str] = None,
                  depth_topic: Optional[str] = None) -> Generator:
    """ROS1 bagからRGB-Dデータを読み込む"""
    if not HAS_ROSBAGS:
        return

    with Reader1(bag_path) as reader:
        # トピックの自動検出
        if rgb_topic is None:
            rgb_topic = find_topic(reader.connections, TOPIC_PATTERNS['rgb'])
        if depth_topic is None:
            depth_topic = find_topic(reader.connections, TOPIC_PATTERNS['depth'])

        print(f"RGB topic: {rgb_topic}")
        print(f"Depth topic: {depth_topic}")

        rgb_buffer = {}
        depth_buffer = {}
        frame_count = 0

        for connection, timestamp, rawdata in reader.messages():
            if frame_count >= max_frames:
                break

            try:
                msg = deserialize_cdr(
                    ros1_to_cdr(rawdata, connection.msgtype),
                    connection.msgtype
                )
            except Exception as e:
                continue

            if connection.topic == rgb_topic:
                rgb = read_image_msg(msg, connection.msgtype)
                rgb_buffer[timestamp] = rgb

            elif connection.topic == depth_topic:
                depth = read_image_msg(msg, connection.msgtype)
                if depth.dtype == np.uint16:
                    depth = depth.astype(np.float32) / 1000.0
                depth_buffer[timestamp] = depth

            # マッチング（近いタイムスタンプ）
            if rgb_buffer and depth_buffer:
                rgb_ts = max(rgb_buffer.keys())
                depth_ts = min(depth_buffer.keys(), key=lambda x: abs(x - rgb_ts))

                if abs(rgb_ts - depth_ts) < 50_000_000:  # 50ms
                    frame_count += 1
                    yield rgb_buffer[rgb_ts], depth_buffer[depth_ts], rgb_ts
                    rgb_buffer.clear()
                    depth_buffer.clear()


def read_ros2_bag(bag_path: str, max_frames: int = 100,
                  rgb_topic: Optional[str] = None,
                  depth_topic: Optional[str] = None) -> Generator:
    """ROS2 bagからRGB-Dデータを読み込む"""
    if not HAS_ROSBAGS:
        return

    with Reader2(bag_path) as reader:
        # トピックの自動検出
        if rgb_topic is None:
            rgb_topic = find_topic(reader.connections, TOPIC_PATTERNS['rgb'])
        if depth_topic is None:
            depth_topic = find_topic(reader.connections, TOPIC_PATTERNS['depth'])

        print(f"RGB topic: {rgb_topic}")
        print(f"Depth topic: {depth_topic}")

        rgb_buffer = {}
        depth_buffer = {}
        frame_count = 0

        for connection, timestamp, rawdata in reader.messages():
            if frame_count >= max_frames:
                break

            try:
                msg = deserialize_cdr(rawdata, connection.msgtype)
            except Exception as e:
                continue

            if connection.topic == rgb_topic:
                rgb = read_image_msg(msg, connection.msgtype)
                rgb_buffer[timestamp] = rgb

            elif connection.topic == depth_topic:
                depth = read_image_msg(msg, connection.msgtype)
                if depth.dtype == np.uint16:
                    depth = depth.astype(np.float32) / 1000.0
                depth_buffer[timestamp] = depth

            # マッチング
            if rgb_buffer and depth_buffer:
                rgb_ts = max(rgb_buffer.keys())
                depth_ts = min(depth_buffer.keys(), key=lambda x: abs(x - rgb_ts))

                if abs(rgb_ts - depth_ts) < 50_000_000:
                    frame_count += 1
                    yield rgb_buffer[rgb_ts], depth_buffer[depth_ts], rgb_ts
                    rgb_buffer.clear()
                    depth_buffer.clear()


def visualize_frame(rgb: np.ndarray, depth: np.ndarray, timestamp: int) -> None:
    """RGB-Dフレームを可視化"""
    if not HAS_MATPLOTLIB:
        print("matplotlib is required")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].imshow(rgb)
    axes[0].set_title(f'RGB - t={timestamp}')
    axes[0].axis('off')

    im = axes[1].imshow(depth, cmap='viridis', vmin=0, vmax=5)
    axes[1].set_title('Depth (m)')
    axes[1].axis('off')
    plt.colorbar(im, ax=axes[1])

    plt.tight_layout()
    plt.show()


def depth_to_pointcloud(depth: np.ndarray, rgb: np.ndarray,
                        camera_params: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
    """深度画像を点群に変換"""
    fx, fy = camera_params['fx'], camera_params['fy']
    cx, cy = camera_params['cx'], camera_params['cy']

    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    mask = (z > 0) & (z < 10)
    points = np.stack([x[mask], y[mask], z[mask]], axis=-1)

    if rgb.ndim == 3:
        colors = rgb[mask] / 255.0
    else:
        colors = np.ones((len(points), 3)) * 0.5

    return points, colors


def visualize_pointcloud(points: np.ndarray, colors: np.ndarray) -> None:
    """点群を可視化"""
    if not HAS_OPEN3D:
        print("open3d is required for point cloud visualization")
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    o3d.visualization.draw_geometries([pcd, coord_frame],
                                       window_name="Point Cloud",
                                       width=1280, height=720)


def interactive_viewer(frames: List[Tuple], camera_params: Dict[str, float]) -> None:
    """インタラクティブビューアー"""
    if not HAS_MATPLOTLIB:
        return

    current_idx = 0
    total_frames = len(frames)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    def update_display(idx: int):
        rgb, depth, ts = frames[idx]

        axes[0].clear()
        axes[0].imshow(rgb)
        axes[0].set_title(f'RGB - Frame {idx}/{total_frames-1}')
        axes[0].axis('off')

        axes[1].clear()
        im = axes[1].imshow(depth, cmap='viridis', vmin=0, vmax=5)
        axes[1].set_title('Depth (m)')
        axes[1].axis('off')

        fig.suptitle(f"Frame {idx} / {total_frames-1} (arrows to navigate, P for pointcloud, Q to quit)")
        fig.canvas.draw()

    def on_key(event):
        nonlocal current_idx
        if event.key == 'right':
            current_idx = min(current_idx + 1, total_frames - 1)
            update_display(current_idx)
        elif event.key == 'left':
            current_idx = max(current_idx - 1, 0)
            update_display(current_idx)
        elif event.key == 'p':
            rgb, depth, _ = frames[current_idx]
            points, colors = depth_to_pointcloud(depth, rgb, camera_params)
            visualize_pointcloud(points, colors)
        elif event.key == 'q':
            plt.close(fig)

    fig.canvas.mpl_connect('key_press_event', on_key)
    update_display(current_idx)
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='ROSbagからRGB-Dデータを可視化',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--bag', type=str, required=True,
                        help='ROSbagファイルのパス')
    parser.add_argument('--version', type=str, required=True,
                        choices=['ros1', 'ros2'],
                        help='ROS version (ros1 or ros2)')
    parser.add_argument('--dataset', type=str, default='voxblox',
                        choices=['eth3d', 'vector', 'robocup', 'voxblox'],
                        help='データセットの種類（カメラパラメータ用）')
    parser.add_argument('--list-topics', action='store_true',
                        help='トピック一覧を表示')
    parser.add_argument('--rgb-topic', type=str, default=None,
                        help='RGBトピック名（自動検出しない場合）')
    parser.add_argument('--depth-topic', type=str, default=None,
                        help='深度トピック名（自動検出しない場合）')
    parser.add_argument('--max-frames', type=int, default=50,
                        help='読み込む最大フレーム数')
    parser.add_argument('--mode', type=str, default='interactive',
                        choices=['interactive', 'single', 'pointcloud'],
                        help='可視化モード')
    parser.add_argument('--frame', type=int, default=0,
                        help='表示するフレーム番号（single/pointcloudモード用）')

    args = parser.parse_args()

    if not HAS_ROSBAGS:
        print("Error: rosbags package is required")
        print("Install with: pip install rosbags")
        sys.exit(1)

    # トピック一覧表示
    if args.list_topics:
        list_topics(args.bag, args.version)
        return

    # データ読み込み
    print(f"Loading {args.version} bag: {args.bag}")

    if args.version == 'ros1':
        reader = read_ros1_bag(args.bag, args.max_frames,
                               args.rgb_topic, args.depth_topic)
    else:
        reader = read_ros2_bag(args.bag, args.max_frames,
                               args.rgb_topic, args.depth_topic)

    frames = list(reader)
    print(f"Loaded {len(frames)} frames")

    if len(frames) == 0:
        print("No frames found. Check topic names with --list-topics")
        sys.exit(1)

    camera_params = CAMERA_PARAMS[args.dataset]

    # 可視化
    if args.mode == 'interactive':
        interactive_viewer(frames, camera_params)
    elif args.mode == 'single':
        idx = min(args.frame, len(frames) - 1)
        rgb, depth, ts = frames[idx]
        visualize_frame(rgb, depth, ts)
    elif args.mode == 'pointcloud':
        idx = min(args.frame, len(frames) - 1)
        rgb, depth, _ = frames[idx]
        points, colors = depth_to_pointcloud(depth, rgb, camera_params)
        print(f"Point cloud: {len(points)} points")
        visualize_pointcloud(points, colors)


if __name__ == '__main__':
    main()
