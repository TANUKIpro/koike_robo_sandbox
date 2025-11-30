#!/usr/bin/env python3
"""
RGB-D データセットの可視化スクリプト

ロボットの見ている光景（RGB画像、深度画像、点群）を可視化します。
対応データセット: TUM RGB-D, ETH3D, VECtor, RoboCup, Voxblox

使用方法:
    # TUM RGB-D データセットの可視化
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk

    # フレームを指定して可視化
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --frame 100

    # 点群のみを表示
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode pointcloud
"""

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

import cv2
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not installed. Some visualization features disabled.")

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: open3d not installed. Point cloud visualization disabled.")


# カメラパラメータ（各データセット用）
CAMERA_PARAMS = {
    'tum_fr1': {'fx': 517.3, 'fy': 516.5, 'cx': 318.6, 'cy': 255.3, 'depth_scale': 5000.0},
    'tum_fr2': {'fx': 520.9, 'fy': 521.0, 'cx': 325.1, 'cy': 249.7, 'depth_scale': 5000.0},
    'tum_fr3': {'fx': 535.4, 'fy': 539.2, 'cx': 320.1, 'cy': 247.6, 'depth_scale': 5000.0},
    'eth3d': {'fx': 527.0, 'fy': 527.0, 'cx': 320.0, 'cy': 240.0, 'depth_scale': 1000.0},
    'vector': {'fx': 386.738, 'fy': 386.738, 'cx': 321.281, 'cy': 238.221, 'depth_scale': 1000.0},
    'robocup': {'fx': 570.3, 'fy': 570.3, 'cx': 319.5, 'cy': 239.5, 'depth_scale': 1000.0},
    'voxblox': {'fx': 525.0, 'fy': 525.0, 'cx': 319.5, 'cy': 239.5, 'depth_scale': 1000.0},
}


class TUMRGBDLoader:
    """TUM RGB-D データセットローダー"""

    def __init__(self, dataset_path: str):
        self.path = Path(dataset_path)
        self.rgb_list = self._load_file_list('rgb.txt')
        self.depth_list = self._load_file_list('depth.txt')
        self.associations = self._create_associations()

        # カメラパラメータの自動選択
        if 'freiburg1' in str(self.path):
            self.camera_params = CAMERA_PARAMS['tum_fr1']
        elif 'freiburg2' in str(self.path):
            self.camera_params = CAMERA_PARAMS['tum_fr2']
        else:
            self.camera_params = CAMERA_PARAMS['tum_fr3']

    def _load_file_list(self, filename: str) -> list:
        """タイムスタンプとファイルパスのリストを読み込む"""
        file_list = []
        filepath = self.path / filename
        if not filepath.exists():
            return file_list

        with open(filepath) as f:
            for line in f:
                if line.startswith('#'):
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    file_list.append((float(parts[0]), parts[1]))
        return file_list

    def _create_associations(self, max_diff: float = 0.02) -> list:
        """RGBと深度画像のタイムスタンプをマッチング"""
        associations = []
        depth_dict = {ts: path for ts, path in self.depth_list}
        depth_timestamps = sorted(depth_dict.keys())

        for rgb_ts, rgb_path in self.rgb_list:
            # 最も近い深度タイムスタンプを探す
            closest_ts = min(depth_timestamps, key=lambda x: abs(x - rgb_ts), default=None)
            if closest_ts is not None and abs(closest_ts - rgb_ts) < max_diff:
                associations.append((rgb_ts, rgb_path, closest_ts, depth_dict[closest_ts]))

        return associations

    def __len__(self) -> int:
        return len(self.associations)

    def __getitem__(self, idx: int) -> Tuple[np.ndarray, np.ndarray, float]:
        """指定インデックスのRGB-Dフレームを取得"""
        if idx >= len(self.associations):
            raise IndexError(f"Index {idx} out of range (max: {len(self.associations)-1})")

        rgb_ts, rgb_path, depth_ts, depth_path = self.associations[idx]

        # RGB画像の読み込み
        rgb = cv2.imread(str(self.path / rgb_path))
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        # 深度画像の読み込み
        depth = cv2.imread(str(self.path / depth_path), cv2.IMREAD_UNCHANGED)
        depth = depth.astype(np.float32) / self.camera_params['depth_scale']

        return rgb, depth, rgb_ts

    def get_camera_params(self) -> Dict[str, float]:
        return self.camera_params


def depth_to_pointcloud(depth: np.ndarray, rgb: np.ndarray,
                        camera_params: Dict[str, float]) -> np.ndarray:
    """深度画像をXYZRGB点群に変換"""
    fx, fy = camera_params['fx'], camera_params['fy']
    cx, cy = camera_params['cx'], camera_params['cy']

    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # 3D座標の計算
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # 有効な点のみ抽出（深度が0でない）
    mask = (z > 0) & (z < 10)  # 10m以内

    points = np.stack([x[mask], y[mask], z[mask]], axis=-1)
    colors = rgb[mask] / 255.0

    return points, colors


def visualize_rgbd_frame(rgb: np.ndarray, depth: np.ndarray,
                         title: str = "RGB-D Frame") -> None:
    """RGB画像と深度画像を並べて表示"""
    if not HAS_MATPLOTLIB:
        print("matplotlib is required for 2D visualization")
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # RGB画像
    axes[0].imshow(rgb)
    axes[0].set_title('RGB Image')
    axes[0].axis('off')

    # 深度画像（カラーマップ）
    depth_viz = axes[1].imshow(depth, cmap='viridis')
    axes[1].set_title('Depth Image')
    axes[1].axis('off')
    plt.colorbar(depth_viz, ax=axes[1], label='Depth (m)')

    # 深度画像（グレースケール、距離に応じた明暗）
    depth_normalized = np.clip(depth / depth.max(), 0, 1)
    axes[2].imshow(depth_normalized, cmap='gray')
    axes[2].set_title('Depth (normalized)')
    axes[2].axis('off')

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def visualize_pointcloud(points: np.ndarray, colors: np.ndarray) -> None:
    """点群をOpen3Dで可視化"""
    if not HAS_OPEN3D:
        print("open3d is required for point cloud visualization")
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # 座標軸を追加
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    print("Point cloud visualization controls:")
    print("  - Mouse drag: Rotate view")
    print("  - Scroll: Zoom")
    print("  - Shift + drag: Pan")
    print("  - Q: Close window")

    o3d.visualization.draw_geometries([pcd, coord_frame],
                                       window_name="Point Cloud Viewer",
                                       width=1280, height=720)


def visualize_robot_view(rgb: np.ndarray, depth: np.ndarray,
                         camera_params: Dict[str, float],
                         show_pointcloud: bool = True) -> None:
    """ロボットの視点を総合的に可視化"""
    print(f"Image size: {rgb.shape[1]}x{rgb.shape[0]}")
    print(f"Depth range: {depth[depth > 0].min():.3f}m - {depth[depth > 0].max():.3f}m")
    print(f"Camera params: fx={camera_params['fx']}, fy={camera_params['fy']}")

    # 2D表示
    visualize_rgbd_frame(rgb, depth)

    # 3D点群表示
    if show_pointcloud and HAS_OPEN3D:
        points, colors = depth_to_pointcloud(depth, rgb, camera_params)
        print(f"Point cloud: {len(points)} points")
        visualize_pointcloud(points, colors)


def interactive_viewer(loader: TUMRGBDLoader) -> None:
    """インタラクティブなフレームビューアー"""
    if not HAS_MATPLOTLIB:
        print("matplotlib is required for interactive viewer")
        return

    current_idx = 0
    total_frames = len(loader)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    def update_display(idx: int):
        rgb, depth, ts = loader[idx]

        axes[0].clear()
        axes[0].imshow(rgb)
        axes[0].set_title(f'RGB - Frame {idx}/{total_frames-1}')
        axes[0].axis('off')

        axes[1].clear()
        im = axes[1].imshow(depth, cmap='viridis', vmin=0, vmax=5)
        axes[1].set_title(f'Depth - t={ts:.3f}')
        axes[1].axis('off')

        fig.suptitle(f"Frame {idx} / {total_frames-1} (Use left/right arrows, Q to quit)")
        fig.canvas.draw()

    def on_key(event):
        nonlocal current_idx
        if event.key == 'right':
            current_idx = min(current_idx + 1, total_frames - 1)
            update_display(current_idx)
        elif event.key == 'left':
            current_idx = max(current_idx - 1, 0)
            update_display(current_idx)
        elif event.key == 'q':
            plt.close(fig)

    fig.canvas.mpl_connect('key_press_event', on_key)
    update_display(current_idx)
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='RGB-D データセットの可視化',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
例:
    # TUM RGB-D データセットの可視化
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk

    # 特定フレームを表示
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --frame 50

    # インタラクティブビューアー
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode interactive

    # 点群のみ表示
    python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode pointcloud
        """
    )
    parser.add_argument('--dataset', type=str, required=True,
                        choices=['tum', 'eth3d', 'vector', 'robocup', 'voxblox'],
                        help='データセットの種類')
    parser.add_argument('--path', type=str, required=True,
                        help='データセットのパス')
    parser.add_argument('--frame', type=int, default=0,
                        help='表示するフレーム番号 (default: 0)')
    parser.add_argument('--mode', type=str, default='all',
                        choices=['all', '2d', 'pointcloud', 'interactive'],
                        help='可視化モード (default: all)')

    args = parser.parse_args()

    # データセットの読み込み
    if args.dataset == 'tum':
        loader = TUMRGBDLoader(args.path)
        print(f"Loaded TUM RGB-D dataset: {len(loader)} frames")
    else:
        print(f"Dataset type '{args.dataset}' is not yet fully implemented.")
        print("Currently, only TUM RGB-D dataset is supported for image-based loading.")
        print("For ROSbag-based datasets (ETH3D, VECtor, RoboCup, Voxblox),")
        print("please refer to the README.md in each dataset folder for loading examples.")
        sys.exit(1)

    if len(loader) == 0:
        print("Error: No frames found in the dataset.")
        print("Make sure you have downloaded and extracted the dataset.")
        sys.exit(1)

    camera_params = loader.get_camera_params()

    # 可視化モードに応じた処理
    if args.mode == 'interactive':
        interactive_viewer(loader)
    else:
        rgb, depth, ts = loader[args.frame]
        print(f"Frame {args.frame}, timestamp: {ts:.3f}")

        if args.mode == '2d':
            visualize_rgbd_frame(rgb, depth, f"Frame {args.frame}")
        elif args.mode == 'pointcloud':
            points, colors = depth_to_pointcloud(depth, rgb, camera_params)
            visualize_pointcloud(points, colors)
        else:  # all
            visualize_robot_view(rgb, depth, camera_params, show_pointcloud=True)


if __name__ == '__main__':
    main()
