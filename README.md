# koike_robo_sandbox

HSRロボットの平面検出タスク検証用サンドボックス環境

## 目的

- 床面とテーブル/棚の面が平行であるという仮定のもと、物体が置かれている平面を検出
- HSRの頭部RGB-Dカメラ（Xtion相当）からの深度画像を使用した把持タスクへの活用
- 実機を使用せずにアルゴリズムの検証を行う

## 動作環境

- ROS2 Humble
- Python 3.10
- Ubuntu 22.04

## ディレクトリ構成

```
koike_robo_sandbox/
├── datasets/                    # RGB-Dデータセット格納用
│   ├── tum_rgbd/               # TUM RGB-D Dataset
│   ├── eth3d/                  # ETH3D SLAM Dataset
│   ├── vector/                 # VECtor Benchmark
│   ├── robocup/                # RoboCup 2023-2024 Dataset
│   ├── voxblox/                # Voxblox Dataset
│   └── README.md               # データセット概要
├── rgbd_visualizer/             # ROS2可視化パッケージ
│   ├── package.xml
│   ├── setup.py
│   ├── rgbd_visualizer/        # Pythonモジュール
│   │   ├── bag_publisher.py    # データセット→ROSトピック変換
│   │   ├── rgbd_viewer.py      # OpenCV可視化
│   │   └── pointcloud_generator.py  # 点群生成
│   ├── launch/                 # Launchファイル
│   └── README.md
└── README.md                    # このファイル
```

## セットアップ

### 1. 依存パッケージのインストール

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-tf2-ros
```

### 2. ROS2パッケージのビルド

```bash
# ワークスペースにシンボリックリンクを作成
cd ~/ros2_ws/src
ln -s /path/to/koike_robo_sandbox/rgbd_visualizer .

# ビルド
cd ~/ros2_ws
colcon build --packages-select rgbd_visualizer
source install/setup.bash
```

## クイックスタート

### 1. データセットのダウンロード（例: TUM RGB-D）

```bash
cd datasets/tum_rgbd
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz
tar xzf rgbd_dataset_freiburg1_desk.tgz
```

### 2. データの可視化

```bash
# 全ノードを起動（OpenCVビューアー + 点群生成）
ros2 launch rgbd_visualizer visualize_tum.launch.py \
    dataset_path:=$HOME/koike_robo_sandbox/datasets/tum_rgbd/rgbd_dataset_freiburg1_desk

# RViz2で可視化
ros2 launch rgbd_visualizer rviz_visualize.launch.py \
    dataset_path:=$HOME/koike_robo_sandbox/datasets/tum_rgbd/rgbd_dataset_freiburg1_desk
```

### 3. 個別ノードの起動

```bash
# Terminal 1: データセットをパブリッシュ
ros2 run rgbd_visualizer bag_publisher --ros-args \
    -p dataset_path:=/path/to/rgbd_dataset_freiburg1_desk

# Terminal 2: OpenCVで可視化
ros2 run rgbd_visualizer rgbd_viewer

# Terminal 3: 点群生成
ros2 run rgbd_visualizer pointcloud_generator

# Terminal 4: RViz2
rviz2
```

## ROSトピック

| トピック | 型 | 説明 |
|---------|------|------|
| `camera/color/image_raw` | sensor_msgs/Image | RGB画像 |
| `camera/depth/image_raw` | sensor_msgs/Image | 深度画像 (16UC1) |
| `camera/camera_info` | sensor_msgs/CameraInfo | カメラ情報 |
| `camera/points` | sensor_msgs/PointCloud2 | 色付き点群 |

## 利用可能なデータセット

| データセット | 特徴 | おすすめ用途 |
|-------------|------|-------------|
| TUM RGB-D | 使いやすい、豊富なシーケンス | 手軽にテストしたい |
| ETH3D | 9軸IMU、高品質キャリブレーション | IMU補正込みでテストしたい |
| VECtor | マルチセンサ（LiDAR、イベントカメラ） | センサ融合のテスト |
| RoboCup 2023-2024 | TIAGoロボット、実競技データ | RoboCup@Home環境に近いデータ |
| Voxblox | Vicon姿勢（サブmm精度） | 高精度Ground Truthが必要 |

詳細は `datasets/README.md` および各サブディレクトリの `README.md` を参照してください。

## RViz2での可視化設定

1. **Fixed Frame**: `camera_depth_optical_frame`
2. **Add → Image**: トピック `/camera/color/image_raw`
3. **Add → Image**: トピック `/camera/depth/image_raw`
4. **Add → PointCloud2**: トピック `/camera/points`
   - Color Transformer: `RGB8`
   - Size: `0.01`
