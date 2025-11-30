# rgbd_visualizer

ROS2 Humble用のRGB-Dデータセット可視化パッケージ

## 概要

TUM RGB-Dなどのデータセットを読み込み、ROSトピックとしてパブリッシュします。
RViz2やOpenCVウィンドウで可視化でき、平面検出アルゴリズムの開発に活用できます。

## 依存パッケージ

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-tf2-ros
```

## ビルド

```bash
cd ~/ros2_ws/src
ln -s /path/to/koike_robo_sandbox/rgbd_visualizer .

cd ~/ros2_ws
colcon build --packages-select rgbd_visualizer
source install/setup.bash
```

## ノード一覧

### 1. bag_publisher

TUM RGB-Dデータセットを読み込み、ROSトピックとしてパブリッシュします。

**パブリッシュトピック:**
- `camera/color/image_raw` (sensor_msgs/Image): RGB画像
- `camera/depth/image_raw` (sensor_msgs/Image): 深度画像 (16UC1)
- `camera/camera_info` (sensor_msgs/CameraInfo): カメラ情報

**パラメータ:**
- `dataset_path` (string): データセットのパス（必須）
- `rate` (double): パブリッシュレート [Hz]（デフォルト: 30.0）
- `loop` (bool): ループ再生（デフォルト: true）

**使用例:**
```bash
ros2 run rgbd_visualizer bag_publisher --ros-args \
    -p dataset_path:=/path/to/rgbd_dataset_freiburg1_desk \
    -p rate:=30.0
```

### 2. rgbd_viewer

RGB-D画像をOpenCVウィンドウで可視化します。

**サブスクライブトピック:**
- `camera/color/image_raw` (sensor_msgs/Image): RGB画像
- `camera/depth/image_raw` (sensor_msgs/Image): 深度画像

**パラメータ:**
- `depth_scale` (double): 深度スケール（デフォルト: 5000.0）
- `max_depth` (double): 表示用最大深度 [m]（デフォルト: 5.0）

**キーボードショートカット:**
- `Q`: 終了
- `S`: 現在のフレームを保存

**使用例:**
```bash
ros2 run rgbd_visualizer rgbd_viewer
```

### 3. pointcloud_generator

深度画像から色付き点群を生成します。

**サブスクライブトピック:**
- `camera/color/image_raw` (sensor_msgs/Image): RGB画像
- `camera/depth/image_raw` (sensor_msgs/Image): 深度画像
- `camera/camera_info` (sensor_msgs/CameraInfo): カメラ情報

**パブリッシュトピック:**
- `camera/points` (sensor_msgs/PointCloud2): 色付き点群

**パラメータ:**
- `depth_scale` (double): 深度スケール（デフォルト: 5000.0）
- `max_depth` (double): 最大深度 [m]（デフォルト: 10.0）
- `min_depth` (double): 最小深度 [m]（デフォルト: 0.1）
- `downsample` (int): ダウンサンプリング係数（デフォルト: 1）

**使用例:**
```bash
ros2 run rgbd_visualizer pointcloud_generator --ros-args \
    -p downsample:=2
```

## Launch ファイル

### visualize_tum.launch.py

TUM RGB-Dデータセットを可視化（全ノード起動）

```bash
ros2 launch rgbd_visualizer visualize_tum.launch.py \
    dataset_path:=/path/to/rgbd_dataset_freiburg1_desk
```

### rviz_visualize.launch.py

RViz2で点群を可視化

```bash
ros2 launch rgbd_visualizer rviz_visualize.launch.py \
    dataset_path:=/path/to/rgbd_dataset_freiburg1_desk
```

## クイックスタート

```bash
# 1. TUM RGB-Dデータセットをダウンロード
cd ~/koike_robo_sandbox/datasets/tum_rgbd
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz
tar xzf rgbd_dataset_freiburg1_desk.tgz

# 2. パッケージをビルド
cd ~/ros2_ws
colcon build --packages-select rgbd_visualizer
source install/setup.bash

# 3. 可視化を起動
ros2 launch rgbd_visualizer visualize_tum.launch.py \
    dataset_path:=$HOME/koike_robo_sandbox/datasets/tum_rgbd/rgbd_dataset_freiburg1_desk
```

## RViz2での可視化

RViz2で以下を追加:
1. **Image**: トピック `/camera/color/image_raw`
2. **Image**: トピック `/camera/depth/image_raw`
3. **PointCloud2**: トピック `/camera/points`
   - Style: `RGB8`
   - Size: `0.01`

## データセット別の設定

| データセット | depth_scale | 備考 |
|-------------|-------------|------|
| TUM RGB-D (freiburg1/2/3) | 5000.0 | デフォルト設定 |
| ETH3D | 1000.0 | mm単位 |
| RoboCup (Astra) | 1000.0 | mm単位 |
