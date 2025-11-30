# Visualization Scripts

RGB-Dデータセットの可視化スクリプト集。ロボットの見ている光景を確認できます。

## 必要なパッケージ

```bash
pip install numpy opencv-python matplotlib open3d rosbags
```

## スクリプト一覧

### 1. visualize_rgbd.py

TUM RGB-D形式（画像ファイル）のデータセットを可視化します。

```bash
# 基本的な使用方法
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk

# 特定のフレームを表示
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --frame 100

# インタラクティブモード（矢印キーでフレーム移動）
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode interactive

# 点群のみ表示
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode pointcloud
```

### 2. visualize_rosbag.py

ROSbag形式のデータセットを可視化します。

```bash
# トピック一覧の確認
python visualize_rosbag.py --bag ../datasets/voxblox/data.bag --version ros1 --list-topics

# ROS1 bag の可視化（Voxblox, ETH3D, VECtor）
python visualize_rosbag.py --bag ../datasets/voxblox/data.bag --version ros1 --dataset voxblox

# ROS2 bag の可視化（RoboCup）
python visualize_rosbag.py --bag ../datasets/robocup/rosbag2_mapping/ --version ros2 --dataset robocup

# 点群表示
python visualize_rosbag.py --bag ../datasets/voxblox/data.bag --version ros1 --mode pointcloud
```

## 可視化モード

| モード | 説明 |
|--------|------|
| `all` | RGB、深度、点群をすべて表示 |
| `2d` | RGB画像と深度画像のみ |
| `pointcloud` | 3D点群のみ |
| `interactive` | 矢印キーでフレームを移動できるビューアー |

## キーボードショートカット（インタラクティブモード）

| キー | 動作 |
|------|------|
| ← → | フレーム移動 |
| P | 現在のフレームの点群を表示 |
| Q | 終了 |

## 出力例

```
Loaded TUM RGB-D dataset: 573 frames
Frame 0, timestamp: 1305031452.791
Image size: 640x480
Depth range: 0.732m - 5.890m
Camera params: fx=517.3, fy=516.5
Point cloud: 245632 points
```

## トラブルシューティング

### matplotlib が表示されない

```bash
# バックエンドを指定
export MPLBACKEND=TkAgg
python visualize_rgbd.py ...
```

### open3d の点群ウィンドウが開かない

```bash
# GUI環境が必要（X11転送など）
ssh -X user@server
python visualize_rgbd.py --mode pointcloud
```

### rosbags でエラーが出る

```bash
# 最新版をインストール
pip install --upgrade rosbags
```
