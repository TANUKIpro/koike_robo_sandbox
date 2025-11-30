# RoboCup 2023-2024 ROSbag Dataset

**RoboCup@Home の開発に最も近い環境のデータセット！**

TIAGoロボット（HSRと類似の構成）にRGB-Dカメラ、Hokuyoレーザー、マイクを搭載し、
RoboCup 2023-2024競技中に収集されたデータセットです。

## 概要

| 項目 | 内容 |
|------|------|
| URL | https://zenodo.org/record/13838208 |
| 形式 | ROS2 bag |
| 内容 | RGB-D（Astra）、レーザー、TF、音声 |
| ロボット | TIAGo（HSRと類似の構成） |
| サイズ | 260GB以上（全データ） |

## 含まれるタスク

- マッピング
- 人追従（Person Following）
- 物体操作（Storing Groceries等）
- ナビゲーション

## データ構造

```
robocup/
├── storing_try_2/                    # Storing Groceries タスク
│   ├── metadata.yaml                 # バッグメタデータ
│   ├── rosbag2_*.db3                 # ROS2 bagデータ
│   ├── behavior_server_*.log         # NAV2 behavior serverログ
│   ├── bt_navigator_*.log            # Behavior Tree navigatorログ
│   ├── controller_server_*.log       # Controller serverログ
│   ├── planner_server_*.log          # Planner serverログ
│   └── launch.log                    # Launch ログ
├── mapping_2023/
│   └── ...
└── README.md
```

## ROSトピック（storing_try_2の例）

| トピック | 型 | メッセージ数 | 説明 |
|---------|----|-----------:|------|
| `/head_front_camera/rgb/image_raw` | sensor_msgs/msg/Image | 2530 | RGB画像 |
| `/head_front_camera/rgb/camera_info` | sensor_msgs/msg/CameraInfo | 2530 | カメラ内部パラメータ |
| `/head_front_camera/depth/image_raw` | sensor_msgs/msg/Image | 2526 | 深度画像 |
| `/scan` | sensor_msgs/msg/LaserScan | 1267 | 2D LiDAR |
| `/scan_raw` | sensor_msgs/msg/LaserScan | 1267 | 2D LiDAR（フィルタなし） |
| `/tf` | tf2_msgs/msg/TFMessage | 7833 | 座標変換 |
| `/tf_static` | tf2_msgs/msg/TFMessage | 7 | 静的座標変換 |
| `/joint_states` | sensor_msgs/msg/JointState | 8446 | 関節状態 |
| `/map` | nav_msgs/msg/OccupancyGrid | 1 | 占有格子地図 |
| `/robot_description` | std_msgs/msg/String | 1 | URDFモデル |
| `/cmd_vel` | geometry_msgs/msg/Twist | 595 | 速度指令 |
| `/say_text` | std_msgs/msg/String | 1 | 音声テキスト |

## RViz2での可視化（推奨）

`rgbd_visualizer`パッケージの専用launchファイルを使用すると、rosbag再生とRViz2可視化を同時に起動できます。

### ビルド

```bash
cd ~/workspace/koike_robo_sandbox  # ワークスペースディレクトリ
colcon build --packages-select rgbd_visualizer
source install/setup.bash
```

### 起動方法

```bash
# 基本的な使用方法
ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2

# ループ再生
ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2 \
    loop:=true

# 0.5倍速で再生
ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2 \
    rate:=0.5

# 10秒後から再生開始
ros2 launch rgbd_visualizer rosbag_rviz.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2 \
    start_offset:=10.0
```

### Launchオプション

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `bag_path` | (必須) | rosbagディレクトリのパス |
| `rate` | 1.0 | 再生速度の倍率 |
| `loop` | false | ループ再生するかどうか |
| `start_offset` | 0.0 | 再生開始位置（秒） |
| `use_sim_time` | true | シミュレーション時間を使用 |
| `rviz_config` | (デフォルト設定) | カスタムRViz設定ファイル |

### RViz2で表示される内容

- **RGB Image**: `/head_front_camera/rgb/image_raw`
- **Depth Image**: `/head_front_camera/depth/image_raw`
- **TF**: 座標変換ツリー
- **LaserScan**: `/scan`
- **Map**: `/map`

## 手動でのROS2再生

```bash
# ROS2環境で再生
ros2 bag play datasets/robocup/storing_try_2/

# 特定のトピックのみ再生
ros2 bag play datasets/robocup/storing_try_2/ \
    --topics /head_front_camera/rgb/image_raw /head_front_camera/depth/image_raw

# シミュレーション時間を有効にして再生
ros2 bag play datasets/robocup/storing_try_2/ --clock

# 別ターミナルでRVizを起動
rviz2
```

## ダウンロード方法

### Zenodoからダウンロード

```bash
cd datasets/robocup

# zenodo-get を使用（推奨）
pip install zenodo-get
zenodo_get 13838208

# または個別ファイルをブラウザからダウンロード
# https://zenodo.org/record/13838208
```

### 容量が大きいため、必要なシーケンスのみダウンロード推奨

1. https://zenodo.org/record/13838208 にアクセス
2. 必要なタスクのbagファイルを選択してダウンロード
3. このディレクトリに配置

## TIAGoロボットのカメラパラメータ（Astra Pro）

```python
# Orbbec Astra Pro（参考値、実際はCameraInfoトピックから取得）
fx = 570.3
fy = 570.3
cx = 319.5
cy = 239.5
depth_scale = 1000.0  # mm -> m
```

## Pythonでの読み込み（rosbags使用）

```python
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np

def read_robocup_bag(bag_path):
    """RoboCup ROS2 bagからRGB-Dデータを読み込む"""
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if 'rgb/image_raw' in connection.topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                yield 'rgb', timestamp, rgb

            elif 'depth/image_raw' in connection.topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                # 深度画像のエンコーディングを確認
                if '16UC1' in msg.encoding:
                    depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                    depth = depth.astype(np.float32) / 1000.0  # mm -> m
                elif '32FC1' in msg.encoding:
                    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                yield 'depth', timestamp, depth

            elif 'camera_info' in connection.topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                camera_info = {
                    'fx': msg.k[0],
                    'fy': msg.k[4],
                    'cx': msg.k[2],
                    'cy': msg.k[5],
                    'width': msg.width,
                    'height': msg.height
                }
                yield 'camera_info', timestamp, camera_info

# TFの読み込み
def read_tf(bag_path):
    """TFデータを読み込む"""
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic in ['/tf', '/tf_static']:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                for transform in msg.transforms:
                    yield {
                        'timestamp': timestamp,
                        'parent': transform.header.frame_id,
                        'child': transform.child_frame_id,
                        'translation': [
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        ],
                        'rotation': [
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w
                        ]
                    }
```

## HSRとTIAGoの比較

| 項目 | HSR | TIAGo |
|------|-----|-------|
| RGB-Dカメラ | Xtion PRO LIVE | Orbbec Astra |
| 解像度 | 640x480 | 640x480 |
| 深度範囲 | 0.8-3.5m | 0.6-8m |
| LiDAR | Hokuyo UST-20LX | Hokuyo URG-04LX |

## 参考リンク

- [Zenodo Dataset](https://zenodo.org/record/13838208)
- [TIAGo Robot](https://pal-robotics.com/robots/tiago/)
- [RoboCup@Home](https://athome.robocup.org/)
- [Gentlebots Team](https://gentlebots.github.io/) - データセット提供チーム
