# VECtor Benchmark

マルチセンサデータセット。RGB-D、LiDAR、IMU、イベントカメラなど多様なセンサデータを提供。

## 概要

| 項目 | 内容 |
|------|------|
| URL | https://star-datasets.github.io/vector/download/ |
| 形式 | ROS bag + テキストファイル |
| 内容 | RGB-D、LiDAR、IMU（9軸）、イベントカメラ、Ground Truth |
| 特徴 | マルチセンサ統合環境 |

## センサ構成

- **RGB-Dカメラ**: Intel RealSense D435i
- **LiDAR**: Ouster OS1-64
- **IMU**: D435i内蔵 + Ouster内蔵
- **イベントカメラ**: Prophesee Gen3
- **Ground Truth**: Leica Total Station

## データ構造

```
vector/
├── corridors/
│   ├── corridors.bag           # ROSbagデータ
│   ├── groundtruth.txt         # Ground Truth姿勢
│   └── calibration/
│       ├── camera_intrinsics.yaml
│       └── extrinsics.yaml
├── units/
│   └── ...
└── README.md
```

## ROSbagトピック

```
/d435i/color/image_raw          # RGB画像 (sensor_msgs/Image)
/d435i/depth/image_rect_raw     # 深度画像 (sensor_msgs/Image)
/d435i/imu                      # D435i IMU (sensor_msgs/Imu)
/ouster/points                  # LiDAR点群 (sensor_msgs/PointCloud2)
/ouster/imu                     # Ouster IMU (sensor_msgs/Imu)
/prophesee/events               # イベントカメラ
/gt/pose                        # Ground Truth姿勢
```

## ダウンロード方法

```bash
cd datasets/vector

# ダウンロードページからファイルを取得
# https://star-datasets.github.io/vector/download/

# 例: corridorsシーケンス
wget https://star-datasets.github.io/vector/data/corridors.bag

# Ground Truthと calibration
wget https://star-datasets.github.io/vector/data/corridors_gt.txt
```

## カメラパラメータ（D435i）

```python
# Intel RealSense D435i
fx = 386.738
fy = 386.738
cx = 321.281
cy = 238.221
depth_scale = 1000.0  # mm -> m
```

## Pythonでの読み込み

```python
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import numpy as np

def read_vector_bag(bag_path):
    """VECtor ROSbagからRGB-Dデータを読み込む"""
    rgb_data = []
    depth_data = []
    imu_data = []

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/d435i/color/image_raw':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                rgb_data.append((timestamp, rgb))

            elif connection.topic == '/d435i/depth/image_rect_raw':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth = depth.astype(np.float32) / 1000.0  # mm -> m
                depth_data.append((timestamp, depth))

            elif connection.topic == '/d435i/imu':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                imu = {
                    'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                    'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
                }
                imu_data.append((timestamp, imu))

    return rgb_data, depth_data, imu_data

# LiDAR点群の読み込み
def read_lidar_points(bag_path):
    """LiDAR点群を読み込む"""
    import struct

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/ouster/points':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                # PointCloud2形式の解析
                # 実際の実装はpoint_cloud2ライブラリを使用推奨
                yield timestamp, msg
```

## ROS1 → ROS2 変換

```bash
pip install rosbags

rosbags-convert corridors.bag --dst corridors_ros2/
```

## 参考リンク

- [VECtor Benchmark 公式サイト](https://star-datasets.github.io/vector/)
- [GitHub](https://github.com/star-datasets/vector)
- [論文](https://arxiv.org/abs/2207.01404)
