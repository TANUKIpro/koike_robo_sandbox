# ETH3D SLAM Dataset

高品質なIMUデータとRGB-D、および正確なGround Truthを提供するデータセット。
カメラ-IMU間の外部キャリブレーション付きで、IMU補正込みのテストに最適。

## 概要

| 項目 | 内容 |
|------|------|
| URL | https://www.eth3d.net/slam_documentation |
| 形式 | ROS bag（圧縮） |
| 内容 | RGB、Depth、IMU（9軸）、Ground Truth姿勢 |
| 特徴 | カメラ-IMU間の外部キャリブレーション付き |

## データ構造

```
eth3d/
├── cables_1/
│   ├── raw_data.bag              # 圧縮されたrosbag（カメラ、IMU、GT）
│   ├── calibration/
│   │   ├── camera_intrinsics.txt # カメラ内部パラメータ
│   │   ├── imu_intrinsics.txt    # IMUキャリブレーション
│   │   └── extrinsics.txt        # カメラ-IMU間の外部パラメータ
│   └── groundtruth.txt           # Ground Truth姿勢
├── einstein_1/
│   └── ...
└── README.md
```

## ROSbagトピック

```
/camera/color/image_raw         # RGB画像 (sensor_msgs/Image)
/camera/depth/image_raw         # 深度画像 (sensor_msgs/Image)
/imu                            # IMUデータ (sensor_msgs/Imu)
/groundtruth                    # Ground Truth姿勢
```

## ダウンロード方法

### ブラウザからダウンロード

1. https://www.eth3d.net/slam_datasets にアクセス
2. 必要なシーケンスを選択してダウンロード

### wgetでダウンロード

```bash
cd datasets/eth3d

# 注意: ETH3Dは直接ダウンロードリンクを提供していないため、
# ブラウザでダウンロードしてこのフォルダに配置してください

# ダウンロード後の展開
unzip cables_1.zip
```

## カメラパラメータ例

```python
# cables_1 シーケンスの例（実際の値はcalibrationファイルを参照）
fx = 527.0
fy = 527.0
cx = 320.0
cy = 240.0
depth_scale = 1000.0  # mm -> m
```

## ROSbagからのデータ抽出

### ROS1環境での抽出

```bash
# RGB画像の抽出
rosrun image_view extract_images _sec_per_frame:=0.0 image:=/camera/color/image_raw

# ROSbagの情報確認
rosbag info raw_data.bag
```

### Pythonでの読み込み（rosbags使用）

```python
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import numpy as np
import cv2

def read_eth3d_bag(bag_path):
    """ETH3D ROSbagからデータを読み込む"""
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/camera/color/image_raw':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                # 画像データの処理
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                yield 'rgb', timestamp, rgb

            elif connection.topic == '/camera/depth/image_raw':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                # 深度データの処理
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth = depth.astype(np.float32) / 1000.0  # mm -> m
                yield 'depth', timestamp, depth

            elif connection.topic == '/imu':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
                gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
                yield 'imu', timestamp, {'accel': accel, 'gyro': gyro}
```

## ROS1 → ROS2 変換

```bash
pip install rosbags

# 変換
rosbags-convert raw_data.bag --dst raw_data_ros2/
```

## 参考リンク

- [ETH3D SLAM Documentation](https://www.eth3d.net/slam_documentation)
- [ETH3D Datasets](https://www.eth3d.net/datasets)
- [論文](https://www.eth3d.net/data/schoeps2019cvpr.pdf)
