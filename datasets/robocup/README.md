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
- 物体操作
- ナビゲーション

## データ構造

```
robocup/
├── mapping_2023/
│   └── rosbag2_*/            # ROS2 bagディレクトリ
│       ├── metadata.yaml
│       └── *.db3
├── person_following_2024/
│   └── ...
├── manipulation_2024/
│   └── ...
└── README.md
```

## ROSトピック（予想される構成）

```
/camera/color/image_raw         # RGB画像 (sensor_msgs/msg/Image)
/camera/depth/image_raw         # 深度画像 (sensor_msgs/msg/Image)
/camera/depth/camera_info       # カメラ情報 (sensor_msgs/msg/CameraInfo)
/scan                           # 2D LiDAR (sensor_msgs/msg/LaserScan)
/tf                             # 座標変換 (tf2_msgs/msg/TFMessage)
/tf_static                      # 静的座標変換
/audio                          # 音声データ
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

## ROS2での再生

```bash
# ROS2環境で再生
ros2 bag play rosbag2_*/

# 特定のトピックのみ再生
ros2 bag play rosbag2_*/ --topics /camera/color/image_raw /camera/depth/image_raw

# RVizで可視化
rviz2
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
            if 'color/image' in connection.topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                yield 'rgb', timestamp, rgb

            elif 'depth/image' in connection.topic:
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
