# Voxblox Dataset

高精度なVicon姿勢データ付きのデータセット。Ground Truthが非常に正確で、
アルゴリズムの精度評価に最適。

## 概要

| 項目 | 内容 |
|------|------|
| URL | https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017 |
| 形式 | ROS bag |
| 内容 | XYZRGB点群、Vicon姿勢（高精度） |
| サイズ | 4.5GB（圧縮） |

## 特徴

- **Vicon Motion Capture**: サブミリメートル精度のGround Truth姿勢
- **XYZRGB点群**: カラー付き点群データ
- **小規模**: 手軽に試せるサイズ

## データ構造

```
voxblox/
├── voxblox_dataset.bag         # メインROSbag
├── cow_and_lady.bag            # cow_and_ladyシーケンス
└── README.md
```

## ROSbagトピック

```
/camera/depth_registered/points       # XYZRGB点群 (sensor_msgs/PointCloud2)
/kinect/vrpn_client/estimated_transform  # Vicon姿勢 (geometry_msgs/TransformStamped)
/tf                                    # 座標変換
```

## ダウンロード方法

```bash
cd datasets/voxblox

# cow_and_lady シーケンス（推奨）
wget http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag

# 別名で保存する場合
wget -O cow_and_lady.bag http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag
```

## カメラパラメータ

```python
# Kinect v2（参考値）
fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5
```

## Pythonでの読み込み

```python
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import numpy as np
import struct

def read_voxblox_pointcloud(bag_path):
    """Voxblox ROSbagから点群データを読み込む"""
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/camera/depth_registered/points':
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)

                # PointCloud2からXYZRGBを抽出
                points = parse_pointcloud2(msg)
                yield timestamp, points

def parse_pointcloud2(msg):
    """PointCloud2メッセージをNumPy配列に変換"""
    # フィールド情報を取得
    field_names = [f.name for f in msg.fields]

    # ポイントデータの読み込み
    point_step = msg.point_step
    row_step = msg.row_step
    data = np.frombuffer(msg.data, dtype=np.uint8)

    # XYZRGBの場合
    points = []
    for i in range(msg.width * msg.height):
        offset = i * point_step
        x = struct.unpack('f', data[offset:offset+4])[0]
        y = struct.unpack('f', data[offset+4:offset+8])[0]
        z = struct.unpack('f', data[offset+8:offset+12])[0]

        # RGBは通常12-15バイト目（パックされている）
        rgb_packed = struct.unpack('I', data[offset+16:offset+20])[0]
        r = (rgb_packed >> 16) & 0xFF
        g = (rgb_packed >> 8) & 0xFF
        b = rgb_packed & 0xFF

        if not np.isnan(x):
            points.append([x, y, z, r, g, b])

    return np.array(points)

def read_vicon_pose(bag_path):
    """Vicon姿勢データを読み込む"""
    poses = []
    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if 'vrpn_client' in connection.topic:
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                pose = {
                    'timestamp': timestamp,
                    'position': [
                        msg.transform.translation.x,
                        msg.transform.translation.y,
                        msg.transform.translation.z
                    ],
                    'orientation': [
                        msg.transform.rotation.x,
                        msg.transform.rotation.y,
                        msg.transform.rotation.z,
                        msg.transform.rotation.w
                    ]
                }
                poses.append(pose)
    return poses
```

## Open3Dでの点群可視化

```python
import open3d as o3d
import numpy as np

def visualize_pointcloud(points):
    """点群を可視化"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    if points.shape[1] >= 6:
        colors = points[:, 3:6] / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd])
```

## ROS1 → ROS2 変換

```bash
pip install rosbags

rosbags-convert data.bag --dst voxblox_ros2/
```

## 参考リンク

- [Voxblox Project](https://github.com/ethz-asl/voxblox)
- [Dataset Page](https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017)
- [論文](https://arxiv.org/abs/1611.03631)
