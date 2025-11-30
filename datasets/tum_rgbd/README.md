# TUM RGB-D Dataset

最も有名で使いやすいRGB-Dデータセット。SLAMや平面検出のベンチマークとして広く使用されています。

## 概要

| 項目 | 内容 |
|------|------|
| URL | https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download |
| 形式 | ROS bag（ROS1）+ 画像ファイル |
| 内容 | RGB画像、Depth画像、IMU（加速度計）、Ground Truth姿勢 |
| カメラ | Microsoft Kinect |
| サイズ | 数百MB〜数GB/シーケンス |

## カメラパラメータ（Kinect v1）

```python
# Freiburg 1 シーケンス用
fx = 517.3
fy = 516.5
cx = 318.6
cy = 255.3
depth_scale = 5000.0  # 深度値をメートルに変換する係数
```

## データ構造

```
tum_rgbd/
├── rgbd_dataset_freiburg1_desk/
│   ├── rgb/                    # RGB画像（PNG）
│   │   ├── 1305031452.791720.png
│   │   └── ...
│   ├── depth/                  # 深度画像（16bit PNG）
│   │   ├── 1305031452.815237.png
│   │   └── ...
│   ├── rgb.txt                 # RGB画像のタイムスタンプとパス
│   ├── depth.txt               # 深度画像のタイムスタンプとパス
│   ├── groundtruth.txt         # Ground Truth姿勢（timestamp tx ty tz qx qy qz qw）
│   └── accelerometer.txt       # 加速度計データ（timestamp ax ay az）
├── rgbd_dataset_freiburg1_room/
│   └── ...
└── README.md
```

## ダウンロード方法

### 推奨シーケンス（平面検出向け）

```bash
cd datasets/tum_rgbd

# fr1/desk - デスク環境（平面検出に最適）
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz
tar xzf rgbd_dataset_freiburg1_desk.tgz

# fr1/room - 部屋全体（より広い環境）
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_room.tgz
tar xzf rgbd_dataset_freiburg1_room.tgz

# fr2/desk - 別のデスク環境
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz
tar xzf rgbd_dataset_freiburg2_desk.tgz

# fr3/long_office_household - オフィス環境（物体多め）
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz
tar xzf rgbd_dataset_freiburg3_long_office_household.tgz
```

### ROSbag形式（ROS1）

```bash
# ROSbagが必要な場合
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk-2hz-with-hierarchies.bag
```

## データの読み込み例（Python）

```python
import cv2
import numpy as np
from pathlib import Path

def load_tum_rgbd(dataset_path, idx=0):
    """TUM RGB-Dデータセットからフレームを読み込む"""
    dataset_path = Path(dataset_path)

    # rgb.txt から画像パスを取得
    rgb_list = []
    with open(dataset_path / 'rgb.txt') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                rgb_list.append((float(parts[0]), parts[1]))

    # depth.txt から画像パスを取得
    depth_list = []
    with open(dataset_path / 'depth.txt') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                depth_list.append((float(parts[0]), parts[1]))

    # 指定インデックスの画像を読み込み
    _, rgb_path = rgb_list[idx]
    _, depth_path = depth_list[idx]

    rgb = cv2.imread(str(dataset_path / rgb_path))
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

    depth = cv2.imread(str(dataset_path / depth_path), cv2.IMREAD_UNCHANGED)
    depth = depth.astype(np.float32) / 5000.0  # メートルに変換

    return rgb, depth
```

## RGB-Depth アソシエーション

RGBと深度画像のタイムスタンプをマッチングするツール:

```bash
# 公式アソシエーションスクリプト
wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py

python associate.py rgb.txt depth.txt > associations.txt
```

## 参考リンク

- [公式サイト](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
- [論文](https://vision.in.tum.de/_media/spezial/bib/sturm12iros.pdf)
- [ベンチマーク評価ツール](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools)
