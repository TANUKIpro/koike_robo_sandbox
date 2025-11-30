# koike_robo_sandbox

HSRロボットの平面検出タスク検証用サンドボックス環境

## 目的

- 床面とテーブル/棚の面が平行であるという仮定のもと、物体が置かれている平面を検出
- HSRの頭部RGB-Dカメラ（Xtion相当）からの深度画像を使用した把持タスクへの活用
- 実機を使用せずにアルゴリズムの検証を行う

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
├── scripts/                     # 可視化・処理スクリプト
│   ├── visualize_rgbd.py       # TUM形式データの可視化
│   ├── visualize_rosbag.py     # ROSbag形式データの可視化
│   └── README.md               # スクリプトの使い方
├── requirements.txt             # Pythonパッケージ依存関係
└── README.md                    # このファイル
```

## セットアップ

```bash
# 依存パッケージのインストール
pip install -r requirements.txt

# または個別にインストール
pip install numpy opencv-python matplotlib open3d rosbags
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
cd scripts

# RGB-Dフレームを表示
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk

# インタラクティブビューアー（矢印キーでフレーム移動）
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode interactive

# 点群表示
python visualize_rgbd.py --dataset tum --path ../datasets/tum_rgbd/rgbd_dataset_freiburg1_desk --mode pointcloud
```

## 利用可能なデータセット

| データセット | 特徴 | おすすめ用途 |
|-------------|------|-------------|
| TUM RGB-D | 使いやすい、豊富なシーケンス | 手軽にテストしたい |
| ETH3D | 9軸IMU、高品質キャリブレーション | IMU補正込みでテストしたい |
| VECtor | マルチセンサ（LiDAR、イベントカメラ） | センサ融合のテスト |
| RoboCup 2023-2024 | TIAGoロボット、実競技データ | RoboCup@Home環境に近いデータ |
| Voxblox | Vicon姿勢（サブmm精度） | 高精度Ground Truthが必要 |

詳細は `datasets/README.md` および各サブディレクトリの `README.md` を参照してください。

## ROS1 → ROS2 変換

TUM等のROS1 bagを使う場合:

```bash
pip install rosbags
rosbags-convert input.bag --dst output_ros2/
```
