# Robot Datasets for Plane Detection

このディレクトリには、平面検出タスクの検証用RGB-Dデータセットを保存します。

## 目的

- 床面とテーブル/棚の面が平行であるという仮定のもと、物体が置かれている平面を検出
- HSRの頭部RGB-Dカメラ（Xtion相当）からの深度画像を使用した把持タスクへの活用

## データセット一覧

| フォルダ | データセット名 | 特徴 | おすすめ用途 |
|----------|---------------|------|-------------|
| `tum_rgbd/` | TUM RGB-D Dataset | 最も有名・使いやすい | 手軽にテストしたい |
| `eth3d/` | ETH3D SLAM Dataset | IMU + RGB-D + Ground Truth | IMU補正込みでテストしたい |
| `vector/` | VECtor Benchmark | マルチセンサ（LiDAR, イベントカメラ等） | IMU補正込みでテストしたい |
| `robocup/` | RoboCup 2023-2024 Dataset | TIAGoロボット、RoboCup@Home環境 | RoboCup環境に近いデータ |
| `voxblox/` | Voxblox Dataset | Vicon姿勢（高精度Ground Truth） | 高精度Ground Truthが欲しい |

## ROS1 bag → ROS2 変換

TUM等のROS1 bagを使う場合、変換が必要です:

```bash
# rosbags パッケージをインストール
pip install rosbags --break-system-packages

# ROS1 bag → ROS2 bag 変換
rosbags-convert input.bag --dst output_ros2/
```

## 必要なPythonパッケージ

```bash
pip install numpy opencv-python open3d matplotlib rosbags
```

## 注意事項

- データセットファイル（`.bag`, `.tgz`, 画像フォルダ等）は `.gitignore` に追加されています
- 各データセットの詳細なダウンロード方法は、各サブディレクトリの `README.md` を参照してください
