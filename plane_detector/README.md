# Plane Detector - RoboCup@Home向け水平平面検出

RGB-Dカメラからの点群データを使用して、RANSAC + 法線ベクトル制約で水平平面（床面、テーブル、棚）を検出し、可視化するROS2パッケージです。

## 特徴

- **RANSAC平面検出**: Open3Dを使用した高速な平面検出
- **法線ベクトル制約**: 重力方向（TFから取得）と平行な平面のみを検出
- **高さベース分類**: 床面 / テーブル / 棚 を高さで自動分類
- **リアルタイム可視化**: RViz2でのMarkerArray表示 + RGB画像へのオーバーレイ

## 依存パッケージ

```bash
# ROS2依存
sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-message-filters

# Python依存
pip install open3d numpy opencv-python
```

## インストール

```bash
# ワークスペースにクローン
cd ~/ros2_ws/src
ln -s /path/to/koike_robo_sandbox/plane_detector .

# ビルド
cd ~/ros2_ws
colcon build --packages-select plane_detector
source install/setup.bash
```

## 使用方法

### storing_try_2データセットでの実行

```bash
# 基本的な使用方法
ros2 launch plane_detector plane_detection.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2

# ループ再生
ros2 launch plane_detector plane_detection.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2 \
    loop:=true

# 0.5倍速で再生
ros2 launch plane_detector plane_detection.launch.py \
    bag_path:=/path/to/datasets/robocup/storing_try_2 \
    rate:=0.5
```

### 単体でノードを起動

```bash
# rosbagを別ターミナルで再生
ros2 bag play /path/to/datasets/robocup/storing_try_2 --clock

# 平面検出ノードを起動
ros2 run plane_detector plane_detector_node --ros-args \
    -p use_sim_time:=true \
    -p depth_scale:=1000.0
```

## Launch引数

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `bag_path` | (必須) | rosbagディレクトリのパス |
| `rate` | 1.0 | 再生速度の倍率 |
| `loop` | false | ループ再生するかどうか |
| `start_offset` | 0.0 | 再生開始位置（秒） |
| `process_rate` | 5.0 | 平面検出の処理レート (Hz) |
| `normal_threshold_deg` | 15.0 | 水平と見なす許容角度（度） |

## ノードパラメータ

### トピック設定

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `rgb_topic` | `/head_front_camera/rgb/image_raw` | RGB画像トピック |
| `depth_topic` | `/head_front_camera/depth/image_raw` | 深度画像トピック |
| `camera_info_topic` | `/head_front_camera/rgb/camera_info` | カメラ情報トピック |

### 座標系設定

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `camera_frame` | `head_front_camera_rgb_optical_frame` | カメラ座標系 |
| `base_frame` | `base_footprint` | ロボット基準座標系 |

### RANSAC設定

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `distance_threshold` | 0.02 | RANSAC距離閾値 (m) |
| `min_plane_points` | 500 | 平面と認識する最小点数 |
| `normal_threshold_deg` | 15.0 | 水平と見なす許容角度（度） |

### 高さによる分類

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `floor_height_max` | 0.15 | 床と見なす最大高さ (m) |
| `table_height_min` | 0.40 | テーブルの最小高さ (m) |
| `table_height_max` | 1.20 | テーブルの最大高さ (m) |

## 出力トピック

| トピック | 型 | 説明 |
|---------|----|----|
| `/detected_planes/markers` | `visualization_msgs/MarkerArray` | 検出平面のRViz2マーカー |
| `/detected_planes/overlay` | `sensor_msgs/Image` | 平面オーバーレイ画像 |
| `/detected_table_surface` | `geometry_msgs/PoseStamped` | 検出したテーブル面の位置・姿勢 |

## 可視化

### RViz2での表示

launchファイルを使用すると、自動的にRViz2が起動し、以下が表示されます：

- **DetectedPlanesOverlay**: 平面検出結果がオーバーレイされたRGB画像
- **DetectedPlanesMarkers**: 検出された平面の3Dマーカー（半透明の板）
- **TF**: 座標変換ツリー
- **LaserScan**: 2D LiDARスキャン

### 平面の色分け

| 平面タイプ | 色 | 高さ範囲 |
|-----------|----|----|
| 床面 (FLOOR) | 灰色 | < 0.15m |
| テーブル (TABLE) | 緑色 | 0.40m - 1.20m |
| 棚 (SHELF) | 青色 | > 1.20m |
| 不明 (UNKNOWN) | 黄色 | その他 |

## アルゴリズム

1. **RGB-D画像取得**: `/head_front_camera/*` トピックから同期取得
2. **TF取得**: `base_footprint` → `camera_frame` の変換を取得
3. **点群生成**: 深度画像とカメラパラメータから3D点群を生成
4. **座標変換**: カメラ座標系から `base_footprint` 座標系へ変換
5. **RANSAC平面検出**: Open3Dで法線制約付きRANSAC
6. **法線フィルタリング**: 重力方向と平行な平面のみ採用
7. **高さ分類**: `base_footprint` からの高さで床/テーブル/棚を分類
8. **可視化**: MarkerArray + 画像オーバーレイを公開

## データセット

### storing_try_2のダウンロード

```bash
cd /path/to/datasets/robocup

# Zenodoから直接ダウンロード
# https://zenodo.org/record/13838208 にアクセスして
# storing_try_2 関連ファイルをダウンロード

# または zenodo-get を使用（Pythonパッケージ）
pip install zenodo-get
zenodo_get 13838208
```

### データ形式

storing_try_2は以下のトピックを含むROS2 bagファイルです：

| トピック | 型 | 説明 |
|---------|----|----|
| `/head_front_camera/rgb/image_raw` | Image | RGB画像 (640x480) |
| `/head_front_camera/depth/image_raw` | Image | 深度画像 (16UC1, mm) |
| `/head_front_camera/rgb/camera_info` | CameraInfo | カメラパラメータ |
| `/tf`, `/tf_static` | TFMessage | 座標変換 |

## 参考

- [plane_detection_approach.md](../plane_detection_approach.md): 設計ドキュメント
- [Open3D RANSAC](http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html#Plane-segmentation): Open3D平面検出
- [RoboCup Dataset](https://zenodo.org/record/13838208): storing_try_2データセット
