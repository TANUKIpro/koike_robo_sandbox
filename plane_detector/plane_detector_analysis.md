# Plane Detector 実装乖離調査・改善タスク一覧

## 1. TF座標系の乖離分析

### 1.1 rosbag (storing_try_2) が提供するTF構造

`frames_2025-11-30_23.33.49.gv` から抽出したTFツリー:

```
map
├── odom
│   └── base_footprint
│       └── base_link
│           ├── torso_fixed_link
│           │   └── torso_lift_link
│           │       └── head_1_link
│           │           └── head_2_link
│           │               └── head_front_camera_link
│           │                   ├── head_front_camera_optical_frame
│           │                   ├── head_front_camera_orbbec_aux_joint_frame
│           │                   │   └── head_front_camera_rgb_frame
│           │                   │       ├── head_front_camera_depth_frame
│           │                   │       │   └── head_front_camera_depth_optical_frame
│           │                   │       └── head_front_camera_rgb_optical_frame
│           │                   ├── head_front_camera_link_color_frame
│           │                   │   └── head_front_camera_link_color_optical_frame
│           │                   └── head_front_camera_link_depth_frame
│           │                       └── head_front_camera_link_depth_optical_frame
│           └── base_laser_link
├── entrance
├── table
├── secure_pick
├── cabinet
└── secure_place
```

### 1.2 現在の実装で使用しているTFフレーム名

**plane_detector_node.py / plane_recorder_node.py:**
```python
camera_frame = 'head_front_camera_rgb_optical_frame'  # ← 問題あり
base_frame = 'base_footprint'  # ← OK
```

### 1.3 乖離点の特定

| 項目 | 現在の実装 | rosbag実際のTF | 状態 |
|------|-----------|----------------|------|
| カメラフレーム | `head_front_camera_rgb_optical_frame` | `head_front_camera_link_color_optical_frame` または `head_front_camera_depth_optical_frame` | ❌ 不一致 |
| ベースフレーム | `base_footprint` | `base_footprint` | ✅ 一致 |
| 深度画像トピック | `/head_front_camera/depth/image_raw` | `/head_front_camera/depth/image_raw` | ✅ 一致 |
| RGB画像トピック | `/head_front_camera/rgb/image_raw` | `/head_front_camera/rgb/image_raw` | ✅ 一致 |

**重要な発見:**
rosbagのTFには2系統のカメラフレームが存在:
1. **静的TF系 (rate: 10000.0)**: `head_front_camera_rgb_optical_frame`, `head_front_camera_depth_optical_frame`
2. **動的TF系 (rate: ~10-20Hz)**: `head_front_camera_link_color_optical_frame`, `head_front_camera_link_depth_optical_frame`

深度画像のフレームIDは `head_front_camera_depth_optical_frame` である可能性が高いが、
実際にrosbagから確認が必要。

---

## 2. plane_detection_approach.md との設計乖離

### 2.1 設計書で想定しているセンサ・座標系

| 項目 | 設計書 (HSR想定) | 実際のrosbag (TIAGo) |
|------|-----------------|---------------------|
| RGB-Dカメラ | Xtion PRO LIVE | Orbbec Astra |
| カメラ座標系 | `head_rgbd_sensor_rgb_frame` | `head_front_camera_*` |
| IMUトピック | `/base_imu/data` | 未確認（存在しない可能性） |
| 点群トピック | `/head_rgbd_sensor/depth/points` | なし（自前生成） |

### 2.2 アルゴリズム設計と実装の乖離

| 設計項目 | 設計書の記述 | 現在の実装 | 乖離 |
|----------|------------|-----------|------|
| **IMU補正** | IMUから重力方向を取得し、基準法線を動的更新 | TFからbase_footprintのZ軸を使用（固定） | ⚠️ IMU未使用 |
| **法線制約** | `normal_threshold_deg = 10.0°` | `normal_threshold_deg = 15.0°` | △ 緩い |
| **最小点数** | `min_plane_points = 1000` | `min_plane_points = 500` | △ 少ない |
| **処理レート** | 必要時のみ実行 | 固定レート（5Hz） | △ 非効率 |
| **平面分類** | 床/テーブル/棚の3分類 | 同じ | ✅ |

### 2.3 設計書で言及されているが未実装の機能

1. **IMUベースの重力方向補正**
   - 設計: `/base_imu/data` トピックから加速度を取得
   - 現状: 未実装（TFのZ軸を固定で使用）

2. **動的環境対応**
   - 設計: 人が通過した際の一時的な遮蔽への対処
   - 現状: 未実装

3. **物体位置との統合**
   - 設計: 平面上の物体候補領域を出力
   - 現状: 未実装

---

## 3. 現在のアルゴリズムの問題点

### 3.1 点群生成の問題

```python
# 現在の実装 (plane_detector_node.py:246-268)
def _create_pointcloud(self, depth: np.ndarray) -> np.ndarray:
    # ダウンサンプリング後の点群生成
    depth_ds = depth[::ds, ::ds]  # downsample_factor=4
```

**問題:**
- ダウンサンプリングが粗い（1/4）
- 深度有効範囲が広すぎる（0.3-5.0m）
- Open3Dの点群正規化を使用していない

### 3.2 RANSAC検出の問題

```python
# 現在の実装
plane_model, inliers = current_pcd.segment_plane(
    distance_threshold=self.distance_threshold,  # 0.02m
    ransac_n=self.ransac_n,  # 3
    num_iterations=self.num_iterations  # 1000
)
```

**問題:**
- `ransac_n=3` は最小値で不安定
- 事前に法線推定を行っていない
- 大きな平面から順に検出するため、テーブルより床が先に検出される

### 3.3 法線フィルタリングの問題

```python
# 現在の実装
gravity_direction = np.array([0.0, 0.0, 1.0])  # 固定値
angle = np.arccos(np.dot(normal, gravity_direction))
```

**問題:**
- ロボットが傾いても重力方向が更新されない
- IMUデータを活用していない

### 3.4 高さ分類の問題

```python
# 現在のパラメータ
floor_height_max = 0.15    # 低すぎる？
table_height_min = 0.40    # TIAGoのテーブル高さと合っているか？
table_height_max = 1.20
```

**問題:**
- TIAGoロボットの実際の環境に最適化されていない
- `0.15m < height < 0.40m` の範囲がUNKNOWN扱いになる

---

## 4. 実装タスク一覧

### Phase 1: TF問題の解決（優先度: 最高）

#### Task 1.1: rosbagのTF構造を実機確認
```bash
# rosbagを再生してTFを確認
ros2 bag play datasets/robocup/storing_try_2 --clock
ros2 run tf2_tools view_frames
```

**確認項目:**
- [ ] RGB画像のframe_id
- [ ] 深度画像のframe_id
- [ ] `head_front_camera_rgb_optical_frame` が存在するか
- [ ] `head_front_camera_link_color_optical_frame` との関係

#### Task 1.2: TFフレーム名の修正
**ファイル:** `plane_detector_node.py`, `plane_recorder_node.py`

```python
# 修正案
self.declare_parameter('camera_frame', 'head_front_camera_link_depth_optical_frame')
# または動的に画像メッセージのframe_idから取得
```

#### Task 1.3: TF待機ロジックの改善
```python
# 現在の実装は0.5秒タイムアウト
transform = self.tf_buffer.lookup_transform(
    self.base_frame, self.camera_frame, stamp,
    timeout=rclpy.duration.Duration(seconds=0.5)
)

# 改善案: TF利用可能まで待機するオプション追加
```

---

### Phase 2: アルゴリズム改善（優先度: 高）

#### Task 2.1: IMUベース重力方向補正の実装
**新規ファイル:** `plane_detector/gravity_estimator.py`

```python
class GravityEstimator:
    """IMUから重力方向を推定"""
    
    def __init__(self, node):
        self.imu_sub = node.create_subscription(
            Imu, '/base_imu/data', self.imu_callback, 10)
        self.gravity_direction = np.array([0.0, 0.0, 1.0])
    
    def imu_callback(self, msg):
        # 加速度から重力方向を推定
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.gravity_direction = -accel / np.linalg.norm(accel)
```

**注意:** storing_try_2にIMUトピックが存在するか要確認

#### Task 2.2: 法線推定の事前計算
```python
# 改善案: Open3Dの法線推定を活用
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)

# 法線が上向きになるよう統一
pcd.orient_normals_towards_camera_location(camera_location)
```

#### Task 2.3: RANSAC パラメータ最適化
```python
# 改善案
distance_threshold = 0.015  # 1.5cm（より厳密に）
ransac_n = 4  # 安定性向上
num_iterations = 2000  # 精度向上
```

#### Task 2.4: 平面検出順序の改善
```python
# 改善案: 高さ優先で検出
def _detect_planes_by_height(self, points_base, target_height_range):
    """指定高さ範囲の平面を優先的に検出"""
    # 1. 高さでROIフィルタリング
    mask = (points_base[:, 2] > target_height_range[0]) & \
           (points_base[:, 2] < target_height_range[1])
    roi_points = points_base[mask]
    
    # 2. ROI内でRANSAC
    ...
```

---

### Phase 3: 高さ分類の改善（優先度: 中）

#### Task 3.1: 分類閾値の見直し
```python
# TIAGo環境に最適化
floor_height_max = 0.20      # 20cm（カーペット等考慮）
low_furniture_max = 0.40     # 低いテーブル/椅子
table_height_min = 0.40
table_height_max = 0.90      # 一般的なテーブル高さ
shelf_height_min = 0.90
shelf_height_max = 1.80
```

#### Task 3.2: 新しい平面タイプの追加
```python
class PlaneType(Enum):
    UNKNOWN = 0
    FLOOR = 1
    LOW_FURNITURE = 2  # 新規
    TABLE = 3
    SHELF_LOW = 4      # 新規
    SHELF_HIGH = 5     # 新規
```

---

### Phase 4: 把持タスク向け機能追加（優先度: 中）

#### Task 4.1: 平面上の物体候補領域検出
**新規ファイル:** `plane_detector/object_region_detector.py`

```python
def detect_objects_on_plane(plane: DetectedPlane, full_pointcloud):
    """平面より上にある点群クラスタを物体候補として検出"""
    # 1. 平面より上の点を抽出
    # 2. EuclideanClusteringでセグメンテーション
    # 3. 各クラスタのバウンディングボックスを計算
    # 4. 把持可能な物体候補として返却
```

#### Task 4.2: 把持候補点の出力
```python
# 新規トピック
/detected_planes/grasp_candidates (geometry_msgs/PoseArray)
```

---

### Phase 5: パフォーマンス最適化（優先度: 低）

#### Task 5.1: 適応的処理レート
```python
def should_process(self):
    """シーンが変化した場合のみ処理"""
    # TFの変化量をチェック
    # 前回との差分が閾値以上なら処理
```

#### Task 5.2: GPU加速（オプション）
```python
# Open3D CUDA対応（要GPU環境）
# または PyTorch3D の活用
```

---

## 5. テスト計画

### 5.1 単体テスト
- [ ] TF変換の正確性
- [ ] 点群生成の正確性
- [ ] RANSAC検出の再現性
- [ ] 高さ分類の正確性

### 5.2 統合テスト
- [ ] storing_try_2 rosbagでの動作確認
- [ ] 検出された平面の高さが妥当か
- [ ] 可視化結果の確認

### 5.3 評価指標
- 検出成功率（%）
- 位置誤差（cm）
- 処理時間（ms/frame）

---

## 6. 優先実装順序

1. **Task 1.1-1.3**: TF問題の解決（まずこれがないと動かない）
2. **Task 2.2**: 法線推定の改善
3. **Task 3.1**: 高さ分類閾値の調整
4. **Task 2.1**: IMU補正（rosbagにIMUがあれば）
5. **Task 2.4**: 平面検出順序の改善
6. **Task 4.1**: 物体候補領域検出

---

## 7. 次のアクション

1. **即時:** rosbagを再生してTFとトピックの実際の名前を確認
2. **短期:** Phase 1のTF問題を解決
3. **中期:** Phase 2-3のアルゴリズム改善
4. **長期:** Phase 4の把持機能追加
