# HSR 平面検出アプローチ設計書

## 1. 背景と目的

### 1.1 課題設定
RoboCup@Home競技において、HSR（Human Support Robot）がテーブルや棚の上に配置された物体を把持するためには、物体が置かれている**平面の検出**が必要となる。

### 1.2 基本仮定
- **床面とテーブル/棚の面は平行である**
- 家庭環境のテーブルや棚は通常水平に設置されている
- この仮定はRoboCup@Home環境では概ね妥当

### 1.3 ゴール
1. RGB-Dカメラからの点群データを用いて水平平面を検出する
2. 床面とテーブル/棚面を高さで区別する
3. 検出結果を把持タスク等の後続処理に提供する

---

## 2. 使用するセンサ・座標系

### 2.1 センサ
| センサ | 座標系 | 用途 |
|--------|--------|------|
| 頭部RGB-Dカメラ | `head_rgbd_sensor_rgb_frame` | 点群取得 |
| IMU | `base_imu_frame` | 重力方向補正 |

### 2.2 関連する座標系（参照: hsr_joints_links.md）
- `base_footprint`: ロボット基準座標系（地面接地点）
- `head_rgbd_sensor_link`: RGB-Dセンサ搭載位置
- `head_rgbd_sensor_rgb_frame`: キャリブレーション後のRGBカメラ光学系基準

---

## 3. アプローチ比較

### 3.1 候補手法

| 手法 | 概要 | メリット | デメリット |
|------|------|----------|------------|
| **RANSAC + 法線制約** | 法線が基準方向（重力逆方向）と近い平面のみ採用 | 計算軽量、誤検出少 | ロボット傾斜時に要補正 |
| **深度画像ベース領域成長** | 深度画像上で直接法線計算・クラスタリング | 点群変換不要 | ノイズに敏感 |
| **多段階アプローチ** | 床→テーブル→棚の順に階層的に検出 | 構造化された出力 | 処理ステップ増加 |

### 3.2 推奨手法
**RANSAC + 法線ベクトル制約 + IMU補正**

理由:
1. 「床と平行」という事前知識を明示的に活用できる
2. Open3D/PCLで容易に実装可能
3. IMU補正でロボット傾斜時もロバスト

---

## 4. システム構成

### 4.1 ノード構成図

```
[HSR Sensors]
     │
     ├── /head_rgbd_sensor/depth/points (sensor_msgs/PointCloud2)
     │
     ├── /base_imu/data (sensor_msgs/Imu)  ※トピック名は要確認
     │
     ▼
[plane_detector_node]
     │
     ├──▶ /detected_table_surface (geometry_msgs/PoseStamped)
     │      - 検出したテーブル面の位置・姿勢
     │
     └──▶ /detected_planes (visualization_msgs/MarkerArray)  ※オプション
            - 可視化用マーカー
```

### 4.2 データフロー

```
1. RGB-Dカメラから点群取得
       ↓
2. IMUから重力方向取得（基準法線を動的更新）
       ↓
3. 点群の前処理（外れ値除去、ダウンサンプリング）
       ↓
4. RANSAC平面検出（法線角度制約付き）
       ↓
5. 高さによる分類（床 / テーブル / 棚）
       ↓
6. 検出結果をPublish
```

---

## 5. アルゴリズム詳細

### 5.1 パラメータ設計

| パラメータ | 推奨値 | 説明 |
|------------|--------|------|
| `normal_threshold_deg` | 10.0° | 水平と見なす許容角度（厳格に設定） |
| `distance_threshold` | 0.02m | RANSAC距離閾値 |
| `min_plane_points` | 500 | 平面と認識する最小点数 |
| `floor_height_fixed` | 0.05m | 床の固定高さ |
| `min_detection_height` | 0.20m | **検出対象の最小高さ（床より上）** |
| `floor_height_max` | 0.15m | 床と見なす最大高さ |
| `table_height_min` | 0.40m | テーブルの最小高さ |
| `table_height_max` | 1.20m | テーブルの最大高さ |

### 5.2 法線角度による平面フィルタリング

```python
# 基準法線（重力逆方向、IMUから取得）
reference_normal = get_gravity_direction_from_imu()

# 検出した平面の法線
plane_normal = np.array([a, b, c])  # RANSAC結果

# 角度計算
angle = np.arccos(np.dot(plane_normal, reference_normal))

# 閾値判定
if angle < np.deg2rad(normal_threshold_deg):
    # 水平平面として採用
```

### 5.3 床除外と高さによる平面分類

**重要**: 床を固定高さ以下として除外し、それより上の水平平面のみを検出する。

```
高さ (Z)
   │
1.2m ┼─────────────── 棚上段
   │     [棚面] ← 検出対象
0.8m ┼─────────────── テーブル/棚中段
   │   [テーブル面] ← 検出対象
0.4m ┼─────────────── table_height_min
   │   [UNKNOWN] ← 検出対象
0.2m ┼─────────────── min_detection_height ★ここより上のみ検出
   │
0.15m┼─────────────── floor_height_max
   │     [床面] ← 除外
0.0m ┼─────────────── base_footprint
```

**床除外ロジック**:
1. 点群から `min_detection_height` より下の点を事前に除去
2. 残った点群に対してのみRANSAC平面検出を実行
3. 検出された平面が `PlaneType.FLOOR` と判定された場合は除外

---

## 6. 実装時の注意点

### 6.1 ロバスト性向上のための対策

| 問題 | 対策 |
|------|------|
| ロボットの姿勢変動 | IMUで重力方向を常時補正 |
| 微妙に傾いたテーブル | 許容角度を5〜10°に設定 |
| 複数段の棚 | 高さでクラスタリング |
| ガラス面で深度取得不可 | RGB情報併用、物体位置から推定 |
| テーブルクロスによる歪み | エッジ検出併用 |

### 6.2 処理速度最適化

1. **ダウンサンプリング**: VoxelGridフィルタで点群を間引く
2. **ROI制限**: 関心領域外の点を事前除去
3. **処理頻度調整**: 毎フレーム処理せず、必要時のみ実行

---

## 7. トピック・サービス仕様（案）

### 7.1 Subscribe

| トピック名 | 型 | 説明 |
|------------|----|----|
| `/head_rgbd_sensor/depth/points` | `sensor_msgs/PointCloud2` | RGB-D点群 |
| `/base_imu/data` | `sensor_msgs/Imu` | IMUデータ |

※トピック名は実環境に合わせて要修正

### 7.2 Publish

| トピック名 | 型 | 説明 |
|------------|----|----|
| `/detected_table_surface` | `geometry_msgs/PoseStamped` | 検出したテーブル面 |
| `/detected_planes` | `visualization_msgs/MarkerArray` | 可視化用（オプション） |

### 7.3 パラメータ

| パラメータ名 | 型 | デフォルト |
|--------------|----|----|
| `normal_threshold_deg` | float | 10.0 |
| `distance_threshold` | float | 0.02 |
| `min_plane_points` | int | 1000 |
| `floor_height_max` | float | 0.15 |
| `table_height_min` | float | 0.40 |
| `table_height_max` | float | 1.20 |

---

## 8. 依存パッケージ

| パッケージ | 用途 |
|------------|------|
| `rclpy` | ROS2 Pythonクライアント |
| `sensor_msgs` | PointCloud2, Imu メッセージ |
| `geometry_msgs` | PoseStamped メッセージ |
| `open3d` | 点群処理、RANSAC平面検出 |
| `numpy` | 数値計算 |
| `tf_transformations` | クォータニオン処理 |
| `sensor_msgs_py` | PointCloud2変換ユーティリティ |

---

## 9. テスト戦略

### 9.1 シミュレーションテスト
1. Gazebo上でテーブル・棚を配置したワールドを作成
2. HSRを各位置に移動させて平面検出を実行
3. 検出結果をRViz2で可視化して確認

### 9.2 実機テスト
1. 固定位置から既知のテーブルを検出
2. 検出した高さ・位置が実測値と一致するか確認
3. ロボットを傾けた状態での検出精度を確認

### 9.3 評価指標
- 検出成功率（%）
- 位置誤差（cm）
- 処理時間（ms/frame）

---

## 10. 今後の拡張案

1. **複数平面の同時追跡**: 棚の各段を区別して管理
2. **物体位置との統合**: 平面上の物体候補領域を出力
3. **動的環境対応**: 人が通過した際の一時的な遮蔽への対処
4. **学習ベース手法との併用**: セマンティックセグメンテーションで家具を認識

---

## 11. 参考資料

- HSR関節・リンク・座標系構造: `hsr_joints_links.md`
- HSR ROS2ドキュメント: https://github.com/hsr-project/hsr_ros2_doc/tree/humble/docs
- Open3D公式ドキュメント: http://www.open3d.org/docs/
- ROS2 Humble公式: https://docs.ros.org/en/humble/

---

## 12. 改訂履歴

| 日付 | 版 | 内容 |
|------|----|----|
| 2025-XX-XX | 1.0 | 初版作成 |
| 2025-11-30 | 1.1 | 床除外ロジック追加、水平平面のみ検出に改善 |
