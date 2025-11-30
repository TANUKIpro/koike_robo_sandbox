# koike_robo_sandbox - Project Context for AI Assistants

このファイルは、AIアシスタント（Claude等）がこのプロジェクトを理解するためのコンテキスト情報を提供します。

## プロジェクト概要

RoboCup@Home向けのRGB-D平面検出システムの開発・実験用リポジトリ。
主にTIAGoロボットのrosbagデータを使用した平面検出アルゴリズムの開発と評価を行っています。

## 主要コンポーネント

### 1. plane_detector（ROS2パッケージ）

RGB-D画像からRANSACを用いて平面を検出するROS2ノード。

**場所**: `/plane_detector/`

**主要ファイル**:
- `plane_detector/plane_detector_node.py`: メイン検出ノード
- `plane_detector/plane_recorder_node.py`: 検出結果の録画ノード
- `launch/plane_detection.launch.py`: 起動ファイル

**特徴**:
- Open3DのRANSACによる平面検出
- TF2を使用した座標変換（カメラ→base_footprint）
- 法線ベクトルによる水平面フィルタリング
- RViz2およびOpenCVによる可視化

### 2. plane_tuner（GUIツール）

平面検出パラメータをチューニングするためのスタンドアロンGUIアプリケーション。

**場所**: `/plane_tuner/`

**背景**:
床面のみが検出され、奥にある棚や椅子の上面が検出されない問題に対処するため、様々なパラメータとアルゴリズムを視覚的に評価・検証できるツールとして作成されました。

**特徴**:
- Tkinterベースの軽量GUI
- ROS2への依存なしで動作（スタンドアロン）
- 複数のアルゴリズム対応（RANSAC, Region Growing, RGB-Guided）
- 深度前処理オプション（バイラテラルフィルタ、ホールフィリング）
- TUMデータセット、rosbag、画像ペアの読み込み対応

**作成の経緯**:
2024年11月の開発セッションにおいて、以下の課題が特定されました：
- 深度センサーは遠距離（2m以上）でデータが粗くなる
- 標準的なRANSACパラメータでは床面しか検出されない
- 「RGBでは平面と認識できるが、深度データが粗い領域」の検出が困難

これらの課題を解決するため、パラメータチューニングとアルゴリズム評価を容易にするGUIツールが必要となり、`plane_tuner`が開発されました。

### 3. rgbd_visualizer（ROS2パッケージ）

RGB-D画像の基本的な可視化ツール群。

**場所**: `/rgbd_visualizer/`

**主要ファイル**:
- `bag_publisher.py`: TUMデータセットのROS2パブリッシャ
- `rgbd_viewer.py`: 基本的なRGB-D表示
- `pointcloud_generator.py`: 点群生成・可視化

## データセット

### TIAGo rosbag

**場所**: `datasets/storing_try_2/` など

**トピック**:
- `/head_front_camera/rgb/image_raw`: RGB画像
- `/head_front_camera/depth/image_raw`: 深度画像（16UC1, mm単位）
- `/head_front_camera/rgb/camera_info`: カメラ情報
- `/tf`, `/tf_static`: TFデータ

**カメラパラメータ**:
- 解像度: 640x480
- depth_scale: 1000.0（mm → m）
- カメラフレーム: `head_front_camera_depth_optical_frame`
- ベースフレーム: `base_footprint`

### TUM RGB-D Dataset

**特徴**:
- depth_scale: 5000.0
- プリセット: freiburg1, freiburg2, freiburg3（それぞれ異なる内部パラメータ）

## 開発における重要な知見

### 平面検出の課題

1. **深度データの品質問題**
   - 近距離（0.5-2m）: 高精度
   - 遠距離（2m以上）: ノイズが増加、点群が疎になる

2. **RANSACパラメータの影響**
   - `distance_threshold`: 0.02mで厳密すぎる場合がある（遠距離）
   - `min_plane_points`: 500では遠距離の平面が検出されにくい
   - `downsample_factor`: 4では点数が不足する場合がある

3. **推奨される改善アプローチ**
   - 深度の前処理（バイラテラルフィルタ）
   - RGB情報を活用した領域分割
   - パラメータの動的調整（深度に応じて）

### TF関連の注意点

- rosbag再生時、TFのタイミングがずれることがある
- `latest_transform`をキャッシュして、lookup失敗時のフォールバックとする
- `use_sim_time: true`の設定が必要

## 使用コマンド例

### plane_tunerの起動

```bash
cd plane_tuner
python plane_tuner.py
```

### plane_detectorの起動（ROS2）

```bash
ros2 launch plane_detector plane_detection.launch.py \
  bag_path:=/path/to/rosbag \
  process_rate:=5.0 \
  min_plane_area:=0.05
```

## 将来の開発方向

1. **RGB-Guided平面検出の改善**: セマンティックセグメンテーションの統合
2. **深度補完**: 機械学習ベースの深度推定との組み合わせ
3. **IMU統合**: 動的な重力方向補正
4. **plane_tunerの結果をplane_detectorに統合**: 最適化されたパラメータの自動適用

## ファイル構成

```
koike_robo_sandbox/
├── .claude/
│   └── claude.md           # このファイル（AIアシスタント用コンテキスト）
├── plane_detector/          # ROS2平面検出パッケージ
├── plane_tuner/             # GUIパラメータチューニングツール
├── rgbd_visualizer/         # RGB-D可視化ツール
├── datasets/                # rosbag等のデータセット
└── README.md               # プロジェクト全体のREADME
```
