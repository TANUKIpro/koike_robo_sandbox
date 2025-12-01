# Plane Detection Parameter Tuner

RGB-D画像からの平面検出パラメータをチューニングするためのGUIアプリケーション。

## 背景・目的

RoboCup@Homeなどのサービスロボット競技において、テーブルや棚の上の物体を操作するためには、まず平面を正確に検出する必要があります。しかし、深度センサーの特性上、以下のような課題があります：

- **近距離では高精度**だが、**遠距離（2m以上）では深度データが粗くなる**
- 床面は検出できても、**奥にある棚や椅子の上面が検出されない**
- パラメータ調整が試行錯誤になりがち

このツールは、**様々なパラメータやアルゴリズムを動的に変化させながら、その効果を視覚的に確認**できるように設計されています。

## 特徴

- **リアルタイムプレビュー**: パラメータ変更即座に検出結果を確認
- **複数アルゴリズム対応**:
  - RANSAC（標準）
  - Iterative RANSAC（複数平面検出）
  - Region Growing
  - RGB-Guided（RGBエッジを活用した平面検出）
- **深度前処理オプション**:
  - バイラテラルフィルタ（ノイズ除去）
  - ホールフィリング（欠損補完）
- **多様な入力形式**:
  - 単一RGB-D画像ペア
  - TUM RGB-Dデータセット
  - ROS2 rosbag

## インストール

```bash
cd plane_tuner
pip install -r requirements.txt
```

### 依存パッケージ

- Python 3.8以上
- numpy
- opencv-python
- open3d
- scipy
- Pillow
- rosbags（ROS2 rosbag読み込み用 - ROS2環境不要）
- PyYAML（オプション：パラメータエクスポート用）

## 使用方法

### 基本起動

```bash
python plane_tuner.py
```

### 画像ペアを指定して起動

```bash
python plane_tuner.py --rgb image_rgb.png --depth image_depth.png
```

### TUMデータセットを読み込んで起動

```bash
python plane_tuner.py --dataset /path/to/rgbd_dataset_freiburg1_desk --preset tum-freiburg1
```

### RoboCup rosbagを読み込んで起動

```bash
# RoboCup TIAGoデータセット（storing_try_2など）
python plane_tuner.py --rosbag datasets/robocup/storing_try_2 --preset robocup-tiago

# 最初の50フレームをスキップして100フレーム読み込み
python plane_tuner.py --rosbag datasets/robocup/storing_try_2 --skip-frames 50 --max-frames 100

# カスタムトピックを指定
python plane_tuner.py --rosbag /path/to/rosbag \
    --rgb-topic /camera/rgb/image_raw \
    --depth-topic /camera/depth/image_raw
```

### カメラパラメータを指定

```bash
# TIAGoロボットのカメラパラメータを使用
python plane_tuner.py --preset tiago --depth-scale 1000

# RoboCup TIAGoデータセット用プリセット
python plane_tuner.py --preset robocup-tiago

# TUMデータセットのカメラパラメータを使用
python plane_tuner.py --preset tum-freiburg1 --depth-scale 5000
```

### RoboCupデータセットのダウンロード

RoboCup rosbagは[Zenodo](https://zenodo.org/record/13838208)からダウンロードできます。

```bash
cd datasets/robocup
pip install zenodo-get
zenodo_get 13838208
```

詳細は `datasets/robocup/README.md` を参照してください。

## GUI操作

### パラメータパネル（左側）

| カテゴリ | パラメータ | 説明 |
|---------|-----------|------|
| **Algorithm** | Algorithm | 平面検出アルゴリズムの選択 |
| **RANSAC** | Distance Threshold | 平面からの許容距離（m） |
| | RANSAC N Points | 1イテレーションで使用する点数 |
| | RANSAC Iterations | 最大イテレーション数 |
| **Plane Filtering** | Min Plane Points | 平面として認識する最小点数 |
| | Min Plane Area | 平面として認識する最小面積（m²） |
| | Max Planes | 検出する最大平面数 |
| **Normal Constraint** | Enable Normal Filter | 法線方向による平面フィルタリング |
| | Normal Threshold | 水平からの許容角度（度） |
| **Point Cloud** | Downsample Factor | ダウンサンプリング係数 |
| | Min/Max Depth | 有効深度範囲（m） |
| **Preprocessing** | Bilateral Filter | バイラテラルフィルタの適用 |
| | Hole Filling | 深度欠損の補完 |

### 画像表示（右側）

- **Side by Side**: RGB（オーバーレイ付き）と深度画像を並べて表示
- **RGB Only**: RGBまたはオーバーレイのみ表示
- **Depth Only**: 深度画像のみ表示（カラーマップ選択可能）

### キーボードショートカット

| キー | 機能 |
|-----|------|
| `Space` / `Enter` | 検出実行 |
| `←` / `→` | 前/次のフレーム |

## アルゴリズム詳細

### RANSAC（標準）

単一の最大平面を検出します。高速ですが、複数平面がある場合は最大のものしか検出されません。

### Iterative RANSAC

複数の平面を順次検出します。最大平面を検出後、その点を除外して次の平面を検出することを繰り返します。

### Region Growing

点群のクラスタリングを行い、各クラスタ内でRANSACを適用します。隣接する点の法線を考慮した検出が可能です。

### RGB-Guided

RGB画像のエッジ（Canny）を検出し、エッジで区切られた領域ごとに平面検出を行います。深度データが粗い遠距離の平面でも、RGBで領域が区別できれば検出しやすくなります。

## パラメータチューニングのヒント

### 床しか検出されない場合

1. **Max Depth を増やす**: 遠くの平面も点群に含める（例: 10.0 → 15.0）
2. **Min Plane Points を下げる**: 点数が少ない平面も検出（例: 500 → 200）
3. **Distance Threshold を上げる**: ノイズの多い遠距離データに対応（例: 0.02 → 0.05）
4. **Downsample Factor を下げる**: より多くの点を使用（例: 4 → 2）
5. **Bilateral Filter を有効化**: 深度ノイズを軽減
6. **RGB-Guided アルゴリズムを試す**: RGB情報を活用

### 平面が検出されすぎる場合

1. **Min Plane Area を上げる**: 小さい平面を除外（例: 0.05 → 0.1）
2. **Min Plane Points を上げる**: 点数が少ない平面を除外
3. **Distance Threshold を下げる**: より厳密な平面適合

### 水平以外の平面も検出したい場合

1. **Enable Normal Filter をOFF**: 法線方向のフィルタを無効化
2. または **Normal Threshold を上げる**: より大きな傾きを許容（例: 15 → 45）

## 出力

### パラメータエクスポート

File → Export Parameters... でYAMLまたはJSON形式でパラメータを保存できます。

```yaml
# example_params.yaml
algorithm: ransac_iterative
distance_threshold: 0.02
ransac_n: 3
num_iterations: 1000
min_plane_points: 500
min_plane_area: 0.05
max_planes: 8
normal_threshold_deg: 15.0
enable_normal_filter: true
downsample_factor: 4
min_depth: 0.1
max_depth: 10.0
```

## ファイル構成

```
plane_tuner/
├── plane_tuner.py          # メインエントリポイント
├── requirements.txt        # 依存パッケージ
├── README.md              # このファイル
├── core/
│   ├── __init__.py
│   ├── image_loader.py    # 画像/rosbag読み込み
│   ├── point_cloud.py     # 点群生成
│   └── plane_detector.py  # 平面検出アルゴリズム
└── gui/
    ├── __init__.py
    ├── main_window.py     # メインウィンドウ
    ├── parameter_panel.py # パラメータ制御パネル
    └── image_canvas.py    # 画像表示キャンバス
```

## 関連プロジェクト

- `plane_detector`: ROS2ノードによる平面検出（本ツールのアルゴリズムを統合可能）
- `rgbd_visualizer`: RGB-D画像の基本的な可視化

## ライセンス

このプロジェクトはkoike_robo_sandboxの一部として提供されています。
