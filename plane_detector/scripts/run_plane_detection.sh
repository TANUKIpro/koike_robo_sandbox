#!/bin/bash
#
# 平面検出システム 1コマンド実行スクリプト
#
# rosbag再生からOpenCVウィンドウ表示までを1コマンドで実行します。
#
# 使用例:
#   ./run_plane_detection.sh /path/to/rosbag
#   ./run_plane_detection.sh /path/to/rosbag --loop
#   ./run_plane_detection.sh /path/to/rosbag --rate 0.5
#   ./run_plane_detection.sh /path/to/rosbag --rviz
#
# オプション:
#   --loop          rosbagをループ再生
#   --rate RATE     再生速度（デフォルト: 1.0）
#   --rviz          RViz2も起動
#   --min-area AREA 最小平面面積 (m^2, デフォルト: 0.05)
#   --help          このヘルプを表示
#

set -e

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$PACKAGE_DIR")"

# デフォルト値
BAG_PATH=""
LOOP="false"
RATE="1.0"
USE_RVIZ="false"
MIN_AREA="0.05"

# ヘルプ表示
show_help() {
    echo "Usage: $0 <bag_path> [options]"
    echo ""
    echo "平面検出システム - rosbag再生からOpenCVウィンドウ表示まで1コマンドで実行"
    echo ""
    echo "Arguments:"
    echo "  bag_path           ROS2 bagディレクトリへのパス"
    echo ""
    echo "Options:"
    echo "  --loop             rosbagをループ再生"
    echo "  --rate RATE        再生速度 (デフォルト: 1.0)"
    echo "  --rviz             RViz2も起動"
    echo "  --min-area AREA    最小平面面積 m^2 (デフォルト: 0.05)"
    echo "  --help             このヘルプを表示"
    echo ""
    echo "Examples:"
    echo "  $0 datasets/robocup/storing_try_2"
    echo "  $0 datasets/robocup/storing_try_2 --loop"
    echo "  $0 datasets/robocup/storing_try_2 --rate 0.5 --loop"
    echo "  $0 datasets/robocup/storing_try_2 --rviz"
    exit 0
}

# 引数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        --help|-h)
            show_help
            ;;
        --loop)
            LOOP="true"
            shift
            ;;
        --rate)
            RATE="$2"
            shift 2
            ;;
        --rviz)
            USE_RVIZ="true"
            shift
            ;;
        --min-area)
            MIN_AREA="$2"
            shift 2
            ;;
        *)
            if [[ -z "$BAG_PATH" ]]; then
                BAG_PATH="$1"
            fi
            shift
            ;;
    esac
done

# bag_pathが指定されているか確認
if [[ -z "$BAG_PATH" ]]; then
    echo "Error: bag_path is required"
    echo ""
    show_help
fi

# 相対パスを絶対パスに変換
if [[ ! "$BAG_PATH" = /* ]]; then
    # 現在のディレクトリからの相対パスを試す
    if [[ -d "$(pwd)/$BAG_PATH" ]]; then
        BAG_PATH="$(pwd)/$BAG_PATH"
    # ワークスペースディレクトリからの相対パスを試す
    elif [[ -d "$WORKSPACE_DIR/$BAG_PATH" ]]; then
        BAG_PATH="$WORKSPACE_DIR/$BAG_PATH"
    fi
fi

# bag_pathが存在するか確認
if [[ ! -d "$BAG_PATH" ]]; then
    echo "Error: bag path not found: $BAG_PATH"
    exit 1
fi

echo "============================================"
echo "  Plane Detection System"
echo "============================================"
echo "Bag path: $BAG_PATH"
echo "Loop: $LOOP"
echo "Rate: $RATE"
echo "RViz: $USE_RVIZ"
echo "Min area: $MIN_AREA m^2"
echo "============================================"

# ROS2ワークスペースをセットアップ
if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    echo "Sourcing workspace: $WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "Warning: Workspace not built. Running colcon build..."
    cd "$WORKSPACE_DIR"
    colcon build --packages-select plane_detector --symlink-install
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# launchコマンドを実行
echo ""
echo "Starting plane detection..."
echo "Press 'Q' in the OpenCV window to quit."
echo ""

ros2 launch plane_detector plane_detection.launch.py \
    bag_path:="$BAG_PATH" \
    loop:="$LOOP" \
    rate:="$RATE" \
    use_rviz:="$USE_RVIZ" \
    min_plane_area:="$MIN_AREA"
