#!/usr/bin/env python3
"""
ROSbagからRGB-Dデータをパブリッシュするノード

TUM RGB-D形式のデータセットを読み込み、ROSトピックとしてパブリッシュします。
RViz2やrgbd_viewerノードで可視化できます。

使用方法:
    ros2 run rgbd_visualizer bag_publisher --ros-args \
        -p dataset_path:=/path/to/rgbd_dataset_freiburg1_desk \
        -p rate:=30.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
from pathlib import Path
import cv2
import numpy as np


# カメラパラメータ（TUM RGB-D用）
CAMERA_PARAMS = {
    'freiburg1': {'fx': 517.3, 'fy': 516.5, 'cx': 318.6, 'cy': 255.3, 'depth_scale': 5000.0},
    'freiburg2': {'fx': 520.9, 'fy': 521.0, 'cx': 325.1, 'cy': 249.7, 'depth_scale': 5000.0},
    'freiburg3': {'fx': 535.4, 'fy': 539.2, 'cx': 320.1, 'cy': 247.6, 'depth_scale': 5000.0},
}


class BagPublisher(Node):
    """TUM RGB-Dデータセットをパブリッシュするノード"""

    def __init__(self):
        super().__init__('bag_publisher')

        # パラメータ
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('loop', True)

        dataset_path = self.get_parameter('dataset_path').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        if not dataset_path:
            self.get_logger().error('dataset_path parameter is required')
            raise ValueError('dataset_path parameter is required')

        self.dataset_path = Path(dataset_path)
        if not self.dataset_path.exists():
            self.get_logger().error(f'Dataset path does not exist: {dataset_path}')
            raise FileNotFoundError(f'Dataset path does not exist: {dataset_path}')

        # カメラパラメータの自動選択
        self.camera_params = self._detect_camera_params()

        # データの読み込み
        self.associations = self._load_associations()
        self.current_idx = 0

        if len(self.associations) == 0:
            self.get_logger().error('No frames found in dataset')
            raise ValueError('No frames found in dataset')

        self.get_logger().info(f'Loaded {len(self.associations)} frames from {dataset_path}')

        # Publisher
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Timer
        period = 1.0 / self.rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'Publishing at {self.rate} Hz')

    def _detect_camera_params(self):
        """データセットパスからカメラパラメータを自動検出"""
        path_str = str(self.dataset_path).lower()
        if 'freiburg1' in path_str:
            return CAMERA_PARAMS['freiburg1']
        elif 'freiburg2' in path_str:
            return CAMERA_PARAMS['freiburg2']
        elif 'freiburg3' in path_str:
            return CAMERA_PARAMS['freiburg3']
        else:
            self.get_logger().warn('Unknown dataset, using freiburg1 parameters')
            return CAMERA_PARAMS['freiburg1']

    def _load_file_list(self, filename: str) -> list:
        """タイムスタンプとファイルパスのリストを読み込む"""
        file_list = []
        filepath = self.dataset_path / filename
        if not filepath.exists():
            return file_list

        with open(filepath) as f:
            for line in f:
                if line.startswith('#'):
                    continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    file_list.append((float(parts[0]), parts[1]))
        return file_list

    def _load_associations(self, max_diff: float = 0.02) -> list:
        """RGBと深度画像のアソシエーションを作成"""
        rgb_list = self._load_file_list('rgb.txt')
        depth_list = self._load_file_list('depth.txt')

        associations = []
        depth_dict = {ts: path for ts, path in depth_list}
        depth_timestamps = sorted(depth_dict.keys())

        for rgb_ts, rgb_path in rgb_list:
            if not depth_timestamps:
                break
            closest_ts = min(depth_timestamps, key=lambda x: abs(x - rgb_ts))
            if abs(closest_ts - rgb_ts) < max_diff:
                associations.append((rgb_ts, rgb_path, closest_ts, depth_dict[closest_ts]))

        return associations

    def _create_camera_info(self, stamp) -> CameraInfo:
        """CameraInfoメッセージを作成"""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.width = 640
        msg.height = 480
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = self.camera_params['fx']
        fy = self.camera_params['fy']
        cx = self.camera_params['cx']
        cy = self.camera_params['cy']

        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def timer_callback(self):
        """定期的にフレームをパブリッシュ"""
        if self.current_idx >= len(self.associations):
            if self.loop:
                self.current_idx = 0
                self.get_logger().info('Looping dataset')
            else:
                self.get_logger().info('Dataset finished')
                self.timer.cancel()
                return

        rgb_ts, rgb_path, depth_ts, depth_path = self.associations[self.current_idx]

        # 画像読み込み
        rgb = cv2.imread(str(self.dataset_path / rgb_path))
        depth = cv2.imread(str(self.dataset_path / depth_path), cv2.IMREAD_UNCHANGED)

        if rgb is None or depth is None:
            self.get_logger().warn(f'Failed to load frame {self.current_idx}')
            self.current_idx += 1
            return

        # BGR -> RGB
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        # タイムスタンプ
        stamp = self.get_clock().now().to_msg()

        # RGB画像をパブリッシュ
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'camera_color_optical_frame'
        self.rgb_pub.publish(rgb_msg)

        # 深度画像をパブリッシュ（16UC1形式）
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        # CameraInfo
        camera_info = self._create_camera_info(stamp)
        self.camera_info_pub.publish(camera_info)

        self.current_idx += 1


def main(args=None):
    rclpy.init(args=args)

    try:
        node = BagPublisher()
        rclpy.spin(node)
    except (ValueError, FileNotFoundError) as e:
        print(f'Error: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
