#!/usr/bin/env python3
"""
深度画像から点群を生成するノード

RGB-D画像から色付き点群を生成し、RViz2で可視化できます。

使用方法:
    ros2 run rgbd_visualizer pointcloud_generator

トピック:
    入力:
        - camera/color/image_raw (sensor_msgs/Image): RGB画像
        - camera/depth/image_raw (sensor_msgs/Image): 深度画像
        - camera/camera_info (sensor_msgs/CameraInfo): カメラ情報

    出力:
        - camera/points (sensor_msgs/PointCloud2): 色付き点群
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import struct


class PointCloudGenerator(Node):
    """深度画像から点群を生成するノード"""

    def __init__(self):
        super().__init__('pointcloud_generator')

        # パラメータ
        self.declare_parameter('rgb_topic', 'camera/color/image_raw')
        self.declare_parameter('depth_topic', 'camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('output_topic', 'camera/points')
        self.declare_parameter('depth_scale', 5000.0)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('downsample', 1)  # ダウンサンプリング係数

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.downsample = self.get_parameter('downsample').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.camera_info = None

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, output_topic, 10)

        # CameraInfo subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # 同期サブスクライバ
        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(f'Point cloud generator started')
        self.get_logger().info(f'  Input: {rgb_topic}, {depth_topic}')
        self.get_logger().info(f'  Output: {output_topic}')

    def camera_info_callback(self, msg: CameraInfo):
        """CameraInfoを保存"""
        self.camera_info = msg

    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        """同期されたRGB-Dフレームから点群を生成"""
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=2.0)
            return

        try:
            # RGB画像の変換
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')

            # 深度画像の変換
            if depth_msg.encoding == '16UC1':
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
                depth_m = depth.astype(np.float32) / self.depth_scale
            elif depth_msg.encoding == '32FC1':
                depth_m = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unknown depth encoding: {depth_msg.encoding}')
                return

            # カメラパラメータ
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            # 点群生成
            pc_msg = self._create_pointcloud(rgb, depth_m, fx, fy, cx, cy, rgb_msg.header)
            self.pc_pub.publish(pc_msg)

        except Exception as e:
            self.get_logger().error(f'Error generating point cloud: {e}')

    def _create_pointcloud(self, rgb: np.ndarray, depth: np.ndarray,
                           fx: float, fy: float, cx: float, cy: float,
                           header) -> PointCloud2:
        """深度画像からPointCloud2メッセージを生成"""
        h, w = depth.shape
        ds = self.downsample

        # ピクセル座標のグリッド生成
        u = np.arange(0, w, ds)
        v = np.arange(0, h, ds)
        u, v = np.meshgrid(u, v)

        # ダウンサンプリング
        depth_ds = depth[::ds, ::ds]
        rgb_ds = rgb[::ds, ::ds]

        # 有効な深度のマスク
        mask = (depth_ds > self.min_depth) & (depth_ds < self.max_depth)

        # 3D座標の計算
        z = depth_ds[mask]
        x = (u[mask] - cx) * z / fx
        y = (v[mask] - cy) * z / fy

        # 色情報
        colors = rgb_ds[mask]

        # PointCloud2メッセージの作成
        points = np.zeros(len(x), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32)
        ])

        points['x'] = x
        points['y'] = y
        points['z'] = z

        # RGBをfloatにパック
        rgb_packed = np.zeros(len(colors), dtype=np.float32)
        for i, (r, g, b) in enumerate(colors):
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_packed[i] = struct.unpack('f', struct.pack('I', rgb_int))[0]
        points['rgb'] = rgb_packed

        # PointCloud2メッセージ
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'camera_depth_optical_frame'
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
