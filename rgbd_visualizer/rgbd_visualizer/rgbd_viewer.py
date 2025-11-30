#!/usr/bin/env python3
"""
RGB-D画像を可視化するノード

ROSトピックからRGB画像と深度画像をサブスクライブし、OpenCVウィンドウで表示します。

使用方法:
    ros2 run rgbd_visualizer rgbd_viewer

トピック:
    - camera/color/image_raw (sensor_msgs/Image): RGB画像
    - camera/depth/image_raw (sensor_msgs/Image): 深度画像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer


class RGBDViewer(Node):
    """RGB-D画像を可視化するノード"""

    def __init__(self):
        super().__init__('rgbd_viewer')

        # パラメータ
        self.declare_parameter('rgb_topic', 'camera/color/image_raw')
        self.declare_parameter('depth_topic', 'camera/depth/image_raw')
        self.declare_parameter('depth_scale', 5000.0)  # TUM用
        self.declare_parameter('max_depth', 5.0)  # 表示用最大深度(m)

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value

        self.bridge = CvBridge()

        # 同期サブスクライバ
        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(f'Subscribing to {rgb_topic} and {depth_topic}')
        self.get_logger().info('Press Q to quit, S to save current frame')

        self.frame_count = 0

    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        """同期されたRGB-Dフレームを処理"""
        try:
            # RGB画像の変換
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # 深度画像の変換
            if depth_msg.encoding == '16UC1':
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
                depth_m = depth.astype(np.float32) / self.depth_scale
            elif depth_msg.encoding == '32FC1':
                depth_m = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unknown depth encoding: {depth_msg.encoding}')
                return

            # 深度画像のカラーマップ
            depth_normalized = np.clip(depth_m / self.max_depth, 0, 1)
            depth_colormap = cv2.applyColorMap(
                (depth_normalized * 255).astype(np.uint8),
                cv2.COLORMAP_VIRIDIS
            )

            # 統計情報
            valid_depth = depth_m[depth_m > 0]
            if len(valid_depth) > 0:
                min_d = valid_depth.min()
                max_d = valid_depth.max()
                mean_d = valid_depth.mean()
            else:
                min_d = max_d = mean_d = 0

            # テキスト表示
            info_text = f'Frame: {self.frame_count} | Depth: {min_d:.2f}-{max_d:.2f}m (mean: {mean_d:.2f}m)'
            cv2.putText(rgb_bgr, info_text, (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 画像を横に並べて表示
            combined = np.hstack([rgb_bgr, depth_colormap])
            cv2.imshow('RGB-D Viewer', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit requested')
                rclpy.shutdown()
            elif key == ord('s'):
                # フレーム保存
                cv2.imwrite(f'rgb_{self.frame_count:05d}.png', rgb_bgr)
                cv2.imwrite(f'depth_{self.frame_count:05d}.png', depth_colormap)
                self.get_logger().info(f'Saved frame {self.frame_count}')

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = RGBDViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
