#!/usr/bin/env python3
"""
平面検出録画ノード - rosbagから平面検出結果をMP4として保存

RGB-Dカメラからの点群データを使用して、RANSACで平面を無差別に検出し、
RGB画像と検出平面のオーバーレイをMP4として保存します。

使用方法:
    # rosbagを再生
    ros2 bag play /path/to/rosbag --clock --loop

    # 録画ノードを起動
    ros2 run plane_detector plane_recorder_node --ros-args \
        -p use_sim_time:=true \
        -p output_path:=/path/to/output.mp4

出力:
    - 左側: RGB画像に検出平面をオーバーレイ
    - 右側: 深度画像（カラーマップ）に検出平面を表示
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from tf2_ros import TransformException

import numpy as np
import open3d as o3d
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum
import os
from datetime import datetime


class PlaneType(Enum):
    """平面の種類"""
    PLANE_1 = 1  # 最初に検出された平面
    PLANE_2 = 2  # 2番目に検出された平面
    PLANE_3 = 3  # 3番目に検出された平面
    PLANE_4 = 4  # 4番目に検出された平面
    PLANE_5 = 5  # 5番目に検出された平面


@dataclass
class DetectedPlane:
    """検出された平面の情報"""
    plane_id: int
    coefficients: np.ndarray  # [a, b, c, d] where ax + by + cz + d = 0
    inlier_indices: np.ndarray
    center: np.ndarray  # 平面の中心点
    normal: np.ndarray  # 法線ベクトル
    points: np.ndarray  # 平面上の点群（カメラ座標系）
    points_base: np.ndarray  # 平面上の点群（base座標系）
    color: Tuple[int, int, int]  # BGR色


class PlaneRecorderNode(Node):
    """平面検出録画ノード"""

    def __init__(self):
        super().__init__('plane_recorder_node')

        # パラメータ宣言
        self._declare_parameters()
        self._get_parameters()

        # 初期化
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.latest_transform = None
        self.frame_count = 0
        self.recorded_frames = 0

        # VideoWriter
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.video_initialized = False

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for camera info (compatible with rosbag)
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # CameraInfo subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            camera_info_qos
        )

        # RGB-D synchronized subscribers
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # 平面の色定義（BGR、区別しやすい色）
        self.plane_colors = [
            (0, 255, 0),    # 緑
            (255, 0, 0),    # 青
            (0, 0, 255),    # 赤
            (255, 255, 0),  # シアン
            (255, 0, 255),  # マゼンタ
            (0, 255, 255),  # 黄
            (128, 0, 255),  # ピンク
            (255, 128, 0),  # 水色
            (0, 128, 255),  # オレンジ
            (128, 255, 0),  # ライム
        ]

        self.get_logger().info('Plane Recorder Node started')
        self.get_logger().info(f'  RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'  Depth topic: {self.depth_topic}')
        self.get_logger().info(f'  Output path: {self.output_path}')
        self.get_logger().info(f'  FPS: {self.output_fps}')
        self.get_logger().info(f'  Max planes: {self.max_planes}')

    def _declare_parameters(self):
        """パラメータの宣言"""
        # トピック設定
        self.declare_parameter('rgb_topic', '/head_front_camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/head_front_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/head_front_camera/rgb/camera_info')

        # 座標系設定（TIAGo rosbag対応）
        self.declare_parameter('camera_frame', 'head_front_camera_depth_optical_frame')
        self.declare_parameter('base_frame', 'base_footprint')

        # 深度処理設定
        self.declare_parameter('depth_scale', 1000.0)  # mm -> m
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('downsample_factor', 4)

        # RANSAC設定
        self.declare_parameter('distance_threshold', 0.02)  # 2cm
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 1000)
        self.declare_parameter('min_plane_points', 500)

        # 平面検出設定（法線制約なしで全平面検出可能）
        self.declare_parameter('filter_horizontal', False)  # 水平平面のみフィルタするか
        self.declare_parameter('normal_threshold_deg', 45.0)  # フィルタ時の許容角度

        # 処理設定
        self.declare_parameter('max_planes', 10)  # より多くの平面を検出
        self.declare_parameter('process_rate', 10.0)  # Hz（録画用に高めに）

        # 出力設定
        self.declare_parameter('output_path', '')  # 空の場合は自動生成
        self.declare_parameter('output_fps', 10.0)
        self.declare_parameter('show_window', True)  # 録画中にプレビュー表示

        # レイアウト設定
        self.declare_parameter('layout', 'horizontal')  # 'horizontal' or 'vertical'

    def _get_parameters(self):
        """パラメータの取得"""
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.depth_scale = self.get_parameter('depth_scale').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.downsample_factor = self.get_parameter('downsample_factor').value

        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.ransac_n = self.get_parameter('ransac_n').value
        self.num_iterations = self.get_parameter('num_iterations').value
        self.min_plane_points = self.get_parameter('min_plane_points').value

        self.filter_horizontal = self.get_parameter('filter_horizontal').value
        self.normal_threshold_deg = self.get_parameter('normal_threshold_deg').value
        self.normal_threshold_rad = np.deg2rad(self.normal_threshold_deg)

        self.max_planes = self.get_parameter('max_planes').value
        self.process_rate = self.get_parameter('process_rate').value

        # 出力パスの設定
        output_path = self.get_parameter('output_path').value
        if not output_path:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_path = f'plane_detection_{timestamp}.mp4'
        self.output_path = output_path

        self.output_fps = self.get_parameter('output_fps').value
        self.show_window = self.get_parameter('show_window').value
        self.layout = self.get_parameter('layout').value

    def camera_info_callback(self, msg: CameraInfo):
        """CameraInfoを保存"""
        if self.camera_info is None:
            self.get_logger().info(f'Received camera info: {msg.width}x{msg.height}')
        self.camera_info = msg

    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        """同期されたRGB-Dフレームを処理"""
        self.frame_count += 1

        # レート制限
        if self.frame_count % max(1, int(30 / self.process_rate)) != 0:
            return

        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=2.0)
            return

        try:
            # 画像の変換
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth = self._convert_depth(depth_msg)

            if depth is None:
                return

            # TF取得（オプション：base座標系への変換用）
            transform = self._get_transform(rgb_msg.header.stamp)

            # 点群生成
            points_camera = self._create_pointcloud(depth)
            if len(points_camera) < self.min_plane_points:
                self.get_logger().warn('Not enough points in point cloud',
                                       throttle_duration_sec=2.0)
                return

            # 点群をbase座標系に変換（TFがある場合）
            if transform is not None:
                points_base = self._transform_points(points_camera, transform)
                gravity_direction = np.array([0.0, 0.0, 1.0])
            else:
                points_base = points_camera
                gravity_direction = np.array([0.0, -1.0, 0.0])  # カメラ座標系

            # 平面検出（無差別）
            detected_planes = self._detect_planes(
                points_camera, points_base, gravity_direction)

            # フレーム作成
            frame = self._create_recording_frame(rgb, depth, detected_planes)

            # 録画
            self._record_frame(frame)

            # プレビュー表示
            if self.show_window:
                self._show_preview(frame)

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _convert_depth(self, depth_msg: Image) -> Optional[np.ndarray]:
        """深度画像を変換"""
        try:
            if depth_msg.encoding == '16UC1':
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
                depth_m = depth.astype(np.float32) / self.depth_scale
            elif depth_msg.encoding == '32FC1':
                depth_m = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unknown depth encoding: {depth_msg.encoding}')
                return None

            return depth_m
        except Exception as e:
            self.get_logger().error(f'Error converting depth: {e}')
            return None

    def _get_transform(self, stamp) -> Optional[np.ndarray]:
        """カメラ座標系からbase_footprint座標系への変換を取得"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            self.latest_transform = transform
            return self._transform_to_matrix(transform)
        except TransformException:
            # 最新のTFが利用できない場合、キャッシュを使用
            if self.latest_transform is not None:
                return self._transform_to_matrix(self.latest_transform)
            return None

    def _transform_to_matrix(self, transform) -> np.ndarray:
        """TransformStampedから4x4変換行列を作成"""
        t = transform.transform.translation
        r = transform.transform.rotation

        # クォータニオンから回転行列
        x, y, z, w = r.x, r.y, r.z, r.w
        rotation_matrix = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

        # 4x4変換行列
        matrix = np.eye(4)
        matrix[:3, :3] = rotation_matrix
        matrix[:3, 3] = [t.x, t.y, t.z]

        return matrix

    def _create_pointcloud(self, depth: np.ndarray) -> np.ndarray:
        """深度画像から点群を生成（カメラ座標系）"""
        h, w = depth.shape
        ds = self.downsample_factor

        # カメラパラメータ
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # ダウンサンプリング
        depth_ds = depth[::ds, ::ds]

        # ピクセル座標のグリッド
        u = np.arange(0, w, ds)
        v = np.arange(0, h, ds)
        u, v = np.meshgrid(u, v)

        # 有効な深度のマスク
        mask = (depth_ds > self.min_depth) & (depth_ds < self.max_depth)

        # 3D座標の計算
        z = depth_ds[mask]
        x = (u[mask] - cx) * z / fx
        y = (v[mask] - cy) * z / fy

        # (N, 3)の点群
        points = np.vstack([x, y, z]).T

        return points

    def _transform_points(self, points: np.ndarray, transform: np.ndarray) -> np.ndarray:
        """点群を変換行列で変換"""
        # ホモジニアス座標に変換
        ones = np.ones((points.shape[0], 1))
        points_h = np.hstack([points, ones])

        # 変換適用
        points_transformed = (transform @ points_h.T).T

        return points_transformed[:, :3]

    def _detect_planes(self, points_camera: np.ndarray, points_base: np.ndarray,
                       gravity_direction: np.ndarray) -> List[DetectedPlane]:
        """RANSACで平面を検出（無差別または水平フィルタ付き）"""
        detected_planes = []

        # Open3D点群に変換（検出はbase座標系で行う）
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_base)

        # カメラ座標系の点群も保持
        pcd_camera = o3d.geometry.PointCloud()
        pcd_camera.points = o3d.utility.Vector3dVector(points_camera)

        remaining_indices = np.arange(len(points_base))

        for i in range(self.max_planes):
            if len(remaining_indices) < self.min_plane_points:
                break

            # 現在の点群を抽出
            current_pcd = pcd.select_by_index(remaining_indices)

            # RANSAC平面検出
            try:
                plane_model, inliers = current_pcd.segment_plane(
                    distance_threshold=self.distance_threshold,
                    ransac_n=self.ransac_n,
                    num_iterations=self.num_iterations
                )
            except Exception:
                break

            if len(inliers) < self.min_plane_points:
                break

            # 平面の法線ベクトル
            a, b, c, d = plane_model
            normal = np.array([a, b, c])
            normal = normal / np.linalg.norm(normal)

            # 水平平面フィルタリング（オプション）
            if self.filter_horizontal:
                # 法線が上向きになるように調整
                if normal[2] < 0:
                    normal = -normal
                angle = np.arccos(np.clip(np.dot(normal, gravity_direction), -1.0, 1.0))
                if angle > self.normal_threshold_rad:
                    # 水平でない平面はスキップするが、点群からは除去
                    mask = np.ones(len(remaining_indices), dtype=bool)
                    mask[inliers] = False
                    remaining_indices = remaining_indices[mask]
                    continue

            # 平面を採用
            actual_indices = remaining_indices[inliers]
            plane_points_base = points_base[actual_indices]
            plane_points_camera = points_camera[actual_indices]

            # 平面の中心
            center = np.mean(plane_points_base, axis=0)

            # 色の取得
            color = self.plane_colors[len(detected_planes) % len(self.plane_colors)]

            detected_plane = DetectedPlane(
                plane_id=len(detected_planes) + 1,
                coefficients=plane_model,
                inlier_indices=actual_indices,
                center=center,
                normal=normal,
                points=plane_points_camera,
                points_base=plane_points_base,
                color=color
            )
            detected_planes.append(detected_plane)

            self.get_logger().debug(
                f'Detected plane {len(detected_planes)}: center={center}, '
                f'points={len(inliers)}'
            )

            # 検出した平面の点を除去
            mask = np.ones(len(remaining_indices), dtype=bool)
            mask[inliers] = False
            remaining_indices = remaining_indices[mask]

        return detected_planes

    def _create_recording_frame(self, rgb: np.ndarray, depth: np.ndarray,
                                planes: List[DetectedPlane]) -> np.ndarray:
        """録画用フレームを作成"""
        h, w = depth.shape

        # RGB画像にオーバーレイ（BGR変換）
        overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # カメラパラメータ
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # 各平面の点を描画
        for plane in planes:
            for point in plane.points:
                x, y, z = point
                if z <= 0:
                    continue

                u = int(fx * x / z + cx)
                v = int(fy * y / z + cy)

                if 0 <= u < w and 0 <= v < h:
                    cv2.circle(overlay, (u, v), 2, plane.color, -1)

        # ラベルを追加
        y_offset = 30
        for i, plane in enumerate(planes):
            # base座標系での高さを表示（TFがある場合）
            height_str = f'{plane.center[2]:.2f}m' if self.latest_transform else ''
            label = f'Plane {plane.plane_id}: {len(plane.points)} pts {height_str}'
            cv2.putText(overlay, label, (10, y_offset + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, plane.color, 2)

        # 検出数表示
        cv2.putText(overlay, f'Detected: {len(planes)} planes',
                    (w - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # フレーム番号表示
        cv2.putText(overlay, f'Frame: {self.recorded_frames}',
                    (w - 200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 深度画像をカラーマップで表示
        depth_normalized = np.clip(depth / self.max_depth, 0, 1)
        depth_colormap = cv2.applyColorMap(
            (depth_normalized * 255).astype(np.uint8),
            cv2.COLORMAP_VIRIDIS
        )

        # 無効な深度は黒に
        depth_colormap[depth <= 0] = [0, 0, 0]

        # 深度画像にも平面をオーバーレイ
        for plane in planes:
            for point in plane.points:
                x, y, z = point
                if z <= 0:
                    continue

                u = int(fx * x / z + cx)
                v = int(fy * y / z + cy)

                if 0 <= u < w and 0 <= v < h:
                    cv2.circle(depth_colormap, (u, v), 2, plane.color, -1)

        # 凡例を追加（深度側）
        cv2.putText(depth_colormap, 'Depth + Planes',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 画像を結合
        if self.layout == 'horizontal':
            # 横に並べる
            h1, w1 = overlay.shape[:2]
            h2, w2 = depth_colormap.shape[:2]
            if h1 != h2:
                scale = h1 / h2
                depth_colormap = cv2.resize(depth_colormap, (int(w2 * scale), h1))
            combined = np.hstack([overlay, depth_colormap])
        else:
            # 縦に並べる
            h1, w1 = overlay.shape[:2]
            h2, w2 = depth_colormap.shape[:2]
            if w1 != w2:
                scale = w1 / w2
                depth_colormap = cv2.resize(depth_colormap, (w1, int(h2 * scale)))
            combined = np.vstack([overlay, depth_colormap])

        return combined

    def _record_frame(self, frame: np.ndarray):
        """フレームを録画"""
        if not self.video_initialized:
            self._init_video_writer(frame)

        if self.video_writer is not None:
            self.video_writer.write(frame)
            self.recorded_frames += 1

            if self.recorded_frames % 100 == 0:
                self.get_logger().info(f'Recorded {self.recorded_frames} frames')

    def _init_video_writer(self, frame: np.ndarray):
        """VideoWriterを初期化"""
        h, w = frame.shape[:2]

        # 出力ディレクトリの作成
        output_dir = os.path.dirname(self.output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # コーデック設定
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        self.video_writer = cv2.VideoWriter(
            self.output_path,
            fourcc,
            self.output_fps,
            (w, h)
        )

        if self.video_writer.isOpened():
            self.get_logger().info(f'Video writer initialized: {self.output_path}')
            self.get_logger().info(f'  Resolution: {w}x{h}')
            self.get_logger().info(f'  FPS: {self.output_fps}')
            self.video_initialized = True
        else:
            self.get_logger().error(f'Failed to initialize video writer: {self.output_path}')
            self.video_writer = None

    def _show_preview(self, frame: np.ndarray):
        """プレビューウィンドウを表示"""
        # 大きすぎる場合はリサイズ
        max_width = 1280
        h, w = frame.shape[:2]
        if w > max_width:
            scale = max_width / w
            frame_preview = cv2.resize(frame, (int(w * scale), int(h * scale)))
        else:
            frame_preview = frame

        cv2.imshow('Plane Detection Recording (Q: quit)', frame_preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            self.get_logger().info('Quit requested')
            self._finalize_video()
            rclpy.shutdown()

    def _finalize_video(self):
        """録画を終了して保存"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Video saved: {self.output_path}')
            self.get_logger().info(f'  Total frames: {self.recorded_frames}')
            self.video_writer = None

    def destroy_node(self):
        """ノードの破棄"""
        self._finalize_video()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PlaneRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
