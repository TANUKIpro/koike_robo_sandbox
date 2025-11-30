#!/usr/bin/env python3
"""
平面検出ノード - RoboCup@Home向け水平平面検出

RGB-Dカメラからの点群データを使用して、RANSAC + 法線ベクトル制約で
水平平面（床面、テーブル、棚）を検出し、可視化します。

使用方法:
    ros2 run plane_detector plane_detector_node

トピック:
    入力:
        - /head_front_camera/rgb/image_raw (sensor_msgs/Image): RGB画像
        - /head_front_camera/depth/image_raw (sensor_msgs/Image): 深度画像
        - /head_front_camera/rgb/camera_info (sensor_msgs/CameraInfo): カメラ情報
        - /tf, /tf_static: 座標変換

    出力:
        - /detected_planes/markers (visualization_msgs/MarkerArray): 検出平面のマーカー
        - /detected_planes/overlay (sensor_msgs/Image): 平面オーバーレイ画像
        - /detected_table_surface (geometry_msgs/PoseStamped): 検出したテーブル面
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
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


class PlaneType(Enum):
    """平面の種類"""
    UNKNOWN = 0
    FLOOR = 1
    TABLE = 2
    SHELF = 3


@dataclass
class DetectedPlane:
    """検出された平面の情報"""
    plane_type: PlaneType
    coefficients: np.ndarray  # [a, b, c, d] where ax + by + cz + d = 0
    inlier_indices: np.ndarray
    center: np.ndarray  # 平面の中心点 (base_footprint座標系)
    height: float  # base_footprintからの高さ
    normal: np.ndarray  # 法線ベクトル (base_footprint座標系)
    points: np.ndarray  # 平面上の点群
    color: Tuple[float, float, float]  # 可視化色


class PlaneDetectorNode(Node):
    """水平平面検出ノード"""

    def __init__(self):
        super().__init__('plane_detector_node')

        # パラメータ宣言
        self._declare_parameters()
        self._get_parameters()

        # 初期化
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.latest_transform = None
        self.frame_count = 0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/detected_planes/markers', 10)
        self.overlay_pub = self.create_publisher(
            Image, '/detected_planes/overlay', 10)
        self.table_pub = self.create_publisher(
            PoseStamped, '/detected_table_surface', 10)

        # QoS for camera info (latched)
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
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

        # 平面の色定義
        self.plane_colors = {
            PlaneType.FLOOR: (0.5, 0.5, 0.5),    # 灰色
            PlaneType.TABLE: (0.0, 1.0, 0.0),    # 緑色
            PlaneType.SHELF: (0.0, 0.5, 1.0),    # 青色
            PlaneType.UNKNOWN: (1.0, 1.0, 0.0),  # 黄色
        }

        self.get_logger().info('Plane Detector Node started')
        self.get_logger().info(f'  RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'  Depth topic: {self.depth_topic}')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  Base frame: {self.base_frame}')
        self.get_logger().info(f'  Show window: {self.show_window}')

    def _declare_parameters(self):
        """パラメータの宣言"""
        # トピック設定
        self.declare_parameter('rgb_topic', '/head_front_camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/head_front_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/head_front_camera/rgb/camera_info')

        # 座標系設定
        self.declare_parameter('camera_frame', 'head_front_camera_rgb_optical_frame')
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

        # 法線角度制約
        self.declare_parameter('normal_threshold_deg', 15.0)

        # 高さによる分類
        self.declare_parameter('floor_height_max', 0.15)
        self.declare_parameter('table_height_min', 0.40)
        self.declare_parameter('table_height_max', 1.20)

        # 処理設定
        self.declare_parameter('max_planes', 5)
        self.declare_parameter('process_rate', 5.0)  # Hz

        # 表示設定
        self.declare_parameter('show_window', True)  # OpenCVウィンドウ表示

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

        self.normal_threshold_deg = self.get_parameter('normal_threshold_deg').value
        self.normal_threshold_rad = np.deg2rad(self.normal_threshold_deg)

        self.floor_height_max = self.get_parameter('floor_height_max').value
        self.table_height_min = self.get_parameter('table_height_min').value
        self.table_height_max = self.get_parameter('table_height_max').value

        self.max_planes = self.get_parameter('max_planes').value
        self.process_rate = self.get_parameter('process_rate').value

        self.show_window = self.get_parameter('show_window').value

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

            # TF取得
            transform = self._get_transform(rgb_msg.header.stamp)
            if transform is None:
                return

            # 点群生成
            points_camera = self._create_pointcloud(depth)
            if len(points_camera) < self.min_plane_points:
                self.get_logger().warn('Not enough points in point cloud',
                                       throttle_duration_sec=2.0)
                return

            # 点群をbase_footprint座標系に変換
            points_base = self._transform_points(points_camera, transform)

            # 重力方向の取得（base_footprint座標系ではZ軸が上向き）
            gravity_direction = np.array([0.0, 0.0, 1.0])

            # 平面検出
            detected_planes = self._detect_planes(
                points_camera, points_base, gravity_direction, rgb)

            # 可視化
            self._publish_markers(detected_planes, rgb_msg.header)
            self._publish_overlay(rgb, depth, detected_planes, rgb_msg.header)

            # テーブル面があれば公開
            for plane in detected_planes:
                if plane.plane_type == PlaneType.TABLE:
                    self._publish_table_surface(plane, rgb_msg.header)
                    break

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
        except TransformException as e:
            # 最新のTFが利用できない場合、キャッシュを使用
            if self.latest_transform is not None:
                return self._transform_to_matrix(self.latest_transform)
            self.get_logger().warn(f'TF not available: {e}', throttle_duration_sec=2.0)
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
        h_ds, w_ds = depth_ds.shape

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
                       gravity_direction: np.ndarray, rgb: np.ndarray) -> List[DetectedPlane]:
        """RANSAC + 法線制約で水平平面を検出"""
        detected_planes = []

        # Open3D点群に変換
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_base)

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

            # 法線が上向きになるように調整
            if normal[2] < 0:
                normal = -normal

            # 重力方向との角度をチェック
            angle = np.arccos(np.clip(np.dot(normal, gravity_direction), -1.0, 1.0))

            if angle < self.normal_threshold_rad:
                # 水平平面として採用
                actual_indices = remaining_indices[inliers]
                plane_points = points_base[actual_indices]

                # 平面の中心と高さ
                center = np.mean(plane_points, axis=0)
                height = center[2]

                # 平面タイプの分類
                plane_type = self._classify_plane(height)

                # 色の取得
                color = self.plane_colors[plane_type]

                detected_plane = DetectedPlane(
                    plane_type=plane_type,
                    coefficients=plane_model,
                    inlier_indices=actual_indices,
                    center=center,
                    height=height,
                    normal=normal,
                    points=plane_points,
                    color=color
                )
                detected_planes.append(detected_plane)

                self.get_logger().info(
                    f'Detected {plane_type.name}: height={height:.2f}m, '
                    f'points={len(inliers)}, angle={np.rad2deg(angle):.1f}deg'
                )

            # 検出した平面の点を除去
            mask = np.ones(len(remaining_indices), dtype=bool)
            mask[inliers] = False
            remaining_indices = remaining_indices[mask]

        return detected_planes

    def _classify_plane(self, height: float) -> PlaneType:
        """高さに基づいて平面を分類"""
        if height < self.floor_height_max:
            return PlaneType.FLOOR
        elif self.table_height_min <= height <= self.table_height_max:
            return PlaneType.TABLE
        elif height > self.table_height_max:
            return PlaneType.SHELF
        else:
            return PlaneType.UNKNOWN

    def _publish_markers(self, planes: List[DetectedPlane], header: Header):
        """検出した平面をMarkerArrayとして公開"""
        marker_array = MarkerArray()

        # 古いマーカーを削除
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.header.frame_id = self.base_frame
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, plane in enumerate(planes):
            # 平面を表すマーカー（CUBE）
            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.base_frame
            marker.ns = 'detected_planes'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # 位置（平面の中心）
            marker.pose.position.x = plane.center[0]
            marker.pose.position.y = plane.center[1]
            marker.pose.position.z = plane.center[2]

            # 姿勢（法線方向に向ける）
            marker.pose.orientation.w = 1.0

            # サイズ（平面の広がりを推定）
            points_xy = plane.points[:, :2]
            if len(points_xy) > 0:
                extent = np.max(points_xy, axis=0) - np.min(points_xy, axis=0)
                marker.scale.x = max(float(extent[0]), 0.1)
                marker.scale.y = max(float(extent[1]), 0.1)
            else:
                marker.scale.x = 0.5
                marker.scale.y = 0.5
            marker.scale.z = 0.01  # 薄い板

            # 色（半透明）
            marker.color.r = float(plane.color[0])
            marker.color.g = float(plane.color[1])
            marker.color.b = float(plane.color[2])
            marker.color.a = 0.6

            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

            marker_array.markers.append(marker)

            # テキストラベル
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = self.base_frame
            text_marker.ns = 'plane_labels'
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = plane.center[0]
            text_marker.pose.position.y = plane.center[1]
            text_marker.pose.position.z = plane.center[2] + 0.1

            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f'{plane.plane_type.name}\n{plane.height:.2f}m'
            text_marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def _publish_overlay(self, rgb: np.ndarray, depth: np.ndarray,
                         planes: List[DetectedPlane], header: Header):
        """RGB画像に検出した平面をオーバーレイして公開"""
        overlay = rgb.copy()
        h, w = depth.shape
        ds = self.downsample_factor

        # カメラパラメータ
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        for plane in planes:
            # 平面上の点をカメラ座標系に逆変換
            if self.latest_transform is None:
                continue

            transform = self._transform_to_matrix(self.latest_transform)
            inv_transform = np.linalg.inv(transform)

            # base_footprint座標系からカメラ座標系へ
            points_h = np.hstack([plane.points, np.ones((len(plane.points), 1))])
            points_camera = (inv_transform @ points_h.T).T[:, :3]

            # カメラ座標系から画像座標系へ投影
            for point in points_camera:
                x, y, z = point
                if z <= 0:
                    continue

                u = int(fx * x / z + cx)
                v = int(fy * y / z + cy)

                if 0 <= u < w and 0 <= v < h:
                    # 色を半透明でオーバーレイ
                    color_bgr = (
                        int(plane.color[2] * 255),
                        int(plane.color[1] * 255),
                        int(plane.color[0] * 255)
                    )
                    # 小さな円で描画（点群なので）
                    cv2.circle(overlay, (u, v), 2, color_bgr, -1)

        # ラベルを追加
        y_offset = 30
        for i, plane in enumerate(planes):
            label = f'{plane.plane_type.name}: {plane.height:.2f}m'
            color_bgr = (
                int(plane.color[2] * 255),
                int(plane.color[1] * 255),
                int(plane.color[0] * 255)
            )
            cv2.putText(overlay, label, (10, y_offset + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_bgr, 2)

        # ROS2メッセージとして公開
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='rgb8')
        overlay_msg.header = header
        self.overlay_pub.publish(overlay_msg)

        # OpenCVウィンドウで表示
        if self.show_window:
            self._show_opencv_window(overlay, depth)

    def _show_opencv_window(self, overlay: np.ndarray, depth: np.ndarray):
        """OpenCVウィンドウで画像を表示"""
        # RGB -> BGR変換（OpenCV用）
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)

        # 深度画像をカラーマップで表示
        depth_normalized = np.clip(depth / self.max_depth, 0, 1)
        depth_colormap = cv2.applyColorMap(
            (depth_normalized * 255).astype(np.uint8),
            cv2.COLORMAP_VIRIDIS
        )

        # 無効な深度は黒に
        depth_colormap[depth <= 0] = [0, 0, 0]

        # 画像を横に並べて表示
        h1, w1 = overlay_bgr.shape[:2]
        h2, w2 = depth_colormap.shape[:2]

        # 高さを揃える
        if h1 != h2:
            scale = h1 / h2
            depth_colormap = cv2.resize(depth_colormap, (int(w2 * scale), h1))

        combined = np.hstack([overlay_bgr, depth_colormap])

        # ウィンドウに表示
        cv2.imshow('Plane Detection (Q: quit, S: save)', combined)

        # キー入力処理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            self.get_logger().info('Quit requested')
            rclpy.shutdown()
        elif key == ord('s') or key == ord('S'):
            filename = f'plane_detection_{self.frame_count:06d}.png'
            cv2.imwrite(filename, combined)
            self.get_logger().info(f'Saved: {filename}')

    def _publish_table_surface(self, plane: DetectedPlane, header: Header):
        """テーブル面のPoseStampedを公開"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = self.base_frame

        # 位置
        pose_msg.pose.position.x = float(plane.center[0])
        pose_msg.pose.position.y = float(plane.center[1])
        pose_msg.pose.position.z = float(plane.center[2])

        # 姿勢（法線方向をZ軸とする）
        # 簡易的にZ軸上向きの姿勢
        pose_msg.pose.orientation.w = 1.0

        self.table_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    node = PlaneDetectorNode()

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
