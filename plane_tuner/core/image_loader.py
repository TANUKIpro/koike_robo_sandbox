"""
Image and rosbag loading utilities for plane_tuner.
Supports:
- Direct RGB + Depth image file pairs
- ROS2 rosbag extraction (using rosbags library - no ROS2 required)
- TUM RGB-D dataset format
- RoboCup dataset (TIAGo rosbag)
"""

import cv2
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any
import struct


# Default RoboCup/TIAGo topics
TIAGO_RGB_TOPIC = "/head_front_camera/rgb/image_raw"
TIAGO_DEPTH_TOPIC = "/head_front_camera/depth/image_raw"
TIAGO_CAMERA_INFO_TOPIC = "/head_front_camera/rgb/camera_info"


@dataclass
class RGBDFrame:
    """Container for RGB-D frame data."""
    rgb: np.ndarray          # RGB image (H, W, 3) uint8
    depth: np.ndarray        # Depth image (H, W) float32 in meters
    timestamp: float = 0.0   # Timestamp in seconds
    rgb_path: str = ""       # Source path for RGB
    depth_path: str = ""     # Source path for Depth

    @property
    def height(self) -> int:
        return self.rgb.shape[0]

    @property
    def width(self) -> int:
        return self.rgb.shape[1]


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters."""
    fx: float = 525.0    # Focal length x
    fy: float = 525.0    # Focal length y
    cx: float = 319.5    # Principal point x
    cy: float = 239.5    # Principal point y
    width: int = 640
    height: int = 480
    depth_scale: float = 1000.0  # Depth units to meters (1000 for mm)

    @classmethod
    def from_tum_freiburg1(cls) -> 'CameraIntrinsics':
        """TUM Freiburg1 camera parameters."""
        return cls(fx=517.3, fy=516.5, cx=318.6, cy=255.3, depth_scale=5000.0)

    @classmethod
    def from_tum_freiburg2(cls) -> 'CameraIntrinsics':
        """TUM Freiburg2 camera parameters."""
        return cls(fx=520.9, fy=521.0, cx=325.1, cy=249.7, depth_scale=5000.0)

    @classmethod
    def from_tum_freiburg3(cls) -> 'CameraIntrinsics':
        """TUM Freiburg3 camera parameters."""
        return cls(fx=535.4, fy=539.2, cx=320.1, cy=247.6, depth_scale=5000.0)

    @classmethod
    def from_tiago(cls) -> 'CameraIntrinsics':
        """TIAGo robot camera parameters (Orbbec Astra Pro)."""
        return cls(fx=570.3, fy=570.3, cx=319.5, cy=239.5, depth_scale=1000.0)

    @classmethod
    def from_robocup_tiago(cls) -> 'CameraIntrinsics':
        """RoboCup 2023-2024 TIAGo dataset camera parameters."""
        return cls(fx=570.3, fy=570.3, cx=319.5, cy=239.5, depth_scale=1000.0)

    @classmethod
    def from_realsense_d435(cls) -> 'CameraIntrinsics':
        """Intel RealSense D435 (typical 640x480)."""
        return cls(fx=615.0, fy=615.0, cx=320.0, cy=240.0, depth_scale=1000.0)

    @classmethod
    def from_camera_info(cls, camera_info: Dict[str, Any]) -> 'CameraIntrinsics':
        """Create from ROS CameraInfo message data."""
        k = camera_info.get('k', camera_info.get('K', [525, 0, 319.5, 0, 525, 239.5, 0, 0, 1]))
        return cls(
            fx=k[0],
            fy=k[4],
            cx=k[2],
            cy=k[5],
            width=camera_info.get('width', 640),
            height=camera_info.get('height', 480),
            depth_scale=1000.0  # Default for TIAGo
        )


class ImageLoader:
    """Load RGB-D images from various sources."""

    def __init__(self, intrinsics: Optional[CameraIntrinsics] = None):
        self.intrinsics = intrinsics or CameraIntrinsics()
        self._frames: List[RGBDFrame] = []
        self._current_index = 0

    def load_image_pair(self, rgb_path: str, depth_path: str,
                        depth_scale: Optional[float] = None) -> Optional[RGBDFrame]:
        """
        Load RGB and depth image pair from files.

        Args:
            rgb_path: Path to RGB image (PNG, JPG, etc.)
            depth_path: Path to depth image (PNG 16-bit, EXR, etc.)
            depth_scale: Override depth scale (units to meters)

        Returns:
            RGBDFrame or None if loading fails
        """
        rgb_path = Path(rgb_path)
        depth_path = Path(depth_path)

        if not rgb_path.exists():
            print(f"RGB image not found: {rgb_path}")
            return None
        if not depth_path.exists():
            print(f"Depth image not found: {depth_path}")
            return None

        # Load RGB
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if rgb is None:
            print(f"Failed to load RGB: {rgb_path}")
            return None
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        # Load depth
        depth = self._load_depth(str(depth_path), depth_scale)
        if depth is None:
            print(f"Failed to load depth: {depth_path}")
            return None

        frame = RGBDFrame(
            rgb=rgb,
            depth=depth,
            rgb_path=str(rgb_path),
            depth_path=str(depth_path)
        )
        self._frames = [frame]
        self._current_index = 0
        return frame

    def _load_depth(self, path: str, depth_scale: Optional[float] = None) -> Optional[np.ndarray]:
        """Load depth image and convert to meters."""
        scale = depth_scale if depth_scale is not None else self.intrinsics.depth_scale

        # Try loading as 16-bit PNG first
        depth = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            return None

        if depth.dtype == np.uint16:
            # 16-bit depth (common format)
            depth_m = depth.astype(np.float32) / scale
        elif depth.dtype == np.float32:
            # Already float (EXR, etc.)
            depth_m = depth
        elif depth.dtype == np.uint8:
            # 8-bit (normalized, need to scale)
            print("Warning: 8-bit depth detected, assuming normalized 0-255 -> 0-10m")
            depth_m = depth.astype(np.float32) / 25.5  # 255 -> 10m
        else:
            depth_m = depth.astype(np.float32) / scale

        # Handle invalid depth values
        depth_m[depth_m <= 0] = 0
        depth_m[np.isnan(depth_m)] = 0
        depth_m[np.isinf(depth_m)] = 0

        return depth_m

    def load_tum_sequence(self, dataset_path: str, max_frames: int = 100) -> int:
        """
        Load TUM RGB-D dataset sequence.

        Args:
            dataset_path: Path to TUM dataset folder (containing rgb/, depth/, associations.txt)
            max_frames: Maximum frames to load

        Returns:
            Number of frames loaded
        """
        dataset_path = Path(dataset_path)

        # Try to find associations file
        assoc_file = dataset_path / "associations.txt"
        if not assoc_file.exists():
            # Try to create from rgb.txt and depth.txt
            assoc_file = self._create_associations(dataset_path)
            if assoc_file is None:
                print("Could not find or create associations.txt")
                return 0

        # Detect which Freiburg dataset
        if "freiburg1" in str(dataset_path).lower():
            self.intrinsics = CameraIntrinsics.from_tum_freiburg1()
        elif "freiburg2" in str(dataset_path).lower():
            self.intrinsics = CameraIntrinsics.from_tum_freiburg2()
        elif "freiburg3" in str(dataset_path).lower():
            self.intrinsics = CameraIntrinsics.from_tum_freiburg3()

        self._frames = []
        with open(assoc_file, 'r') as f:
            for i, line in enumerate(f):
                if i >= max_frames:
                    break

                parts = line.strip().split()
                if len(parts) < 4:
                    continue

                ts_rgb, rgb_file, ts_depth, depth_file = parts[:4]
                rgb_path = dataset_path / rgb_file
                depth_path = dataset_path / depth_file

                if not rgb_path.exists() or not depth_path.exists():
                    continue

                rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
                if rgb is None:
                    continue
                rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

                depth = self._load_depth(str(depth_path))
                if depth is None:
                    continue

                frame = RGBDFrame(
                    rgb=rgb,
                    depth=depth,
                    timestamp=float(ts_rgb),
                    rgb_path=str(rgb_path),
                    depth_path=str(depth_path)
                )
                self._frames.append(frame)

        self._current_index = 0
        return len(self._frames)

    def _create_associations(self, dataset_path: Path) -> Optional[Path]:
        """Create associations file from rgb.txt and depth.txt."""
        rgb_txt = dataset_path / "rgb.txt"
        depth_txt = dataset_path / "depth.txt"

        if not rgb_txt.exists() or not depth_txt.exists():
            return None

        def read_file_list(path):
            data = {}
            with open(path, 'r') as f:
                for line in f:
                    if line.startswith('#'):
                        continue
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        ts = float(parts[0])
                        data[ts] = parts[1]
            return data

        rgb_data = read_file_list(rgb_txt)
        depth_data = read_file_list(depth_txt)

        # Associate by timestamp (within 0.02s)
        associations = []
        for ts_rgb, rgb_file in sorted(rgb_data.items()):
            best_ts = None
            best_diff = float('inf')
            for ts_depth in depth_data:
                diff = abs(ts_rgb - ts_depth)
                if diff < best_diff and diff < 0.02:
                    best_diff = diff
                    best_ts = ts_depth

            if best_ts is not None:
                associations.append((ts_rgb, rgb_file, best_ts, depth_data[best_ts]))

        # Write associations file
        assoc_path = dataset_path / "associations.txt"
        with open(assoc_path, 'w') as f:
            for ts_rgb, rgb_file, ts_depth, depth_file in associations:
                f.write(f"{ts_rgb:.6f} {rgb_file} {ts_depth:.6f} {depth_file}\n")

        return assoc_path

    def load_rosbag_sequence(
        self,
        bag_path: str,
        rgb_topic: str = TIAGO_RGB_TOPIC,
        depth_topic: str = TIAGO_DEPTH_TOPIC,
        camera_info_topic: str = TIAGO_CAMERA_INFO_TOPIC,
        max_frames: int = 100,
        skip_frames: int = 0
    ) -> int:
        """
        Load multiple frames from ROS2 rosbag.

        Uses rosbags library (no ROS2 installation required).

        Args:
            bag_path: Path to rosbag directory or file
            rgb_topic: RGB image topic name
            depth_topic: Depth image topic name
            camera_info_topic: Camera info topic name
            max_frames: Maximum frames to load
            skip_frames: Skip first N frames

        Returns:
            Number of frames loaded
        """
        try:
            return self._load_rosbag_sequence_rosbags(
                bag_path, rgb_topic, depth_topic, camera_info_topic, max_frames, skip_frames
            )
        except ImportError as e:
            print(f"rosbags library not available: {e}")
            print("Install with: pip install rosbags")
            return 0

    def _load_rosbag_sequence_rosbags(
        self,
        bag_path: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        max_frames: int,
        skip_frames: int
    ) -> int:
        """Load rosbag using rosbags library (no ROS2 required)."""
        from pathlib import Path as RosbagPath

        # Try new API first (rosbags >= 0.10), then fall back to old API
        try:
            return self._load_rosbag_new_api(
                bag_path, rgb_topic, depth_topic, camera_info_topic, max_frames, skip_frames
            )
        except ImportError:
            return self._load_rosbag_old_api(
                bag_path, rgb_topic, depth_topic, camera_info_topic, max_frames, skip_frames
            )

    def _load_rosbag_new_api(
        self,
        bag_path: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        max_frames: int,
        skip_frames: int
    ) -> int:
        """Load rosbag using new rosbags API (>= 0.10)."""
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore

        bag_path = Path(bag_path)

        # Create type store for ROS2 Humble (or Foxy as fallback)
        try:
            typestore = get_typestore(Stores.ROS2_HUMBLE)
        except AttributeError:
            typestore = get_typestore(Stores.ROS2_FOXY)

        # Collect RGB and depth messages by timestamp (deserialize immediately)
        rgb_messages: Dict[int, Any] = {}
        depth_messages: Dict[int, Any] = {}
        camera_info = None

        print(f"Reading rosbag from: {bag_path}")

        with AnyReader([bag_path], default_typestore=typestore) as reader:
            # Print available topics
            print("Available topics:")
            for conn in reader.connections:
                print(f"  {conn.topic}: {conn.msgtype}")

            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic

                if topic == camera_info_topic and camera_info is None:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    camera_info = {
                        'k': list(msg.k),
                        'width': msg.width,
                        'height': msg.height
                    }
                    self.intrinsics = CameraIntrinsics.from_camera_info(camera_info)
                    print(f"Camera intrinsics loaded: fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}")

                elif topic == rgb_topic:
                    # Deserialize immediately within the context
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    rgb_messages[timestamp] = msg

                elif topic == depth_topic:
                    # Deserialize immediately within the context
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    depth_messages[timestamp] = msg

        return self._process_deserialized_messages(
            rgb_messages, depth_messages, max_frames, skip_frames
        )

    def _load_rosbag_old_api(
        self,
        bag_path: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        max_frames: int,
        skip_frames: int
    ) -> int:
        """Load rosbag using old rosbags API (< 0.10)."""
        from rosbags.rosbag2 import Reader
        from rosbags.serde import deserialize_cdr

        bag_path = Path(bag_path)

        # Collect RGB and depth messages by timestamp (deserialize immediately)
        rgb_messages: Dict[int, Any] = {}
        depth_messages: Dict[int, Any] = {}
        camera_info = None

        print(f"Reading rosbag from: {bag_path}")

        with Reader(bag_path) as reader:
            # Print available topics
            print("Available topics:")
            for conn in reader.connections:
                print(f"  {conn.topic}: {conn.msgtype}")

            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic

                if topic == camera_info_topic and camera_info is None:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    camera_info = {
                        'k': list(msg.k),
                        'width': msg.width,
                        'height': msg.height
                    }
                    self.intrinsics = CameraIntrinsics.from_camera_info(camera_info)
                    print(f"Camera intrinsics loaded: fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}")

                elif topic == rgb_topic:
                    # Deserialize immediately
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    rgb_messages[timestamp] = msg

                elif topic == depth_topic:
                    # Deserialize immediately
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    depth_messages[timestamp] = msg

        return self._process_deserialized_messages(
            rgb_messages, depth_messages, max_frames, skip_frames
        )

    def _process_deserialized_messages(
        self,
        rgb_messages: Dict[int, Any],
        depth_messages: Dict[int, Any],
        max_frames: int,
        skip_frames: int
    ) -> int:
        """Process already-deserialized rosbag messages into frames."""
        print(f"Found {len(rgb_messages)} RGB and {len(depth_messages)} depth messages")

        self._frames = []
        rgb_timestamps = sorted(rgb_messages.keys())

        frame_count = 0
        for rgb_ts in rgb_timestamps:
            if frame_count < skip_frames:
                frame_count += 1
                continue

            if len(self._frames) >= max_frames:
                break

            # Find closest depth timestamp
            best_depth_ts = None
            best_diff = float('inf')
            for depth_ts in depth_messages:
                diff = abs(rgb_ts - depth_ts)
                if diff < best_diff:
                    best_diff = diff
                    best_depth_ts = depth_ts

            # Allow up to 100ms difference
            if best_depth_ts is None or best_diff > 100_000_000:  # 100ms in nanoseconds
                frame_count += 1
                continue

            # Get already-deserialized messages
            rgb_msg = rgb_messages[rgb_ts]
            depth_msg = depth_messages[best_depth_ts]

            # Convert RGB
            rgb = self._convert_ros_image(rgb_msg)
            if rgb is None:
                frame_count += 1
                continue

            # Convert Depth
            depth = self._convert_ros_depth(depth_msg)
            if depth is None:
                frame_count += 1
                continue

            frame = RGBDFrame(
                rgb=rgb,
                depth=depth,
                timestamp=rgb_ts / 1e9,
                rgb_path=f"rosbag:{rgb_ts}",
                depth_path=f"rosbag:{best_depth_ts}"
            )
            self._frames.append(frame)
            frame_count += 1

        self._current_index = 0
        print(f"Loaded {len(self._frames)} RGB-D frames from rosbag")
        return len(self._frames)

    def _convert_ros_image(self, msg) -> Optional[np.ndarray]:
        """Convert ROS Image message to numpy array."""
        try:
            if msg.encoding in ['rgb8']:
                rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                rgb = rgb.reshape((msg.height, msg.width, 3))
                return rgb
            elif msg.encoding in ['bgr8']:
                bgr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                bgr = bgr.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            else:
                print(f"Unsupported RGB encoding: {msg.encoding}")
                return None
        except Exception as e:
            print(f"Error converting RGB image: {e}")
            return None

    def _convert_ros_depth(self, msg) -> Optional[np.ndarray]:
        """Convert ROS depth Image message to numpy array (in meters)."""
        try:
            if msg.encoding == '16UC1':
                depth = np.frombuffer(bytes(msg.data), dtype=np.uint16)
                depth = depth.reshape((msg.height, msg.width))
                depth_m = depth.astype(np.float32) / self.intrinsics.depth_scale
            elif msg.encoding == '32FC1':
                depth_m = np.frombuffer(bytes(msg.data), dtype=np.float32)
                depth_m = depth_m.reshape((msg.height, msg.width))
            else:
                print(f"Unsupported depth encoding: {msg.encoding}")
                return None

            # Handle invalid values
            depth_m[depth_m <= 0] = 0
            depth_m[np.isnan(depth_m)] = 0
            depth_m[np.isinf(depth_m)] = 0

            return depth_m
        except Exception as e:
            print(f"Error converting depth image: {e}")
            return None

    def load_robocup_rosbag(
        self,
        bag_path: str,
        max_frames: int = 100,
        skip_frames: int = 0
    ) -> int:
        """
        Convenience method to load RoboCup TIAGo rosbag with default topics.

        Args:
            bag_path: Path to RoboCup rosbag (e.g., datasets/robocup/storing_try_2)
            max_frames: Maximum frames to load
            skip_frames: Skip first N frames

        Returns:
            Number of frames loaded
        """
        # Set TIAGo intrinsics as default
        self.intrinsics = CameraIntrinsics.from_robocup_tiago()

        return self.load_rosbag_sequence(
            bag_path,
            rgb_topic=TIAGO_RGB_TOPIC,
            depth_topic=TIAGO_DEPTH_TOPIC,
            camera_info_topic=TIAGO_CAMERA_INFO_TOPIC,
            max_frames=max_frames,
            skip_frames=skip_frames
        )

    def load_rosbag_frame(self, bag_path: str, rgb_topic: str, depth_topic: str,
                          frame_index: int = 0) -> Optional[RGBDFrame]:
        """
        Extract a single frame from ROS2 rosbag.

        Note: For loading multiple frames, use load_rosbag_sequence() instead.
        """
        count = self.load_rosbag_sequence(
            bag_path, rgb_topic, depth_topic,
            max_frames=frame_index + 1, skip_frames=frame_index
        )
        if count > 0:
            return self._frames[0]
        return None

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def current_frame(self) -> Optional[RGBDFrame]:
        if 0 <= self._current_index < len(self._frames):
            return self._frames[self._current_index]
        return None

    def get_frame(self, index: int) -> Optional[RGBDFrame]:
        if 0 <= index < len(self._frames):
            self._current_index = index
            return self._frames[index]
        return None

    def next_frame(self) -> Optional[RGBDFrame]:
        return self.get_frame(self._current_index + 1)

    def prev_frame(self) -> Optional[RGBDFrame]:
        return self.get_frame(self._current_index - 1)


def save_frame_pair(frame: RGBDFrame, output_dir: str, prefix: str = "frame"):
    """Save RGB-D frame as image pair for later use."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # Save RGB
    rgb_bgr = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(output_path / f"{prefix}_rgb.png"), rgb_bgr)

    # Save depth as 16-bit PNG (millimeters)
    depth_mm = (frame.depth * 1000).astype(np.uint16)
    cv2.imwrite(str(output_path / f"{prefix}_depth.png"), depth_mm)

    print(f"Saved to {output_path}")
