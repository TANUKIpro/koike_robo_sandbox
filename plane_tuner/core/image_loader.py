"""
Image and rosbag loading utilities for plane_tuner.
Supports:
- Direct RGB + Depth image file pairs
- ROS2 rosbag extraction
- TUM RGB-D dataset format
"""

import cv2
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List
import struct


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
        """TIAGo robot camera parameters (typical)."""
        return cls(fx=554.25, fy=554.25, cx=320.5, cy=240.5, depth_scale=1000.0)

    @classmethod
    def from_realsense_d435(cls) -> 'CameraIntrinsics':
        """Intel RealSense D435 (typical 640x480)."""
        return cls(fx=615.0, fy=615.0, cx=320.0, cy=240.0, depth_scale=1000.0)


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

    def load_rosbag_frame(self, bag_path: str, rgb_topic: str, depth_topic: str,
                          frame_index: int = 0) -> Optional[RGBDFrame]:
        """
        Extract a single frame from ROS2 rosbag.

        Note: Requires rosbag2_py which is part of ROS2 installation.
        Falls back to mcap direct reading if rosbag2_py not available.
        """
        try:
            return self._load_rosbag_ros2(bag_path, rgb_topic, depth_topic, frame_index)
        except ImportError:
            print("rosbag2_py not available, trying mcap direct read...")
            return self._load_rosbag_mcap(bag_path, rgb_topic, depth_topic, frame_index)

    def _load_rosbag_ros2(self, bag_path: str, rgb_topic: str, depth_topic: str,
                          frame_index: int) -> Optional[RGBDFrame]:
        """Load from rosbag using rosbag2_py."""
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image

        storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        rgb_msg = None
        depth_msg = None
        rgb_count = 0
        depth_count = 0

        while reader.has_next():
            topic, data, ts = reader.read_next()

            if topic == rgb_topic:
                if rgb_count == frame_index:
                    rgb_msg = deserialize_message(data, Image)
                rgb_count += 1
            elif topic == depth_topic:
                if depth_count == frame_index:
                    depth_msg = deserialize_message(data, Image)
                depth_count += 1

            if rgb_msg and depth_msg:
                break

        if rgb_msg is None or depth_msg is None:
            return None

        # Convert RGB
        if rgb_msg.encoding in ['rgb8', 'bgr8']:
            rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8)
            rgb = rgb.reshape((rgb_msg.height, rgb_msg.width, 3))
            if rgb_msg.encoding == 'bgr8':
                rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        else:
            print(f"Unsupported RGB encoding: {rgb_msg.encoding}")
            return None

        # Convert Depth
        if depth_msg.encoding == '16UC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.uint16)
            depth = depth.reshape((depth_msg.height, depth_msg.width))
            depth_m = depth.astype(np.float32) / self.intrinsics.depth_scale
        elif depth_msg.encoding == '32FC1':
            depth_m = np.frombuffer(depth_msg.data, dtype=np.float32)
            depth_m = depth_m.reshape((depth_msg.height, depth_msg.width))
        else:
            print(f"Unsupported depth encoding: {depth_msg.encoding}")
            return None

        return RGBDFrame(rgb=rgb, depth=depth_m, timestamp=ts / 1e9)

    def _load_rosbag_mcap(self, bag_path: str, rgb_topic: str, depth_topic: str,
                          frame_index: int) -> Optional[RGBDFrame]:
        """Load from mcap rosbag directly without ROS2."""
        try:
            from mcap.reader import make_reader
            from mcap_ros2.decoder import DecoderFactory
        except ImportError:
            print("mcap and mcap-ros2-support packages required.")
            print("Install with: pip install mcap mcap-ros2-support")
            return None

        bag_file = Path(bag_path)
        if bag_file.is_dir():
            # Find .mcap file in directory
            mcap_files = list(bag_file.glob("*.mcap"))
            if not mcap_files:
                print(f"No .mcap file found in {bag_path}")
                return None
            bag_file = mcap_files[0]

        rgb_data = None
        depth_data = None
        rgb_count = 0
        depth_count = 0

        with open(bag_file, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])

            for schema, channel, message, decoded_msg in reader.iter_decoded_messages():
                if channel.topic == rgb_topic:
                    if rgb_count == frame_index:
                        rgb_data = decoded_msg
                    rgb_count += 1
                elif channel.topic == depth_topic:
                    if depth_count == frame_index:
                        depth_data = decoded_msg
                    depth_count += 1

                if rgb_data and depth_data:
                    break

        if rgb_data is None or depth_data is None:
            return None

        # Convert messages to numpy arrays
        rgb = np.array(rgb_data.data, dtype=np.uint8)
        rgb = rgb.reshape((rgb_data.height, rgb_data.width, 3))
        if rgb_data.encoding == 'bgr8':
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        if depth_data.encoding == '16UC1':
            depth = np.array(depth_data.data, dtype=np.uint16)
            depth = depth.reshape((depth_data.height, depth_data.width))
            depth_m = depth.astype(np.float32) / self.intrinsics.depth_scale
        else:
            depth_m = np.array(depth_data.data, dtype=np.float32)
            depth_m = depth_m.reshape((depth_data.height, depth_data.width))

        return RGBDFrame(rgb=rgb, depth=depth_m)

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
