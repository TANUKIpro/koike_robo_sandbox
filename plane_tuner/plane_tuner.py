#!/usr/bin/env python3
"""
Plane Detection Parameter Tuner

A GUI application for tuning plane detection parameters on RGB-D data.
Supports various algorithms and provides real-time visualization of detection results.

Usage:
    python plane_tuner.py [--rgb RGB_PATH] [--depth DEPTH_PATH] [--dataset TUM_PATH]
    python plane_tuner.py --rosbag ROSBAG_PATH [--max-frames N]

Examples:
    # Launch GUI only
    python plane_tuner.py

    # Load image pair at startup
    python plane_tuner.py --rgb image_rgb.png --depth image_depth.png

    # Load TUM dataset at startup
    python plane_tuner.py --dataset /path/to/rgbd_dataset_freiburg1_desk

    # Load RoboCup rosbag at startup
    python plane_tuner.py --rosbag datasets/robocup/storing_try_2 --preset robocup-tiago

    # Load rosbag with custom topics
    python plane_tuner.py --rosbag /path/to/rosbag --rgb-topic /camera/rgb --depth-topic /camera/depth
"""

import sys
import argparse
from pathlib import Path

# Add package to path
package_dir = Path(__file__).parent
sys.path.insert(0, str(package_dir))

from gui.main_window import MainWindow
from core.image_loader import (
    ImageLoader, CameraIntrinsics,
    TIAGO_RGB_TOPIC, TIAGO_DEPTH_TOPIC, TIAGO_CAMERA_INFO_TOPIC
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plane Detection Parameter Tuner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    # Image pair
    parser.add_argument(
        '--rgb', type=str, default=None,
        help='Path to RGB image'
    )
    parser.add_argument(
        '--depth', type=str, default=None,
        help='Path to depth image'
    )

    # Dataset options
    parser.add_argument(
        '--dataset', type=str, default=None,
        help='Path to TUM RGB-D dataset directory'
    )
    parser.add_argument(
        '--rosbag', type=str, default=None,
        help='Path to ROS2 rosbag directory'
    )

    # Rosbag options
    parser.add_argument(
        '--rgb-topic', type=str, default=TIAGO_RGB_TOPIC,
        help=f'RGB image topic (default: {TIAGO_RGB_TOPIC})'
    )
    parser.add_argument(
        '--depth-topic', type=str, default=TIAGO_DEPTH_TOPIC,
        help=f'Depth image topic (default: {TIAGO_DEPTH_TOPIC})'
    )
    parser.add_argument(
        '--max-frames', type=int, default=100,
        help='Maximum frames to load from dataset/rosbag (default: 100)'
    )
    parser.add_argument(
        '--skip-frames', type=int, default=0,
        help='Skip first N frames (default: 0)'
    )

    # Camera parameters
    parser.add_argument(
        '--depth-scale', type=float, default=None,
        help='Depth scale factor (mm to meters: 1000.0 for TIAGo, 5000.0 for TUM)'
    )
    parser.add_argument(
        '--preset', type=str, default=None,
        choices=['tum-freiburg1', 'tum-freiburg2', 'tum-freiburg3', 'tiago', 'robocup-tiago', 'realsense-d435'],
        help='Camera intrinsics preset'
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Create application
    app = MainWindow()

    # Apply camera preset if specified
    if args.preset:
        presets = {
            'tum-freiburg1': CameraIntrinsics.from_tum_freiburg1(),
            'tum-freiburg2': CameraIntrinsics.from_tum_freiburg2(),
            'tum-freiburg3': CameraIntrinsics.from_tum_freiburg3(),
            'tiago': CameraIntrinsics.from_tiago(),
            'robocup-tiago': CameraIntrinsics.from_robocup_tiago(),
            'realsense-d435': CameraIntrinsics.from_realsense_d435(),
        }
        app.image_loader.intrinsics = presets[args.preset]

    # Override depth scale if specified
    if args.depth_scale:
        app.image_loader.intrinsics.depth_scale = args.depth_scale

    # Load initial data
    if args.rosbag:
        # Load rosbag
        print(f"Loading rosbag from: {args.rosbag}")
        count = app.image_loader.load_rosbag_sequence(
            args.rosbag,
            rgb_topic=args.rgb_topic,
            depth_topic=args.depth_topic,
            max_frames=args.max_frames,
            skip_frames=args.skip_frames
        )
        if count > 0:
            app._update_frame_slider()
            frame = app.image_loader.get_frame(0)
            if frame:
                app._set_current_frame(frame)
            app.status_var.set(f"Loaded {count} frames from rosbag")
        else:
            print("Warning: Failed to load rosbag. Check that:")
            print("  1. The path is correct and contains rosbag data")
            print("  2. 'rosbags' package is installed: pip install rosbags")

    elif args.dataset:
        # Load TUM dataset
        count = app.image_loader.load_tum_sequence(args.dataset, max_frames=args.max_frames)
        if count > 0:
            app._update_frame_slider()
            frame = app.image_loader.get_frame(0)
            if frame:
                app._set_current_frame(frame)
            app.status_var.set(f"Loaded {count} frames from TUM dataset")

    elif args.rgb and args.depth:
        # Load image pair
        frame = app.image_loader.load_image_pair(args.rgb, args.depth, args.depth_scale)
        if frame:
            app._set_current_frame(frame)
        else:
            print(f"Warning: Failed to load image pair: {args.rgb}, {args.depth}")

    # Run application
    app.run()


if __name__ == '__main__':
    main()
