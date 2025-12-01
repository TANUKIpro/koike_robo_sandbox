"""
Point cloud generation utilities.
"""

import numpy as np
from typing import Tuple, Optional
from .image_loader import RGBDFrame, CameraIntrinsics


def create_point_cloud(
    frame: RGBDFrame,
    intrinsics: CameraIntrinsics,
    min_depth: float = 0.1,
    max_depth: float = 10.0,
    downsample_factor: int = 1
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate 3D point cloud from RGB-D frame.

    Args:
        frame: RGBDFrame containing RGB and depth images
        intrinsics: Camera intrinsic parameters
        min_depth: Minimum valid depth (meters)
        max_depth: Maximum valid depth (meters)
        downsample_factor: Skip every N pixels (1 = no downsampling)

    Returns:
        Tuple of:
        - points: (N, 3) float32 array of 3D points in camera frame
        - colors: (N, 3) uint8 array of RGB colors
        - pixel_coords: (N, 2) int array of (u, v) pixel coordinates
    """
    depth = frame.depth
    rgb = frame.rgb

    h, w = depth.shape
    ds = max(1, int(downsample_factor))

    # Create pixel coordinate grids
    u = np.arange(0, w, ds)
    v = np.arange(0, h, ds)
    u_grid, v_grid = np.meshgrid(u, v)

    # Get downsampled depth
    depth_ds = depth[::ds, ::ds]

    # Create valid depth mask
    mask = (depth_ds > min_depth) & (depth_ds < max_depth) & (~np.isnan(depth_ds))

    # Extract valid points
    z = depth_ds[mask]
    u_valid = u_grid[mask].astype(np.float32)
    v_valid = v_grid[mask].astype(np.float32)

    if len(z) == 0:
        return np.zeros((0, 3), dtype=np.float32), \
               np.zeros((0, 3), dtype=np.uint8), \
               np.zeros((0, 2), dtype=np.int32)

    # Back-project to 3D (camera coordinates)
    # x = (u - cx) * z / fx
    # y = (v - cy) * z / fy
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.cx, intrinsics.cy

    x = (u_valid - cx) * z / fx
    y = (v_valid - cy) * z / fy

    points = np.stack([x, y, z], axis=-1).astype(np.float32)

    # Get colors for valid points
    rgb_ds = rgb[::ds, ::ds]
    colors = rgb_ds[mask].astype(np.uint8)

    # Pixel coordinates
    pixel_coords = np.stack([u_valid.astype(np.int32), v_valid.astype(np.int32)], axis=-1)

    return points, colors, pixel_coords


def transform_points(
    points: np.ndarray,
    rotation_matrix: Optional[np.ndarray] = None,
    translation: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Transform points from one coordinate frame to another.

    Args:
        points: (N, 3) array of 3D points
        rotation_matrix: (3, 3) rotation matrix
        translation: (3,) translation vector

    Returns:
        (N, 3) transformed points
    """
    if rotation_matrix is None and translation is None:
        return points

    transformed = points.copy()

    if rotation_matrix is not None:
        transformed = transformed @ rotation_matrix.T

    if translation is not None:
        transformed = transformed + translation

    return transformed


def estimate_normals(
    points: np.ndarray,
    k_neighbors: int = 30,
    search_radius: float = 0.1
) -> np.ndarray:
    """
    Estimate point normals using Open3D.

    Args:
        points: (N, 3) array of 3D points
        k_neighbors: Number of neighbors for normal estimation
        search_radius: Search radius for neighbors

    Returns:
        (N, 3) array of unit normal vectors
    """
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=search_radius,
            max_nn=k_neighbors
        )
    )

    # Orient normals consistently (towards camera)
    pcd.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))

    return np.asarray(pcd.normals)


def filter_points_by_depth_gradient(
    frame: RGBDFrame,
    gradient_threshold: float = 0.1
) -> np.ndarray:
    """
    Create mask for points with low depth gradient (smooth regions).
    Useful for filtering out depth discontinuities.

    Args:
        frame: RGBDFrame
        gradient_threshold: Maximum allowed depth gradient

    Returns:
        (H, W) boolean mask where True = valid smooth region
    """
    depth = frame.depth.copy()
    depth[depth <= 0] = np.nan

    # Compute gradients
    grad_x = np.abs(np.gradient(depth, axis=1))
    grad_y = np.abs(np.gradient(depth, axis=0))
    grad_mag = np.sqrt(grad_x**2 + grad_y**2)

    # Create mask for smooth regions
    mask = grad_mag < gradient_threshold
    mask[np.isnan(grad_mag)] = False

    return mask
