"""
Plane detection algorithms for RGB-D data.
Supports multiple detection methods with configurable parameters.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum
from scipy.spatial import ConvexHull
import cv2


class PlaneAlgorithm(Enum):
    """Available plane detection algorithms."""
    RANSAC = "ransac"
    RANSAC_ITERATIVE = "ransac_iterative"  # Multiple planes via iterative RANSAC
    REGION_GROWING = "region_growing"
    RGB_GUIDED = "rgb_guided"  # Use RGB edges to guide plane detection


@dataclass
class PlaneDetectionParams:
    """Parameters for plane detection."""
    # Algorithm selection
    algorithm: PlaneAlgorithm = PlaneAlgorithm.RANSAC_ITERATIVE

    # RANSAC parameters
    distance_threshold: float = 0.02  # Max distance from plane (meters)
    ransac_n: int = 3  # Points per RANSAC iteration
    num_iterations: int = 1000  # Max RANSAC iterations

    # Plane filtering
    min_plane_points: int = 500  # Minimum inliers for valid plane
    min_plane_area: float = 0.05  # Minimum area (m^2)
    max_planes: int = 8  # Maximum planes to detect

    # Normal constraint (for horizontal plane detection)
    normal_threshold_deg: float = 15.0  # Max angle from vertical (degrees)
    enable_normal_filter: bool = True  # Filter by normal direction

    # Pre-processing
    downsample_factor: int = 4  # Point cloud downsampling
    min_depth: float = 0.1  # Minimum depth (meters)
    max_depth: float = 10.0  # Maximum depth (meters)

    # Region growing parameters
    smoothness_threshold: float = 10.0  # Degrees
    curvature_threshold: float = 1.0

    # RGB-guided parameters
    canny_low: int = 50
    canny_high: int = 150
    rgb_region_min_size: int = 1000  # Minimum RGB region size in pixels

    # Depth pre-processing
    apply_bilateral_filter: bool = False
    bilateral_d: int = 5
    bilateral_sigma_color: float = 75.0
    bilateral_sigma_space: float = 75.0

    # Hole filling
    apply_hole_filling: bool = False
    hole_fill_kernel_size: int = 5

    @property
    def normal_threshold_rad(self) -> float:
        return np.deg2rad(self.normal_threshold_deg)


@dataclass
class DetectedPlane:
    """Detected plane information."""
    plane_id: int
    coefficients: np.ndarray  # [a, b, c, d] where ax + by + cz + d = 0
    center: np.ndarray  # (3,) centroid of plane points
    normal: np.ndarray  # (3,) unit normal vector
    height: float  # Z coordinate of center (height from ground)
    area: float  # Estimated area in m^2
    inlier_indices: np.ndarray  # Indices of points belonging to this plane
    inlier_count: int
    color: Tuple[int, int, int] = (0, 255, 0)  # RGB color for visualization

    # Optional: 2D pixel mask
    pixel_mask: Optional[np.ndarray] = None


# Predefined colors for planes
PLANE_COLORS = [
    (0, 255, 0),    # Green
    (0, 128, 255),  # Blue
    (255, 0, 0),    # Red
    (255, 255, 0),  # Yellow
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Cyan
    (255, 128, 0),  # Orange
    (128, 0, 255),  # Purple
]


class PlaneDetector:
    """Multi-algorithm plane detector for RGB-D data."""

    def __init__(self, params: Optional[PlaneDetectionParams] = None):
        self.params = params or PlaneDetectionParams()
        self._gravity_direction = np.array([0.0, 0.0, 1.0])  # Default: Z-up

    def set_gravity_direction(self, direction: np.ndarray):
        """Set expected gravity direction for horizontal plane filtering."""
        self._gravity_direction = direction / np.linalg.norm(direction)

    def preprocess_depth(self, depth: np.ndarray) -> np.ndarray:
        """Apply pre-processing to depth image."""
        processed = depth.copy()

        # Hole filling
        if self.params.apply_hole_filling:
            mask = (processed <= 0) | np.isnan(processed)
            if np.any(mask):
                # Use inpainting for hole filling
                mask_uint8 = mask.astype(np.uint8) * 255
                # Convert to 16-bit for inpainting
                depth_16 = (processed * 1000).astype(np.uint16)
                depth_16[mask] = 0
                filled = cv2.inpaint(depth_16, mask_uint8, 3, cv2.INPAINT_TELEA)
                processed = filled.astype(np.float32) / 1000.0

        # Bilateral filter
        if self.params.apply_bilateral_filter:
            # Convert to uint16 for bilateral filter
            valid_mask = (processed > 0) & (~np.isnan(processed))
            depth_norm = processed.copy()
            depth_norm[~valid_mask] = 0
            max_val = np.max(depth_norm[valid_mask]) if np.any(valid_mask) else 1.0
            depth_8bit = (depth_norm / max_val * 255).astype(np.uint8)

            filtered = cv2.bilateralFilter(
                depth_8bit,
                self.params.bilateral_d,
                self.params.bilateral_sigma_color,
                self.params.bilateral_sigma_space
            )
            processed = filtered.astype(np.float32) / 255.0 * max_val
            processed[~valid_mask] = depth[~valid_mask]

        return processed

    def detect(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray] = None,
        pixel_coords: Optional[np.ndarray] = None,
        image_shape: Optional[Tuple[int, int]] = None
    ) -> List[DetectedPlane]:
        """
        Detect planes in point cloud.

        Args:
            points: (N, 3) array of 3D points
            colors: Optional (N, 3) RGB colors
            pixel_coords: Optional (N, 2) pixel coordinates
            image_shape: Optional (H, W) for creating pixel masks

        Returns:
            List of DetectedPlane objects
        """
        if len(points) < self.params.min_plane_points:
            return []

        algorithm = self.params.algorithm

        if algorithm == PlaneAlgorithm.RANSAC:
            return self._detect_ransac_single(points, colors, pixel_coords, image_shape)
        elif algorithm == PlaneAlgorithm.RANSAC_ITERATIVE:
            return self._detect_ransac_iterative(points, colors, pixel_coords, image_shape)
        elif algorithm == PlaneAlgorithm.REGION_GROWING:
            return self._detect_region_growing(points, colors, pixel_coords, image_shape)
        elif algorithm == PlaneAlgorithm.RGB_GUIDED:
            # RGB-guided requires RGB image
            if colors is not None and image_shape is not None:
                return self._detect_rgb_guided(points, colors, pixel_coords, image_shape)
            else:
                # Fall back to iterative RANSAC
                return self._detect_ransac_iterative(points, colors, pixel_coords, image_shape)
        else:
            return self._detect_ransac_iterative(points, colors, pixel_coords, image_shape)

    def _detect_ransac_single(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray],
        pixel_coords: Optional[np.ndarray],
        image_shape: Optional[Tuple[int, int]]
    ) -> List[DetectedPlane]:
        """Detect single dominant plane using RANSAC."""
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        try:
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=self.params.distance_threshold,
                ransac_n=self.params.ransac_n,
                num_iterations=self.params.num_iterations
            )
        except Exception as e:
            print(f"RANSAC failed: {e}")
            return []

        if len(inliers) < self.params.min_plane_points:
            return []

        inliers = np.array(inliers)
        plane = self._create_plane_from_model(
            plane_model, points, inliers, 0, colors, pixel_coords, image_shape
        )

        if plane is not None:
            return [plane]
        return []

    def _detect_ransac_iterative(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray],
        pixel_coords: Optional[np.ndarray],
        image_shape: Optional[Tuple[int, int]]
    ) -> List[DetectedPlane]:
        """Detect multiple planes using iterative RANSAC."""
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        detected_planes = []
        remaining_indices = np.arange(len(points))

        for plane_idx in range(self.params.max_planes):
            if len(remaining_indices) < self.params.min_plane_points:
                break

            current_pcd = pcd.select_by_index(remaining_indices.tolist())

            try:
                plane_model, inliers_local = current_pcd.segment_plane(
                    distance_threshold=self.params.distance_threshold,
                    ransac_n=self.params.ransac_n,
                    num_iterations=self.params.num_iterations
                )
            except Exception:
                break

            if len(inliers_local) < self.params.min_plane_points:
                break

            # Map local indices back to global
            inliers_global = remaining_indices[inliers_local]

            plane = self._create_plane_from_model(
                plane_model, points, inliers_global, plane_idx,
                colors, pixel_coords, image_shape
            )

            if plane is not None:
                detected_planes.append(plane)

            # Remove inliers from remaining points
            mask = np.ones(len(remaining_indices), dtype=bool)
            mask[inliers_local] = False
            remaining_indices = remaining_indices[mask]

        return detected_planes

    def _detect_region_growing(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray],
        pixel_coords: Optional[np.ndarray],
        image_shape: Optional[Tuple[int, int]]
    ) -> List[DetectedPlane]:
        """Detect planes using region growing segmentation."""
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Estimate normals
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30
            )
        )

        # Use DBSCAN clustering as alternative to region growing
        # (Open3D doesn't have direct region growing for planar segmentation)
        labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=50))

        detected_planes = []
        unique_labels = set(labels)
        unique_labels.discard(-1)  # Remove noise label

        for plane_idx, label in enumerate(sorted(unique_labels)):
            if plane_idx >= self.params.max_planes:
                break

            cluster_indices = np.where(labels == label)[0]

            if len(cluster_indices) < self.params.min_plane_points:
                continue

            # Fit plane to cluster using RANSAC
            cluster_pcd = pcd.select_by_index(cluster_indices.tolist())

            try:
                plane_model, inliers_local = cluster_pcd.segment_plane(
                    distance_threshold=self.params.distance_threshold,
                    ransac_n=self.params.ransac_n,
                    num_iterations=100
                )
            except Exception:
                continue

            if len(inliers_local) < self.params.min_plane_points:
                continue

            inliers_global = cluster_indices[inliers_local]

            plane = self._create_plane_from_model(
                plane_model, points, inliers_global, len(detected_planes),
                colors, pixel_coords, image_shape
            )

            if plane is not None:
                detected_planes.append(plane)

        return detected_planes

    def _detect_rgb_guided(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        pixel_coords: np.ndarray,
        image_shape: Tuple[int, int]
    ) -> List[DetectedPlane]:
        """
        Detect planes using RGB guidance.
        Uses color edges to define regions, then fits planes to each region.
        """
        import open3d as o3d

        h, w = image_shape

        # Reconstruct RGB image from point cloud colors
        rgb_image = np.zeros((h, w, 3), dtype=np.uint8)
        for i, (u, v) in enumerate(pixel_coords):
            if 0 <= u < w and 0 <= v < h:
                rgb_image[v, u] = colors[i]

        # Edge detection
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, self.params.canny_low, self.params.canny_high)

        # Dilate edges to create region boundaries
        kernel = np.ones((3, 3), np.uint8)
        edges_dilated = cv2.dilate(edges, kernel, iterations=2)

        # Find connected components (regions between edges)
        edges_inv = 255 - edges_dilated
        num_labels, labels = cv2.connectedComponents(edges_inv)

        detected_planes = []
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Create pixel-to-point mapping
        pixel_to_point = {}
        for i, (u, v) in enumerate(pixel_coords):
            pixel_to_point[(int(u), int(v))] = i

        for label in range(1, min(num_labels, self.params.max_planes + 1)):
            # Get pixels in this region
            region_mask = labels == label
            region_size = np.sum(region_mask)

            if region_size < self.params.rgb_region_min_size:
                continue

            # Find points in this region
            point_indices = []
            for v in range(h):
                for u in range(w):
                    if region_mask[v, u] and (u, v) in pixel_to_point:
                        point_indices.append(pixel_to_point[(u, v)])

            if len(point_indices) < self.params.min_plane_points:
                continue

            point_indices = np.array(point_indices)

            # Fit plane to region points
            region_pcd = pcd.select_by_index(point_indices.tolist())

            try:
                plane_model, inliers_local = region_pcd.segment_plane(
                    distance_threshold=self.params.distance_threshold * 2,  # More lenient
                    ransac_n=self.params.ransac_n,
                    num_iterations=self.params.num_iterations
                )
            except Exception:
                continue

            inlier_ratio = len(inliers_local) / len(point_indices)
            if inlier_ratio < 0.5:  # Less than 50% are planar
                continue

            inliers_global = point_indices[inliers_local]

            plane = self._create_plane_from_model(
                plane_model, points, inliers_global, len(detected_planes),
                colors, pixel_coords, image_shape
            )

            if plane is not None:
                detected_planes.append(plane)

        return detected_planes

    def _create_plane_from_model(
        self,
        plane_model: np.ndarray,
        points: np.ndarray,
        inlier_indices: np.ndarray,
        plane_idx: int,
        colors: Optional[np.ndarray],
        pixel_coords: Optional[np.ndarray],
        image_shape: Optional[Tuple[int, int]]
    ) -> Optional[DetectedPlane]:
        """Create DetectedPlane object from RANSAC model."""
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        normal_len = np.linalg.norm(normal)
        if normal_len < 1e-6:
            return None
        normal = normal / normal_len

        # Ensure normal points "up" (positive Z in base frame)
        if normal[2] < 0:
            normal = -normal
            d = -d

        # Check normal constraint
        if self.params.enable_normal_filter:
            angle = np.arccos(np.clip(np.dot(normal, self._gravity_direction), -1.0, 1.0))
            if angle > self.params.normal_threshold_rad:
                return None

        # Get plane points
        plane_points = points[inlier_indices]

        # Calculate area
        area = self._calculate_plane_area(plane_points)
        if area < self.params.min_plane_area:
            return None

        # Calculate center
        center = np.mean(plane_points, axis=0)

        # Create pixel mask if possible
        pixel_mask = None
        if pixel_coords is not None and image_shape is not None:
            h, w = image_shape
            pixel_mask = np.zeros((h, w), dtype=bool)
            plane_pixel_coords = pixel_coords[inlier_indices]
            for u, v in plane_pixel_coords:
                if 0 <= u < w and 0 <= v < h:
                    pixel_mask[int(v), int(u)] = True

        return DetectedPlane(
            plane_id=plane_idx + 1,
            coefficients=np.array([a, b, c, d]),
            center=center,
            normal=normal,
            height=center[2],
            area=area,
            inlier_indices=inlier_indices,
            inlier_count=len(inlier_indices),
            color=PLANE_COLORS[plane_idx % len(PLANE_COLORS)],
            pixel_mask=pixel_mask
        )

    def _calculate_plane_area(self, points: np.ndarray) -> float:
        """Calculate plane area using convex hull projection."""
        if len(points) < 3:
            return 0.0

        try:
            # Project to XY plane (assuming horizontal planes)
            points_2d = points[:, :2]
            hull = ConvexHull(points_2d)
            return hull.volume  # In 2D, volume is area
        except Exception:
            # Fallback: bounding box
            x_range = np.max(points[:, 0]) - np.min(points[:, 0])
            y_range = np.max(points[:, 1]) - np.min(points[:, 1])
            return x_range * y_range * 0.5


def create_overlay(
    rgb: np.ndarray,
    planes: List[DetectedPlane],
    alpha: float = 0.5
) -> np.ndarray:
    """
    Create RGB overlay with detected planes.

    Args:
        rgb: (H, W, 3) RGB image
        planes: List of DetectedPlane with pixel_mask
        alpha: Overlay transparency

    Returns:
        (H, W, 3) RGB image with plane overlays
    """
    overlay = rgb.copy()
    h, w = rgb.shape[:2]

    for plane in planes:
        if plane.pixel_mask is None:
            continue

        # Create colored overlay for this plane
        mask = plane.pixel_mask
        if mask.shape != (h, w):
            continue

        color = np.array(plane.color, dtype=np.float32)
        overlay[mask] = (
            overlay[mask].astype(np.float32) * (1 - alpha) +
            color * alpha
        ).astype(np.uint8)

    return overlay


def draw_plane_info(
    image: np.ndarray,
    planes: List[DetectedPlane],
    intrinsics=None
) -> np.ndarray:
    """Draw plane information text on image."""
    result = image.copy()

    y_offset = 30
    for plane in planes:
        text = f"Plane{plane.plane_id}: H={plane.height:.2f}m A={plane.area:.2f}m2"
        color = plane.color
        cv2.putText(result, text, (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        y_offset += 25

    return result
