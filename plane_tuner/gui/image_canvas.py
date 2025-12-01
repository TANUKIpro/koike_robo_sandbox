"""
Image display canvas for RGB-D visualization.
"""

import tkinter as tk
from tkinter import ttk
import numpy as np
from PIL import Image, ImageTk
from typing import Optional, Tuple
import cv2


class ImageCanvas(ttk.Frame):
    """Canvas for displaying RGB and depth images with plane overlays."""

    def __init__(self, parent, width: int = 640, height: int = 480):
        super().__init__(parent)
        self.display_width = width
        self.display_height = height

        self._rgb_image: Optional[np.ndarray] = None
        self._depth_image: Optional[np.ndarray] = None
        self._overlay_image: Optional[np.ndarray] = None

        self._photo_rgb: Optional[ImageTk.PhotoImage] = None
        self._photo_depth: Optional[ImageTk.PhotoImage] = None

        self._create_widgets()

    def _create_widgets(self):
        """Create canvas widgets."""
        # Main container
        main_frame = ttk.Frame(self)
        main_frame.pack(fill='both', expand=True)

        # Display mode selector
        mode_frame = ttk.Frame(main_frame)
        mode_frame.pack(fill='x', pady=5)

        self.display_mode = tk.StringVar(value='side_by_side')
        modes = [
            ('Side by Side', 'side_by_side'),
            ('RGB Only', 'rgb_only'),
            ('Depth Only', 'depth_only'),
            ('Overlay Only', 'overlay_only')
        ]
        for text, mode in modes:
            rb = ttk.Radiobutton(
                mode_frame, text=text, value=mode,
                variable=self.display_mode,
                command=self._update_display
            )
            rb.pack(side='left', padx=5)

        # Depth colormap selector
        ttk.Label(mode_frame, text="  Depth Colormap:").pack(side='left', padx=5)
        self.colormap_var = tk.StringVar(value='jet')
        colormaps = ['jet', 'viridis', 'plasma', 'inferno', 'magma', 'gray']
        colormap_cb = ttk.Combobox(
            mode_frame, textvariable=self.colormap_var,
            values=colormaps, state='readonly', width=10
        )
        colormap_cb.pack(side='left', padx=5)
        colormap_cb.bind('<<ComboboxSelected>>', lambda e: self._update_display())

        # Canvas for image display
        self.canvas = tk.Canvas(
            main_frame,
            width=self.display_width * 2,
            height=self.display_height,
            bg='black'
        )
        self.canvas.pack(fill='both', expand=True, pady=5)

        # Info panel
        info_frame = ttk.Frame(main_frame)
        info_frame.pack(fill='x', pady=5)

        self.info_label = ttk.Label(info_frame, text="No image loaded")
        self.info_label.pack(side='left', padx=5)

        self.depth_info_label = ttk.Label(info_frame, text="")
        self.depth_info_label.pack(side='right', padx=5)

        # Bind mouse motion for depth readout
        self.canvas.bind('<Motion>', self._on_mouse_move)

    def set_images(
        self,
        rgb: Optional[np.ndarray] = None,
        depth: Optional[np.ndarray] = None,
        overlay: Optional[np.ndarray] = None
    ):
        """
        Set images to display.

        Args:
            rgb: (H, W, 3) RGB image (uint8)
            depth: (H, W) depth image (float32, meters)
            overlay: (H, W, 3) RGB image with plane overlay (uint8)
        """
        self._rgb_image = rgb
        self._depth_image = depth
        self._overlay_image = overlay

        self._update_display()
        self._update_info()

    def _update_display(self):
        """Update canvas display based on current mode."""
        mode = self.display_mode.get()

        # Clear canvas
        self.canvas.delete('all')

        if mode == 'rgb_only':
            img = self._overlay_image if self._overlay_image is not None else self._rgb_image
            if img is not None:
                self._display_single_image(img, 0)

        elif mode == 'depth_only':
            if self._depth_image is not None:
                depth_colored = self._colorize_depth(self._depth_image)
                self._display_single_image(depth_colored, 0)

        elif mode == 'overlay_only':
            if self._overlay_image is not None:
                self._display_single_image(self._overlay_image, 0)
            elif self._rgb_image is not None:
                self._display_single_image(self._rgb_image, 0)

        else:  # side_by_side
            # Left: RGB or Overlay
            left_img = self._overlay_image if self._overlay_image is not None else self._rgb_image
            if left_img is not None:
                self._display_single_image(left_img, 0)

            # Right: Depth
            if self._depth_image is not None:
                depth_colored = self._colorize_depth(self._depth_image)
                self._display_single_image(depth_colored, self.display_width)

    def _display_single_image(self, img: np.ndarray, x_offset: int):
        """Display a single image on the canvas."""
        # Resize to display size
        h, w = img.shape[:2]
        scale = min(self.display_width / w, self.display_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)

        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Convert to PIL Image
        if len(resized.shape) == 2:
            pil_img = Image.fromarray(resized)
        else:
            pil_img = Image.fromarray(resized)

        # Create PhotoImage
        photo = ImageTk.PhotoImage(pil_img)

        # Center in canvas area
        x = x_offset + (self.display_width - new_w) // 2
        y = (self.display_height - new_h) // 2

        self.canvas.create_image(x, y, anchor='nw', image=photo)

        # Keep reference to prevent garbage collection
        if x_offset == 0:
            self._photo_rgb = photo
        else:
            self._photo_depth = photo

    def _colorize_depth(self, depth: np.ndarray) -> np.ndarray:
        """Convert depth to colored image."""
        # Normalize depth
        valid_mask = (depth > 0) & (~np.isnan(depth))
        if not np.any(valid_mask):
            return np.zeros((*depth.shape, 3), dtype=np.uint8)

        min_d = np.min(depth[valid_mask])
        max_d = np.max(depth[valid_mask])

        if max_d - min_d < 1e-6:
            normalized = np.zeros_like(depth)
        else:
            normalized = (depth - min_d) / (max_d - min_d)
            normalized = np.clip(normalized, 0, 1)

        normalized_uint8 = (normalized * 255).astype(np.uint8)

        # Apply colormap
        colormap_name = self.colormap_var.get()
        colormap_map = {
            'jet': cv2.COLORMAP_JET,
            'viridis': cv2.COLORMAP_VIRIDIS,
            'plasma': cv2.COLORMAP_PLASMA,
            'inferno': cv2.COLORMAP_INFERNO,
            'magma': cv2.COLORMAP_MAGMA,
            'gray': None
        }

        cmap = colormap_map.get(colormap_name, cv2.COLORMAP_JET)

        if cmap is None:
            colored = cv2.cvtColor(normalized_uint8, cv2.COLOR_GRAY2RGB)
        else:
            colored = cv2.applyColorMap(normalized_uint8, cmap)
            colored = cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)

        # Set invalid pixels to black
        colored[~valid_mask] = 0

        return colored

    def _update_info(self):
        """Update info labels."""
        if self._rgb_image is None:
            self.info_label.config(text="No image loaded")
            return

        h, w = self._rgb_image.shape[:2]
        self.info_label.config(text=f"Image: {w}x{h}")

        if self._depth_image is not None:
            valid_mask = (self._depth_image > 0) & (~np.isnan(self._depth_image))
            if np.any(valid_mask):
                min_d = np.min(self._depth_image[valid_mask])
                max_d = np.max(self._depth_image[valid_mask])
                mean_d = np.mean(self._depth_image[valid_mask])
                self.depth_info_label.config(
                    text=f"Depth: {min_d:.2f}m - {max_d:.2f}m (mean: {mean_d:.2f}m)"
                )
            else:
                self.depth_info_label.config(text="No valid depth")

    def _on_mouse_move(self, event):
        """Handle mouse movement for depth readout."""
        if self._depth_image is None:
            return

        # Check if mouse is over depth area (right side in side_by_side mode)
        mode = self.display_mode.get()
        if mode == 'side_by_side' and event.x < self.display_width:
            return  # Over RGB area
        if mode == 'rgb_only' or mode == 'overlay_only':
            return  # No depth displayed

        # Calculate image coordinates
        h, w = self._depth_image.shape
        scale = min(self.display_width / w, self.display_height / h)

        if mode == 'side_by_side':
            x_offset = self.display_width + (self.display_width - int(w * scale)) // 2
        else:
            x_offset = (self.display_width - int(w * scale)) // 2

        y_offset = (self.display_height - int(h * scale)) // 2

        img_x = int((event.x - x_offset) / scale)
        img_y = int((event.y - y_offset) / scale)

        if 0 <= img_x < w and 0 <= img_y < h:
            depth_val = self._depth_image[img_y, img_x]
            if depth_val > 0 and not np.isnan(depth_val):
                self.depth_info_label.config(
                    text=f"Depth at ({img_x}, {img_y}): {depth_val:.3f}m"
                )


class PlaneInfoPanel(ttk.Frame):
    """Panel displaying detected plane information."""

    def __init__(self, parent):
        super().__init__(parent)
        self._create_widgets()

    def _create_widgets(self):
        """Create widgets."""
        # Header
        header = ttk.Label(self, text="Detected Planes", font=('TkDefaultFont', 10, 'bold'))
        header.pack(fill='x', pady=5)

        # Plane list
        columns = ('id', 'height', 'area', 'points')
        self.tree = ttk.Treeview(self, columns=columns, show='headings', height=8)

        self.tree.heading('id', text='ID')
        self.tree.heading('height', text='Height (m)')
        self.tree.heading('area', text='Area (m2)')
        self.tree.heading('points', text='Points')

        self.tree.column('id', width=40, anchor='center')
        self.tree.column('height', width=80, anchor='center')
        self.tree.column('area', width=80, anchor='center')
        self.tree.column('points', width=80, anchor='center')

        self.tree.pack(fill='both', expand=True)

        # Summary
        self.summary_label = ttk.Label(self, text="No planes detected")
        self.summary_label.pack(fill='x', pady=5)

    def update_planes(self, planes: list):
        """Update plane list."""
        # Clear existing items
        for item in self.tree.get_children():
            self.tree.delete(item)

        if not planes:
            self.summary_label.config(text="No planes detected")
            return

        # Add plane info
        for plane in planes:
            self.tree.insert('', 'end', values=(
                f"Plane{plane.plane_id}",
                f"{plane.height:.3f}",
                f"{plane.area:.3f}",
                plane.inlier_count
            ), tags=(f'plane{plane.plane_id}',))

            # Set row color
            color_hex = '#{:02x}{:02x}{:02x}'.format(*plane.color)
            self.tree.tag_configure(f'plane{plane.plane_id}', foreground=color_hex)

        self.summary_label.config(
            text=f"Total: {len(planes)} planes, {sum(p.inlier_count for p in planes)} points"
        )
