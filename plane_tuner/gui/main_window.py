"""
Main GUI window for plane detection parameter tuning.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
from pathlib import Path
import threading
import time
from typing import Optional

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from gui.parameter_panel import ParameterPanel
from gui.image_canvas import ImageCanvas, PlaneInfoPanel
from core.image_loader import (
    ImageLoader, RGBDFrame, CameraIntrinsics,
    TIAGO_RGB_TOPIC, TIAGO_DEPTH_TOPIC
)
from core.point_cloud import create_point_cloud
from core.plane_detector import (
    PlaneDetector, PlaneDetectionParams, create_overlay, draw_plane_info
)


# Default paths for RoboCup datasets
DEFAULT_ROBOCUP_PATH = "datasets/robocup/storing_try_2"


class CameraIntrinsicsDialog(tk.Toplevel):
    """Dialog for setting camera intrinsics."""

    def __init__(self, parent, intrinsics: CameraIntrinsics):
        super().__init__(parent)
        self.title("Camera Intrinsics")
        self.geometry("350x350")
        self.transient(parent)
        self.grab_set()

        self.intrinsics = intrinsics
        self.result = None

        self._create_widgets()

    def _create_widgets(self):
        frame = ttk.Frame(self, padding=10)
        frame.pack(fill='both', expand=True)

        # Preset selector
        ttk.Label(frame, text="Presets:").grid(row=0, column=0, sticky='w', pady=5)
        presets = ['Custom', 'TUM Freiburg1', 'TUM Freiburg2', 'TUM Freiburg3',
                   'TIAGo', 'RoboCup TIAGo', 'RealSense D435']
        self.preset_var = tk.StringVar(value='Custom')
        preset_cb = ttk.Combobox(frame, textvariable=self.preset_var,
                                 values=presets, state='readonly')
        preset_cb.grid(row=0, column=1, sticky='ew', pady=5)
        preset_cb.bind('<<ComboboxSelected>>', self._on_preset_change)

        # Intrinsic parameters
        row = 1
        self.vars = {}

        params = [
            ('fx', 'Focal Length X', self.intrinsics.fx),
            ('fy', 'Focal Length Y', self.intrinsics.fy),
            ('cx', 'Principal Point X', self.intrinsics.cx),
            ('cy', 'Principal Point Y', self.intrinsics.cy),
            ('width', 'Image Width', self.intrinsics.width),
            ('height', 'Image Height', self.intrinsics.height),
            ('depth_scale', 'Depth Scale', self.intrinsics.depth_scale),
        ]

        for name, label, default in params:
            ttk.Label(frame, text=label + ":").grid(row=row, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=default)
            entry = ttk.Entry(frame, textvariable=var)
            entry.grid(row=row, column=1, sticky='ew', pady=2)
            self.vars[name] = var
            row += 1

        # Buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=row, column=0, columnspan=2, pady=20)

        ttk.Button(btn_frame, text="OK", command=self._on_ok).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel).pack(side='left', padx=5)

        frame.columnconfigure(1, weight=1)

    def _on_preset_change(self, event=None):
        preset = self.preset_var.get()
        presets_map = {
            'TUM Freiburg1': CameraIntrinsics.from_tum_freiburg1(),
            'TUM Freiburg2': CameraIntrinsics.from_tum_freiburg2(),
            'TUM Freiburg3': CameraIntrinsics.from_tum_freiburg3(),
            'TIAGo': CameraIntrinsics.from_tiago(),
            'RoboCup TIAGo': CameraIntrinsics.from_robocup_tiago(),
            'RealSense D435': CameraIntrinsics.from_realsense_d435(),
        }

        if preset in presets_map:
            intr = presets_map[preset]
            self.vars['fx'].set(intr.fx)
            self.vars['fy'].set(intr.fy)
            self.vars['cx'].set(intr.cx)
            self.vars['cy'].set(intr.cy)
            self.vars['width'].set(intr.width)
            self.vars['height'].set(intr.height)
            self.vars['depth_scale'].set(intr.depth_scale)

    def _on_ok(self):
        self.result = CameraIntrinsics(
            fx=self.vars['fx'].get(),
            fy=self.vars['fy'].get(),
            cx=self.vars['cx'].get(),
            cy=self.vars['cy'].get(),
            width=int(self.vars['width'].get()),
            height=int(self.vars['height'].get()),
            depth_scale=self.vars['depth_scale'].get()
        )
        self.destroy()

    def _on_cancel(self):
        self.destroy()


class MainWindow:
    """Main application window."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Plane Detection Parameter Tuner")
        self.root.geometry("1400x900")

        # State
        self.image_loader = ImageLoader()
        self.plane_detector = PlaneDetector()
        self.current_frame: Optional[RGBDFrame] = None
        self.current_points: Optional[np.ndarray] = None
        self.current_colors: Optional[np.ndarray] = None
        self.current_pixel_coords: Optional[np.ndarray] = None

        # Auto-update flag
        self.auto_update = tk.BooleanVar(value=True)

        self._create_menu()
        self._create_widgets()
        self._setup_bindings()

    def _create_menu(self):
        """Create menu bar."""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Load RGB Image...", command=self._load_rgb)
        file_menu.add_command(label="Load Depth Image...", command=self._load_depth)
        file_menu.add_command(label="Load Image Pair...", command=self._load_image_pair)
        file_menu.add_separator()
        file_menu.add_command(label="Load TUM Dataset...", command=self._load_tum_dataset)
        file_menu.add_command(label="Load ROS2 Bag...", command=self._load_rosbag)
        file_menu.add_command(label="Load RoboCup Rosbag...", command=self._load_robocup_rosbag)
        file_menu.add_separator()
        file_menu.add_command(label="Save Current Frame...", command=self._save_frame)
        file_menu.add_command(label="Export Parameters...", command=self._export_params)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Settings menu
        settings_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Settings", menu=settings_menu)
        settings_menu.add_command(label="Camera Intrinsics...", command=self._edit_intrinsics)
        settings_menu.add_checkbutton(label="Auto Update", variable=self.auto_update)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self._show_about)

    def _create_widgets(self):
        """Create main widgets."""
        # Main paned window
        paned = ttk.PanedWindow(self.root, orient='horizontal')
        paned.pack(fill='both', expand=True, padx=5, pady=5)

        # Left panel: Parameters
        left_frame = ttk.Frame(paned)
        paned.add(left_frame, weight=0)

        ttk.Label(left_frame, text="Parameters", font=('TkDefaultFont', 12, 'bold')).pack(pady=5)

        self.param_panel = ParameterPanel(left_frame, on_change=self._on_params_changed)
        self.param_panel.pack(fill='both', expand=True)

        # Right panel: Visualization
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=1)

        # Image canvas (expandable)
        self.image_canvas = ImageCanvas(right_frame, width=640, height=480)
        self.image_canvas.pack(fill='both', expand=True)

        # Frame control bar (directly below image canvas)
        frame_bar = ttk.Frame(right_frame)
        frame_bar.pack(fill='x', pady=(5, 0))

        # Previous button
        self.prev_btn = ttk.Button(frame_bar, text="<", width=3, command=self._prev_frame)
        self.prev_btn.pack(side='left', padx=2)

        # Frame slider
        self.frame_slider_var = tk.IntVar(value=0)
        self.frame_slider = ttk.Scale(
            frame_bar, from_=0, to=0,
            variable=self.frame_slider_var,
            orient='horizontal',
            command=self._on_frame_slider
        )
        self.frame_slider.pack(side='left', fill='x', expand=True, padx=5)

        # Next button
        self.next_btn = ttk.Button(frame_bar, text=">", width=3, command=self._next_frame)
        self.next_btn.pack(side='left', padx=2)

        # Frame label
        self.frame_label = ttk.Label(frame_bar, text="Frame: 0/0", width=15)
        self.frame_label.pack(side='left', padx=10)

        # Bottom: Plane info and controls
        bottom_frame = ttk.Frame(right_frame)
        bottom_frame.pack(fill='x', pady=5)

        # Plane info
        self.plane_info = PlaneInfoPanel(bottom_frame)
        self.plane_info.pack(side='left', fill='both', expand=True, padx=5)

        # Control buttons (detection only)
        ctrl_frame = ttk.Frame(bottom_frame)
        ctrl_frame.pack(side='right', fill='y', padx=5)

        ttk.Button(ctrl_frame, text="Run Detection", command=self._run_detection).pack(pady=2, fill='x')

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ttk.Label(
            self.root, textvariable=self.status_var,
            relief='sunken', anchor='w'
        )
        self.status_bar.pack(fill='x', side='bottom')

    def _setup_bindings(self):
        """Setup keyboard bindings."""
        self.root.bind('<Left>', lambda e: self._prev_frame())
        self.root.bind('<Right>', lambda e: self._next_frame())
        self.root.bind('<Return>', lambda e: self._run_detection())
        self.root.bind('<space>', lambda e: self._run_detection())

    def _load_rgb(self):
        """Load RGB image."""
        path = filedialog.askopenfilename(
            title="Select RGB Image",
            filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp"), ("All files", "*.*")]
        )
        if path:
            self._temp_rgb_path = path
            self.status_var.set(f"Loaded RGB: {Path(path).name}")

    def _load_depth(self):
        """Load depth image."""
        path = filedialog.askopenfilename(
            title="Select Depth Image",
            filetypes=[("Images", "*.png *.exr *.tiff"), ("All files", "*.*")]
        )
        if path:
            if hasattr(self, '_temp_rgb_path'):
                frame = self.image_loader.load_image_pair(self._temp_rgb_path, path)
                if frame:
                    self._set_current_frame(frame)
                    del self._temp_rgb_path
            else:
                messagebox.showinfo("Info", "Please load RGB image first")

    def _load_image_pair(self):
        """Load RGB-D image pair."""
        rgb_path = filedialog.askopenfilename(
            title="Select RGB Image",
            filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp"), ("All files", "*.*")]
        )
        if not rgb_path:
            return

        depth_path = filedialog.askopenfilename(
            title="Select Depth Image",
            filetypes=[("Images", "*.png *.exr *.tiff"), ("All files", "*.*")]
        )
        if not depth_path:
            return

        frame = self.image_loader.load_image_pair(rgb_path, depth_path)
        if frame:
            self._set_current_frame(frame)
        else:
            messagebox.showerror("Error", "Failed to load image pair")

    def _load_tum_dataset(self):
        """Load TUM RGB-D dataset."""
        path = filedialog.askdirectory(title="Select TUM Dataset Folder")
        if not path:
            return

        self.status_var.set("Loading TUM dataset...")
        self.root.update()

        count = self.image_loader.load_tum_sequence(path, max_frames=200)
        if count > 0:
            self._update_frame_slider()
            frame = self.image_loader.get_frame(0)
            if frame:
                self._set_current_frame(frame)
            self.status_var.set(f"Loaded {count} frames from TUM dataset")
        else:
            messagebox.showerror("Error", "Failed to load TUM dataset")

    def _load_rosbag(self):
        """Load ROS2 rosbag."""
        path = filedialog.askdirectory(title="Select ROS2 Bag Directory")
        if not path:
            return

        # Ask for topics
        dialog = RosbagTopicDialog(self.root)
        self.root.wait_window(dialog)

        if dialog.result:
            rgb_topic, depth_topic, max_frames = dialog.result

            self.status_var.set("Loading rosbag...")
            self.root.update()

            count = self.image_loader.load_rosbag_sequence(
                path, rgb_topic, depth_topic, max_frames=max_frames
            )
            if count > 0:
                self._update_frame_slider()
                frame = self.image_loader.get_frame(0)
                if frame:
                    self._set_current_frame(frame)
                self.status_var.set(f"Loaded {count} frames from rosbag")
            else:
                messagebox.showerror("Error", "Failed to load rosbag. Check console for details.")

    def _load_robocup_rosbag(self):
        """Load RoboCup TIAGo rosbag with default settings."""
        # Try default path first
        default_path = Path(__file__).parent.parent.parent / DEFAULT_ROBOCUP_PATH
        if default_path.exists():
            initial_dir = str(default_path.parent)
        else:
            initial_dir = str(Path.cwd())

        path = filedialog.askdirectory(
            title="Select RoboCup Rosbag Directory (e.g., storing_try_2)",
            initialdir=initial_dir
        )
        if not path:
            return

        # Show loading dialog
        dialog = RobocupLoadDialog(self.root)
        self.root.wait_window(dialog)

        if dialog.result:
            max_frames, skip_frames = dialog.result

            self.status_var.set("Indexing RoboCup rosbag (lazy loading)...")
            self.root.update()

            count = self.image_loader.load_robocup_rosbag(
                path, max_frames=max_frames, skip_frames=skip_frames
            )
            if count > 0:
                self._update_frame_slider()
                self.status_var.set(f"Loading frame 1/{count}...")
                self.root.update()
                frame = self.image_loader.get_frame(0)
                if frame:
                    self._set_current_frame(frame)
                mode = "lazy" if self.image_loader.is_lazy_mode else "memory"
                self.status_var.set(f"Indexed {count} frames ({mode} loading)")
            else:
                messagebox.showerror(
                    "Error",
                    "Failed to load RoboCup rosbag.\n\n"
                    "Please ensure:\n"
                    "1. The rosbag is downloaded (see datasets/robocup/README.md)\n"
                    "2. 'rosbags' package is installed: pip install rosbags\n\n"
                    "Check console for detailed error messages."
                )

    def _save_frame(self):
        """Save current frame."""
        if self.current_frame is None:
            messagebox.showinfo("Info", "No frame loaded")
            return

        path = filedialog.askdirectory(title="Select Output Directory")
        if path:
            from core.image_loader import save_frame_pair
            save_frame_pair(self.current_frame, path)
            self.status_var.set(f"Saved frame to {path}")

    def _export_params(self):
        """Export current parameters to file."""
        path = filedialog.asksaveasfilename(
            title="Export Parameters",
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml"), ("JSON", "*.json")]
        )
        if not path:
            return

        params = self.param_panel.get_params()
        ext = Path(path).suffix.lower()

        if ext == '.yaml':
            import yaml
            with open(path, 'w') as f:
                yaml.dump(self._params_to_dict(params), f, default_flow_style=False)
        else:
            import json
            with open(path, 'w') as f:
                json.dump(self._params_to_dict(params), f, indent=2)

        self.status_var.set(f"Exported parameters to {path}")

    def _params_to_dict(self, params: PlaneDetectionParams) -> dict:
        """Convert parameters to dictionary."""
        return {
            'algorithm': params.algorithm.value,
            'distance_threshold': params.distance_threshold,
            'ransac_n': params.ransac_n,
            'num_iterations': params.num_iterations,
            'min_plane_points': params.min_plane_points,
            'min_plane_area': params.min_plane_area,
            'max_planes': params.max_planes,
            'normal_threshold_deg': params.normal_threshold_deg,
            'enable_normal_filter': params.enable_normal_filter,
            'downsample_factor': params.downsample_factor,
            'min_depth': params.min_depth,
            'max_depth': params.max_depth,
            'apply_bilateral_filter': params.apply_bilateral_filter,
            'bilateral_d': params.bilateral_d,
            'bilateral_sigma_color': params.bilateral_sigma_color,
            'bilateral_sigma_space': params.bilateral_sigma_space,
            'apply_hole_filling': params.apply_hole_filling,
            'canny_low': params.canny_low,
            'canny_high': params.canny_high,
            'rgb_region_min_size': params.rgb_region_min_size,
        }

    def _edit_intrinsics(self):
        """Edit camera intrinsics."""
        dialog = CameraIntrinsicsDialog(self.root, self.image_loader.intrinsics)
        self.root.wait_window(dialog)

        if dialog.result:
            self.image_loader.intrinsics = dialog.result
            self.status_var.set("Camera intrinsics updated")
            if self.auto_update.get() and self.current_frame:
                self._run_detection()

    def _show_about(self):
        """Show about dialog."""
        messagebox.showinfo(
            "About",
            "Plane Detection Parameter Tuner\n\n"
            "A GUI tool for tuning plane detection parameters.\n"
            "Supports multiple algorithms including RANSAC, Region Growing,\n"
            "and RGB-guided detection.\n\n"
            "Created for RoboCup@Home plane detection development."
        )

    def _set_current_frame(self, frame: RGBDFrame):
        """Set current frame and update display."""
        self.current_frame = frame

        # Generate point cloud
        params = self.param_panel.get_params()
        self.current_points, self.current_colors, self.current_pixel_coords = create_point_cloud(
            frame,
            self.image_loader.intrinsics,
            min_depth=params.min_depth,
            max_depth=params.max_depth,
            downsample_factor=params.downsample_factor
        )

        # Update display
        self.image_canvas.set_images(rgb=frame.rgb, depth=frame.depth)

        if self.auto_update.get():
            self._run_detection()

    def _run_detection(self):
        """Run plane detection with current parameters."""
        if self.current_frame is None:
            return

        self.status_var.set("Running detection...")
        self.root.update()

        start_time = time.time()

        # Get parameters and update detector
        params = self.param_panel.get_params()
        self.plane_detector.params = params

        # Preprocess depth if needed
        depth = self.current_frame.depth
        if params.apply_bilateral_filter or params.apply_hole_filling:
            depth = self.plane_detector.preprocess_depth(depth)

        # Regenerate point cloud with current params
        points, colors, pixel_coords = create_point_cloud(
            RGBDFrame(rgb=self.current_frame.rgb, depth=depth),
            self.image_loader.intrinsics,
            min_depth=params.min_depth,
            max_depth=params.max_depth,
            downsample_factor=params.downsample_factor
        )

        # Run detection
        h, w = self.current_frame.rgb.shape[:2]
        planes = self.plane_detector.detect(
            points, colors, pixel_coords,
            image_shape=(h, w)
        )

        elapsed = time.time() - start_time

        # Update visualization
        if planes:
            overlay = create_overlay(self.current_frame.rgb, planes, alpha=0.5)
            overlay = draw_plane_info(overlay, planes)
        else:
            overlay = self.current_frame.rgb

        self.image_canvas.set_images(
            rgb=self.current_frame.rgb,
            depth=depth,
            overlay=overlay
        )

        # Update plane info
        self.plane_info.update_planes(planes)

        self.status_var.set(
            f"Detection complete: {len(planes)} planes found in {elapsed:.3f}s "
            f"({len(points)} points)"
        )

    def _on_params_changed(self, params: PlaneDetectionParams):
        """Handle parameter change."""
        self.plane_detector.params = params
        if self.auto_update.get() and self.current_frame:
            self._run_detection()

    def _prev_frame(self):
        """Go to previous frame."""
        self._load_frame_at(self.image_loader._current_index - 1)

    def _next_frame(self):
        """Go to next frame."""
        self._load_frame_at(self.image_loader._current_index + 1)

    def _on_frame_slider(self, value):
        """Handle frame slider change."""
        index = int(float(value))
        if index != self.image_loader._current_index:
            self._load_frame_at(index)

    def _load_frame_at(self, index: int):
        """Load and display frame at given index."""
        count = self.image_loader.frame_count
        if index < 0 or index >= count:
            return

        # Show loading status for lazy loading
        if self.image_loader.is_lazy_mode:
            self.status_var.set(f"Loading frame {index + 1}/{count}...")
            self.frame_label.config(text=f"Loading: {index + 1}/{count}")
            self.root.update()

        frame = self.image_loader.get_frame(index)
        if frame:
            self._set_current_frame(frame)
            self._update_frame_label()

    def _update_frame_slider(self):
        """Update frame slider range."""
        count = self.image_loader.frame_count
        if count > 1:
            self.frame_slider.config(to=count - 1)
        else:
            self.frame_slider.config(to=0)
        self._update_frame_label()

    def _update_frame_label(self):
        """Update frame label."""
        count = self.image_loader.frame_count
        current = self.image_loader._current_index
        self.frame_label.config(text=f"Frame: {current + 1}/{count}")
        self.frame_slider_var.set(current)

    def run(self):
        """Start the application."""
        self.root.mainloop()


class RosbagTopicDialog(tk.Toplevel):
    """Dialog for rosbag topic selection."""

    def __init__(self, parent):
        super().__init__(parent)
        self.title("Rosbag Topics")
        self.geometry("450x220")
        self.transient(parent)
        self.grab_set()

        self.result = None
        self._create_widgets()

    def _create_widgets(self):
        frame = ttk.Frame(self, padding=10)
        frame.pack(fill='both', expand=True)

        # RGB topic
        ttk.Label(frame, text="RGB Topic:").grid(row=0, column=0, sticky='w', pady=5)
        self.rgb_var = tk.StringVar(value=TIAGO_RGB_TOPIC)
        ttk.Entry(frame, textvariable=self.rgb_var, width=40).grid(row=0, column=1, pady=5)

        # Depth topic
        ttk.Label(frame, text="Depth Topic:").grid(row=1, column=0, sticky='w', pady=5)
        self.depth_var = tk.StringVar(value=TIAGO_DEPTH_TOPIC)
        ttk.Entry(frame, textvariable=self.depth_var, width=40).grid(row=1, column=1, pady=5)

        # Max frames
        ttk.Label(frame, text="Max Frames:").grid(row=2, column=0, sticky='w', pady=5)
        self.max_frames_var = tk.IntVar(value=100)
        ttk.Spinbox(frame, textvariable=self.max_frames_var, from_=1, to=2000, width=10).grid(
            row=2, column=1, sticky='w', pady=5
        )

        # Buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=3, column=0, columnspan=2, pady=20)

        ttk.Button(btn_frame, text="OK", command=self._on_ok).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel).pack(side='left', padx=5)

    def _on_ok(self):
        self.result = (self.rgb_var.get(), self.depth_var.get(), self.max_frames_var.get())
        self.destroy()

    def _on_cancel(self):
        self.destroy()


class RobocupLoadDialog(tk.Toplevel):
    """Dialog for RoboCup rosbag loading options."""

    def __init__(self, parent):
        super().__init__(parent)
        self.title("RoboCup Rosbag Options")
        self.geometry("400x200")
        self.transient(parent)
        self.grab_set()

        self.result = None
        self._create_widgets()

    def _create_widgets(self):
        frame = ttk.Frame(self, padding=10)
        frame.pack(fill='both', expand=True)

        # Info label
        info = ttk.Label(
            frame,
            text="Loading RoboCup TIAGo rosbag (Lazy Loading)\n"
                 "Frames are loaded on-demand to save memory.\n"
                 "(Topics: /head_front_camera/rgb/image_raw, depth/image_raw)",
            justify='center'
        )
        info.grid(row=0, column=0, columnspan=2, pady=10)

        # Max frames (0 = unlimited)
        ttk.Label(frame, text="Max Frames (0=all):").grid(row=1, column=0, sticky='w', pady=5)
        self.max_frames_var = tk.IntVar(value=0)
        ttk.Spinbox(frame, textvariable=self.max_frames_var, from_=0, to=10000, width=10).grid(
            row=1, column=1, sticky='w', pady=5
        )

        # Skip frames
        ttk.Label(frame, text="Skip First N Frames:").grid(row=2, column=0, sticky='w', pady=5)
        self.skip_frames_var = tk.IntVar(value=0)
        ttk.Spinbox(frame, textvariable=self.skip_frames_var, from_=0, to=5000, width=10).grid(
            row=2, column=1, sticky='w', pady=5
        )

        # Buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=3, column=0, columnspan=2, pady=15)

        ttk.Button(btn_frame, text="Load", command=self._on_ok).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Cancel", command=self._on_cancel).pack(side='left', padx=5)

    def _on_ok(self):
        self.result = (self.max_frames_var.get(), self.skip_frames_var.get())
        self.destroy()

    def _on_cancel(self):
        self.destroy()


def main():
    """Main entry point."""
    app = MainWindow()
    app.run()


if __name__ == '__main__':
    main()
