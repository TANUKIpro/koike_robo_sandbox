"""
Parameter control panel for plane detection tuning.
"""

import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, Any, Optional
from dataclasses import fields

import sys
sys.path.insert(0, str(__file__).rsplit('/', 2)[0])
from core.plane_detector import PlaneDetectionParams, PlaneAlgorithm


class ParameterPanel(ttk.Frame):
    """Panel with sliders and controls for plane detection parameters."""

    def __init__(self, parent, on_change: Optional[Callable[[PlaneDetectionParams], None]] = None):
        super().__init__(parent)
        self.on_change = on_change
        self.params = PlaneDetectionParams()

        self._create_widgets()

    def _create_widgets(self):
        """Create all parameter control widgets."""
        # Main scrollable canvas
        canvas = tk.Canvas(self, width=320)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Enable mouse wheel scrolling
        def on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", on_mousewheel)

        row = 0

        # Algorithm selection
        row = self._add_section_header("Algorithm", row)
        self.algorithm_var = tk.StringVar(value=self.params.algorithm.value)
        row = self._add_combobox(
            "Algorithm", self.algorithm_var,
            [a.value for a in PlaneAlgorithm],
            row
        )

        # RANSAC Parameters
        row = self._add_section_header("RANSAC Parameters", row)

        self.distance_threshold_var = tk.DoubleVar(value=self.params.distance_threshold)
        row = self._add_slider(
            "Distance Threshold (m)", self.distance_threshold_var,
            0.005, 0.10, 0.005, row
        )

        self.ransac_n_var = tk.IntVar(value=self.params.ransac_n)
        row = self._add_slider(
            "RANSAC N Points", self.ransac_n_var,
            3, 10, 1, row, is_int=True
        )

        self.num_iterations_var = tk.IntVar(value=self.params.num_iterations)
        row = self._add_slider(
            "RANSAC Iterations", self.num_iterations_var,
            100, 5000, 100, row, is_int=True
        )

        # Plane Filtering
        row = self._add_section_header("Plane Filtering", row)

        self.min_plane_points_var = tk.IntVar(value=self.params.min_plane_points)
        row = self._add_slider(
            "Min Plane Points", self.min_plane_points_var,
            50, 2000, 50, row, is_int=True
        )

        self.min_plane_area_var = tk.DoubleVar(value=self.params.min_plane_area)
        row = self._add_slider(
            "Min Plane Area (m2)", self.min_plane_area_var,
            0.01, 1.0, 0.01, row
        )

        self.max_planes_var = tk.IntVar(value=self.params.max_planes)
        row = self._add_slider(
            "Max Planes", self.max_planes_var,
            1, 10, 1, row, is_int=True
        )

        # Normal Constraint
        row = self._add_section_header("Normal Constraint", row)

        self.enable_normal_filter_var = tk.BooleanVar(value=self.params.enable_normal_filter)
        row = self._add_checkbox(
            "Enable Normal Filter", self.enable_normal_filter_var, row
        )

        self.normal_threshold_deg_var = tk.DoubleVar(value=self.params.normal_threshold_deg)
        row = self._add_slider(
            "Normal Threshold (deg)", self.normal_threshold_deg_var,
            1.0, 45.0, 1.0, row
        )

        # Point Cloud Processing
        row = self._add_section_header("Point Cloud Processing", row)

        self.downsample_factor_var = tk.IntVar(value=self.params.downsample_factor)
        row = self._add_slider(
            "Downsample Factor", self.downsample_factor_var,
            1, 8, 1, row, is_int=True
        )

        self.min_depth_var = tk.DoubleVar(value=self.params.min_depth)
        row = self._add_slider(
            "Min Depth (m)", self.min_depth_var,
            0.0, 1.0, 0.05, row
        )

        self.max_depth_var = tk.DoubleVar(value=self.params.max_depth)
        row = self._add_slider(
            "Max Depth (m)", self.max_depth_var,
            1.0, 15.0, 0.5, row
        )

        # Depth Pre-processing
        row = self._add_section_header("Depth Pre-processing", row)

        self.apply_bilateral_filter_var = tk.BooleanVar(value=self.params.apply_bilateral_filter)
        row = self._add_checkbox(
            "Bilateral Filter", self.apply_bilateral_filter_var, row
        )

        self.bilateral_d_var = tk.IntVar(value=self.params.bilateral_d)
        row = self._add_slider(
            "Bilateral D", self.bilateral_d_var,
            3, 15, 2, row, is_int=True
        )

        self.bilateral_sigma_color_var = tk.DoubleVar(value=self.params.bilateral_sigma_color)
        row = self._add_slider(
            "Bilateral Sigma Color", self.bilateral_sigma_color_var,
            10.0, 200.0, 10.0, row
        )

        self.bilateral_sigma_space_var = tk.DoubleVar(value=self.params.bilateral_sigma_space)
        row = self._add_slider(
            "Bilateral Sigma Space", self.bilateral_sigma_space_var,
            10.0, 200.0, 10.0, row
        )

        self.apply_hole_filling_var = tk.BooleanVar(value=self.params.apply_hole_filling)
        row = self._add_checkbox(
            "Hole Filling", self.apply_hole_filling_var, row
        )

        # RGB-Guided Parameters
        row = self._add_section_header("RGB-Guided Parameters", row)

        self.canny_low_var = tk.IntVar(value=self.params.canny_low)
        row = self._add_slider(
            "Canny Low", self.canny_low_var,
            10, 150, 10, row, is_int=True
        )

        self.canny_high_var = tk.IntVar(value=self.params.canny_high)
        row = self._add_slider(
            "Canny High", self.canny_high_var,
            50, 300, 10, row, is_int=True
        )

        self.rgb_region_min_size_var = tk.IntVar(value=self.params.rgb_region_min_size)
        row = self._add_slider(
            "RGB Region Min Size", self.rgb_region_min_size_var,
            100, 5000, 100, row, is_int=True
        )

        # Apply button
        row += 1
        apply_btn = ttk.Button(
            self.scrollable_frame, text="Apply Changes",
            command=self._on_apply
        )
        apply_btn.grid(row=row, column=0, columnspan=3, pady=10, sticky='ew')

        # Reset button
        row += 1
        reset_btn = ttk.Button(
            self.scrollable_frame, text="Reset to Defaults",
            command=self._on_reset
        )
        reset_btn.grid(row=row, column=0, columnspan=3, pady=5, sticky='ew')

    def _add_section_header(self, text: str, row: int) -> int:
        """Add section header."""
        label = ttk.Label(
            self.scrollable_frame, text=text,
            font=('TkDefaultFont', 10, 'bold')
        )
        label.grid(row=row, column=0, columnspan=3, pady=(15, 5), sticky='w')
        return row + 1

    def _add_slider(self, label: str, var: tk.Variable, min_val: float,
                    max_val: float, step: float, row: int, is_int: bool = False) -> int:
        """Add labeled slider with value display."""
        # Label
        lbl = ttk.Label(self.scrollable_frame, text=label)
        lbl.grid(row=row, column=0, sticky='w', padx=5, pady=2)

        # Value display
        if is_int:
            val_lbl = ttk.Label(self.scrollable_frame, text=str(int(var.get())))
        else:
            val_lbl = ttk.Label(self.scrollable_frame, text=f"{var.get():.3f}")
        val_lbl.grid(row=row, column=2, sticky='e', padx=5)

        # Slider
        resolution = step if not is_int else 1
        slider = ttk.Scale(
            self.scrollable_frame, from_=min_val, to=max_val,
            variable=var, orient='horizontal'
        )
        slider.grid(row=row, column=1, sticky='ew', padx=5, pady=2)

        # Update value label on change
        def update_label(*args):
            if is_int:
                val_lbl.config(text=str(int(var.get())))
            else:
                val_lbl.config(text=f"{var.get():.3f}")
        var.trace_add('write', update_label)

        # Make column 1 expandable
        self.scrollable_frame.columnconfigure(1, weight=1)

        return row + 1

    def _add_checkbox(self, label: str, var: tk.BooleanVar, row: int) -> int:
        """Add labeled checkbox."""
        cb = ttk.Checkbutton(self.scrollable_frame, text=label, variable=var)
        cb.grid(row=row, column=0, columnspan=3, sticky='w', padx=5, pady=2)
        return row + 1

    def _add_combobox(self, label: str, var: tk.StringVar,
                      values: list, row: int) -> int:
        """Add labeled combobox."""
        lbl = ttk.Label(self.scrollable_frame, text=label)
        lbl.grid(row=row, column=0, sticky='w', padx=5, pady=2)

        cb = ttk.Combobox(
            self.scrollable_frame, textvariable=var,
            values=values, state='readonly'
        )
        cb.grid(row=row, column=1, columnspan=2, sticky='ew', padx=5, pady=2)
        return row + 1

    def _on_apply(self):
        """Apply current parameter values."""
        self.params = self.get_params()
        if self.on_change:
            self.on_change(self.params)

    def _on_reset(self):
        """Reset parameters to defaults."""
        default_params = PlaneDetectionParams()
        self.set_params(default_params)
        if self.on_change:
            self.on_change(default_params)

    def get_params(self) -> PlaneDetectionParams:
        """Get current parameter values."""
        return PlaneDetectionParams(
            algorithm=PlaneAlgorithm(self.algorithm_var.get()),
            distance_threshold=self.distance_threshold_var.get(),
            ransac_n=int(self.ransac_n_var.get()),
            num_iterations=int(self.num_iterations_var.get()),
            min_plane_points=int(self.min_plane_points_var.get()),
            min_plane_area=self.min_plane_area_var.get(),
            max_planes=int(self.max_planes_var.get()),
            normal_threshold_deg=self.normal_threshold_deg_var.get(),
            enable_normal_filter=self.enable_normal_filter_var.get(),
            downsample_factor=int(self.downsample_factor_var.get()),
            min_depth=self.min_depth_var.get(),
            max_depth=self.max_depth_var.get(),
            apply_bilateral_filter=self.apply_bilateral_filter_var.get(),
            bilateral_d=int(self.bilateral_d_var.get()),
            bilateral_sigma_color=self.bilateral_sigma_color_var.get(),
            bilateral_sigma_space=self.bilateral_sigma_space_var.get(),
            apply_hole_filling=self.apply_hole_filling_var.get(),
            canny_low=int(self.canny_low_var.get()),
            canny_high=int(self.canny_high_var.get()),
            rgb_region_min_size=int(self.rgb_region_min_size_var.get())
        )

    def set_params(self, params: PlaneDetectionParams):
        """Set parameter values from PlaneDetectionParams."""
        self.algorithm_var.set(params.algorithm.value)
        self.distance_threshold_var.set(params.distance_threshold)
        self.ransac_n_var.set(params.ransac_n)
        self.num_iterations_var.set(params.num_iterations)
        self.min_plane_points_var.set(params.min_plane_points)
        self.min_plane_area_var.set(params.min_plane_area)
        self.max_planes_var.set(params.max_planes)
        self.normal_threshold_deg_var.set(params.normal_threshold_deg)
        self.enable_normal_filter_var.set(params.enable_normal_filter)
        self.downsample_factor_var.set(params.downsample_factor)
        self.min_depth_var.set(params.min_depth)
        self.max_depth_var.set(params.max_depth)
        self.apply_bilateral_filter_var.set(params.apply_bilateral_filter)
        self.bilateral_d_var.set(params.bilateral_d)
        self.bilateral_sigma_color_var.set(params.bilateral_sigma_color)
        self.bilateral_sigma_space_var.set(params.bilateral_sigma_space)
        self.apply_hole_filling_var.set(params.apply_hole_filling)
        self.canny_low_var.set(params.canny_low)
        self.canny_high_var.set(params.canny_high)
        self.rgb_region_min_size_var.set(params.rgb_region_min_size)

        self.params = params
