#!/usr/bin/env python3
"""
AXIOM Ground Station - Rover Arm Control & Telemetry GUI (Light Mode)
---------------------------------------------------------------------
- Interactive PyQt6 & pyqtgraph Ground Station interface.
- Subscribes to `/rover/telemetry` for live sensor data (BMP388 & BNO055).
- Publishes arm joint control commands to `/rover/arm_commands`.
- Standardizes Home Position to 0.0° across all joints on startup.
- Telemetry CSV logging is handled on-board by the Pico hardware.
"""

import sys
import json
import time
import random
import math
from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGridLayout, QGroupBox, QFrame
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QKeyEvent

import pyqtgraph as pg

# Conditional ROS 2 Import
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class GroundStationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AXIOM Ground Station - Rover Arm Control")
        self.resize(1150, 780)

        # -------------------------------------------------------------------
        # System States & Startup Home Position (0.0° / Claw Open)
        # -------------------------------------------------------------------
        self.mode = "SIMULATED"  # Modes: 'SIMULATED' or 'LIVE ROS 2'

        self.arm_angles = {
            "lower_elbow": 0.0,  # Smraza 45KG (0° - 270°)
            "upper_joint": 0.0,  # Miuzei 25KG (0° - 180°)
            "wrist": 0.0,        # MG996R (0° - 180°)
            "claw": 0.0,         # MG996R (0.0° = Fully Open)
            "base_dir": 0.0      # GM9 DC Motor (0.0 = Stopped)
        }

        # Telemetry Data Buffers for Real-Time Plotting
        self.time_history = []
        self.temp_history = []
        self.pressure_history = []
        self.alt_history = []
        self.acc_x_history = []
        self.acc_y_history = []
        self.acc_z_history = []
        self.max_data_points = 50

        # -------------------------------------------------------------------
        # ROS 2 Integration
        # -------------------------------------------------------------------
        self.ros_node = None
        self.ros_pub = None
        self.ros_sub = None

        if ROS2_AVAILABLE:
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = rclpy.create_node('ground_station_gui')
            self.ros_pub = self.ros_node.create_publisher(String, '/rover/arm_commands', 10)
            self.ros_sub = self.ros_node.create_subscription(
                String, '/rover/telemetry', self.ros_telemetry_callback, 10
            )

        # Build UI Layout
        self.init_ui()

        # Main Event Loop Timer (50 ms / 20 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(50)

        # Publish 0° Home Position on startup
        self.publish_arm_commands()

    # -----------------------------------------------------------------------
    # Light Mode UI Construction & Styling
    # -----------------------------------------------------------------------
    def init_ui(self):
        # Apply Clean Light Mode Stylesheet
        self.setStyleSheet("""
            QMainWindow { background-color: #f8f9fa; }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #ced4da;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: #ffffff;
                color: #212529;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 2px 8px;
                color: #0d6efd;
            }
            QLabel { color: #212529; font-size: 13px; }
            QFrame { border: none; }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # 1. Header Bar
        main_layout.addWidget(self.create_header())

        # 2. Arm Status & Angle Readout Panel
        main_layout.addWidget(self.create_arm_panel())

        # 3. Real-Time Telemetry Plots (pyqtgraph)
        main_layout.addWidget(self.create_telemetry_plots())

        # 4. Live Metric Summary Cards
        main_layout.addWidget(self.create_metrics_bar())

        # 5. Keyboard Legend Footer
        main_layout.addWidget(self.create_footer_legend())

    def create_header(self):
        header_frame = QFrame()
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(0, 0, 0, 0)

        title = QLabel("AXIOM ROVER GROUND STATION")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #0d6efd;")

        self.mode_label = QLabel(f"MODE: {self.mode}")
        self.mode_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.update_mode_banner_style()

        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(self.mode_label)
        return header_frame

    def create_arm_panel(self):
        group = QGroupBox("ARM ACTUATOR LIVE READOUTS (0° HOME DEFAULT)")
        layout = QGridLayout(group)

        self.joint_labels = {}
        joints = [
            ("Lower Elbow (GP0):", "lower_elbow", "0.0°"),
            ("Upper Joint (GP1):", "upper_joint", "0.0°"),
            ("Wrist Twist (GP4):", "wrist", "0.0°"),
            ("Claw State (GP12):", "claw", "0.0° (OPEN)"),
            ("Base Motor (GP6/7/11):", "base_dir", "0.0 (STOPPED)")
        ]

        for idx, (name, key, default_str) in enumerate(joints):
            lbl_name = QLabel(name)
            lbl_name.setStyleSheet("color: #495057; font-weight: bold;")
            lbl_val = QLabel(default_str)
            lbl_val.setStyleSheet("color: #198754; font-weight: bold; font-size: 14px;")
            self.joint_labels[key] = lbl_val

            col = (idx % 3) * 2
            row = idx // 3
            layout.addWidget(lbl_name, row, col)
            layout.addWidget(lbl_val, row, col + 1)

        return group

    def create_telemetry_plots(self):
        # Configure Light Mode Theme for pyqtgraph
        pg.setConfigOption('background', '#ffffff')
        pg.setConfigOption('foreground', '#212529')

        plot_layout = QHBoxLayout()

        # BMP388 Environmental Plot
        self.bmp_plot = pg.PlotWidget(title="BMP388 Environmental Telemetry")
        self.bmp_plot.showGrid(x=True, y=True, alpha=0.3)
        self.bmp_plot.addLegend()
        self.curve_temp = self.bmp_plot.plot(pen=pg.mkPen('#dc3545', width=2), name="Temp (°C)")
        self.curve_press = self.bmp_plot.plot(pen=pg.mkPen('#0d6efd', width=2), name="Pressure (hPa / 10)")
        self.curve_alt = self.bmp_plot.plot(pen=pg.mkPen('#198754', width=2), name="Alt (m)")

        # BNO05