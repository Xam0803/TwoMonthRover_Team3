#!/usr/bin/env python3
"""AXIOM Ground Station - Rover Arm Control & Telemetry GUI"""

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
        # System States & Defaults
        # -------------------------------------------------------------------
        self.mode = "SIMULATED"  # Initial Mode: 'SIMULATED' or 'LIVE ROS 2'

        # Default Home Position: All Servos at 0.0° (Claw Open), Base Stopped
        self.arm_angles = {
            "lower_elbow": 0.0,  # Smraza 45KG (0° - 270°)
            "upper_joint": 0.0,  # Miuzei 25KG (0° - 180°)
            "wrist": 0.0,        # MG996R (0° - 180°)
            "claw": 0.0,         # MG996R (0° = Open, 180° = Closed)
            "base_dir": 0.0      # GM9 DC Motor (-1.0 to +1.0)
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
        # ROS 2 Integration Setup
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

        # Build UI Layout & Theme
        self.init_ui()

        # Main Event Loop Timer (50 ms / 20 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(50)

        # Publish initial Home position (0° / Claw Open) on startup
        self.publish_arm_commands()

    # -----------------------------------------------------------------------
    # UI Initialization & Styling
    # -----------------------------------------------------------------------
    def init_ui(self):
        # Apply Dark Aesthetic Theme
        self.setStyleSheet("""
            QMainWindow { background-color: #181825; }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #313244;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: #1e1e2e;
                color: #cdd6f4;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 2px 8px;
                color: #89b4fa;
            }
            QLabel { color: #cdd6f4; font-size: 13px; }
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

        # 3. Telemetry Plotting Dashboard (pyqtgraph)
        main_layout.addWidget(self.create_telemetry_plots())

        # 4. Live Metric Summary Cards
        main_layout.addWidget(self.create_metrics_bar())

        # 5. Keyboard Shortcut Guide Footer
        main_layout.addWidget(self.create_footer_legend())

    def create_header(self):
        header_frame = QFrame()
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(0, 0, 0, 0)

        title = QLabel("AXIOM ROVER GROUND STATION")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #cba6f7;")

        self.mode_label = QLabel(f"MODE: {self.mode}")
        self.mode_label.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        self.update_mode_banner_style()

        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(self.mode_label)
        return header_frame

    def create_arm_panel(self):
        group = QGroupBox("ARM ACTUATOR LIVE READOUTS & TELEMETRY")
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
            lbl_name.setStyleSheet("color: #a6adc8; font-weight: bold;")
            lbl_val = QLabel(default_str)
            lbl_val.setStyleSheet("color: #a6e3a1; font-weight: bold; font-size: 14px;")
            self.joint_labels[key] = lbl_val

            col = (idx % 3) * 2
            row = idx // 3
            layout.addWidget(lbl_name, row, col)
            layout.addWidget(lbl_val, row, col + 1)

        return group

    def create_telemetry_plots(self):
        pg.setConfigOption('background', '#1e1e2e')
        pg.setConfigOption('foreground', '#cdd6f4')

        plot_layout = QHBoxLayout()

        # BMP388 Environmental Plot
        self.bmp_plot = pg.PlotWidget(title="BMP388 Environmental Telemetry")
        self.bmp_plot.showGrid(x=True, y=True, alpha=0.3)
        self.bmp_plot.addLegend()
        self.curve_temp = self.bmp_plot.plot(pen=pg.mkPen('#f38ba8', width=2), name="Temp (°C)")
        self.curve_press = self.bmp_plot.plot(pen=pg.mkPen('#89b4fa', width=2), name="Pressure (hPa / 10)")
        self.curve_alt = self.bmp_plot.plot(pen=pg.mkPen('#a6e3a1', width=2), name="Alt (m)")

        # BNO055 Acceleration Plot
        self.bno_plot = pg.PlotWidget(title="BNO055 3-Axis Acceleration")
        self.bno_plot.showGrid(x=True, y=True, alpha=0.3)
        self.bno_plot.addLegend()
        self.curve_acc_x = self.bno_plot.plot(pen=pg.mkPen('#fab387', width=2), name="Acc X (m/s²)")
        self.curve_acc_y = self.bno_plot.plot(pen=pg.mkPen('#f9e2af', width=2), name="Acc Y (m/s²)")
        self.curve_acc_z = self.bno_plot.plot(pen=pg.mkPen('#94e2d5', width=2), name="Acc Z (m/s²)")

        plot_layout.addWidget(self.bmp_plot)
        plot_layout.addWidget(self.bno_plot)

        container = QFrame()
        container.setLayout(plot_layout)
        return container

    def create_metrics_bar(self):
        group = QGroupBox("SENSOR METRICS SUMMARY")
        layout = QHBoxLayout(group)

        self.card_temp = QLabel("Temp: -- °C")
        self.card_press = QLabel("Pressure: -- hPa")
        self.card_alt = QLabel("Altitude: -- m")
        self.card_acc = QLabel("Acc (X,Y,Z): --, --, --")

        for card in [self.card_temp, self.card_press, self.card_alt, self.card_acc]:
            card.setStyleSheet("""
                background-color: #313244;
                border-radius: 6px;
                padding: 8px;
                font-weight: bold;
                color: #89b4fa;
            """)
            layout.addWidget(card)

        return group

    def create_footer_legend(self):
        footer = QLabel(
            "KEYBOARD CONTROLS:  [W/S] Elbow  |  [I/K] Upper Joint  |  [J/L] Wrist  |  "
            "[U/O] Claw (Open/Close)  |  [A/D] Base Motor  |  [R] Zero/Home Arm  |  [T] Toggle Mode"
        )
        footer.setAlignment(Qt.AlignmentFlag.AlignCenter)
        footer.setStyleSheet("color: #7f849c; font-weight: bold; font-size: 12px; padding: 4px;")
        return footer

    # -----------------------------------------------------------------------
    # ROS 2 & Command Publishing Logic
    # -----------------------------------------------------------------------
    def publish_arm_commands(self):
        """Publishes the current arm command JSON payload over ROS 2."""
        if ROS2_AVAILABLE and self.ros_pub:
            msg = String()
            msg.data = json.dumps(self.arm_angles)
            self.ros_pub.publish(msg)

    def ros_telemetry_callback(self, msg):
        """Processes incoming ROS 2 telemetry when in LIVE ROS 2 mode."""
        if self.mode == "LIVE ROS 2":
            try:
                payload = json.loads(msg.data)
                self.process_telemetry_payload(payload)
            except Exception as e:
                print(f"[ERROR] Failed to parse ROS 2 telemetry JSON: {e}")

    # -----------------------------------------------------------------------
    # Main Event Loop & Telemetry Generator
    # -----------------------------------------------------------------------
    def update_loop(self):
        # 1. Spin ROS 2 Node (if active)
        if ROS2_AVAILABLE and self.ros_node:
            rclpy.spin_once(self.ros_node, timeout_sec=0.001)

        # 2. Handle Simulation Mode Telemetry Generation
        if self.mode == "SIMULATED":
            t = time.time()
            mock_payload = {
                "timestamp": datetime.now().strftime("%H:%M:%S"),
                "bmp388": {
                    "temp_c": round(24.5 + math.sin(t * 0.5) * 1.5 + random.uniform(-0.1, 0.1), 2),
                    "pressure_hpa": round(1013.25 + math.cos(t * 0.2) * 4.0 + random.uniform(-0.3, 0.3), 2),
                    "alt_m": round(95.0 + math.sin(t * 0.1) * 2.0 + random.uniform(-0.2, 0.2), 2)
                },
                "bno055": {
                    "acc_x": round(random.uniform(-0.15, 0.15), 2),
                    "acc_y": round(random.uniform(-0.15, 0.15), 2),
                    "acc_z": round(9.81 + random.uniform(-0.08, 0.08), 2)
                }
            }
            self.process_telemetry_payload(mock_payload)

    def process_telemetry_payload(self, data):
        """Updates internal buffers, graphs, and metric cards from telemetry."""
        bmp = data.get("bmp388", {})
        bno = data.get("bno055", {})

        temp = bmp.get("temp_c", 0.0)
        press = bmp.get("pressure_hpa", 0.0)
        alt = bmp.get("alt_m", 0.0)
        acc_x = bno.get("acc_x", 0.0)
        acc_y = bno.get("acc_y", 0.0)
        acc_z = bno.get("acc_z", 0.0)

        # Maintain Fixed-Length History Buffers
        self.time_history.append(time.time())
        self.temp_history.append(temp)
        self.pressure_history.append(press / 10.0)  # Scaled for graph visibility
        self.alt_history.append(alt)
        self.acc_x_history.append(acc_x)
        self.acc_y_history.append(acc_y)
        self.acc_z_history.append(acc_z)

        if len(self.time_history) > self.max_data_points:
            self.time_history.pop(0)
            self.temp_history.pop(0)
            self.pressure_history.pop(0)
            self.alt_history.pop(0)
            self.acc_x_history.pop(0)
            self.acc_y_history.pop(0)
            self.acc_z_history.pop(0)

        # Update pyqtgraph Curves
        x_vals = list(range(len(self.time_history)))
        self.curve_temp.setData(x_vals, self.temp_history)
        self.curve_press.setData(x_vals, self.pressure_history)
        self.curve_alt.setData(x_vals, self.alt_history)
        self.curve_acc_x.setData(x_vals, self.acc_x_history)
        self.curve_acc_y.setData(x_vals, self.acc_y_history)
        self.curve_acc_z.setData(x_vals, self.acc_z_history)

        # Update Summary Cards
        self.card_temp.setText(f"Temp: {temp:.2f} °C")
        self.card_press.setText(f"Pressure: {press:.2f} hPa")
        self.card_alt.setText(f"Altitude: {alt:.2f} m")
        self.card_acc.setText(f"Acc (X,Y,Z): {acc_x:.1f}, {acc_y:.1f}, {acc_z:.1f} m/s²")

    # -----------------------------------------------------------------------
    # Keyboard Interaction & Mode Switching
    # -----------------------------------------------------------------------
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        step = 5.0  # Angle increment per keystroke

        # [T] Toggle Mode
        if key == Qt.Key.Key_T:
            self.mode = "LIVE ROS 2" if self.mode == "SIMULATED" else "SIMULATED"
            self.update_mode_banner_style()

        # [R] Reset to Home Position (0.0° / Claw Open)
        elif key == Qt.Key.Key_R:
            self.arm_angles["lower_elbow"] = 0.0
            self.arm_angles["upper_joint"] = 0.0
            self.arm_angles["wrist"] = 0.0
            self.arm_angles["claw"] = 0.0
            self.arm_angles["base_dir"] = 0.0
            self.publish_arm_commands()

        # [W / S] Lower Elbow (0° - 270°)
        elif key == Qt.Key.Key_W:
            self.arm_angles["lower_elbow"] = min(270.0, self.arm_angles["lower_elbow"] + step)
            self.publish_arm_commands()
        elif key == Qt.Key.Key_S:
            self.arm_angles["lower_elbow"] = max(0.0, self.arm_angles["lower_elbow"] - step)
            self.publish_arm_commands()

        # [I / K] Upper Joint (0° - 180°)
        elif key == Qt.Key.Key_I:
            self.arm_angles["upper_joint"] = min(180.0, self.arm_angles["upper_joint"] + step)
            self.publish_arm_commands()
        elif key == Qt.Key.Key_K:
            self.arm_angles["upper_joint"] = max(0.0, self.arm_angles["upper_joint"] - step)
            self.publish_arm_commands()

        # [J / L] Wrist Twist (0° - 180°)
        elif key == Qt.Key.Key_L:
            self.arm_angles["wrist"] = min(180.0, self.arm_angles["wrist"] + step)
            self.publish_arm_commands()
        elif key == Qt.Key.Key_J:
            self.arm_angles["wrist"] = max(0.0, self.arm_angles["wrist"] - step)
            self.publish_arm_commands()

        # [U / O] Claw Control (0° = Open, 180° = Closed)
        elif key == Qt.Key.Key_O:
            self.arm_angles["claw"] = min(180.0, self.arm_angles["claw"] + step)  # Close
            self.publish_arm_commands()
        elif key == Qt.Key.Key_U:
            self.arm_angles["claw"] = max(0.0, self.arm_angles["claw"] - step)  # Open
            self.publish_arm_commands()

        # [A / D] Base DC Motor Direction (-1.0 to +1.0)
        elif key == Qt.Key.Key_D:
            self.arm_angles["base_dir"] = min(1.0, round(self.arm_angles["base_dir"] + 0.2, 1))
            self.publish_arm_commands()
        elif key == Qt.Key.Key_A:
            self.arm_angles["base_dir"] = max(-1.0, round(self.arm_angles["base_dir"] - 0.2, 1))
            self.publish_arm_commands()

        self.update_angle_displays()

    def update_angle_displays(self):
        """Reflects current angle values on UI readouts."""
        self.joint_labels["lower_elbow"].setText(f"{self.arm_angles['lower_elbow']:.1f}°")
        self.joint_labels["upper_joint"].setText(f"{self.arm_angles['upper_joint']:.1f}°")
        self.joint_labels["wrist"].setText(f"{self.arm_angles['wrist']:.1f}°")

        claw_deg = self.arm_angles["claw"]
        claw_status = "OPEN" if claw_deg == 0.0 else f"{claw_deg:.1f}°"
        self.joint_labels["claw"].setText(f"{claw_deg:.1f}° ({claw_status})")

        base_val = self.arm_angles["base_dir"]
        base_status = "STOPPED" if base_val == 0.0 else ("RIGHT" if base_val > 0 else "LEFT")
        self.joint_labels["base_dir"].setText(f"{base_val:.1f} ({base_status})")

    def update_mode_banner_style(self):
        self.mode_label.setText(f"MODE: {self.mode}")
        if self.mode == "LIVE ROS 2":
            self.mode_label.setStyleSheet("""
                background-color: #a6e3a1; color: #11111b;
                padding: 4px 12px; border-radius: 6px; font-weight: bold;
            """)
        else:
            self.mode_label.setStyleSheet("""
                background-color: #f9e2af; color: #11111b;
                padding: 4px 12px; border-radius: 6px; font-weight: bold;
            """)

    def closeEvent(self, event):
        """Clean shutdown handler."""
        if ROS2_AVAILABLE and self.ros_node:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        event.accept()


# ---------------------------------------------------------------------------
# Application Entry Point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    gui = GroundStationApp()
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()