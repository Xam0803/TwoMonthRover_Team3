import sys
import os
import json
import random
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QPushButton
)
from PyQt6.QtCore import Qt, QPointF, QTimer
from PyQt6.QtGui import QPainter, QPen, QColor, QKeyEvent
import pyqtgraph as pg

os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"

# Global pyqtgraph performance configs for X11 forwarding
pg.setConfigOption('background', '#121212')
pg.setConfigOption('foreground', '#AAAAAA')
pg.setConfigOption('antialias', False)


class BaseRotationIndicator(QWidget):
    """Shows right/left semi-circle highlights based on rotation direction."""
    def __init__(self):
        super().__init__()
        self.setMinimumSize(100, 100)
        self.direction = 0  # 0: Idle, 1: CW (Right), -1: CCW (Left)

    def set_direction(self, direction):
        if self.direction != direction:
            self.direction = direction
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect().adjusted(15, 15, -15, -15)
        
        # Background ring (Dark Gray)
        painter.setPen(QPen(QColor(50, 50, 50), 4))
        painter.drawEllipse(rect)

        # Active direction semi-circle (Green)
        if self.direction != 0:
            active_pen = QPen(QColor(0, 255, 100), 6)
            active_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(active_pen)

            if self.direction == 1:
                # Right side: Start at 12 o'clock (90), sweep Clockwise (-180) through 3 o'clock
                painter.drawArc(rect, 90 * 16, -180 * 16)
            elif self.direction == -1:
                # Left side: Start at 12 o'clock (90), sweep Counter-Clockwise (180) through 9 o'clock
                painter.drawArc(rect, 90 * 16, 180 * 16)


class JointVisualizer(QWidget):
    """Standard dial visualizer for angle-constrained joints."""
    def __init__(self, title="Joint"):
        super().__init__()
        self.title = title
        self.angle = 0.0
        self.setMinimumSize(100, 100)

    def set_angle(self, angle):
        new_angle = max(-180.0, min(180.0, float(angle)))
        if self.angle != new_angle:
            self.angle = new_angle
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)

        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2 - 10
        radius = min(w, h) / 2 - 20

        painter.setPen(QPen(QColor("#333338"), 3))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(self.angle)
        painter.setPen(QPen(QColor("#00FF88"), 4))
        painter.drawLine(0, 0, 0, int(-radius))
        painter.restore()

        painter.setPen(QColor("#00FF88"))
        painter.drawText(
            self.rect(),
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter,
            f"{self.angle:.1f}°"
        )


class GroundStationROSNode(Node):
    def __init__(self, telemetry_callback):
        super().__init__('axiom_ground_station_node')
        self.telemetry_callback = telemetry_callback
        
        self.telemetry_sub = self.create_subscription(
            String, '/rover/telemetry', self.telemetry_callback, 10
        )
        self.arm_cmd_pub = self.create_publisher(
            String, '/rover/arm_commands', 10
        )

    def publish_arm_targets(self, angles_dict):
        msg = String()
        msg.data = json.dumps(angles_dict)
        self.arm_cmd_pub.publish(msg)


class GroundStationCanvas(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        
        self.ros_node = ros_node
        self.base_direction = 0
        self.angles = {
            "lower_elbow": 0.0,
            "upper_joint": 0.0,
            "wrist": 0.0
        }

        # 1. Keyboard focus configuration
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # 2. Key processing timer
        self.pressed_keys = set()
        self.key_timer = QTimer(self)
        self.key_timer.setInterval(33)  # ~30 FPS
        self.key_timer.timeout.connect(self._process_held_keys)
        self.key_timer.start()

        self.reset_timer = QTimer(self)
        self.reset_timer.setInterval(30)
        self.reset_timer.timeout.connect(self._step_reset)

        # 3. Chart data arrays
        self.time_data = list(range(50))
        self.temp_data = [25.0] * 50
        self.press_data = [1000.0] * 50

        # 4. Build user interface
        self.init_ui()

        # 5. Telemetry update timer
        self.mock_timer = QTimer(self)
        self.mock_timer.timeout.connect(self.update_mock_telemetry)
        self.mock_timer.start(1000)

    def init_ui(self):
        self.setWindowTitle("AXIOM Ground Station")
        self.resize(1050, 650)
        self.setStyleSheet("background-color: #121214;")

        main_layout = QVBoxLayout(self)
        top_layout = QHBoxLayout()

        # LEFT PANEL (Controls & Info) 
        left_panel = QVBoxLayout()
        title_label = QLabel("AXIOM GROUND STATION")
        title_label.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;")
        
        controls_label = QLabel(
            "KEYBOARD CONTROLS:\n\n"
            "• A / D : Base Rotation (Hold)\n"
            "• W / S : Lower Elbow Joint\n"
            "• I / K : Upper Joint\n"
            "• J / L : Wrist / Claw Rotation"
        )
        controls_label.setStyleSheet("color: #AAAAAA; font-size: 13px; line-height: 1.5;")

        left_panel.addWidget(title_label)
        left_panel.addSpacing(15)
        left_panel.addWidget(controls_label)
        left_panel.addStretch()

        btn_style = "font-weight: bold; font-size: 12px; border-radius: 5px; padding: 10px;"

        self.power_btn = QPushButton("POWER SYSTEM")
        self.power_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.power_btn.setStyleSheet(btn_style + "background-color: #222226; color: #55555d; border: 1px solid #333338;")
        left_panel.addWidget(self.power_btn)

        left_panel.addSpacing(8)

        self.reset_btn = QPushButton("RESET POSITION")
        self.reset_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.reset_btn.setStyleSheet(btn_style + "background-color: #1f1215; color: #ff4444; border: 1px solid #441a1c;")
        self.reset_btn.clicked.connect(self.start_smooth_reset)
        left_panel.addWidget(self.reset_btn)

        top_layout.addLayout(left_panel, stretch=1)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setStyleSheet("color: #222225;")
        top_layout.addWidget(line)

        # RIGHT PANEL (Indicators & Graphs)
        right_panel = QVBoxLayout()
        viz_header = QLabel("JOINT KINEMATIC DISPLAY")
        viz_header.setStyleSheet("color: #00FF88; font-size: 14px; font-weight: bold;")
        right_panel.addWidget(viz_header, alignment=Qt.AlignmentFlag.AlignHCenter)

        grid_layout = QGridLayout()

        # Base Rotation Indicator
        base_container = QWidget()
        base_layout = QVBoxLayout(base_container)
        base_layout.setContentsMargins(5, 5, 5, 5)
        lbl_base_title = QLabel("BASE ROTATION")
        lbl_base_title.setStyleSheet("color: #888888; font-size: 11px; font-weight: bold;")
        
        self.viz_base = BaseRotationIndicator()
        self.lbl_base_val = QLabel("IDLE")
        self.lbl_base_val.setStyleSheet("color: #00FF88; font-size: 11px; font-weight: bold;")
        self.lbl_base_val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        base_layout.addWidget(lbl_base_title, alignment=Qt.AlignmentFlag.AlignHCenter)
        base_layout.addWidget(self.viz_base, alignment=Qt.AlignmentFlag.AlignCenter)
        base_layout.addWidget(self.lbl_base_val)

        # Other Joints
        self.viz_lower = JointVisualizer("Lower Elbow")
        self.viz_upper = JointVisualizer("Upper Joint")
        self.viz_wrist = JointVisualizer("Wrist / Claw")

        grid_layout.addWidget(base_container, 0, 0)
        grid_layout.addWidget(self.create_labeled_widget("LOWER ELBOW JOINT", self.viz_lower), 0, 1)
        grid_layout.addWidget(self.create_labeled_widget("UPPER JOINT", self.viz_upper), 1, 0)
        grid_layout.addWidget(self.create_labeled_widget("WRIST / CLAW TWIST", self.viz_wrist), 1, 1)

        right_panel.addLayout(grid_layout)

        # Live Graphs
        charts_layout = QHBoxLayout()
        
        self.plot_temp = pg.PlotWidget(title="Temperature (°C)")
        self.plot_temp.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.curve_temp = self.plot_temp.plot(self.time_data, self.temp_data, pen=pg.mkPen(color='#ff5555', width=2))
        charts_layout.addWidget(self.plot_temp)
        
        self.plot_press = pg.PlotWidget(title="Pressure (hPa)")
        self.plot_press.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.curve_press = self.plot_press.plot(self.time_data, self.press_data, pen=pg.mkPen(color='#5555ff', width=2))
        charts_layout.addWidget(self.plot_press)

        right_panel.addLayout(charts_layout)
        
        # Live Time Readout Under Graphs 
        self.lbl_telemetry = QLabel("TIME: --:--:--")
        self.lbl_telemetry.setStyleSheet("color: #00FF64; font-family: monospace; font-size: 13px; font-weight: bold; padding: 5px;")
        right_panel.addWidget(self.lbl_telemetry)

        top_layout.addLayout(right_panel, stretch=2)
        main_layout.addLayout(top_layout, stretch=3)

        # BOTTOM PANEL (6 Live Metric Readout Cards) 
        bottom_panel = QFrame()
        bottom_panel.setStyleSheet("border-top: 2px solid #222225; background-color: #0d0d0f; padding: 10px;")
        
        bottom_layout = QVBoxLayout(bottom_panel)
        reserved_label = QLabel("LIVE SENSOR READINGS")
        reserved_label.setStyleSheet("color: #00FF88; font-size: 12px; font-weight: bold;")
        bottom_layout.addWidget(reserved_label, alignment=Qt.AlignmentFlag.AlignLeft)

        grid_sensor = QGridLayout()
        grid_sensor.setSpacing(10)

        def create_metric_card(title_text):
            card = QFrame()
            card.setStyleSheet("background-color: #16161a; border: 1px solid #2a2a30; border-radius: 6px;")
            c_layout = QVBoxLayout(card)
            c_layout.setContentsMargins(8, 6, 8, 6)
            
            lbl_title = QLabel(title_text)
            lbl_title.setStyleSheet("color: #888888; font-size: 10px; font-weight: bold;")
            
            lbl_val = QLabel("--")
            lbl_val.setStyleSheet("color: #00FF88; font-size: 15px; font-weight: bold; font-family: monospace;")
            lbl_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            c_layout.addWidget(lbl_title, alignment=Qt.AlignmentFlag.AlignHCenter)
            c_layout.addWidget(lbl_val, alignment=Qt.AlignmentFlag.AlignHCenter)
            return card, lbl_val

        card_temp, self.lbl_val_temp = create_metric_card("TEMPERATURE")
        card_press, self.lbl_val_press = create_metric_card("PRESSURE")
        card_alt, self.lbl_val_alt = create_metric_card("ALTITUDE")
        card_yaw, self.lbl_val_yaw = create_metric_card("YAW")
        card_pitch, self.lbl_val_pitch = create_metric_card("PITCH")
        card_roll, self.lbl_val_roll = create_metric_card("ROLL")

        grid_sensor.addWidget(card_temp, 0, 0)
        grid_sensor.addWidget(card_press, 0, 1)
        grid_sensor.addWidget(card_alt, 0, 2)
        grid_sensor.addWidget(card_yaw, 0, 3)
        grid_sensor.addWidget(card_pitch, 0, 4)
        grid_sensor.addWidget(card_roll, 0, 5)

        bottom_layout.addLayout(grid_sensor)
        main_layout.addWidget(bottom_panel, stretch=1)



    def create_labeled_widget(self, title_text, visualizer):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(5, 5, 5, 5)

        lbl = QLabel(title_text)
        lbl.setStyleSheet("color: #888888; font-size: 11px; font-weight: bold;")
        
        layout.addWidget(lbl, alignment=Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(visualizer)
        return container

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.setFocus()

    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.pressed_keys.add(event.key())
            self._update_base_indicator()

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.pressed_keys.discard(event.key())
            self._update_base_indicator()

    def _update_base_indicator(self):
        if Qt.Key.Key_D in self.pressed_keys and Qt.Key.Key_A not in self.pressed_keys:
            self.base_direction = 1
            self.viz_base.set_direction(1)
            self.lbl_base_val.setText("CLOCKWISE")
        elif Qt.Key.Key_A in self.pressed_keys and Qt.Key.Key_D not in self.pressed_keys:
            self.base_direction = -1
            self.viz_base.set_direction(-1)
            self.lbl_base_val.setText("COUNTER-CLOCKWISE")
        else:
            self.base_direction = 0
            self.viz_base.set_direction(0)
            self.lbl_base_val.setText("IDLE")
        
        self.update_visualizers()

    def _process_held_keys(self):
        if not self.pressed_keys:
            return

        step = 1.5
        changed = False

        if Qt.Key.Key_W in self.pressed_keys:
            self.angles["lower_elbow"] = min(90.0, self.angles["lower_elbow"] + step)
            changed = True
        if Qt.Key.Key_S in self.pressed_keys:
            self.angles["lower_elbow"] = max(-90.0, self.angles["lower_elbow"] - step)
            changed = True

        if Qt.Key.Key_I in self.pressed_keys:
            self.angles["upper_joint"] = min(90.0, self.angles["upper_joint"] + step)
            changed = True
        if Qt.Key.Key_K in self.pressed_keys:
            self.angles["upper_joint"] = max(-90.0, self.angles["upper_joint"] - step)
            changed = True

        if Qt.Key.Key_J in self.pressed_keys:
            self.angles["wrist"] = max(-180.0, self.angles["wrist"] - step)
            changed = True
        if Qt.Key.Key_L in self.pressed_keys:
            self.angles["wrist"] = min(180.0, self.angles["wrist"] + step)
            changed = True

        if changed:
            self.update_visualizers()

    def update_visualizers(self):
        self.viz_lower.set_angle(self.angles["lower_elbow"])
        self.viz_upper.set_angle(self.angles["upper_joint"])
        self.viz_wrist.set_angle(self.angles["wrist"])
        
        if self.ros_node:
            cmd_payload = dict(self.angles)
            cmd_payload["base_dir"] = self.base_direction
            self.ros_node.publish_arm_targets(cmd_payload)

    def start_smooth_reset(self):
        if not self.reset_timer.isActive():
            self.reset_timer.start()

    def _step_reset(self):
        reset_speed = 1.5
        all_at_zero = True

        for joint_key in ["lower_elbow", "upper_joint", "wrist"]:
            current_val = self.angles[joint_key]
            if abs(current_val) > reset_speed:
                all_at_zero = False
                if current_val > 0:
                    self.angles[joint_key] -= reset_speed
                else:
                    self.angles[joint_key] += reset_speed
            else:
                self.angles[joint_key] = 0.0

        self.update_visualizers()

        if all_at_zero:
            self.reset_timer.stop()

    def update_telemetry_ui(self, data):
        """Updates metric cards when real ROS 2 telemetry arrives."""
        if "bmp388" in data:
            bmp = data["bmp388"]
            if "temp_c" in bmp:
                self.lbl_val_temp.setText(f"{bmp['temp_c']:.1f} °C")
            if "pressure_hpa" in bmp:
                self.lbl_val_press.setText(f"{bmp['pressure_hpa']:.1f} hPa")
            if "alt_m" in bmp:
                self.lbl_val_alt.setText(f"{bmp['alt_m']:.2f} m")
            
        if "bno055" in data:
            bno = data["bno055"]
            if "yaw" in bno:
                self.lbl_val_yaw.setText(f"{bno['yaw']:.1f}°")
            if "pitch" in bno:
                self.lbl_val_pitch.setText(f"{bno['pitch']:.1f}°")
            if "roll" in bno:
                self.lbl_val_roll.setText(f"{bno['roll']:.1f}°")

    def update_mock_telemetry(self):
        """Updates pyqtgraph chart data and bottom live sensor cards."""
        new_temp = self.temp_data[-1] + random.uniform(-0.5, 0.5)
        new_press = self.press_data[-1] + random.uniform(-1.5, 1.5)
        
        self.temp_data = self.temp_data[1:] + [new_temp]
        self.press_data = self.press_data[1:] + [new_press]
        
        self.curve_temp.setData(self.time_data, self.temp_data)
        self.curve_press.setData(self.time_data, self.press_data)
        
        # Update time under graphs
        timestamp = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
        self.lbl_telemetry.setText(f"TIME: {timestamp}")

        # Update 6 Live Metric Cards with mock readings
        mock_alt = 93.81 + random.uniform(-0.1, 0.1)
        mock_yaw = random.uniform(-1.0, 1.0)
        mock_pitch = random.uniform(-0.5, 0.5)
        mock_roll = random.uniform(-0.5, 0.5)

        self.lbl_val_temp.setText(f"{new_temp:.1f} °C")
        self.lbl_val_press.setText(f"{new_press:.1f} hPa")
        self.lbl_val_alt.setText(f"{mock_alt:.2f} m")
        self.lbl_val_yaw.setText(f"{mock_yaw:.1f}°")
        self.lbl_val_pitch.setText(f"{mock_pitch:.1f}°")
        self.lbl_val_roll.setText(f"{mock_roll:.1f}°")


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    window = None
    def handle_incoming_telemetry(msg):
        try:
            data = json.loads(msg.data)
            if window:
                window.update_telemetry_ui(data)
        except Exception:
            pass

    ros_node = GroundStationROSNode(handle_incoming_telemetry)
    window = GroundStationCanvas(ros_node)
    window.show()

    # Spin ROS 2 events smoothly without thrashing CPU
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(40)  # 25 Hz spin rate

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
