import sys
import os
import json
import time
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGridLayout, QLabel, QFrame, QPushButton, QDial)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
import pyqtgraph as pg

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
                # Right side: Start at 12 o'clock (90°), sweep Clockwise (-180°) through 3 o'clock
                painter.drawArc(rect, 90 * 16, -180 * 16)
            elif self.direction == -1:
                # Left side: Start at 12 o'clock (90°), sweep Counter-Clockwise (180°) through 9 o'clock
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
    """ROS 2 Node handling telemetry subscriptions and arm command publishing."""
    def __init__(self, telemetry_callback):
        super().__init__('axiom_ground_station')
        self.telemetry_callback = telemetry_callback

        # Subscriber for Orange Pi telemetry JSON packets
        self.sub_telemetry = self.create_subscription(
            String,
            '/rover/telemetry',
            self.on_telemetry_received,
            10
        )

        # Publisher for arm command JSON payloads
        self.pub_commands = self.create_publisher(
            String,
            '/rover/arm_commands',
            10
        )

    def on_telemetry_received(self, msg):
        try:
            data = json.loads(msg.data)
            if self.telemetry_callback:
                self.telemetry_callback(data)
        except json.JSONDecodeError:
            self.get_logger().error("Received malformed JSON telemetry packet.")

    def send_arm_command(self, cmd_dict):
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.pub_commands.publish(msg)


class GroundStationCanvas(QWidget):
    """Main Ground Station GUI application with high-contrast daylight theme & circular dials."""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("AXIOM Ground Station - ROS 2 Humble")
        self.resize(1200, 750)
        self.setStyleSheet("background-color: #F2F4F7;")

        # Active joint angle states (Degrees)
        self.joint_base = 0.0
        self.joint_lower_elbow = 0.0
        self.joint_upper = 0.0
        self.joint_wrist = 0.0
        self.base_dir = 0
        self.pressed_keys = set()

        # Telemetry plot data buffers (last 30 samples)
        self.max_samples = 30
        self.time_data = list(range(self.max_samples))
        self.temp_data = [25.0] * self.max_samples
        self.press_data = [1013.25] * self.max_samples

        self.init_ui()

        # ROS 2 executor timer (~33 Hz)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(30)

        # Control polling timer (30 Hz)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.process_control_inputs)
        self.control_timer.start(33)

        # Fallback mock telemetry generator (1 Hz)
        self.mock_timer = QTimer(self)
        self.mock_timer.timeout.connect(self.update_mock_telemetry)
        self.mock_timer.start(1000)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(12, 12, 12, 12)

        # --- TOP SECTION: CONTROLS, KINEMATIC DIALS & CHARTS ---
        top_layout = QHBoxLayout()

        # --- LEFT PANEL (Controls & Info) ---
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
        lbl_ctrls = QLabel(instructions)
        lbl_ctrls.setStyleSheet("color: #1A1D20; font-size: 11px; border: 1px solid #DCDFE6; background: #F8F9FA; padding: 6px; border-radius: 4px;")
        left_layout.addWidget(lbl_ctrls)

        # Wall-E image display
        lbl_walle = QLabel()
        pixmap_walle = QPixmap("walle.png")
        if not pixmap_walle.isNull():
            lbl_walle.setPixmap(pixmap_walle.scaledToWidth(200, Qt.TransformationMode.SmoothTransformation))
            lbl_walle.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_walle.setStyleSheet("border: none; background: transparent; padding-top: 4px;")
            left_layout.addWidget(lbl_walle, alignment=Qt.AlignmentFlag.AlignCenter)

        # Reset button
        btn_reset = QPushButton("RESET POSITION")
        btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #D32F2F;
                color: #FFFFFF;
                font-weight: bold;
                font-size: 11px;
                padding: 8px;
                border-radius: 4px;
                border: none;
            }
            QPushButton:hover {
                background-color: #B71C1C;
            }
        """)
        btn_reset.clicked.connect(self.reset_arm_position)
        left_layout.addWidget(btn_reset)

        left_layout.addStretch()
        top_layout.addWidget(left_panel, stretch=1)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setStyleSheet("color: #222225;")
        top_layout.addWidget(line)

        # --- RIGHT PANEL (Indicators & Graphs) ---
        right_panel = QVBoxLayout()
        viz_header = QLabel("JOINT KINEMATIC DISPLAY")
        viz_header.setStyleSheet("color: #00FF88; font-size: 14px; font-weight: bold;")
        right_panel.addWidget(viz_header, alignment=Qt.AlignmentFlag.AlignHCenter)

        grid_dials = QHBoxLayout()
        grid_dials.setSpacing(12)

        def create_circular_dial(title_text, min_val=-180, max_val=180):
            container = QFrame()
            container.setStyleSheet("background-color: #F8F9FA; border: 1.5px solid #000000; border-radius: 6px;")
            c_layout = QVBoxLayout(container)
            c_layout.setContentsMargins(6, 6, 6, 6)

            lbl_title = QLabel(title_text)
            lbl_title.setStyleSheet("color: #4A5568; font-size: 10px; font-weight: bold;")

            dial = QDial()
            dial.setRange(min_val, max_val)
            dial.setNotchesVisible(True)
            dial.setEnabled(False)  # Display-only circular gauge
            dial.setFixedSize(70, 70)
            dial.setStyleSheet("QDial { background-color: #FFFFFF; }")

            lbl_val = QLabel("0.0°")
            lbl_val.setStyleSheet("color: #000000; font-size: 12px; font-weight: bold; font-family: monospace;")

            c_layout.addWidget(lbl_title, alignment=Qt.AlignmentFlag.AlignHCenter)
            c_layout.addWidget(dial, alignment=Qt.AlignmentFlag.AlignHCenter)
            c_layout.addWidget(lbl_val, alignment=Qt.AlignmentFlag.AlignHCenter)
            return container, dial, lbl_val

        card_dial_base, self.dial_base, self.lbl_val_dial_base = create_circular_dial("BASE ROTATION")
        card_dial_elbow, self.dial_elbow, self.lbl_val_dial_elbow = create_circular_dial("LOWER ELBOW")
        card_dial_upper, self.dial_upper, self.lbl_val_dial_upper = create_circular_dial("UPPER JOINT")
        card_dial_wrist, self.dial_wrist, self.lbl_val_dial_wrist = create_circular_dial("WRIST TWIST")

        grid_dials.addWidget(card_dial_base)
        grid_dials.addWidget(card_dial_elbow)
        grid_dials.addWidget(card_dial_upper)
        grid_dials.addWidget(card_dial_wrist)

        dials_layout.addLayout(grid_dials)
        mid_right_layout.addWidget(dials_frame)

        # --- CHARTS SECTION ---
        charts_layout = QHBoxLayout()

        # Temperature Plot
        self.plot_temp = pg.PlotWidget(title="<b>TEMPERATURE (°C)</b>")
        self.plot_temp.showGrid(x=True, y=True, alpha=0.3)
        self.curve_temp = self.plot_temp.plot(pen=pg.mkPen(color='#D32F2F', width=2))

        # Pressure Plot
        self.plot_press = pg.PlotWidget(title="<b>PRESSURE (hPa)</b>")
        self.plot_press.showGrid(x=True, y=True, alpha=0.3)
        self.curve_press = self.plot_press.plot(pen=pg.mkPen(color='#0288D1', width=2))

        charts_layout.addWidget(self.plot_temp)
        charts_layout.addWidget(self.plot_press)

        right_panel.addLayout(charts_layout)
        
        # --- Live Time Readout Under Graphs ---
        self.lbl_telemetry = QLabel("TIME: --:--:--")
        self.lbl_telemetry.setStyleSheet("color: #007A33; font-family: monospace; font-size: 13px; font-weight: bold; padding: 2px;")
        mid_right_layout.addWidget(self.lbl_telemetry)

        top_layout.addLayout(mid_right_layout, stretch=3)
        main_layout.addLayout(top_layout, stretch=3)

        # --- BOTTOM PANEL (6 Live Metric Readout Cards) ---
        bottom_panel = QFrame()
        bottom_panel.setStyleSheet("background-color: #FFFFFF; border: 2px solid #C0C4CC; border-radius: 8px; padding: 10px;")
        
        bottom_layout = QVBoxLayout(bottom_panel)
        reserved_label = QLabel("LIVE SENSOR READINGS")
        reserved_label.setStyleSheet("color: #000000; font-size: 12px; font-weight: bold;")
        bottom_layout.addWidget(reserved_label, alignment=Qt.AlignmentFlag.AlignLeft)

        grid_sensor = QGridLayout()
        grid_sensor.setSpacing(10)

        def create_metric_card(title_text):
            card = QFrame()
            card.setStyleSheet("background-color: #F8F9FA; border: 1.5px solid #000000; border-radius: 6px;")
            c_layout = QVBoxLayout(card)
            c_layout.setContentsMargins(8, 6, 8, 6)
            
            lbl_title = QLabel(title_text)
            lbl_title.setStyleSheet("color: #4A5568; font-size: 11px; font-weight: bold;")
            
            lbl_val = QLabel("--")
            lbl_val.setStyleSheet("color: #000000; font-size: 15px; font-weight: bold; font-family: monospace;")
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

    def keyPressEvent(self, event):
        self.pressed_keys.add(event.key())
        if event.key() == Qt.Key.Key_R:
            self.reset_arm_position()

    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())

    def process_control_inputs(self):
        """Polls held keys, updates joint target positions, and updates circular dials."""
        step = 1.0
        moved = False

        if Qt.Key.Key_A in self.pressed_keys:
            self.joint_base -= step
            self.base_dir = -1
            moved = True
        elif Qt.Key.Key_D in self.pressed_keys:
            self.joint_base += step
            self.base_dir = 1
            moved = True
        else:
            self.base_dir = 0

        if Qt.Key.Key_W in self.pressed_keys:
            self.joint_lower_elbow += step
            moved = True
        if Qt.Key.Key_S in self.pressed_keys:
            self.joint_lower_elbow -= step
            moved = True

        if Qt.Key.Key_I in self.pressed_keys:
            self.joint_upper += step
            moved = True
        if Qt.Key.Key_K in self.pressed_keys:
            self.joint_upper -= step
            moved = True

        if Qt.Key.Key_J in self.pressed_keys:
            self.joint_wrist -= step
            moved = True
        if Qt.Key.Key_L in self.pressed_keys:
            self.joint_wrist += step
            moved = True

        if moved:
            self.update_dial_visualizers()
            cmd = {
                "lower_elbow": self.joint_lower_elbow,
                "upper_joint": self.joint_upper,
                "wrist": self.joint_wrist,
                "base_dir": self.base_dir
            }
            self.ros_node.send_arm_command(cmd)

    def update_dial_visualizers(self):
        """Updates the needle position and numerical readout for all 4 circular dials."""
        # Base
        self.dial_base.setValue(int(self.joint_base))
        self.lbl_val_dial_base.setText(f"{self.joint_base:.1f}°")

        # Lower Elbow
        self.dial_elbow.setValue(int(self.joint_lower_elbow))
        self.lbl_val_dial_elbow.setText(f"{self.joint_lower_elbow:.1f}°")

        # Upper Joint
        self.dial_upper.setValue(int(self.joint_upper))
        self.lbl_val_dial_upper.setText(f"{self.joint_upper:.1f}°")

        # Wrist
        self.dial_wrist.setValue(int(self.joint_wrist))
        self.lbl_val_dial_wrist.setText(f"{self.joint_wrist:.1f}°")

    def reset_arm_position(self):
        self.joint_base = 0.0
        self.joint_lower_elbow = 0.0
        self.joint_upper = 0.0
        self.joint_wrist = 0.0
        self.base_dir = 0
        self.update_dial_visualizers()

        cmd = {
            "lower_elbow": 0.0,
            "upper_joint": 0.0,
            "wrist": 0.0,
            "base_dir": 0
        }
        self.ros_node.send_arm_command(cmd)

    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def update_telemetry_ui(self, data):
        """Updates metric cards and charts upon receipt of real ROS 2 telemetry."""
        if self.mock_timer.isActive():
            self.mock_timer.stop()

        if "timestamp" in data:
            self.lbl_telemetry.setText(f"TIME: {data['timestamp']}")

        if "bmp388" in data:
            bmp = data["bmp388"]
            temp = bmp.get('temp_c')
            press = bmp.get('pressure_hpa')
            alt = bmp.get('alt_m')

            if temp is not None:
                self.lbl_val_temp.setText(f"{temp:.1f} °C")
                self.temp_data = self.temp_data[1:] + [float(temp)]
                self.curve_temp.setData(self.time_data, self.temp_data)

            if press is not None:
                self.lbl_val_press.setText(f"{press:.1f} hPa")
                self.press_data = self.press_data[1:] + [float(press)]
                self.curve_press.setData(self.time_data, self.press_data)

            if alt is not None:
                self.lbl_val_alt.setText(f"{alt:.2f} m")

        if "bno055" in data:
            bno = data["bno055"]
            if "yaw" in bno:
                self.lbl_val_yaw.setText(f"{bno['yaw']:.1f}°")
            if "pitch" in bno:
                self.lbl_val_pitch.setText(f"{bno['pitch']:.1f}°")
            if "roll" in bno:
                self.lbl_val_roll.setText(f"{bno['roll']:.1f}°")

    def update_mock_telemetry(self):
        """Fallback mock generator operating until real ROS 2 telemetry arrives."""
        current_time = time.strftime("%Y/%m/%d %H:%M:%S")
        mock_data = {
            "timestamp": current_time,
            "bmp388": {
                "temp_c": 24.5,
                "pressure_hpa": 1012.8,
                "alt_m": 94.2
            },
            "bno055": {
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0
            }
        }
        self.update_telemetry_ui(mock_data)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    ros_node = GroundStationROSNode(telemetry_callback=None)
    gui = GroundStationCanvas(ros_node)
    ros_node.telemetry_callback = gui.update_telemetry_ui

    gui.show()
    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
