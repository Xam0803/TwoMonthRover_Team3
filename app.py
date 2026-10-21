import sys
import os
import json
import time
import math
import csv

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QFrame, QPushButton
)
from PyQt6.QtCore import Qt, QPointF, QRectF, QTimer
from PyQt6.QtGui import QPainter, QPen, QColor, QPixmap
import pyqtgraph as pg

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configure environment for X11 forwarding / High-DPI display
os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"

# Configure pyqtgraph for high-visibility outdoor daylight mode
pg.setConfigOption('background', 'w')       # White plot background
pg.setConfigOption('foreground', 'k')       # Black axes and text
pg.setConfigOption('antialias', False)      # Optimize render speed over X11


class BaseRotationIndicator(QWidget):
    """Custom circular ring visualizer highlighting CW/CCW base rotation direction."""
    def __init__(self, title="BASE ROTATION"):
        super().__init__()
        self.title = title
        self.direction = 0  # 0: Idle, 1: CW (Right), -1: CCW (Left)
        self.setMinimumSize(110, 110)

    def set_direction(self, direction):
        if self.direction != direction:
            self.direction = direction
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2 - 3
        radius = min(w, h) / 2 - 25

        # Title at top
        painter.setPen(QColor("#4A5568"))
        font = painter.font()
        font.setBold(True)
        font.setPointSize(9)
        painter.setFont(font)
        painter.drawText(
            self.rect().adjusted(0, 2, 0, 0),
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter,
            self.title
        )

        circle_rect = QRectF(cx - radius, cy - radius, 2 * radius, 2 * radius)

        # Outer ring
        painter.setPen(QPen(QColor("#718096"), 4))
        painter.drawEllipse(circle_rect)

        # Active direction semi-circle arc
        if self.direction != 0:
            active_pen = QPen(QColor("#00C853"), 6)
            active_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(active_pen)

            if self.direction == 1:
                painter.drawArc(circle_rect, 90 * 16, -180 * 16)
            elif self.direction == -1:
                painter.drawArc(circle_rect, 90 * 16, 180 * 16)

        # Status readout below
        status_text = "CLOCKWISE" if self.direction == 1 else "COUNTER-CW" if self.direction == -1 else "IDLE"
        painter.setPen(QColor("#00C853" if self.direction != 0 else "#718096"))
        font.setPointSize(9)
        painter.setFont(font)
        painter.drawText(
            self.rect(),
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter,
            status_text
        )


class JointVisualizer(QWidget):
    """Custom circular dial visualizer supporting up to 270° joint angles."""
    def __init__(self, title="JOINT", max_angle=180.0):
        super().__init__()
        self.title = title
        self.max_angle = float(max_angle)
        self.angle = 0.0
        self.setMinimumSize(110, 110)

    def set_angle(self, angle):
        new_angle = max(0.0, min(self.max_angle, float(angle)))
        if self.angle != new_angle:
            self.angle = new_angle
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2 - 3
        radius = min(w, h) / 2 - 25

        # Title
        painter.setPen(QColor("#4A5568"))
        font = painter.font()
        font.setBold(True)
        font.setPointSize(9)
        painter.setFont(font)
        painter.drawText(
            self.rect().adjusted(0, 2, 0, 0),
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter,
            self.title
        )

        # Outer Ring
        painter.setPen(QPen(QColor("#4A5568"), 3))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Rotating Pointer Needle
        painter.save()
        painter.translate(cx, cy)
        # Offset angle visual representation relative to center
        painter.rotate(self.angle - (self.max_angle / 2.0))
        painter.setPen(QPen(QColor("#00C853"), 4))
        painter.drawLine(0, 0, 0, int(-radius + 4))
        painter.restore()

        # Readout
        painter.setPen(QColor("#009624"))
        font.setPointSize(10)
        painter.setFont(font)
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

        self.sub_telemetry = self.create_subscription(
            String, '/rover/telemetry', self.on_telemetry_received, 10
        )
        self.pub_commands = self.create_publisher(
            String, '/rover/arm_commands', 10
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
    """Main Ground Station GUI supporting 5-actuator control."""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("AXIOM Ground Station - ROS 2 Humble")
        self.resize(1200, 780)
        self.setStyleSheet("background-color: #F2F4F7;")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.sim_mode = True

        # Joint angle states
        self.joint_base_dir = 0
        self.joint_lower_elbow = 135.0  # Default center for 270° Smraza servo
        self.joint_upper = 90.0         # Default center for 180° Miuzei servo
        self.joint_wrist = 90.0         # Default center for 180° MG996R servo
        self.joint_claw = 90.0          # Default center for 180° MG996R servo
        self.pressed_keys = set()

        # Telemetry plot data buffers
        self.max_samples = 50
        self.time_data = list(range(self.max_samples))
        self.temp_data = [25.0] * self.max_samples
        self.press_data = [980.0] * self.max_samples

        self.csv_filename = f"telemetry_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_csv_log()
        self.init_ui()

        # Timers
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(30)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.process_control_inputs)
        self.control_timer.start(33)

        self.mock_timer = QTimer(self)
        self.mock_timer.timeout.connect(self.update_mock_telemetry)
        self.mock_timer.start(1000)

    def init_csv_log(self):
        try:
            with open(self.csv_filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "temp_c", "pressure_hpa", "alt_m", "yaw", "pitch", "roll"])
        except Exception as e:
            print(f"Failed to initialize CSV log file: {e}")

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(12, 12, 12, 12)

        top_layout = QHBoxLayout()
        top_layout.setSpacing(10)

        # Left Control Panel
        left_panel = QFrame()
        left_panel.setStyleSheet("QFrame { background-color: #FFFFFF; border: 2px solid #C0C4CC; border-radius: 8px; padding: 8px; }")
        left_layout = QVBoxLayout(left_panel)

        ctrl_heading = QLabel("ARM CONTROLS")
        ctrl_heading.setStyleSheet("color: #000000; font-size: 13px; font-weight: bold; padding-bottom: 2px;")
        left_layout.addWidget(ctrl_heading, alignment=Qt.AlignmentFlag.AlignHCenter)

        self.lbl_mode = QLabel("MODE: SIMULATED (Press 'T' for Live ROS 2)")
        self.lbl_mode.setStyleSheet("color: #D32F2F; font-weight: bold; font-size: 10px; background: #FFEBEE; padding: 4px; border-radius: 4px;")
        self.lbl_mode.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_layout.addWidget(self.lbl_mode)

        instructions = (
            "<b>BASE (GM9 DC):</b> A / D<br>"
            "<b>LOWER ELBOW (45KG 270°):</b> W / S<br>"
            "<b>UPPER JOINT (25KG 180°):</b> I / K<br>"
            "<b>WRIST TWIST (MG996R):</b> J / L<br>"
            "<b>CLAW OPEN/CLOSE (MG996R):</b> U / O<br><br>"
            "<i>Hold key for continuous movement.<br>"
            "Press <b>R</b> to Reset Position.<br>"
            "Press <b>T</b> to Toggle Sim / Live Mode.</i>"
        )
        lbl_ctrls = QLabel(instructions)
        lbl_ctrls.setStyleSheet("color: #1A1D20; font-size: 11px; border: 1px solid #DCDFE6; background: #F8F9FA; padding: 6px; border-radius: 4px;")
        left_layout.addWidget(lbl_ctrls)

        lbl_walle = QLabel()
        pixmap_walle = QPixmap("walle.png")
        if not pixmap_walle.isNull():
            lbl_walle.setPixmap(pixmap_walle.scaledToWidth(180, Qt.TransformationMode.SmoothTransformation))
            lbl_walle.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_walle.setStyleSheet("border: none; background: transparent; padding-top: 4px;")
            left_layout.addWidget(lbl_walle, alignment=Qt.AlignmentFlag.AlignCenter)

        btn_reset = QPushButton("RESET POSITION")
        btn_reset.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        btn_reset.setStyleSheet("QPushButton { background-color: #D32F2F; color: #FFFFFF; font-weight: bold; font-size: 11px; padding: 8px; border-radius: 4px; border: none; } QPushButton:hover { background-color: #B71C1C; }")
        btn_reset.clicked.connect(self.reset_arm_position)
        left_layout.addWidget(btn_reset)

        left_layout.addStretch()
        top_layout.addWidget(left_panel, stretch=1)

        # Right Panel
        right_panel = QVBoxLayout()
        viz_header = QLabel("JOINT KINEMATIC DISPLAY")
        viz_header.setStyleSheet("color: #007A33; font-size: 14px; font-weight: bold;")
        right_panel.addWidget(viz_header, alignment=Qt.AlignmentFlag.AlignHCenter)

        dials_grid = QGridLayout()
        dials_grid.setSpacing(10)

        self.viz_base = BaseRotationIndicator("BASE ROTATION")
        self.viz_elbow = JointVisualizer("ELBOW (270°)", max_angle=270.0)
        self.viz_upper = JointVisualizer("UPPER JOINT (180°)", max_angle=180.0)
        self.viz_wrist = JointVisualizer("WRIST / CLAW (180°)", max_angle=180.0)

        dials_grid.addWidget(self.viz_base, 0, 0)
        dials_grid.addWidget(self.viz_elbow, 0, 1)
        dials_grid.addWidget(self.viz_upper, 1, 0)
        dials_grid.addWidget(self.viz_wrist, 1, 1)

        right_panel.addLayout(dials_grid)
        self.update_dial_visualizers()

        # Plots
        charts_layout = QHBoxLayout()
        self.plot_temp = pg.PlotWidget(title="<b>Temperature (°C)</b>")
        self.plot_temp.showGrid(x=True, y=True, alpha=0.3)
        self.curve_temp = self.plot_temp.plot(pen=pg.mkPen(color='#D32F2F', width=2))

        self.plot_press = pg.PlotWidget(title="<b>Pressure (hPa)</b>")
        self.plot_press.showGrid(x=True, y=True, alpha=0.3)
        self.curve_press = self.plot_press.plot(pen=pg.mkPen(color='#0288D1', width=2))

        charts_layout.addWidget(self.plot_temp)
        charts_layout.addWidget(self.plot_press)
        right_panel.addLayout(charts_layout)

        self.lbl_telemetry = QLabel("TIME: --:--:--")
        self.lbl_telemetry.setStyleSheet("color: #007A33; font-family: monospace; font-size: 13px; font-weight: bold; padding: 2px;")
        right_panel.addWidget(self.lbl_telemetry)

        top_layout.addLayout(right_panel, stretch=3)
        main_layout.addLayout(top_layout, stretch=3)

        # Bottom Sensor Display Cards
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
        card_yaw, self.lbl_val_yaw = create_metric_card("ACCEL X")
        card_pitch, self.lbl_val_pitch = create_metric_card("ACCEL Y")
        card_roll, self.lbl_val_roll = create_metric_card("ACCEL Z")

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
        elif event.key() == Qt.Key.Key_T:
            self.toggle_mode()

    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())

    def toggle_mode(self):
        self.sim_mode = not self.sim_mode
        if self.sim_mode:
            self.lbl_mode.setText("MODE: SIMULATED (Press 'T' for Live ROS 2)")
            self.lbl_mode.setStyleSheet("color: #D32F2F; font-weight: bold; font-size: 10px; background: #FFEBEE; padding: 4px; border-radius: 4px;")
        else:
            self.lbl_mode.setText("MODE: LIVE ROS 2 (Press 'T' for Simulation)")
            self.lbl_mode.setStyleSheet("color: #007A33; font-weight: bold; font-size: 10px; background: #E8F5E9; padding: 4px; border-radius: 4px;")

    def process_control_inputs(self):
        step = 1.5
        moved = False

        # Base GM9 Motor Controls
        if Qt.Key.Key_A in self.pressed_keys:
            self.joint_base_dir = -1
            moved = True
        elif Qt.Key.Key_D in self.pressed_keys:
            self.joint_base_dir = 1
            moved = True
        else:
            self.joint_base_dir = 0

        # Lower Elbow Servo (Smraza 45KG: 0 - 270°)
        if Qt.Key.Key_W in self.pressed_keys:
            self.joint_lower_elbow = min(270.0, self.joint_lower_elbow + step)
            moved = True
        if Qt.Key.Key_S in self.pressed_keys:
            self.joint_lower_elbow = max(0.0, self.joint_lower_elbow - step)
            moved = True

        # Upper Joint Servo (Miuzei 25KG: 0 - 180°)
        if Qt.Key.Key_I in self.pressed_keys:
            self.joint_upper = min(180.0, self.joint_upper + step)
            moved = True
        if Qt.Key.Key_K in self.pressed_keys:
            self.joint_upper = max(0.0, self.joint_upper - step)
            moved = True

        # Wrist Twist Servo (MG996R: 0 - 180°)
        if Qt.Key.Key_J in self.pressed_keys:
            self.joint_wrist = max(0.0, self.joint_wrist - step)
            moved = True
        if Qt.Key.Key_L in self.pressed_keys:
            self.joint_wrist = min(180.0, self.joint_wrist + step)
            moved = True

        # Claw Open/Close Servo (MG996R: 0 - 180°)
        if Qt.Key.Key_U in self.pressed_keys:
            self.joint_claw = min(180.0, self.joint_claw + step)
            moved = True
        if Qt.Key.Key_O in self.pressed_keys:
            self.joint_claw = max(0.0, self.joint_claw - step)
            moved = True

        self.update_dial_visualizers()

        if moved:
            cmd = {
                "lower_elbow": self.joint_lower_elbow,
                "upper_joint": self.joint_upper,
                "wrist": self.joint_wrist,
                "claw": self.joint_claw,
                "base_dir": self.joint_base_dir
            }
            self.ros_node.send_arm_command(cmd)

    def update_dial_visualizers(self):
        self.viz_base.set_direction(self.joint_base_dir)
        self.viz_elbow.set_angle(self.joint_lower_elbow)
        self.viz_upper.set_angle(self.joint_upper)
        self.viz_wrist.set_angle(self.joint_wrist)

    def reset_arm_position(self):
        self.joint_base_dir = 0
        self.joint_lower_elbow = 135.0
        self.joint_upper = 90.0
        self.joint_wrist = 90.0
        self.joint_claw = 90.0
        self.pressed_keys.clear()
        self.update_dial_visualizers()

        cmd = {
            "lower_elbow": 135.0,
            "upper_joint": 90.0,
            "wrist": 90.0,
            "claw": 90.0,
            "base_dir": 0
        }
        self.ros_node.send_arm_command(cmd)

    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def render_telemetry_data(self, data):
        current_ts = data.get("timestamp", time.strftime("%Y/%m/%d %H:%M:%S"))
        self.lbl_telemetry.setText(f"TIME: {current_ts}")

        temp_val, press_val, alt_val = "", "", ""
        acc_x, acc_y, acc_z = "", "", ""

        if "bmp388" in data:
            bmp = data["bmp388"]
            temp = bmp.get('temp_c')
            press = bmp.get('pressure_hpa')
            alt = bmp.get('alt_m')

            if temp is not None:
                temp_val = f"{float(temp):.2f}"
                self.lbl_val_temp.setText(f"{temp:.1f} °C")
                self.temp_data = self.temp_data[1:] + [float(temp)]
                self.curve_temp.setData(self.time_data, self.temp_data)

            if press is not None:
                press_val = f"{float(press):.2f}"
                self.lbl_val_press.setText(f"{press:.1f} hPa")
                self.press_data = self.press_data[1:] + [float(press)]
                self.curve_press.setData(self.time_data, self.press_data)

            if alt is not None:
                alt_val = f"{float(alt):.2f}"
                self.lbl_val_alt.setText(f"{alt:.2f} m")

        if "bno055" in data:
            bno = data["bno055"]
            if "acc_x" in bno and bno['acc_x'] is not None:
                acc_x = f"{float(bno['acc_x']):.2f}"
                self.lbl_val_yaw.setText(f"{bno['acc_x']:.2f}")
            if "acc_y" in bno and bno['acc_y'] is not None:
                acc_y = f"{float(bno['acc_y']):.2f}"
                self.lbl_val_pitch.setText(f"{bno['acc_y']:.2f}")
            if "acc_z" in bno and bno['acc_z'] is not None:
                acc_z = f"{float(bno['acc_z']):.2f}"
                self.lbl_val_roll.setText(f"{bno['acc_z']:.2f}")

        try:
            with open(self.csv_filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([current_ts, temp_val, press_val, alt_val, acc_x, acc_y, acc_z])
        except Exception:
            pass

    def update_telemetry_ui(self, data):
        if not self.sim_mode:
            self.render_telemetry_data(data)

    def update_mock_telemetry(self):
        if not self.sim_mode:
            return

        current_time = time.strftime("%Y/%m/%d %H:%M:%S")
        t = time.time()

        mock_data = {
            "timestamp": current_time,
            "bmp388": {
                "temp_c": 25.0 + 2.0 * math.sin(t / 8.0),
                "pressure_hpa": 980.0 + 5.0 * math.cos(t / 12.0),
                "alt_m": 94.2 + 0.5 * math.sin(t / 15.0)
            },
            "bno055": {
                "acc_x": 0.1 * math.sin(t),
                "acc_y": 0.05 * math.cos(t),
                "acc_z": 9.81
            }
        }
        self.render_telemetry_data(mock_data)


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