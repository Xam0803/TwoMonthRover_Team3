import sys
import os
import json
import time
import math

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

        # Centered square bounding rect ensures a PERFECT CIRCLE (never an oval)
        circle_rect = QRectF(cx - radius, cy - radius, 2 * radius, 2 * radius)

        # Outer ring (Gray)
        painter.setPen(QPen(QColor("#718096"), 4))
        painter.drawEllipse(circle_rect)

        # Active direction semi-circle arc (Vivid Green)
        if self.direction != 0:
            active_pen = QPen(QColor("#00C853"), 6)
            active_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(active_pen)

            if self.direction == 1:
                # Right side: CW arc
                painter.drawArc(circle_rect, 90 * 16, -180 * 16)
            elif self.direction == -1:
                # Left side: CCW arc
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
    """Custom circular dial visualizer with rotating needle pointer for joint angles."""
    def __init__(self, title="JOINT"):
        super().__init__()
        self.title = title
        self.angle = 0.0
        self.setMinimumSize(110, 110)

    def set_angle(self, angle):
        new_angle = max(-180.0, min(180.0, float(angle)))
        if self.angle != new_angle:
            self.angle = new_angle
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

        # Outer Dial Ring
        painter.setPen(QPen(QColor("#4A5568"), 3))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Rotating Pointer Needle
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(self.angle)
        painter.setPen(QPen(QColor("#00C853"), 4))
        painter.drawLine(0, 0, 0, int(-radius + 4))
        painter.restore()

        # Angle Numerical Readout below
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
    """Main Ground Station GUI application with high-contrast daylight theme."""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("AXIOM Ground Station - ROS 2 Humble")
        self.resize(1200, 780)
        self.setStyleSheet("background-color: #F2F4F7;")

        # Mode State: Starts in Simulated Mode (Press 'T' to toggle)
        self.sim_mode = True

        # Active joint angle states (Degrees) - Default Home Position
        self.joint_base = 0.0
        self.joint_lower_elbow = 0.0
        self.joint_upper = 0.0
        self.joint_wrist = 0.0
        self.base_dir = 0
        self.pressed_keys = set()

        # Telemetry plot data buffers (last 50 samples)
        self.max_samples = 50
        self.time_data = list(range(self.max_samples))
        self.temp_data = [25.0] * self.max_samples
        self.press_data = [980.0] * self.max_samples

        self.init_ui()

        # ROS 2 executor timer (~33 Hz)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(30)

        # Control polling timer (30 Hz)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.process_control_inputs)
        self.control_timer.start(33)

        # Continuous mock telemetry generator (1 Hz)
        self.mock_timer = QTimer(self)
        self.mock_timer.timeout.connect(self.update_mock_telemetry)
        self.mock_timer.start(1000)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(12, 12, 12, 12)

        # --- TOP SECTION: CONTROLS, KINEMATIC DIALS & CHARTS ---
        top_layout = QHBoxLayout()
        top_layout.setSpacing(10)

        # --- LEFT PANEL (Controls & Wall-E Logo) ---
        left_panel = QFrame()
        left_panel.setStyleSheet("""
            QFrame {
                background-color: #FFFFFF;
                border: 2px solid #C0C4CC;
                border-radius: 8px;
                padding: 8px;
            }
        """)
        left_layout = QVBoxLayout(left_panel)

        ctrl_heading = QLabel("ARM CONTROLS")
        ctrl_heading.setStyleSheet("color: #000000; font-size: 13px; font-weight: bold; padding-bottom: 2px;")
        left_layout.addWidget(ctrl_heading, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Live / Simulated Mode Status Banner
        self.lbl_mode = QLabel("MODE: SIMULATED (Press 'T' for Live ROS 2)")
        self.lbl_mode.setStyleSheet("color: #D32F2F; font-weight: bold; font-size: 10px; background: #FFEBEE; padding: 4px; border-radius: 4px;")
        self.lbl_mode.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_layout.addWidget(self.lbl_mode)

        instructions = (
            "<b>BASE ROTATION:</b> A / D<br>"
            "<b>LOWER ELBOW:</b> W / S<br>"
            "<b>UPPER JOINT:</b> I / K<br>"
            "<b>WRIST TWIST:</b> J / L<br><br>"
            "<i>Hold key for continuous movement.<br>"
            "Press <b>R</b> to Reset Position.<br>"
            "Press <b>T</b> to Toggle Sim / Live Mode.</i>"
        )
        lbl_ctrls = QLabel(instructions)
        lbl_ctrls.setStyleSheet("color: #1A1D20; font-size: 11px; border: 1px solid #DCDFE6; background: #F8F9FA; padding: 6px; border-radius: 4px;")
        left_layout.addWidget(lbl_ctrls)

        # Wall-E image display
        lbl_walle = QLabel()
        pixmap_walle = QPixmap("walle.png")
        if not pixmap_walle.isNull():
            lbl_walle.setPixmap(pixmap_walle.scaledToWidth(180, Qt.TransformationMode.SmoothTransformation))
            lbl_walle.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_walle.setStyleSheet("border: none; background: transparent; padding-top: 4px;")
            left_layout.addWidget(lbl_walle, alignment=Qt.AlignmentFlag.AlignCenter)

        # Reset button (NoFocus prevents stealing key events)
        btn_reset = QPushButton("RESET POSITION")
        btn_reset.setFocusPolicy(Qt.FocusPolicy.NoFocus)
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

        # --- RIGHT PANEL (Original 2x2 Kinematic Layout & Graphs) ---
        right_panel = QVBoxLayout()

        viz_header = QLabel("JOINT KINEMATIC DISPLAY")
        viz_header.setStyleSheet("color: #007A33; font-size: 14px; font-weight: bold;")
        right_panel.addWidget(viz_header, alignment=Qt.AlignmentFlag.AlignHCenter)

        # 2x2 Kinematic Dial Grid matching original layout
        dials_grid = QGridLayout()
        dials_grid.setSpacing(10)

        self.viz_base = BaseRotationIndicator("BASE ROTATION")
        self.viz_elbow = JointVisualizer("LOWER ELBOW JOINT")
        self.viz_upper = JointVisualizer("UPPER JOINT")
        self.viz_wrist = JointVisualizer("WRIST / CLAW TWIST")

        dials_grid.addWidget(self.viz_base, 0, 0)
        dials_grid.addWidget(self.viz_elbow, 0, 1)
        dials_grid.addWidget(self.viz_upper, 1, 0)
        dials_grid.addWidget(self.viz_wrist, 1, 1)

        right_panel.addLayout(dials_grid)

        # Initial dial values
        self.update_dial_visualizers()

        # Charts Section
        charts_layout = QHBoxLayout()

        # Temperature Plot
        self.plot_temp = pg.PlotWidget(title="<b>Temperature (°C)</b>")
        self.plot_temp.showGrid(x=True, y=True, alpha=0.3)
        self.curve_temp = self.plot_temp.plot(pen=pg.mkPen(color='#D32F2F', width=2))

        # Pressure Plot
        self.plot_press = pg.PlotWidget(title="<b>Pressure (hPa)</b>")
        self.plot_press.showGrid(x=True, y=True, alpha=0.3)
        self.curve_press = self.plot_press.plot(pen=pg.mkPen(color='#0288D1', width=2))

        charts_layout.addWidget(self.plot_temp)
        charts_layout.addWidget(self.plot_press)

        right_panel.addLayout(charts_layout)
        
        # System Time Readout under graphs
        self.lbl_telemetry = QLabel("TIME: --:--:--")
        self.lbl_telemetry.setStyleSheet("color: #007A33; font-family: monospace; font-size: 13px; font-weight: bold; padding: 2px;")
        right_panel.addWidget(self.lbl_telemetry)

        top_layout.addLayout(right_panel, stretch=3)
        main_layout.addLayout(top_layout, stretch=3)

        # --- BOTTOM SECTION: 6 LIVE SENSOR READOUT CARDS ---
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
        elif event.key() == Qt.Key.Key_T:
            self.toggle_mode()

    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())

    def toggle_mode(self):
        """Toggles between simulated data mode and live ROS 2 telemetry mode."""
        self.sim_mode = not self.sim_mode
        if self.sim_mode:
            self.lbl_mode.setText("MODE: SIMULATED (Press 'T' for Live ROS 2)")
            self.lbl_mode.setStyleSheet("color: #D32F2F; font-weight: bold; font-size: 10px; background: #FFEBEE; padding: 4px; border-radius: 4px;")
        else:
            self.lbl_mode.setText("MODE: LIVE ROS 2 (Press 'T' for Simulation)")
            self.lbl_mode.setStyleSheet("color: #007A33; font-weight: bold; font-size: 10px; background: #E8F5E9; padding: 4px; border-radius: 4px;")

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

        self.update_dial_visualizers()

        if moved:
            cmd = {
                "lower_elbow": self.joint_lower_elbow,
                "upper_joint": self.joint_upper,
                "wrist": self.joint_wrist,
                "base_dir": self.base_dir
            }
            self.ros_node.send_arm_command(cmd)

    def update_dial_visualizers(self):
        """Updates needle position and directional arcs for custom dial widgets."""
        self.viz_base.set_direction(self.base_dir)
        self.viz_elbow.set_angle(self.joint_lower_elbow)
        self.viz_upper.set_angle(self.joint_upper)
        self.viz_wrist.set_angle(self.joint_wrist)

    def reset_arm_position(self):
        """Resets all arm joints to zero (home position) and clears key press buffer."""
        self.joint_base = 0.0
        self.joint_lower_elbow = 0.0
        self.joint_upper = 0.0
        self.joint_wrist = 0.0
        self.base_dir = 0
        self.pressed_keys.clear()
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

    def render_telemetry_data(self, data):
        """Renders telemetry payload dictionary onto UI plots and metrics."""
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

    def update_telemetry_ui(self, data):
        """Callback for live ROS 2 telemetry packets."""
        if not self.sim_mode:
            self.render_telemetry_data(data)

    def update_mock_telemetry(self):
        """Generates dynamic live-scrolling simulated telemetry data when in Simulation Mode."""
        if not self.sim_mode:
            return

        current_time = time.strftime("%Y/%m/%d %H:%M:%S")
        t = time.time()

        # Dynamic rolling sine-wave simulation for temperature & pressure curves
        mock_temp = 25.0 + 2.0 * math.sin(t / 8.0) + (t % 0.4 - 0.2)
        mock_press = 980.0 + 5.0 * math.cos(t / 12.0) + (t % 0.8 - 0.4)
        mock_alt = 94.2 + 0.5 * math.sin(t / 15.0)

        mock_data = {
            "timestamp": current_time,
            "bmp388": {
                "temp_c": mock_temp,
                "pressure_hpa": mock_press,
                "alt_m": mock_alt
            },
            "bno055": {
                "yaw": (t * 4.0) % 360.0 - 180.0,
                "pitch": 10.0 * math.sin(t / 6.0),
                "roll": 5.0 * math.cos(t / 8.0)
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