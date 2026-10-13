import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QPushButton
)
from PyQt6.QtCore import Qt, QPointF, QTimer
from PyQt6.QtGui import QPainter, QPen, QColor, QKeyEvent
import os
os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
class GroundStationROSNode(Node):
    def __init__(self, telemetry_callback):
        super().__init__('axiom_ground_station_node')
        self.telemetry_callback = telemetry_callback
        
        # 1. Telemetry Subscriber (Reads BMP388 & BNO055 from Orange Pi)
        self.telemetry_sub = self.create_subscription(
            String, '/rover/telemetry', self.telemetry_callback, 10
        )
        
        # 2. Arm / Joint Target Publisher (Matches "Publish to Arm Positions Topic" in Flowchart)
        self.arm_cmd_pub = self.create_publisher(
            String, '/rover/arm_commands', 10
        )

    def publish_arm_targets(self, angles_dict):
        msg = String()
        msg.data = json.dumps(angles_dict)
        self.arm_cmd_pub.publish(msg)

class JointVisualizer(QWidget):
    def __init__(self, title="Joint"):
        super().__init__()
        self.title = title
        self.angle = 0.0
        self.setMinimumSize(120, 120)

    def set_angle(self, angle):
        self.angle = max(-180.0, min(180.0, float(angle)))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

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


class GroundStationCanvas(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.angles = {
            "base": 0.0,
            "lower_elbow": 0.0,
            "upper_joint": 0.0,
            "wrist": 0.0
        }

        self.pressed_keys = set()
        
        self.key_timer = QTimer(self)
        self.key_timer.setInterval(16)
        self.key_timer.timeout.connect(self._process_held_keys)
        self.key_timer.start()

        self.reset_timer = QTimer(self)
        self.reset_timer.setInterval(20)
        self.reset_timer.timeout.connect(self._step_reset)

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("AXIOM Ground Station")
        self.resize(1000, 600)
        self.setStyleSheet("background-color: #121214;")
        
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        main_layout = QVBoxLayout()

        top_layout = QHBoxLayout()

        left_panel = QVBoxLayout()
        title_label = QLabel("AXIOM GROUND STATION")
        title_label.setStyleSheet("color: #FFFFFF; font-size: 18px; font-weight: bold;")
        
        controls_label = QLabel(
            "KEYBOARD CONTROLS:\n\n"
            "• A / D : Base Rotation\n"
            "• W / S : Lower Elbow Joint\n"
            "• I / K : Upper Joint\n"
            "• J / L : Wrist / Claw Rotation"
        )
        controls_label.setStyleSheet("color: #AAAAAA; font-size: 13px; line-height: 1.5;")

        left_panel.addWidget(title_label)
        left_panel.addSpacing(15)
        left_panel.addWidget(controls_label)
        left_panel.addStretch()

        btn_style_base = (
            "font-weight: bold; font-size: 12px; border-radius: 5px; padding: 10px;"
        )

        self.power_btn = QPushButton("POWER SYSTEM")
        self.power_btn.setStyleSheet(
            btn_style_base + "background-color: #222226; color: #55555d; border: 1px solid #333338;"
        )
        left_panel.addWidget(self.power_btn)

        left_panel.addSpacing(8)

        self.reset_btn = QPushButton("RESET POSITION")
        self.reset_btn.setStyleSheet(
            btn_style_base + "background-color: #1f1215; color: #ff4444; border: 1px solid #441a1c;"
        )
        self.reset_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.reset_btn.clicked.connect(self.start_smooth_reset)
        left_panel.addWidget(self.reset_btn)

        top_layout.addLayout(left_panel, stretch=1)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.VLine)
        line.setStyleSheet("color: #222225;")
        top_layout.addWidget(line)

        right_panel = QVBoxLayout()
        viz_header = QLabel("JOINT KINEMATIC DISPLAY")
        viz_header.setStyleSheet("color: #00FF88; font-size: 14px; font-weight: bold;")
        right_panel.addWidget(viz_header, alignment=Qt.AlignmentFlag.AlignHCenter)

        grid_layout = QGridLayout()

        self.viz_base = JointVisualizer("Base")
        self.viz_lower = JointVisualizer("Lower Elbow")
        self.viz_upper = JointVisualizer("Upper Joint")
        self.viz_wrist = JointVisualizer("Wrist / Claw")

        grid_layout.addWidget(self.create_labeled_widget("BASE ROTATION", self.viz_base), 0, 0)
        grid_layout.addWidget(self.create_labeled_widget("LOWER ELBOW JOINT", self.viz_lower), 0, 1)
        grid_layout.addWidget(self.create_labeled_widget("UPPER JOINT", self.viz_upper), 1, 0)
        grid_layout.addWidget(self.create_labeled_widget("WRIST / CLAW TWIST", self.viz_wrist), 1, 1)

        right_panel.addLayout(grid_layout)
        top_layout.addLayout(right_panel, stretch=2)

        main_layout.addLayout(top_layout, stretch=3)

        bottom_panel = QFrame()
        bottom_panel.setStyleSheet("border-top: 2px solid #222225; background-color: #0d0d0f; padding: 5px;")
        
        bottom_layout = QVBoxLayout(bottom_panel)
        reserved_label = QLabel("LIVE ROS 2 TELEMETRY & SENSOR FEED")
        reserved_label.setStyleSheet("color: #00FF88; font-size: 12px; font-weight: bold;")
        bottom_layout.addWidget(reserved_label, alignment=Qt.AlignmentFlag.AlignLeft)

        sensor_layout = QHBoxLayout()
        self.time_label = QLabel("TIME: --:--:--")
        self.time_label.setStyleSheet("color: #888888; font-size: 12px;")

        self.bmp_label = QLabel("BMP388: Temp -- °C | Press -- hPa | Alt -- m")
        self.bmp_label.setStyleSheet("color: #FFFFFF; font-size: 12px;")

        self.bno_label = QLabel("BNO055: Yaw --° | Pitch --° | Roll --°")
        self.bno_label.setStyleSheet("color: #FFFFFF; font-size: 12px;")

        sensor_layout.addWidget(self.time_label)
        sensor_layout.addWidget(self.bmp_label)
        sensor_layout.addWidget(self.bno_label)
        
        bottom_layout.addLayout(sensor_layout)

        main_layout.addWidget(bottom_panel, stretch=1)

        self.setLayout(main_layout)

    def create_labeled_widget(self, title_text, visualizer):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(5, 5, 5, 5)

        lbl = QLabel(title_text)
        lbl.setStyleSheet("color: #888888; font-size: 11px; font-weight: bold;")
        
        layout.addWidget(lbl, alignment=Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(visualizer)
        return container

    def start_smooth_reset(self):
        if not self.reset_timer.isActive():
            self.reset_timer.start()

    def _step_reset(self):
        reset_speed = 0.75
        all_at_zero = True

        for joint_key in self.angles:
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
        """Callback to update GUI telemetry labels when ROS 2 message arrives."""
        if "timestamp" in data:
            self.time_label.setText(f"TIME: {data['timestamp']}")
        
        if "bmp388" in data:
            bmp = data["bmp388"]
            self.bmp_label.setText(
                f"BMP388: Temp {bmp.get('temp_c', '--')} °C | "
                f"Press {bmp.get('pressure_hpa', '--')} hPa | "
                f"Alt {bmp.get('alt_m', '--')} m"
            )
            
        if "bno055" in data:
            bno = data["bno055"]
            self.bno_label.setText(
                f"BNO055: Yaw {bno.get('yaw', '--')}° | "
                f"Pitch {bno.get('pitch', '--')}° | "
                f"Roll {bno.get('roll', '--')}°"
            )

    def update_visualizers(self):
        self.viz_base.set_angle(self.angles["base"])
        self.viz_lower.set_angle(self.angles["lower_elbow"])
        self.viz_upper.set_angle(self.angles["upper_joint"])
        self.viz_wrist.set_angle(self.angles["wrist"])
        self.ros_node.publish_arm_targets(self.angles)
        
    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.pressed_keys.add(event.key())

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat() and event.key() in self.pressed_keys:
            self.pressed_keys.remove(event.key())

    def _process_held_keys(self):
        if not self.pressed_keys:
            return

        step = 1.0

        if Qt.Key.Key_A in self.pressed_keys:
            self.angles["base"] -= step
        if Qt.Key.Key_D in self.pressed_keys:
            self.angles["base"] += step

        if Qt.Key.Key_W in self.pressed_keys:
            self.angles["lower_elbow"] += step
        if Qt.Key.Key_S in self.pressed_keys:
            self.angles["lower_elbow"] -= step

        if Qt.Key.Key_I in self.pressed_keys:
            self.angles["upper_joint"] += step
        if Qt.Key.Key_K in self.pressed_keys:
            self.angles["upper_joint"] -= step

        if Qt.Key.Key_J in self.pressed_keys:
            self.angles["wrist"] -= step
        if Qt.Key.Key_L in self.pressed_keys:
            self.angles["wrist"] += step

        self.update_visualizers()


def main():
    # Initialize ROS 2
    rclpy.init()
    
    app = QApplication(sys.argv)
    
    window = None
    def handle_incoming_telemetry(msg):
        data = json.loads(msg.data)
        if window:
            window.update_telemetry_ui(data)

    ros_node = GroundStationROSNode(handle_incoming_telemetry)
    window = GroundStationCanvas(ros_node)
    window.show()

    # Spin ROS 2 background events inside PyQt6 loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(10)

    sys.exit(app.exec())

if __name__ == "__main__":
    main()