#!/usr/bin/env python3
"""
Orange Pi Onboard ROS 2 Node
- Interfaces with Raspberry Pi Pico over USB Serial (/dev/ttyACM0)
- Parses sensor telemetry from Pico and publishes JSON to `/rover/telemetry`
- Subscribes to `/rover/arm_commands` and sends formatted motor string to Pico
"""

import sys
import json
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class OrangePiRoverNode(Node):
    def __init__(self):
        super().__init__('orange_pi_rover_node')

        self.telemetry_pub = self.create_publisher(String, '/rover/telemetry', 10)
        self.arm_sub = self.create_subscription(
            String, '/rover/arm_commands', self.arm_command_callback, 10
        )

        self.ser = None
        self.serial_connected = False
        self.port_candidates = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        
        self.init_serial()
        self.timer = self.create_timer(0.05, self.read_serial_and_publish_telemetry)

    def init_serial(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().error("pyserial library missing! Run: pip3 install pyserial")
            return

        for port in self.port_candidates:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.serial_connected = True
                self.get_logger().info(f"Connected to Pi Pico on {port} at 115200 baud.")
                break
            except (serial.SerialException, FileNotFoundError):
                continue

        if not self.serial_connected:
            self.get_logger().warn("Pico offline. Running fallback mode.")

    def read_serial_and_publish_telemetry(self):
        timestamp = datetime.now().strftime("%Y/%m/%d %H:%M:%S")

        if self.serial_connected and self.ser.in_waiting > 0:
            try:
                raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if raw_line.startswith("log:"):
                    self.get_logger().info(f"[PICO LOG] {raw_line[4:]}")
                    return

                if raw_line.startswith("sensor_data:"):
                    data_str = raw_line.replace("sensor_data:", "").strip()
                    parts = [p.strip() for p in data_str.split(",")]

                    if len(parts) >= 6:
                        acc_x, acc_y, acc_z = float(parts[0]), float(parts[1]), float(parts[2])
                        alt_m, pressure_hpa, temp_c = float(parts[3]), float(parts[4]), float(parts[5])

                        telemetry_payload = {
                            "timestamp": timestamp,
                            "bmp388": {
                                "temp_c": round(temp_c, 2),
                                "pressure_hpa": round(pressure_hpa, 2),
                                "alt_m": round(alt_m, 2)
                            },
                            "bno055": {
                                "acc_x": round(acc_x, 2),
                                "acc_y": round(acc_y, 2),
                                "acc_z": round(acc_z, 2)
                            }
                        }

                        msg = String()
                        msg.data = json.dumps(telemetry_payload)
                        self.telemetry_pub.publish(msg)
                        return

            except Exception as e:
                self.get_logger().error(f"Serial parsing error: {e}")

        elif not self.serial_connected:
            telemetry_payload = {
                "timestamp": timestamp,
                "bmp388": {"temp_c": 26.67, "pressure_hpa": 1002.03, "alt_m": 93.81},
                "bno055": {"acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81}
            }
            msg = String()
            msg.data = json.dumps(telemetry_payload)
            self.telemetry_pub.publish(msg)

    def arm_command_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            
            if isinstance(cmd_data, dict):
                v0 = cmd_data.get("lower_elbow", 135.0)  # Smraza 45KG Servo
                v1 = cmd_data.get("upper_joint", 90.0)   # Miuzei 25KG Servo
                v2 = cmd_data.get("wrist", 90.0)         # MG996R Servo
                v3 = cmd_data.get("claw", 90.0)          # MG996R Servo
                v4 = cmd_data.get("base_dir", 0.0)       # GM9 DC Motor
            elif isinstance(cmd_data, list) and len(cmd_data) >= 5:
                v0, v1, v2, v3, v4 = cmd_data[:5]
            else:
                return

            motor_cmd_str = f"motor:{v0:.2f},{v1:.2f},{v2:.2f},{v3:.2f},{v4:.2f}\n"

            if self.serial_connected and self.ser and self.ser.is_open:
                self.ser.write(motor_cmd_str.encode('utf-8'))
                self.get_logger().info(f"Pico TX: {motor_cmd_str.strip()}")

        except Exception as e:
            self.get_logger().error(f"Arm command handler error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OrangePiRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
