#!/usr/bin/env python3
"""
Orange Pi Onboard ROS 2 Node
- Reads BMP388 & BNO055 sensors over I2C and publishes telemetry to `/rover/telemetry`
- Subscribes to `/rover/arm_commands` to handle joint target angles
"""

import sys
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import board
    import busio
    import adafruit_bmp388
    import adafruit_bno055
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class OrangePiRoverNode(Node):
    def __init__(self):
        super().__init__('orange_pi_rover_node')

        self.telemetry_pub = self.create_publisher(String, '/rover/telemetry', 10)
        self.arm_sub = self.create_subscription(
            String, '/rover/arm_commands', self.arm_command_callback, 10
        )

        if HARDWARE_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.bmp388 = adafruit_bmp388.BMP388(i2c)
                self.bno055 = adafruit_bno055.BNO055_I2C(i2c)
                self.get_logger().info("Hardware Sensors (BMP388 & BNO055) Initialized.")
            except Exception as e:
                self.get_logger().error(f"I2C sensor connection failed: {e}")
                HARDWARE_AVAILABLE = False

        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def publish_telemetry(self):
        timestamp = datetime.now().strftime("%Y/%m/%d %H:%M:%S")

        if HARDWARE_AVAILABLE:
            temp_c = round(self.bmp388.temperature, 2)
            pressure_hpa = round(self.bmp388.pressure, 2)
            altitude_m = round(self.bmp388.altitude, 2)

            yaw, pitch, roll = self.bno055.euler
            yaw = round(yaw, 2) if yaw is not None else 0.0
            pitch = round(pitch, 2) if pitch is not None else 0.0
            roll = round(roll, 2) if roll is not None else 0.0
        else:
            temp_c, pressure_hpa, altitude_m = 26.67, 1002.03, 93.81
            yaw, pitch, roll = 6.9, -22.38, 27.01

        telemetry_payload = {
            "timestamp": timestamp,
            "bmp388": {
                "temp_c": temp_c,
                "pressure_hpa": pressure_hpa,
                "alt_m": altitude_m
            },
            "bno055": {
                "yaw": yaw,
                "pitch": pitch,
                "roll": roll
            }
        }

        msg = String()
        msg.data = json.dumps(telemetry_payload)
        self.telemetry_pub.publish(msg)

    def arm_command_callback(self, msg):
        try:
            target_angles = json.loads(msg.data)
            self.get_logger().info(f"Target angles received: {target_angles}")
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON payload on /rover/arm_commands")


def main(args=None):
    rclpy.init(args=args)
    node = OrangePiRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
