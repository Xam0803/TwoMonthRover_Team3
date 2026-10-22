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
import serial
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rover_comms.msg import ArmControl, Telemetry



class GSNode(Node):
    def __init__(self):
        super().__init__('gs_node')
        self.publisher_ = self.create_publisher(ArmControl, 'telem', 10)
        self.subscription = self.create_subscription(
            ArmControl, 'arm_control', self.listener_callback, 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def listener_callback(self, msg):
        self.get_logger().info(
            '\n'
            f'Telemetry:\n'
            f'Altitude: {msg.altitude}\n'
            f'Temperature: {msg.temperature}\n'
            f'Pressure: {msg.pressure}\n'
            f'Acceleration: <{msg.acceleration.x}, {msg.acceleration.y}, {msg.acceleration.z}>\n'
        )

    def tick(self):
        msg = ArmControl()
        msg.axis0 = 0
        msg.lower = 45
        msg.upper = 90
        msg.wrist = 135
        msg.claw = 45
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GSNode()
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
