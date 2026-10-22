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



class OrangePiRoverNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.publisher_ = self.create_publisher(Telemetry, 'telem', 10)
        self.subscription = self.create_subscription(
            ArmControl, 'arm_control', self.listener_callback, 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def listener_callback(self, msg):
        self.get_logger().info(
            '\n'
            f'Control: axes=({msg.axis0:.2f}, {msg.lower:.2f}, '
            f'{msg.upper:.2f}, {msg.wrist:.2f}) claw={msg.claw:.2f}'
        )

    def tick(self):
        msg = Telemetry()
        msg.altitude = 0.0
        msg.pressure = 0.5
        msg.temperature = -0.5
        msg.acceleration.x = 1.0
        msg.acceleration.y = 1.0
        msg.acceleration.z = 1.0
        self.publisher_.publish(msg)

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
