#!/usr/bin/env python3
"""
Orange Pi Onboard ROS 2 Node
- Subscribes to ArmControl on /rover/arm_command and forwards it to the Pico
- Reads telemetry lines from the Pico and publishes Telemetry on /rover/telemetry
- Reconnects automatically if the Pico is unplugged or resets

Serial protocol (newline-terminated ASCII lines):
    Orange Pi -> Pico:  A,<axis0>,<lower>,<upper>,<wrist>,<claw>\n
                        e.g.  A,90,45,90,135,45
    Pico -> Orange Pi:  T,<altitude>,<temperature>,<pressure>,<ax>,<ay>,<az>\n
                        e.g.  T,12.5,23.1,1013.2,0.01,-0.02,9.81
    Any other line from the Pico is logged as debug output.
"""

import time

import rclpy
import serial
from rclpy.node import Node
from rover_comms.msg import ArmControl, Telemetry


class OrangePiRoverNode(Node):
    def __init__(self):
        super().__init__('arm_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)        # ignored by USB CDC, harmless
        self.declare_parameter('send_hz', 20.0)       # max rate commands go to the Pico
        self.declare_parameter('reconnect_s', 2.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.reconnect_s = self.get_parameter('reconnect_s').value

        self.ser = None
        self.rx_buffer = b''
        self.last_connect_attempt = 0.0
        self.latest_cmd = None   # newest command line not yet sent
        self.last_sent = None

        self.publisher_ = self.create_publisher(Telemetry, 'rover/telemetry', 10)
        self.subscription = self.create_subscription(
            ArmControl, 'rover/arm_command', self.command_callback, 10)

        send_period = 1.0 / self.get_parameter('send_hz').value
        self.send_timer = self.create_timer(send_period, self.send_tick)
        self.read_timer = self.create_timer(0.01, self.read_tick)  # poll serial at 100 Hz

        self.connect()

    # ---------- serial connection ----------

    def connect(self):
        self.last_connect_attempt = time.monotonic()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1)
            self.ser.reset_input_buffer()
            self.rx_buffer = b''
            self.last_sent = None  # resend current command after reconnect
            self.get_logger().info(f'Connected to Pico on {self.port}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(
                f'Could not open {self.port}: {e}. Retrying in {self.reconnect_s:.0f}s',
                throttle_duration_sec=10.0)

    def disconnect(self, reason):
        self.get_logger().error(f'Serial connection lost: {reason}')
        try:
            if self.ser:
                self.ser.close()
        except serial.SerialException:
            pass
        self.ser = None

    def ensure_connected(self):
        if self.ser is None and time.monotonic() - self.last_connect_attempt > self.reconnect_s:
            self.connect()
        return self.ser is not None

    # ---------- ROS -> Pico ----------

    def command_callback(self, msg):
        # Only keep the newest command; send_tick forwards it at a fixed rate
        # so a fast publisher can't flood the serial link.
        self.latest_cmd = (
            f'A,{int(msg.axis0)},{int(msg.lower)},{int(msg.upper)},'
            f'{int(msg.wrist)},{int(msg.claw)}\n'
        )

    def send_tick(self):
        if self.latest_cmd is None or self.latest_cmd == self.last_sent:
            return
        if not self.ensure_connected():
            return
        try:
            self.ser.write(self.latest_cmd.encode('ascii'))
            self.last_sent = self.latest_cmd
            self.get_logger().debug(f'-> Pico: {self.latest_cmd.strip()}')
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as e:
            self.disconnect(e)

    # ---------- Pico -> ROS ----------

    def read_tick(self):
        if not self.ensure_connected():
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buffer += self.ser.read(waiting)
        except (serial.SerialException, OSError) as e:
            self.disconnect(e)
            return

        while b'\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
            self.handle_line(line.decode('ascii', errors='replace').strip())

        if len(self.rx_buffer) > 4096:  # garbage with no newlines; drop it
            self.rx_buffer = b''

    def handle_line(self, line):
        if not line:
            return
        if not line.startswith('T,'):
            self.get_logger().info(f'Pico: {line}')
            return
        parts = line.split(',')
        if len(parts) != 7:
            self.get_logger().warn(f'Malformed telemetry: {line}')
            return
        try:
            alt, temp, press, ax, ay, az = (float(p) for p in parts[1:])
        except ValueError:
            self.get_logger().warn(f'Malformed telemetry: {line}')
            return

        msg = Telemetry()
        msg.altitude = alt
        msg.temperature = temp
        msg.pressure = press
        msg.acceleration.x = ax
        msg.acceleration.y = ay
        msg.acceleration.z = az
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()