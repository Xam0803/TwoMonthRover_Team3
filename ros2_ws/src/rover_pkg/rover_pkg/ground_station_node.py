#!/usr/bin/env python3
"""
Ground Station ROS 2 Node
- Keyboard arm control through a pygame window (window must have focus)
- Publishes ArmControl to /rover/arm_command
- Subscribes to Telemetry on /rover/telemetry and shows it in the window

Controls (hold keys to move, rate-based):
    A / D          Base rotation (axis0)
    W / S          Lower arm (shoulder)
    UP / DOWN      Upper arm (elbow)
    LEFT / RIGHT   Wrist
    Q / E          Claw close / open
    SHIFT (hold)   Fine mode (slow movement)
    R              Return to home position
    SPACE          Pause / resume sending commands
    ESC            Quit
"""

import time

import pygame
import rclpy
from rclpy.node import Node
from rover_comms.msg import ArmControl, Telemetry

PUBLISH_HZ = 20.0
SPEED = 60.0          # degrees per second while a key is held
FINE_FACTOR = 0.25    # speed multiplier while SHIFT is held
TELEMETRY_TIMEOUT = 1.0  # seconds before telemetry is shown as stale

# joint: (decrease key, increase key, min, max, home)
JOINTS = {
    'axis0': (pygame.K_a,    pygame.K_d,     0.0, 180.0,  90.0),
    'lower': (pygame.K_s,    pygame.K_w,     0.0, 180.0,  45.0),
    'upper': (pygame.K_DOWN, pygame.K_UP,    0.0, 180.0,  90.0),
    'wrist': (pygame.K_LEFT, pygame.K_RIGHT, 0.0, 180.0, 135.0),
    'claw':  (pygame.K_q,    pygame.K_e,     0.0,  90.0,  45.0),
}

LABELS = {
    'axis0': 'Base   [A/D]',
    'lower': 'Lower  [W/S]',
    'upper': 'Upper  [Up/Down]',
    'wrist': 'Wrist  [Left/Right]',
    'claw':  'Claw   [Q/E]',
}


class GSNode(Node):
    def __init__(self):
        super().__init__('gs_node')
        self.publisher_ = self.create_publisher(ArmControl, 'rover/arm_command', 10)
        self.subscription = self.create_subscription(
            Telemetry, 'rover/telemetry', self.listener_callback, 10)

        pygame.init()
        self.screen = pygame.display.set_mode((560, 420))
        pygame.display.set_caption('Rover Ground Station')
        self.font = pygame.font.Font(None, 26)
        self.small = pygame.font.Font(None, 20)

        self.angles = {j: cfg[4] for j, cfg in JOINTS.items()}
        self.paused = False
        self.running = True
        self.telemetry = None
        self.last_telemetry_time = 0.0
        self.last_tick = time.monotonic()

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)
        self.get_logger().info('Ground station ready. Focus the pygame window to drive.')

    def listener_callback(self, msg):
        self.telemetry = msg
        self.last_telemetry_time = time.monotonic()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_r:
                    self.angles = {j: cfg[4] for j, cfg in JOINTS.items()}
                    self.get_logger().info('Returning to home position')
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                    self.get_logger().info('Paused' if self.paused else 'Resumed')

    def update_angles(self, dt):
        keys = pygame.key.get_pressed()
        fine = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        step = SPEED * (FINE_FACTOR if fine else 1.0) * dt
        for joint, (dec, inc, lo, hi, _) in JOINTS.items():
            delta = (step if keys[inc] else 0.0) - (step if keys[dec] else 0.0)
            self.angles[joint] = max(lo, min(hi, self.angles[joint] + delta))

    def tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        self.handle_events()
        if not self.paused:
            self.update_angles(dt)
            msg = ArmControl()
            for joint, value in self.angles.items():
                setattr(msg, joint, int(round(value)))
            self.publisher_.publish(msg)
        self.draw()

    def draw(self):
        bg, fg, dim = (25, 28, 34), (230, 230, 230), (140, 140, 150)
        good, bad, accent = (90, 200, 120), (220, 90, 90), (90, 160, 230)
        self.screen.fill(bg)
        y = 15

        status = 'PAUSED (Space to resume)' if self.paused else 'SENDING'
        self.text(status, 15, y, bad if self.paused else good)
        y += 35

        for joint, (_, _, lo, hi, _) in JOINTS.items():
            value = self.angles[joint]
            self.text(LABELS[joint], 15, y, fg)
            self.text(f'{value:6.1f}°', 210, y, fg)
            bar_x, bar_w = 300, 240
            pygame.draw.rect(self.screen, (60, 64, 72), (bar_x, y + 3, bar_w, 12))
            fill = int(bar_w * (value - lo) / (hi - lo))
            pygame.draw.rect(self.screen, accent, (bar_x, y + 3, fill, 12))
            y += 30

        y += 10
        stale = time.monotonic() - self.last_telemetry_time > TELEMETRY_TIMEOUT
        if self.telemetry is None:
            self.text('Telemetry: waiting...', 15, y, bad)
        else:
            t = self.telemetry
            self.text('Telemetry' + ('  (STALE)' if stale else ''), 15, y, bad if stale else good)
            y += 28
            for line in (
                f'Altitude:     {t.altitude:.2f}',
                f'Temperature:  {t.temperature:.2f}',
                f'Pressure:     {t.pressure:.2f}',
                f'Accel:        <{t.acceleration.x:.2f}, {t.acceleration.y:.2f}, '
                f'{t.acceleration.z:.2f}>',
            ):
                self.text(line, 15, y, fg)
                y += 24

        self.text('Shift: fine   R: home   Space: pause   Esc: quit',
                  15, 395, dim, self.small)
        pygame.display.flip()

    def text(self, s, x, y, color, font=None):
        self.screen.blit((font or self.font).render(s, True, color), (x, y))


def main(args=None):
    rclpy.init(args=args)
    node = GSNode()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()