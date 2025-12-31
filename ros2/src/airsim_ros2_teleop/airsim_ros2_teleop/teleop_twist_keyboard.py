#!/usr/bin/env python3
import sys
import threading
from select import select

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

MSG = """
ROS2 Teleop (Arrows) -> Publishing to cmd_vel
---------------------------------------------
Arrow keys:
  ↑  : forward (+x)
  ↓  : backward (-x)
  ←  : strafe left (+y)
  →  : strafe right (-y)

Shift + Arrow keys (swift / fast):
  Shift+↑, Shift+↓, Shift+←, Shift+→

Swift arrows (extra keys):
  PageUp   : up (+z)
  PageDown : down (-z)
  Home     : rotate left (+yaw)
  End      : rotate right (-yaw)

Speed:
  q/z : increase/decrease max speeds by 10%
  w/x : increase/decrease only linear speed by 10%
  e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

def vels(speed, turn):
    return f"currently:\tspeed {speed:.3f}\tturn {turn:.3f}"

# --- Key parsing (supports arrows + shift-arrows on typical terminals) ---
ESC = "\x1b"

def get_key(settings, timeout: float) -> str:
    """Returns a logical key token: UP/DOWN/LEFT/RIGHT, SHIFT_UP..., PGUP, etc, or '' on timeout."""
    if sys.platform == 'win32':
        ch = msvcrt.getwch()
        # Windows arrow keys come as a prefix + code
        if ch in ("\x00", "\xe0"):
            code = msvcrt.getwch()
            return {
                "H": "UP",
                "P": "DOWN",
                "K": "LEFT",
                "M": "RIGHT",
                "I": "PGUP",
                "Q": "PGDN",
                "G": "HOME",
                "O": "END",
            }.get(code, "")
        return ch

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if not rlist:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return ""

    c1 = sys.stdin.read(1)

    # Not an escape sequence → normal key
    if c1 != ESC:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return c1

    # Escape sequence: read more (non-blocking small reads)
    seq = c1
    rlist, _, _ = select([sys.stdin], [], [], 0.001)
    while rlist:
        seq += sys.stdin.read(1)
        rlist, _, _ = select([sys.stdin], [], [], 0.001)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # Common arrow sequences:
    #  ESC [ A/B/C/D
    # Shift-arrows often: ESC [ 1 ; 2 A/B/C/D  (varies by terminal)
    if seq.startswith("\x1b["):
        tail = seq[2:]

        # Plain arrows
        if tail == "A": return "UP"
        if tail == "B": return "DOWN"
        if tail == "C": return "RIGHT"
        if tail == "D": return "LEFT"

        # Shift + arrows (common xterm style)
        if tail == "1;2A": return "SHIFT_UP"
        if tail == "1;2B": return "SHIFT_DOWN"
        if tail == "1;2C": return "SHIFT_RIGHT"
        if tail == "1;2D": return "SHIFT_LEFT"

        # Home/End (varies)
        if tail in ("H", "1~", "7~"): return "HOME"
        if tail in ("F", "4~", "8~"): return "END"

        # PgUp/PgDn
        if tail == "5~": return "PGUP"
        if tail == "6~": return "PGDN"

    return ""


def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restore_terminal_settings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class PublishThread(threading.Thread):
    def __init__(self, node: Node, rate_hz: float, stamped: bool, frame_id: str):
        super().__init__(daemon=True)
        self.node = node
        self.stamped = stamped
        self.frame_id = frame_id

        self.publisher = node.create_publisher(
            TwistStamped if stamped else Twist,
            'cmd_vel',
            10
        )

        self.x = self.y = self.z = self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0

        self.cond = threading.Condition()
        self.done = False
        self.timeout = (1.0 / rate_hz) if rate_hz and rate_hz > 0.0 else None

        self.start()

    def update(self, x, y, z, th, speed, turn):
        with self.cond:
            self.x, self.y, self.z, self.th = x, y, z, th
            self.speed, self.turn = speed, turn
            self.cond.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done and rclpy.ok():
            with self.cond:
                self.cond.wait(self.timeout)

                lin_x = self.x * self.speed
                lin_y = self.y * self.speed
                lin_z = self.z * self.speed
                ang_z = self.th * self.turn

            if self.stamped:
                msg = TwistStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.twist.linear.x = lin_x
                msg.twist.linear.y = lin_y
                msg.twist.linear.z = lin_z
                msg.twist.angular.z = ang_z
            else:
                msg = Twist()
                msg.linear.x = lin_x
                msg.linear.y = lin_y
                msg.linear.z = lin_z
                msg.angular.z = ang_z

            self.publisher.publish(msg)

        # stop on exit
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)
        else:
            self.publisher.publish(Twist())


class TeleopArrows(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        # Parameters similar to ROS1 script
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 0.2)
        self.declare_parameter('speed_limit', 1000.0)
        self.declare_parameter('turn_limit', 1000.0)
        self.declare_parameter('repeat_rate', 0.0)
        self.declare_parameter('key_timeout', 0.5)
        self.declare_parameter('stamped', False)
        self.declare_parameter('frame_id', '')

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.speed_limit = float(self.get_parameter('speed_limit').value)
        self.turn_limit = float(self.get_parameter('turn_limit').value)
        self.repeat_rate = float(self.get_parameter('repeat_rate').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.stamped = bool(self.get_parameter('stamped').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # Movement bindings (x, y, z, yaw)
        self.move_bindings = {
            "w": (1, 0, 0, 0),
            "s": (-1, 0, 0, 0),
            "d": (0, 1, 0, 0),     # left strafe
            "a": (0, -1, 0, 0),   # right strafe

            # "swift arrows" (fast multiplier applied below)
            "W": (0, 0, 1, 0),
            "S": (0, 0, -1, 0),
            "A": (0, 0, 0, -1),
            "D": (0, 0, 0, 1),
        }

        self.speed_bindings = {
            # 'q': (1.1, 1.1),
            'z': (0.9, 0.9),
            'w': (1.1, 1.0),
            'x': (0.9, 1.0),
            'e': (1.0, 1.1),
            'c': (1.0, 0.9),
        }

        self.swift_multiplier = 2.0  # "swift" = 2x speed/turn

        self.pub_thread = PublishThread(
            node=self,
            rate_hz=self.repeat_rate,
            stamped=self.stamped,
            frame_id=self.frame_id
        )

    def loop(self):
        settings = save_terminal_settings()

        x = y = z = th = 0.0
        status = 0

        try:
            print(MSG)
            print(vels(self.speed, self.turn))

            self.pub_thread.update(x, y, z, th, self.speed, self.turn)

            while rclpy.ok():
                key = get_key(settings, self.key_timeout)
                swift = False
                if key in ("W", "S", "A", "D"):
                    swift = True

                if key in self.move_bindings:
                    x, y, z, th = self.move_bindings[key]
                elif key in self.speed_bindings:
                    self.speed = min(self.speed_limit, self.speed * self.speed_bindings[key][0])
                    self.turn = min(self.turn_limit, self.turn * self.speed_bindings[key][1])
                    if self.speed == self.speed_limit:
                        print("Linear speed limit reached!")
                    if self.turn == self.turn_limit:
                        print("Angular speed limit reached!")
                    print(vels(self.speed, self.turn))
                    if status == 14:
                        print(MSG)
                    status = (status + 1) % 15
                    # keep current motion
                else:
                    # timeout -> stop only if we were moving
                    if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                        continue
                    x = y = z = th = 0.0
                    if key == '\x03':  # Ctrl-C
                        break

                sp = self.speed * (self.swift_multiplier if swift else 1.0)
                tr = self.turn * (self.swift_multiplier if swift else 1.0)

                self.pub_thread.update(x, y, z, th, sp, tr)

        finally:
            self.pub_thread.stop()
            restore_terminal_settings(settings)


def main():
    rclpy.init()
    node = TeleopArrows()
    try:
        node.loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
