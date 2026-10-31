#!/usr/bin/env python3
"""Keyboard control for the two retractable climbing arms.

Publishes std_msgs/Float64 (radians) on /arm_left/cmd_pos and /arm_right/cmd_pos.
Works with Gazebo (through ros_gz_bridge) and with the RViz-only simulator.
Run it in its own terminal:  ros2 run pathsense_description arm_teleop_keyboard.py
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

HELP = """
PathSense climbing-arm teleop
---------------------------------------------
  w / s : both arms up / down
  e / d : LEFT arm up / down
  r / f : RIGHT arm up / down

  1 : drive pose        (0.40 rad, as in the concept render)
  2 : arms flat forward (0.00 rad)
  3 : climb / press     (-0.35 rad, nose down, lifts the front)
  4 : retract           (3.00 rad, folded back over the tracks)
  5 : vertical          (1.57 rad)

  + / - : bigger / smaller step     (now {step:.2f} rad)
  q or Ctrl-C : quit
---------------------------------------------
"""

LOWER, UPPER = -0.60, 3.00
PRESETS = {'1': 0.40, '2': 0.0, '3': -0.35, '4': 3.00, '5': 1.57}


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop_keyboard')
        self.pub_l = self.create_publisher(Float64, '/arm_left/cmd_pos', 10)
        self.pub_r = self.create_publisher(Float64, '/arm_right/cmd_pos', 10)
        self.left = self.declare_parameter('start', 0.40).value
        self.right = self.left
        self.step = 0.10

    def publish(self):
        self.left = min(max(self.left, LOWER), UPPER)
        self.right = min(max(self.right, LOWER), UPPER)
        self.pub_l.publish(Float64(data=float(self.left)))
        self.pub_r.publish(Float64(data=float(self.right)))
        print(f'\rleft arm: {self.left:+.2f} rad   right arm: {self.right:+.2f} rad   step: {self.step:.2f}   ', end='')


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ArmTeleop()
    print(HELP.format(step=node.step))
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key in ('q', '\x03'):
                break
            if key == 'w':
                node.left += node.step; node.right += node.step
            elif key == 's':
                node.left -= node.step; node.right -= node.step
            elif key == 'e':
                node.left += node.step
            elif key == 'd':
                node.left -= node.step
            elif key == 'r':
                node.right += node.step
            elif key == 'f':
                node.right -= node.step
            elif key in PRESETS:
                node.left = node.right = PRESETS[key]
            elif key == '+':
                node.step = min(node.step + 0.05, 0.5)
            elif key == '-':
                node.step = max(node.step - 0.05, 0.05)
            elif key == '':
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            node.publish()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()
        print()


if __name__ == '__main__':
    main()
