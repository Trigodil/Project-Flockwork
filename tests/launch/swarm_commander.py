"""Swarm-wide commands mid-flight: move the target or switch formation.

Interactive by default, or timed steps with --script, in seconds since start:
--script='25 formation v; 40 goto 0 6; 60 formation circle'
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

SHAPES = ('grid', 'line', 'column', 'v', 'circle')


class SwarmCommander(Node):
    def __init__(self):
        super().__init__('swarm_commander')
        self.target_pub = self.create_publisher(Point, '/swarm/target', 10)
        self.formation_pub = self.create_publisher(String, '/swarm/formation', 10)

    def run(self, parts):
        if len(parts) == 3 and parts[0] == 'goto':
            try:
                x, y = float(parts[1]), float(parts[2])
            except ValueError:
                return 'x and y must be numbers'
            self.target_pub.publish(Point(x=x, y=y, z=0.0))
            return f'goto {x} {y}'
        if len(parts) == 2 and parts[0] == 'formation':
            if parts[1] not in SHAPES:
                return f"unknown shape, pick one of {', '.join(SHAPES)}"
            self.formation_pub.publish(String(data=parts[1]))
            return f'formation {parts[1]}'
        return "format: 'goto <x> <y>' or 'formation <shape>'"


def parse_script(script):
    steps = []
    for step in script.split(';'):
        parts = step.split()
        if parts:
            steps.append((float(parts[0]), parts[1:]))
    return sorted(steps, key=lambda s: s[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--script', default='')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SwarmCommander()
    try:
        if args.script:
            start = time.monotonic()
            for at, parts in parse_script(args.script):
                while rclpy.ok() and time.monotonic() - start < at:
                    rclpy.spin_once(node, timeout_sec=0.1)
                node.get_logger().info(node.run(parts))
        else:
            print(f"Commands: 'goto <x> <y>', 'formation <{'|'.join(SHAPES)}>'. Ctrl+C to quit.")
            while rclpy.ok():
                line = input('> ').strip()
                if line:
                    print(node.run(line.split()))
                    rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
