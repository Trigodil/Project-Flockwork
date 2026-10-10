"""Interactive target-setting tool.

Type a drone name (or 'all') and an x,y target, the drone flies there.
manual_target outranks target_override, so it also works over the mean-field layer.
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class ManualTarget(Node):
    def __init__(self, drone_names):
        super().__init__('manual_target')
        self.pubs = {
            n: self.create_publisher(Point, f'/{n}/manual_target', 10) for n in drone_names
        }

    def send(self, name, x, y):
        msg = Point(x=x, y=y, z=0.0)
        if name == 'all':
            for pub in self.pubs.values():
                pub.publish(msg)
        elif name in self.pubs:
            self.pubs[name].publish(msg)
        else:
            print(f"Unknown drone '{name}', known: {list(self.pubs)} or 'all'")


def main():
    num_drones = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    drone_names = [f'x3_{i}' for i in range(num_drones)]

    rclpy.init()
    node = ManualTarget(drone_names)

    print(f"Drones: {', '.join(drone_names)} (or 'all')")
    print("Enter: <drone> <x> <y>   e.g. 'x3_0 3 -2' or 'all 5 5'. Ctrl+C to quit.")
    try:
        while rclpy.ok():
            line = input('> ').strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3:
                print('Format: <drone> <x> <y>')
                continue
            name, xs, ys = parts
            try:
                x, y = float(xs), float(ys)
            except ValueError:
                print('x and y must be numbers')
                continue
            node.send(name, x, y)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
