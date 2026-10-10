"""Waypoint follower.

Publishes a shared path's current waypoint to each drone's manual_target,
advancing per drone once it is within --radius of that waypoint.
"""
import argparse

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class PathFollower(Node):
    def __init__(self, drone_names, waypoints, radius, loop):
        super().__init__('path_follower')
        self.waypoints = waypoints
        self.radius = radius
        self.loop = loop
        self.index = {n: 0 for n in drone_names}
        self.pubs = {
            n: self.create_publisher(Point, f'/{n}/manual_target', 10) for n in drone_names
        }
        for n in drone_names:
            self.create_subscription(
                Odometry, f'/{n}/odom', (lambda msg, n=n: self.odom_cb(n, msg)), 10)
        self.create_timer(0.5, self.publish_targets)

    def odom_cb(self, name, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        tx, ty = self.waypoints[self.index[name]]
        if ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5 < self.radius:
            if self.index[name] < len(self.waypoints) - 1:
                self.index[name] += 1
            elif self.loop:
                self.index[name] = 0

    def publish_targets(self):
        for name, pub in self.pubs.items():
            tx, ty = self.waypoints[self.index[name]]
            pub.publish(Point(x=tx, y=ty, z=0.0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', required=True)
    parser.add_argument('--waypoints', required=True)
    parser.add_argument('--radius', type=float, default=1.0)
    parser.add_argument('--loop', action='store_true')
    args, _ = parser.parse_known_args()

    drone_names = args.drones.split(',')
    waypoints = []
    for pair in args.waypoints.split(';'):
        parts = pair.strip().split(',')
        if len(parts) != 2:
            parser.error(f"waypoint '{pair}' is not 'x,y'")
        waypoints.append((float(parts[0]), float(parts[1])))

    rclpy.init()
    node = PathFollower(drone_names, waypoints, args.radius, args.loop)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
