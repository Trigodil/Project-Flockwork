#!/usr/bin/env python3
"""RViz click-to-target bridge.

Subscribes to RViz's "Publish Point" tool output (/clicked_point) and
forwards it as a manual_target to the selected drone, 'all' by default.
The controller ranks manual_target above the automatic target_override
layer, so this still works when swarm.launch.py is running
mean_field_controller_node. Also republishes each drone's odom position as markers so
they show up in RViz while clicking. Run alongside an already running
swarm.launch.py or stress_test.launch.py, not part of either. Not
part of the swarm_control package on purpose, same as the rest of
tests/.
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class RvizTargetBridge(Node):
    def __init__(self, drone_names):
        super().__init__('rviz_target_bridge')
        self.declare_parameter('selected_drone', 'all')
        self.target_pubs = {
            n: self.create_publisher(Point, f'/{n}/manual_target', 10) for n in drone_names
        }
        self.positions = {}
        for n in drone_names:
            self.create_subscription(
                Odometry, f'/{n}/odom', (lambda msg, n=n: self.odom_cb(n, msg)), 10)
        self.create_subscription(PointStamped, '/clicked_point', self.click_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/drone_markers', 10)
        self.create_timer(0.5, self.publish_markers)

    def odom_cb(self, name, msg):
        self.positions[name] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def click_cb(self, msg):
        selected = self.get_parameter('selected_drone').value
        target = Point(x=msg.point.x, y=msg.point.y, z=0.0)
        if selected == 'all':
            for pub in self.target_pubs.values():
                pub.publish(target)
        elif selected in self.target_pubs:
            self.target_pubs[selected].publish(target)
        else:
            self.get_logger().warn(f"Unknown selected_drone '{selected}'")
            return
        self.get_logger().info(f'Sent ({msg.point.x:.2f}, {msg.point.y:.2f}) to {selected}')

    def publish_markers(self):
        array = MarkerArray()
        for i, (name, (x, y)) in enumerate(self.positions.items()):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'drones'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.a = 1.0
            array.markers.append(m)
        self.marker_pub.publish(array)


def main():
    num_drones = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    drone_names = [f'x3_{i}' for i in range(num_drones)]

    rclpy.init()
    node = RvizTargetBridge(drone_names)
    print(f'Bridging RViz /clicked_point to target_override for: {", ".join(drone_names)}')
    print("Default target is 'all'. To pick one drone from another terminal:")
    print('  ros2 param set /rviz_target_bridge selected_drone x3_0')
    print('  ros2 param set /rviz_target_bridge selected_drone all')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
