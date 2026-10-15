#!/usr/bin/env python3
import csv
import math
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from stanley_controller.geometry import front_axle_position, yaw_from_quaternion


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_file', os.path.expanduser('~/stanley_waypoints.csv'))
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('max_jump_distance', 1.0)
        # Stanley regulates the FRONT axle, so the reference path is stored as the
        # front-axle trajectory. Odometry reports the rear axle, hence the projection.
        self.declare_parameter('record_front_axle', True)
        self.declare_parameter('wheelbase', 2.7)
        # Distance from the odometry reference point to the front axle.
        # < 0 means the odometry reports the rear axle, i.e. offset = wheelbase.
        self.declare_parameter('front_axle_offset', -1.0)

        odom_topic = self.get_parameter('odom_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_jump_distance = self.get_parameter('max_jump_distance').value
        self.record_front_axle = self.get_parameter('record_front_axle').value
        self.front_axle_offset = self.get_parameter('front_axle_offset').value
        if self.front_axle_offset < 0.0:
            self.front_axle_offset = self.get_parameter('wheelbase').value

        self._points = []
        self._last_point = None
        self._recording = True

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        reference = f'front axle (offset={self.front_axle_offset} m)' if self.record_front_axle \
            else 'odometry frame origin'
        self.get_logger().info(
            f'Recording {reference} from {odom_topic} (min_distance={self.min_distance} m). '
            f'Press Ctrl+C to stop and write {self.output_file}'
        )

    def _odom_cb(self, msg):
        if not self._recording:
            return

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.record_front_axle:
            x, y = front_axle_position(x, y, yaw, self.front_axle_offset)

        if self._last_point is not None:
            jump = math.hypot(x - self._last_point[0], y - self._last_point[1])
            if jump > self.max_jump_distance:
                self._recording = False
                self.get_logger().warn(
                    f'Odometry jumped {jump:.2f} m in one update (likely a simulation '
                    'reset). Stopping recording to avoid corrupting the path; '
                    f'{len(self._points)} waypoints recorded so far were kept.'
                )
                return
            if jump < self.min_distance:
                return

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        self._points.append((x, y, yaw, v, w))
        self._last_point = (x, y)

    def save(self):
        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])
            writer.writerows(self._points)
        self.get_logger().info(f'Saved {len(self._points)} waypoints to {self.output_file}')


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
