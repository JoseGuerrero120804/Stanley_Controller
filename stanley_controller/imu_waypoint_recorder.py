#!/usr/bin/env python3
"""
Record a new waypoints.csv on the physical QCar from its IMU and tachometer.

1. Start it with the car still at the start of the route: the gyro bias is
   estimated and the car's current pose becomes the local origin (0, 0, 0).
2. Drive the route (joystick / teleop).
3. Ctrl+C: writes the waypoints CSV (x, y, yaw, v; same format as
   bag_to_waypoints.py) and a figure of the trajectory.

The integration is the same ImuIntegrator the imu_odometry node uses, so when
the controller later starts from the same spot, its odometry is in this frame.
"""
import csv
from datetime import datetime
import math
import os
import signal

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu

from stanley_controller.imu_integration import ImuIntegrator


class ImuWaypointRecorder(Node):

    def __init__(self):
        super().__init__('imu_waypoint_recorder')
        self.declare_parameter('imu_topic', '/qcar/imu')
        self.declare_parameter('speed_topic', '/qcar/velocity')
        # 'x' (vector.x, as bag_to_waypoints.py) or 'norm' (|vector| signed by x).
        self.declare_parameter('speed_field', 'x')
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('bias_window', 3.0)
        self.declare_parameter('min_distance', 0.05)
        self.declare_parameter('output_file', os.path.expanduser('~/waypoints.csv'))
        self.declare_parameter('plot_file', os.path.expanduser('~/waypoints.png'))
        self.declare_parameter('odom_topic', '/qcar/waypoint_recorder/odom')
        self.declare_parameter('input_timeout', 0.5)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.speed_field = p('speed_field')
        self.min_distance = p('min_distance')
        self.output_file = os.path.expanduser(p('output_file'))
        self.plot_file = os.path.expanduser(p('plot_file'))
        self.input_timeout = p('input_timeout')

        self._imu = ImuIntegrator(yaw_sign=p('yaw_sign'), bias_window=p('bias_window'))
        self._speed = 0.0
        self._last_speed_rx = None
        self._last_imu_rx = None
        self._trajectory = []   # every integrated pose, for the figure
        self._waypoints = []    # (x, y, yaw, v) spaced >= min_distance

        self.create_subscription(Vector3Stamped, p('speed_topic'), self._speed_cb, 10)
        self.create_subscription(Imu, p('imu_topic'), self._imu_cb, 50)
        self._pub = self.create_publisher(Odometry, p('odom_topic'), 10)
        self.create_timer(0.05, self._status)
        self.get_logger().info(
            f'Keep the car STILL: estimating the gyro bias for {p("bias_window")} s, then '
            'its current pose becomes the origin (0, 0, 0).')

    def _speed_cb(self, msg):
        self._last_speed_rx = self.get_clock().now()
        if self.speed_field == 'x':
            self._speed = msg.vector.x
        else:
            speed = math.hypot(msg.vector.x, msg.vector.y)
            self._speed = -speed if msg.vector.x < 0.0 else speed

    def _imu_cb(self, msg):
        self._last_imu_rx = self.get_clock().now()
        if self._last_speed_rx is None:
            return  # no speed yet: neither the bias nor the motion can be judged
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self._last_imu_rx.nanoseconds * 1e-9
        gaps = self._imu.gaps
        event = self._imu.update(t, msg.angular_velocity.z, self._speed)
        if event == 'bias_ready':
            self.get_logger().info(
                f'Gyro bias {self._imu.bias:.5f} rad/s ({self._imu.bias_samples} samples). '
                'Origin set at (0, 0, 0). Drive the route now; Ctrl+C to save.')
            self._add_waypoint(force=True)
        elif event == 'bias_failed':
            self.get_logger().error(
                'The car moved during the bias window: the gyro bias could not be estimated. '
                'Stop (Ctrl+C) and restart with the car still.')
        if self._imu.gaps > gaps:
            self.get_logger().warn('IMU data gap (> 0.5 s): motion during the gap is lost; '
                                   'consider recording again.', throttle_duration_sec=2.0)
        if self._imu.ready:
            self._trajectory.append((self._imu.x, self._imu.y))
            self._add_waypoint()

    def _add_waypoint(self, force=False):
        x, y = self._imu.x, self._imu.y
        if not force and self._waypoints:
            lx, ly = self._waypoints[-1][:2]
            if math.hypot(x - lx, y - ly) < self.min_distance:
                return
        self._waypoints.append((x, y, self._imu.theta, self._speed))

    def _status(self):
        now = self.get_clock().now()
        for name, last in (('speed', self._last_speed_rx), ('IMU', self._last_imu_rx)):
            if last is None or (now - last).nanoseconds * 1e-9 > self.input_timeout:
                self.get_logger().warn(f'No {name} data.', throttle_duration_sec=3.0)
                return
        if not self._imu.ready:
            return
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'qcar_reference'
        odom.pose.pose.position.x = self._imu.x
        odom.pose.pose.position.y = self._imu.y
        odom.pose.pose.orientation.z = math.sin(self._imu.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._imu.theta / 2.0)
        odom.twist.twist.linear.x = self._speed
        odom.twist.twist.angular.z = self._imu.yaw_rate
        self._pub.publish(odom)

    def save(self):
        if len(self._waypoints) < 2:
            self.get_logger().warn('Fewer than 2 waypoints recorded; nothing saved.')
            return
        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        if os.path.exists(self.output_file):
            stem, ext = os.path.splitext(self.output_file)
            backup = f'{stem}_backup_{datetime.now():%Y%m%d_%H%M%S}{ext}'
            os.rename(self.output_file, backup)
            self.get_logger().info(f'Existing file moved to {backup}')
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'v'])
            for x, y, yaw, v in self._waypoints:
                writer.writerow([f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}', f'{v:.4f}'])

        length = sum(math.hypot(self._waypoints[i + 1][0] - self._waypoints[i][0],
                                self._waypoints[i + 1][1] - self._waypoints[i][1])
                     for i in range(len(self._waypoints) - 1))
        self.get_logger().info(
            f'Saved {len(self._waypoints)} waypoints ({length:.2f} m) to {self.output_file}')
        self._plot(length)

    def _plot(self, length):
        if not self.plot_file:
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        traj = list(zip(*self._trajectory))
        wx, wy, wyaw, _ = zip(*self._waypoints)
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.plot(traj[0], traj[1], color='0.6', linewidth=1, label='IMU trajectory')
        ax.plot(wx, wy, 'b.', markersize=4, label=f'Waypoints (every {self.min_distance} m)')
        step = max(1, len(wx) // 25)
        ax.quiver(wx[::step], wy[::step], [math.cos(a) for a in wyaw[::step]],
                  [math.sin(a) for a in wyaw[::step]], color='r', width=0.003,
                  label='Heading')
        ax.plot(0.0, 0.0, 'go', markersize=10, label='Origin (start)')
        ax.plot(wx[-1], wy[-1], 'rs', markersize=10, label='End')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title(f'Recorded trajectory: {len(wx)} waypoints, {length:.2f} m, total turn '
                     f'{math.degrees(sum_turn(wyaw)):.0f} deg\n'
                     f'gyro bias {self._imu.bias:.5f} rad/s')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        out_dir = os.path.dirname(self.plot_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        fig.savefig(self.plot_file, dpi=200)
        self.get_logger().info(f'Saved trajectory figure to {self.plot_file}')


def sum_turn(yaws):
    """Total signed heading change (rad) along a sequence of wrapped yaw angles."""
    return sum(math.atan2(math.sin(b - a), math.cos(b - a)) for a, b in zip(yaws, yaws[1:]))


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Handle Ctrl+C / SIGTERM ourselves so the files are always written.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = ImuWaypointRecorder()
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
