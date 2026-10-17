#!/usr/bin/env python3
import csv
import math
import os

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from stanley_controller.geometry import normalize_angle
from stanley_controller.imu_integration import ImuIntegrator


class DeadReckoning(Node):
    """
    Pose estimate of the physical QCar from its own sensors.

    The speed v comes from the tachometer (/qcar/velocity) and the heading from
    heading_source:
      'imu'      theta' = yaw_sign * (gyro_z - bias)   (same integration as
                 bag_to_waypoints.py, so the pose is in the frame of waypoints.csv;
                 the gyro bias is estimated while the car is still at start-up)
      'steering' theta' = v tan(delta) / L, delta from the steering command
    and x' = v cos(theta), y' = v sin(theta) at the rear axle.

    Publishes nav_msgs/Odometry of the reference point rear_axle_offset ahead
    of the rear axle, starting from the first pose of the waypoints file.
    """

    def __init__(self):
        super().__init__('dead_reckoning')
        self.declare_parameter('path_file', os.path.expanduser('~/qcar_real_waypoints.csv'))
        self.declare_parameter('heading_source', 'steering')
        self.declare_parameter('speed_topic', '/qcar/velocity')
        # 'x' (vector.x, as bag_to_waypoints.py) or 'norm' (|vector| signed by x).
        self.declare_parameter('speed_field', 'norm')
        self.declare_parameter('imu_topic', '/qcar/imu')
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('bias_window', 1.0)
        self.declare_parameter('command_topic', '/qcar2_motor_speed_cmd')
        # 'motor_commands' (qcar2_interfaces/MotorCommands) or 'vector3_stamped' (y = steering).
        self.declare_parameter('command_type', 'motor_commands')
        self.declare_parameter('odom_topic', '/qcar/dead_reckoning/odom')
        self.declare_parameter('steering_sign', 1.0)
        # Must match stanley_controller: physical steering = command + trim.
        self.declare_parameter('steering_trim', 0.0)
        self.declare_parameter('wheelbase', 0.256)
        # Distance from the rear axle forward to the published reference point.
        self.declare_parameter('rear_axle_offset', 0.127)
        self.declare_parameter('publish_rate', 50.0)
        # Stop publishing if the sensors go silent for this long (s): a frozen pose
        # must never look like a live one to the controller.
        self.declare_parameter('input_timeout', 0.5)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.heading_source = p('heading_source')
        if self.heading_source not in ('imu', 'steering'):
            raise ValueError("heading_source must be 'imu' or 'steering'")
        self.speed_field = p('speed_field')
        self.yaw_sign = p('yaw_sign')
        self.bias_window = p('bias_window')
        self.steering_sign = p('steering_sign')
        self.steering_trim = p('steering_trim')
        self.wheelbase = p('wheelbase')
        self.rear_axle_offset = p('rear_axle_offset')
        self.input_timeout = p('input_timeout')

        ref_x, ref_y, yaw = self._start_pose(p('path_file'))
        self._theta = yaw
        self._x = ref_x - self.rear_axle_offset * math.cos(yaw)
        self._y = ref_y - self.rear_axle_offset * math.sin(yaw)
        self._speed = 0.0
        self._delta = 0.0
        self._yaw_rate = 0.0
        self._last_time = None

        # IMU mode: same integration as imu_waypoint_recorder (shared class), on
        # the rear axle; the gyro bias is estimated while the car is still.
        self._imu = ImuIntegrator(self._x, self._y, self._theta, self.yaw_sign,
                                  self.bias_window)
        self._last_speed_rx = None
        self._last_imu_rx = None

        self.create_subscription(Vector3Stamped, p('speed_topic'), self._speed_cb, 10)
        if self.heading_source == 'imu':
            self.create_subscription(Imu, p('imu_topic'), self._imu_cb, 50)
        elif p('command_type') == 'vector3_stamped':
            self.create_subscription(Vector3Stamped, p('command_topic'), self._vector_cmd_cb, 10)
        else:
            from qcar2_interfaces.msg import MotorCommands
            self.create_subscription(MotorCommands, p('command_topic'), self._command_cb, 10)
        self._pub = self.create_publisher(Odometry, p('odom_topic'), 10)
        self.create_timer(1.0 / p('publish_rate'), self._step)
        self.get_logger().info(
            f'Dead reckoning ({self.heading_source}) from start pose x={ref_x:.3f} m, '
            f'y={ref_y:.3f} m, yaw={math.degrees(yaw):.1f} deg. Place the car at this pose'
            + (' and keep it still while the gyro bias is estimated.'
               if self.heading_source == 'imu' else '.'))

    @staticmethod
    def _start_pose(path_file):
        with open(path_file, newline='') as f:
            row = next(csv.DictReader(f))
        x = float(row.get('ref_x') or row['x'])
        y = float(row.get('ref_y') or row['y'])
        return x, y, float(row['yaw'])

    def _speed_cb(self, msg):
        self._last_speed_rx = self.get_clock().now()
        if self.speed_field == 'x':
            self._speed = msg.vector.x
        else:
            speed = math.hypot(msg.vector.x, msg.vector.y)
            self._speed = -speed if msg.vector.x < 0.0 else speed

    def _command_cb(self, msg):
        if 'steering_angle' in msg.motor_names:
            value = msg.values[list(msg.motor_names).index('steering_angle')]
            # Undo the steering convention to get the physical angle (+ = left).
            self._delta = self.steering_sign * value + self.steering_trim

    def _vector_cmd_cb(self, msg):
        self._delta = self.steering_sign * msg.vector.y + self.steering_trim

    def _imu_cb(self, msg):
        self._last_imu_rx = self.get_clock().now()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        event = self._imu.update(t, msg.angular_velocity.z, self._speed)
        if event == 'bias_ready':
            self.get_logger().info(
                f'Gyro bias {self._imu.bias:.5f} rad/s from {self._imu.bias_samples} '
                'samples. Publishing odometry.')
        elif event == 'bias_failed':
            self.get_logger().warn('The car was moving during the bias window; '
                                   'gyro bias not corrected.')
        self._x, self._y, self._theta = self._imu.x, self._imu.y, self._imu.theta
        self._yaw_rate = self._imu.yaw_rate

    def _inputs_stale(self, now):
        stale = []
        inputs = [('speed', self._last_speed_rx)]
        if self.heading_source == 'imu':
            inputs.append(('IMU', self._last_imu_rx))
        for name, last in inputs:
            if last is None or (now - last).nanoseconds * 1e-9 > self.input_timeout:
                stale.append(name)
        return stale

    def _step(self):
        now = self.get_clock().now()
        stale = self._inputs_stale(now)
        if stale:
            # Publish nothing: the controller then sees a stale pose and stops the car.
            self.get_logger().warn(
                f'No {" / ".join(stale)} data for more than {self.input_timeout} s; '
                'odometry paused.', throttle_duration_sec=2.0)
            self._last_time = None
            return
        if self.heading_source == 'imu':
            if not self._imu.ready:
                return  # no odometry until the gyro bias is known
        else:
            dt = 0.0 if self._last_time is None \
                else (now - self._last_time).nanoseconds * 1e-9
            self._last_time = now
            dt = min(max(dt, 0.0), 0.1)
            v = self._speed
            self._x += v * math.cos(self._theta) * dt
            self._y += v * math.sin(self._theta) * dt
            self._yaw_rate = v * math.tan(self._delta) / self.wheelbase
            self._theta = normalize_angle(self._theta + self._yaw_rate * dt)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'qcar_reference'
        odom.pose.pose.position.x = self._x + self.rear_axle_offset * math.cos(self._theta)
        odom.pose.pose.position.y = self._y + self.rear_axle_offset * math.sin(self._theta)
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        odom.twist.twist.linear.x = self._speed
        odom.twist.twist.angular.z = self._yaw_rate
        self._pub.publish(odom)


def main():
    rclpy.init()
    node = DeadReckoning()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
