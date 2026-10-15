#!/usr/bin/env python3
import csv
import math
import os
import signal

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from stanley_controller.geometry import normalize_angle, yaw_from_quaternion
from stanley_controller.mocap_utils import y_up_to_planar, yaw_from_y_up_quaternion
from stanley_controller.path_utils import classify_segments, load_path
from stanley_controller.speed_controller import SpeedController
from stanley_controller.stanley import StanleyController

# Parameters that can be changed live (`ros2 param set`) without restarting the
# node, mapped to the (controller, attribute) they update.
_STANLEY_PARAMS = {
    'stanley_gain': 'k',
    'softening_gain': 'k_soft',
    'max_steering_angle': 'max_steering_angle',
    'goal_tolerance': 'goal_tolerance',
}
_SPEED_PARAMS = {
    'kp': 'kp',
    'ki': 'ki',
    'kd': 'kd',
    'integral_limit': 'integral_limit',
    'derivative_filter_alpha': 'derivative_filter_alpha',
    'feedforward_gain': 'feedforward_gain',
}
_NODE_PARAMS = ('wheelbase', 'target_speed', 'target_speed_curve',
                'curve_curvature_threshold')

_LOG_COLUMNS = ['t', 'x_front', 'y_front', 'v_ref', 'v', 'u', 'delta',
                'cross_track_error', 'heading_error']


class StanleyNode(Node):
    """
    Path following with Stanley (lateral) + PID (longitudinal) control.

    Reads a recorded front-axle path, and on every control step:
      steering  delta = Stanley(pose, v)                 (rad)
      throttle  u     = PID(v_ref - v)                   (m/s velocity command)
    and publishes them according to command_mode:
      'twist'    (Prius) Twist; the AckermannSteering plugin sets its steering
                 angle as phi = atan(L * w / v_x), so sending
                 w = u * tan(delta) / L reproduces exactly delta.
      'qcar_sim' / 'user_command' Vector3Stamped (/qcar_sim/user_command in
                 simulation, /qcar/user_command on the car): x = u, y = delta
                 (rad, steering_sign * (delta - steering_trim)). On the physical
                 QCar u is a throttle duty cycle, not m/s.
      'motor_commands' (physical QCar 2) qcar2_interfaces/MotorCommands:
                 steering_angle = steering_sign * (delta - steering_trim),
                 motor_throttle = u.

    The pose comes from nav_msgs/Odometry (simulation, dead_reckoning node) or
    from a geometry_msgs/PoseStamped (live OptiTrack); the PID speed feedback
    from the odometry twist or from the QCar tachometer (/qcar/velocity).
    """

    def __init__(self):
        super().__init__('stanley_controller')
        self.declare_parameter('path_file', os.path.expanduser('~/stanley_waypoints.csv'))
        # > 0 if the path file stores a reference point (with yaw) instead of the
        # front axle: waypoints are shifted forward by this distance (m).
        self.declare_parameter('path_front_axle_offset', 0.0)
        self.declare_parameter('odom_topic', '/odom')
        # 'odom' (nav_msgs/Odometry on odom_topic) or 'pose_stamped' (pose_topic).
        self.declare_parameter('pose_source', 'odom')
        self.declare_parameter('pose_topic', '/vrpn_mocap/QCar_Red/pose')
        # True if pose_topic is in Motive's Y-up frame (x = X, y = -Z, yaw about +Y).
        self.declare_parameter('pose_y_up', False)
        # Added to the measured yaw (rad): rigid body axis vs. true car heading.
        self.declare_parameter('yaw_offset', 0.0)
        # 'odom' (odometry twist) or 'qcar_velocity' (Vector3Stamped tachometer).
        self.declare_parameter('speed_source', 'odom')
        self.declare_parameter('speed_topic', '/qcar/velocity')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_mode', 'twist')
        # -1.0 if the vehicle's steering convention is inverted (+ = right).
        self.declare_parameter('steering_sign', 1.0)
        # Mechanical steering offset (rad): the angle the car actually steers with a
        # zero command, + = left. It is subtracted from every steering command.
        self.declare_parameter('steering_trim', 0.0)
        self.declare_parameter('log_file', '')
        self.declare_parameter('odom_is_front_axle', False)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('search_window', 50)
        self.declare_parameter('odom_timeout', 0.5)

        self.declare_parameter('wheelbase', 2.7)
        # Distance from the odometry reference point to the front axle.
        # < 0 means the odometry reports the rear axle, i.e. offset = wheelbase.
        self.declare_parameter('front_axle_offset', -1.0)
        self.declare_parameter('stanley_gain', 1.0)
        self.declare_parameter('softening_gain', 1.0)
        self.declare_parameter('max_steering_angle', 0.3)
        self.declare_parameter('goal_tolerance', 0.3)

        self.declare_parameter('target_speed', 0.5)
        self.declare_parameter('target_speed_curve', 0.5)
        self.declare_parameter('curve_curvature_threshold', 0.1)

        self.declare_parameter('kp', 0.3)
        self.declare_parameter('ki', 3.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('max_command', 0.5)
        self.declare_parameter('min_command', -0.5)
        self.declare_parameter('integral_limit', 5.0)
        self.declare_parameter('derivative_filter_alpha', 0.7)
        self.declare_parameter('feedforward_gain', 0.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        path_file = p('path_file')
        self.path = load_path(path_file, p('path_front_axle_offset'))

        self.command_mode = p('command_mode')
        if self.command_mode == 'user_command':
            self.command_mode = 'qcar_sim'  # same Vector3Stamped interface
        if self.command_mode not in ('twist', 'qcar_sim', 'motor_commands'):
            raise ValueError("command_mode must be 'twist', 'qcar_sim', 'user_command' or "
                             f"'motor_commands', got {self.command_mode!r}")
        self.pose_source = p('pose_source')
        if self.pose_source not in ('odom', 'pose_stamped'):
            raise ValueError("pose_source must be 'odom' or 'pose_stamped', "
                             f'got {self.pose_source!r}')
        self.speed_source = p('speed_source')
        if self.speed_source not in ('odom', 'qcar_velocity'):
            raise ValueError("speed_source must be 'odom' or 'qcar_velocity', "
                             f'got {self.speed_source!r}')
        self.pose_y_up = p('pose_y_up')
        self.yaw_offset = p('yaw_offset')
        self.steering_sign = p('steering_sign')
        self.steering_trim = p('steering_trim')
        self.wheelbase = p('wheelbase')
        front_axle_offset = p('front_axle_offset')
        if front_axle_offset < 0.0:
            front_axle_offset = self.wheelbase

        self.stanley = StanleyController(
            self.path, front_axle_offset, p('stanley_gain'), p('softening_gain'),
            p('max_steering_angle'), p('goal_tolerance'), p('search_window'),
            p('odom_is_front_axle'))
        self.speed_controller = SpeedController(
            p('kp'), p('ki'), p('kd'), p('min_command'), p('max_command'),
            p('integral_limit'), p('derivative_filter_alpha'), p('feedforward_gain'))
        for name in _NODE_PARAMS:
            setattr(self, name, p(name))
        self._segment_is_curve = classify_segments(self.path, self.curve_curvature_threshold)
        self.odom_timeout = p('odom_timeout')
        self.log_file = p('log_file')

        self._pose = None
        self._speed = 0.0
        self._last_odom_time = None
        self._last_speed_time = None
        self._last_step_time = None
        self._start_time = None
        self._goal_reached = False
        self._log_rows = []

        self.add_on_set_parameters_callback(self._on_parameter_update)

        if self.command_mode == 'twist':
            msg_type = Twist
        elif self.command_mode == 'qcar_sim':
            msg_type = Vector3Stamped
        else:
            from qcar2_interfaces.msg import MotorCommands
            msg_type = MotorCommands
        self._cmd_msg_type = msg_type
        self._cmd_topic = p('cmd_vel_topic')
        self._cmd_pub = self.create_publisher(msg_type, self._cmd_topic, 10)
        if self.pose_source == 'odom':
            self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 10)
        else:
            self.create_subscription(PoseStamped, p('pose_topic'), self._pose_cb, 10)
        if self.speed_source == 'qcar_velocity':
            self.create_subscription(Vector3Stamped, p('speed_topic'), self._speed_cb, 10)
        self.create_timer(1.0 / p('control_rate'), self._control_loop)

        self.get_logger().info(
            f'Loaded {len(self.path)} waypoints from {path_file}. '
            f'Stanley k={self.stanley.k}, k_s={self.stanley.k_soft}, '
            f'|delta|<={self.stanley.max_steering_angle} rad. '
            f'PID Kp={self.speed_controller.kp}, Ki={self.speed_controller.ki}, '
            f'Kd={self.speed_controller.kd}, v_ref={self.target_speed}/'
            f'{self.target_speed_curve} m/s (straight/curve)'
        )

    def _on_parameter_update(self, params):
        for param in params:
            if param.name in _STANLEY_PARAMS:
                setattr(self.stanley, _STANLEY_PARAMS[param.name], param.value)
            elif param.name in _SPEED_PARAMS:
                setattr(self.speed_controller, _SPEED_PARAMS[param.name], param.value)
            elif param.name == 'front_axle_offset':
                self.stanley.wheelbase = param.value if param.value >= 0.0 else self.wheelbase
            elif param.name == 'max_command':
                self.speed_controller.u_max = param.value
            elif param.name == 'min_command':
                self.speed_controller.u_min = param.value
            elif param.name in ('yaw_offset', 'steering_sign', 'steering_trim'):
                setattr(self, param.name, param.value)
            elif param.name in _NODE_PARAMS:
                setattr(self, param.name, param.value)
                if param.name == 'curve_curvature_threshold':
                    self._segment_is_curve = classify_segments(self.path, param.value)
                if (param.name == 'wheelbase'
                        and self.get_parameter('front_axle_offset').value < 0.0):
                    self.stanley.wheelbase = param.value
        return SetParametersResult(successful=True)

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._pose = (x, y, normalize_angle(yaw + self.yaw_offset))
        if self.speed_source == 'odom':
            # Wheel-odometry forward speed: the tachometer feedback for the PID.
            self._speed = msg.twist.twist.linear.x
        self._last_odom_time = self.get_clock().now()

    def _pose_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        if self.pose_y_up:
            x, y = y_up_to_planar(p.x, p.z)
            yaw = yaw_from_y_up_quaternion(q.x, q.y, q.z, q.w)
        else:
            x, y = p.x, p.y
            yaw = yaw_from_quaternion(q)
        self._pose = (x, y, normalize_angle(yaw + self.yaw_offset))
        self._last_odom_time = self.get_clock().now()

    def _speed_cb(self, msg):
        self._last_speed_time = self.get_clock().now()
        # QCar tachometer: planar velocity vector, sign from the forward component.
        speed = math.hypot(msg.vector.x, msg.vector.y)
        self._speed = -speed if msg.vector.x < 0.0 else speed

    def _publish_command(self, u, delta):
        if self.command_mode == 'twist':
            cmd = Twist()
            cmd.linear.x = u
            if abs(u) > 1e-3:
                cmd.angular.z = u * math.tan(delta) / self.wheelbase
        elif self.command_mode == 'qcar_sim':
            cmd = Vector3Stamped()
            # vehicle_controller times commands out against this stamp (sim time).
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = u
            cmd.vector.y = self.steering_sign * (delta - self.steering_trim)
        else:
            cmd = self._cmd_msg_type()
            cmd.motor_names = ['steering_angle', 'motor_throttle']
            cmd.values = [float(self.steering_sign * (delta - self.steering_trim)), float(u)]
        self._cmd_pub.publish(cmd)

    def _stop(self):
        self._publish_command(0.0, 0.0)
        self.speed_controller.reset()
        self._last_step_time = None

    def _control_loop(self):
        # Another node commanding the vehicle (e.g. a teleop bridge left running)
        # overrides our commands and the car will not follow the path.
        if self.count_publishers(self._cmd_topic) > 1:
            self.get_logger().warn(
                f'Other nodes are also publishing on {self._cmd_topic}; they override '
                'the Stanley commands. Stop them (teleop / qcar_teleop_bridge).',
                throttle_duration_sec=5.0)
        if self._pose is None:
            self.get_logger().info(
                'Waiting for the vehicle pose...', throttle_duration_sec=5.0)
            return
        if self._goal_reached:
            # Keep commanding a stop: a real vehicle may hold its last command.
            self._publish_command(0.0, 0.0)
            return

        now = self.get_clock().now()
        if (now - self._last_odom_time).nanoseconds * 1e-9 > self.odom_timeout:
            self.get_logger().warn('Vehicle pose is stale, holding the vehicle.',
                                   throttle_duration_sec=2.0)
            self._stop()
            return
        if self.speed_source == 'qcar_velocity' and (
                self._last_speed_time is None
                or (now - self._last_speed_time).nanoseconds * 1e-9 > self.odom_timeout):
            self.get_logger().warn('Speed feedback is stale, holding the vehicle.',
                                   throttle_duration_sec=2.0)
            self._stop()
            return

        dt = 0.0 if self._last_step_time is None \
            else (now - self._last_step_time).nanoseconds * 1e-9
        self._last_step_time = now
        if self._start_time is None:
            self._start_time = now

        x, y, yaw = self._pose
        v = self._speed
        lateral = self.stanley.compute(x, y, yaw, v)

        if lateral.at_path_end:
            self._goal_reached = True
            self._stop()
            self.get_logger().info('Goal reached, stopping.')
            self._save_log()
            return

        v_ref = (self.target_speed_curve if self._segment_is_curve[lateral.segment_index]
                 else self.target_speed)
        u = self.speed_controller.update(v_ref, v, dt)

        self._publish_command(u, lateral.delta)

        self._log_rows.append((
            (now - self._start_time).nanoseconds * 1e-9, lateral.front_x, lateral.front_y,
            v_ref, v, u, lateral.delta, lateral.cross_track_error, lateral.heading_error))

    def _save_log(self):
        if not self.log_file or not self._log_rows:
            return
        out_dir = os.path.dirname(self.log_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(_LOG_COLUMNS)
            writer.writerows(self._log_rows)
        self.get_logger().info(f'Saved {len(self._log_rows)} control samples to {self.log_file}')
        self._log_rows = []


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Handle Ctrl+C / SIGTERM ourselves so the context is still alive to send
    # the final stop command (essential on the physical car).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        node = StanleyNode()
    except RuntimeError as e:
        rclpy.logging.get_logger('stanley_controller').error(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node._publish_command(0.0, 0.0)
            node.get_logger().info('Sent stop command.')
        node._save_log()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
