#!/usr/bin/env python3
from geometry_msgs.msg import Twist, Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class QCarTeleopBridge(Node):
    """
    Drive the simulated QCar with teleop_twist_keyboard (Step 1, path recording).

    qcar_gazebo's vehicle_controller listens to Vector3Stamped on
    /qcar_sim/user_command (x = speed m/s, y = steering angle rad) and stops the
    car if the newest command stamp is older than 1 s. teleop_twist_keyboard only
    publishes on key presses, so the last Twist is re-sent at a fixed rate with a
    fresh stamp: linear.x -> speed, angular.z -> steering angle (clamped).
    """

    def __init__(self):
        super().__init__('qcar_teleop_bridge')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/qcar_sim/user_command')
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self._speed = 0.0
        self._steering = 0.0

        self._pub = self.create_publisher(
            Vector3Stamped, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self._twist_cb, 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self._publish)

    def _twist_cb(self, msg):
        self._speed = max(-self.max_speed, min(msg.linear.x, self.max_speed))
        self._steering = max(-self.max_steering_angle,
                             min(msg.angular.z, self.max_steering_angle))

    def _publish(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = self._speed
        cmd.vector.y = self._steering
        self._pub.publish(cmd)


def main():
    rclpy.init()
    node = QCarTeleopBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
