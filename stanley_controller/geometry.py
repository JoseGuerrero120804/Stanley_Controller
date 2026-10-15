import math


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def front_axle_position(x, y, yaw, wheelbase):
    """Project a rear-axle position forward to the centre of the front axle."""
    return x + wheelbase * math.cos(yaw), y + wheelbase * math.sin(yaw)
