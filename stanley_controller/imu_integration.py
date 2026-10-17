"""
IMU + tachometer dead reckoning shared by imu_waypoint_recorder and dead_reckoning.

Both nodes must integrate in exactly the same way, otherwise the recorded
waypoints and the live odometry the controller follows end up in different
frames. The math is the one of bag_to_waypoints.py:

    yaw <- yaw + yaw_sign * (gyro_z - bias) * dt
    x   <- x + v * cos(yaw) * dt
    y   <- y + v * sin(yaw) * dt

with trapezoidal averaging between consecutive IMU samples. The gyro bias is
the mean gyro_z while the car is still during the first bias_window seconds.
"""
import math

from stanley_controller.geometry import normalize_angle


class ImuIntegrator:

    def __init__(self, x=0.0, y=0.0, theta=0.0, yaw_sign=1.0, bias_window=3.0,
                 still_speed=0.02, max_dt=0.5, min_bias_samples=10):
        self.x = x
        self.y = y
        self.theta = theta
        self.yaw_sign = yaw_sign
        self.bias_window = bias_window
        self.still_speed = still_speed
        self.max_dt = max_dt
        self.min_bias_samples = min_bias_samples

        self.bias = None          # rad/s, None until estimated
        self.bias_samples = 0
        self.yaw_rate = 0.0
        self.gaps = 0             # samples skipped because dt was invalid (data gaps)
        self._bias_sum = 0.0
        self._bias_start = None
        self._last = None         # (t, wz, v)

    @property
    def ready(self):
        return self.bias is not None

    def update(self, t, wz, v):
        """
        Process one IMU sample (time s, gyro z rad/s) with the current speed v (m/s).

        Returns 'bias_ready' or 'bias_failed' on the sample that ends the bias
        window, otherwise None.
        """
        event = None
        if self.bias is None:
            if self._bias_start is None:
                self._bias_start = t
            if abs(v) < self.still_speed:
                self._bias_sum += wz
                self.bias_samples += 1
            if t - self._bias_start >= self.bias_window:
                if self.bias_samples >= self.min_bias_samples:
                    self.bias = self._bias_sum / self.bias_samples
                    event = 'bias_ready'
                else:
                    self.bias = 0.0
                    event = 'bias_failed'
            self._last = (t, wz, v)
            return event

        t0, wz0, v0 = self._last
        self._last = (t, wz, v)
        dt = t - t0
        if dt <= 0.0 or dt > self.max_dt:
            self.gaps += 1
            return None
        w = self.yaw_sign * (0.5 * (wz + wz0) - self.bias)
        vm = 0.5 * (v + v0)
        yaw_mid = self.theta + 0.5 * w * dt
        self.theta = normalize_angle(self.theta + w * dt)
        self.x += vm * math.cos(yaw_mid) * dt
        self.y += vm * math.sin(yaw_mid) * dt
        self.yaw_rate = w
        return None
