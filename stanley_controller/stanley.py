from dataclasses import dataclass
import math

from stanley_controller.geometry import front_axle_position, normalize_angle


@dataclass
class StanleyOutput:
    delta: float           # saturated steering angle (rad), + = left (REP-103)
    cross_track_error: float  # e_ct (m), + = front axle LEFT of the path
    heading_error: float   # theta_e = theta_path - theta_vehicle, in [-pi, pi]
    front_x: float
    front_y: float
    segment_index: int     # index i of the nearest segment w_i -> w_{i+1}
    at_path_end: bool      # front axle has reached / passed the last waypoint


class StanleyController:
    """
    Stanley geometric lateral controller.

    delta = theta_e + arctan(k * e_ct / (k_s + v)), evaluated at the front axle.

    The cross-track error follows the sign convention e_ct > 0 when the front
    axle is to the LEFT of the directed segment w1 -> w2 (2D cross product).
    With ROS steering convention (delta > 0 turns LEFT), a car that is left of
    the path must steer RIGHT to return, so the correction term is applied as
    -arctan(k * e_ct / (k_s + v)).
    """

    def __init__(self, path, wheelbase, k=1.0, k_soft=1.0, max_steering_angle=0.3,
                 goal_tolerance=0.3, search_window=50, pose_is_front_axle=False):
        self.path = path
        self.wheelbase = wheelbase
        self.k = k
        self.k_soft = k_soft
        self.max_steering_angle = max_steering_angle
        self.goal_tolerance = goal_tolerance
        self.search_window = search_window
        # Set True if the odometry already reports the front-axle position.
        self.pose_is_front_axle = pose_is_front_axle
        self.reset()

    def reset(self):
        self._segment_idx = 0

    def _nearest_segment(self, px, py):
        """
        Find the nearest segment, searching only a bounded window ahead.

        A whole-path search could jump to a spatially close but later part of
        the path (e.g. where a loop comes back near its start).
        Returns (segment index, projection parameter t in [0, 1], distance).
        """
        last_seg = len(self.path) - 2
        search_end = min(self._segment_idx + self.search_window, last_seg)
        best = None
        for i in range(self._segment_idx, search_end + 1):
            (x1, y1), (x2, y2) = self.path[i], self.path[i + 1]
            dx, dy = x2 - x1, y2 - y1
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-12:
                t = 0.0
            else:
                t = ((px - x1) * dx + (py - y1) * dy) / seg_len2
            t_clamped = max(0.0, min(t, 1.0))
            dist = math.hypot(px - (x1 + t_clamped * dx), py - (y1 + t_clamped * dy))
            if best is None or dist < best[2]:
                best = (i, t, dist)
        self._segment_idx = best[0]
        return best

    def compute(self, x, y, yaw, v):
        """Steering command for a vehicle at (x, y, yaw) moving at speed v (m/s)."""
        if self.pose_is_front_axle:
            fx, fy = x, y
        else:
            fx, fy = front_axle_position(x, y, yaw, self.wheelbase)

        i, t, dist = self._nearest_segment(fx, fy)
        (x1, y1), (x2, y2) = self.path[i], self.path[i + 1]
        seg_dx, seg_dy = x2 - x1, y2 - y1

        # Sign from the 2D cross product (w2 - w1) x (p_f - w1): > 0 => left.
        cross = seg_dx * (fy - y1) - seg_dy * (fx - x1)
        e_ct = math.copysign(dist, cross)

        theta_path = math.atan2(seg_dy, seg_dx)
        theta_e = normalize_angle(theta_path - yaw)

        # abs(v): keeps the denominator >= k_s even if the car briefly rolls back.
        correction = math.atan2(self.k * e_ct, self.k_soft + abs(v))
        delta = theta_e - correction
        delta = max(-self.max_steering_angle, min(delta, self.max_steering_angle))

        last_seg = len(self.path) - 2
        gx, gy = self.path[-1]
        at_end = i == last_seg and (t >= 1.0 or math.hypot(gx - fx, gy - fy)
                                    < self.goal_tolerance)

        return StanleyOutput(delta, e_ct, theta_e, fx, fy, i, at_end)
