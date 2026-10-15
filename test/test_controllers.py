import math

from stanley_controller.speed_controller import SpeedController
from stanley_controller.stanley import StanleyController

STRAIGHT_PATH = [(float(i), 0.0) for i in range(20)]


def make_stanley(**kwargs):
    params = {'wheelbase': 0.256, 'k': 1.0, 'k_soft': 1.0, 'max_steering_angle': 0.3}
    params.update(kwargs)
    return StanleyController(STRAIGHT_PATH, **params)


def test_on_path_aligned_gives_zero_steering():
    out = make_stanley().compute(2.0, 0.0, 0.0, 1.0)
    assert abs(out.cross_track_error) < 1e-9
    assert abs(out.delta) < 1e-9


def test_cross_track_sign_left_positive_and_steers_back():
    left = make_stanley().compute(2.0, 0.1, 0.0, 1.0)
    right = make_stanley().compute(2.0, -0.1, 0.0, 1.0)
    assert left.cross_track_error > 0.0 and left.delta < 0.0
    assert right.cross_track_error < 0.0 and right.delta > 0.0


def test_front_axle_projection_and_heading_error():
    out = make_stanley(max_steering_angle=1.0).compute(2.0, 0.0, 0.2, 0.0)
    assert math.isclose(out.front_x, 2.0 + 0.256 * math.cos(0.2))
    assert math.isclose(out.heading_error, -0.2)
    # Front axle drifted left (e_ct > 0), heading also left: both push delta right.
    assert out.delta < -0.2


def test_steering_saturation():
    out = make_stanley().compute(2.0, 5.0, 0.0, 0.0)
    assert out.delta == -0.3


def test_path_end_detected():
    out = make_stanley().compute(19.5, 0.0, 0.0, 1.0)
    assert out.at_path_end


def test_pid_converges_on_velocity_plant():
    pid = SpeedController(0.3, 3.0, 0.0, -3.0, 3.0, 5.0)
    v = 0.0
    for _ in range(200):
        u = pid.update(1.5, v, 0.05)
        v = u
    assert abs(v - 1.5) < 1e-3


def test_pid_output_is_saturated():
    pid = SpeedController(10.0, 0.0, 0.0, -0.5, 0.5)
    assert pid.update(5.0, 0.0, 0.05) == 0.5
    assert pid.update(-5.0, 0.0, 0.05) == -0.5


def test_pid_anti_windup_freezes_integral_in_saturation():
    pid = SpeedController(1.0, 1.0, 0.0, -0.5, 0.5, integral_limit=100.0)
    for _ in range(100):
        pid.update(5.0, 0.0, 0.05)
    assert pid.integral == 0.0
    # As soon as the error reverses, the output leaves saturation immediately.
    assert pid.update(0.0, 0.4, 0.05) < 0.0


def test_pid_derivative_on_measurement_has_no_setpoint_kick():
    pid = SpeedController(0.0, 0.0, 1.0, -10.0, 10.0, derivative_filter_alpha=0.0)
    pid.update(0.0, 0.0, 0.05)
    assert pid.update(5.0, 0.0, 0.05) == 0.0
