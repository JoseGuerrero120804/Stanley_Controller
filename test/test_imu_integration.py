import math

from stanley_controller.imu_integration import ImuIntegrator


def feed(integ, t0, duration, wz, v, rate=100.0):
    n = int(round(duration * rate))
    for i in range(n):
        integ.update(t0 + i / rate, wz, v)
    return t0 + n / rate


def test_origin_and_bias_while_still():
    integ = ImuIntegrator(bias_window=1.0)
    t = feed(integ, 0.0, 1.5, 0.004, 0.0)   # still, constant gyro bias
    assert integ.ready and math.isclose(integ.bias, 0.004)
    feed(integ, t, 2.0, 0.004, 0.0)          # bias removed: no drift while still
    assert abs(integ.theta) < 1e-9 and integ.x == 0.0 and integ.y == 0.0


def test_straight_line_from_origin():
    integ = ImuIntegrator(bias_window=1.0)
    t = feed(integ, 0.0, 1.5, 0.0, 0.0)
    feed(integ, t, 10.0, 0.0, 0.2)
    assert math.isclose(integ.x, 2.0, abs_tol=0.01)
    assert abs(integ.y) < 1e-9


def test_left_quarter_turn():
    integ = ImuIntegrator(bias_window=1.0)
    t = feed(integ, 0.0, 1.5, 0.01, 0.0)
    w, v = math.pi / 2 / 5.0, 0.2            # 90 deg in 5 s, radius v / w
    feed(integ, t, 5.0, w + 0.01, v)
    r = v / w
    assert math.isclose(integ.theta, math.pi / 2, abs_tol=0.01)
    assert math.isclose(integ.x, r, abs_tol=0.01) and math.isclose(integ.y, r, abs_tol=0.01)


def test_moving_during_bias_window_is_reported():
    integ = ImuIntegrator(bias_window=1.0)
    events = [integ.update(i / 100.0, 0.1, 0.3) for i in range(150)]
    assert 'bias_failed' in events and integ.bias == 0.0


def test_data_gap_is_skipped_and_counted():
    integ = ImuIntegrator(bias_window=1.0)
    t = feed(integ, 0.0, 1.5, 0.0, 0.0)
    integ.update(t + 2.0, 0.0, 0.2)          # 2 s gap
    assert integ.gaps == 1 and integ.x == 0.0
