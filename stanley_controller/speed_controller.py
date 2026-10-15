class SpeedController:
    """
    Discrete-time PID for longitudinal speed, with anti-windup clamping.

    e(t)   = v_ref - v(t)
    e_i(t) = e_i(t-1) + e(t) * dt
    e_d(t) = d e / dt, taken on the measurement (-dv/dt) so that steps in
             v_ref (e.g. straight -> curve) do not cause a derivative kick,
             and low-pass filtered to reject tachometer noise.
    u(t)   = Kp * e + Ki * e_i + Kd * e_d   (+ optional feed-forward Kff * v_ref)

    The output u is clipped to [u_min, u_max]. Anti-windup is done twice:
    the integral state is clamped to +/- integral_limit, and integration is
    frozen while u is saturated and the error would push it further into
    saturation (conditional integration).
    """

    def __init__(self, kp, ki, kd, u_min=-0.5, u_max=0.5, integral_limit=float('inf'),
                 derivative_filter_alpha=0.7, feedforward_gain=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.u_min = u_min
        self.u_max = u_max
        self.integral_limit = integral_limit
        # alpha in [0, 1): 0 = no filtering, closer to 1 = heavier filtering.
        self.derivative_filter_alpha = derivative_filter_alpha
        self.feedforward_gain = feedforward_gain
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.derivative = 0.0
        self._prev_measurement = None

    def update(self, v_ref, v, dt):
        """Return the saturated command u for one control step of length dt (s)."""
        error = v_ref - v

        if self._prev_measurement is None or dt <= 0.0:
            raw_derivative = 0.0
        else:
            raw_derivative = -(v - self._prev_measurement) / dt
        self._prev_measurement = v
        a = self.derivative_filter_alpha
        self.derivative = a * self.derivative + (1.0 - a) * raw_derivative

        candidate_integral = self.integral + error * max(dt, 0.0)
        candidate_integral = max(-self.integral_limit,
                                 min(candidate_integral, self.integral_limit))

        u_unsat = (self.feedforward_gain * v_ref
                   + self.kp * error
                   + self.ki * candidate_integral
                   + self.kd * self.derivative)

        winding_up = ((u_unsat > self.u_max and error > 0.0)
                      or (u_unsat < self.u_min and error < 0.0))
        if not winding_up:
            self.integral = candidate_integral
        else:
            u_unsat = (self.feedforward_gain * v_ref
                       + self.kp * error
                       + self.ki * self.integral
                       + self.kd * self.derivative)

        return max(self.u_min, min(u_unsat, self.u_max))
