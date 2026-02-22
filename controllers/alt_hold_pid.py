#!/usr/bin/env python3
"""
Step 2 controller — PID altitude hold.
Roll/pitch/yaw stay neutral (1500). Only throttle is controlled by PID.
PID algorithm from hover2/nn_hover2/hover_pid.py.

Call reset(target_alt) before first compute() — sets target and clears PID state.
"""

RC_MID = 1500   # neutral RC value for all axes
RC_MIN = 1000   # minimum RC value (full stick down/left)
RC_MAX = 2000   # maximum RC value (full stick up/right)

# base throttle — conservative value below typical hover point
# the PID integral will find the real hover throttle automatically
# recommendation: if drone barely lifts at 1300, lower to 1250
BASE_THROTTLE = 1300


def _clamp_rc(value):
    """Keep value within valid RC range [1000, 2000]."""
    return max(RC_MIN, min(RC_MAX, int(value)))


class PID:
    """
    Standard PID controller with anti-windup.
    Copied from hover2/nn_hover2/hover_pid.py.
    """

    def __init__(self, kp, ki, kd, output_limits=(-1, 1)):
        self.kp = kp                    # proportional gain — reacts to current error
        self.ki = ki                    # integral gain — eliminates steady-state error
        self.kd = kd                    # derivative gain — dampens oscillations
        self.limits = output_limits     # clamp output to this range
        self._integral = 0.0            # accumulated error over time
        self._last_error = None         # previous error for derivative calculation

    def reset(self):
        """Clear PID state for fresh start."""
        self._integral = 0.0
        self._last_error = None

    def update(self, error, dt):
        """Compute PID output from error and time step."""

        # integral: accumulate error over time
        self._integral += error * dt

        # anti-windup: prevent integral from growing too large
        # if integral is huge, PID output will be stuck at max even after error is gone
        max_integral = (self.limits[1] - self.limits[0]) / max(self.ki, 1e-6)
        self._integral = max(-max_integral, min(max_integral, self._integral))

        # derivative: rate of change of error (zero on first call)
        d = (error - self._last_error) / dt if self._last_error is not None else 0.0
        self._last_error = error

        # PID formula: P + I + D
        out = self.kp * error + self.ki * self._integral + self.kd * d

        # clamp output to limits
        return max(self.limits[0], min(self.limits[1], out))


class PIDController:
    """
    PID altitude hold controller.
    throttle = BASE_THROTTLE + PID(target_alt - current_alt)

    If drone is too low  → PID positive → more throttle
    If drone is too high → PID negative → less throttle

    BASE_THROTTLE (1300) is intentionally conservative.
    The PID integral will gradually find the real hover throttle.
    """

    def __init__(self):
        self.target_alt = None  # set by reset() before first compute()

        # PID gains scaled for RC output [1000-2000]:
        #   kp=150  → 1m error gives +150 RC offset
        #   ki=25   → integral finds hover throttle over time
        #   kd=100  → derivative dampens oscillations
        #   limits=(-300, 500) → throttle range [1000, 1800]
        #     asymmetric: more room to climb (+500) than descend (-300)
        self.alt_pid = PID(kp=150, ki=25, kd=100, output_limits=(-300, 500))

    def reset(self, target_alt):
        """Set target altitude and clear PID state. Call before first compute()."""
        self.target_alt = target_alt
        self.alt_pid.reset()

    def compute(self, roll, pitch, yaw, altitude, dt):
        """Roll/pitch/yaw stay neutral. Only throttle is adjusted by PID."""

        # error > 0 means drone is below target (need more throttle)
        alt_error = self.target_alt - altitude

        # PID computes throttle offset from BASE_THROTTLE
        pid_output = self.alt_pid.update(alt_error, dt)

        # final throttle = base + PID correction, clamped to [1000, 2000]
        return RC_MID, RC_MID, RC_MID, _clamp_rc(BASE_THROTTLE + pid_output)
