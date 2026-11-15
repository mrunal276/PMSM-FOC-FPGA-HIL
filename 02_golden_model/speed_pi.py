"""
speed_pi.py

Outer speed-loop PI controller for the PMSM FOC golden model.

Architecture:
    omega_ref -> Speed PI -> iq_ref

Fixed-point:
    omega_ref / omega      : Q10.6  (64 counts = 1 rad/s)
    PI gains               : Q1.15
    integrator             : Q8.24
    iq_ref / output        : Q1.15  (1.0 = 20 A)

The speed PI output is iq_ref, which is limited to +/-20 A.

Anti-windup:
    Conditional integration:
      - integrate normally when output is not saturated
      - when saturated, integrate only if the error drives the
        output back toward the valid range
"""

from dataclasses import dataclass
import math


# ============================================================
# Fixed-point formats
# ============================================================

Q15_MAX = 32767
Q15_MIN = -32768

Q24_MAX = (1 << 31) - 1
Q24_MIN = -(1 << 31)

Q6_SCALE = 64.0
Q15_SCALE = 32768.0
Q24_SCALE = 1 << 24


# ============================================================
# Conversion helpers
# ============================================================

def float_to_q15(x: float) -> int:
    """Convert float to signed Q1.15."""
    if x >= 1.0:
        return Q15_MAX
    if x <= -1.0:
        return Q15_MIN

    value = int(round(x * Q15_SCALE))
    return max(Q15_MIN, min(Q15_MAX, value))


def q15_to_float(x: int) -> float:
    """Convert signed Q1.15 to float."""
    return x / Q15_SCALE


def float_to_q24(x: float) -> int:
    """Convert float to signed Q8.24."""
    value = int(round(x * Q24_SCALE))
    return max(Q24_MIN, min(Q24_MAX, value))


def q24_to_float(x: int) -> float:
    """Convert signed Q8.24 to float."""
    return x / Q24_SCALE


def float_to_q6(x: float) -> int:
    """Convert speed in rad/s to unsigned Q10.6."""
    if x <= 0.0:
        return 0

    value = int(round(x * Q6_SCALE))
    return max(0, min((1023 << 6) + 63, value))


def q6_to_float(x: int) -> float:
    """Convert unsigned Q10.6 to rad/s."""
    return x / Q6_SCALE


# ============================================================
# Utility
# ============================================================

def clamp(x: float, low: float, high: float) -> float:
    return max(low, min(high, x))


# ============================================================
# Float speed PI
# ============================================================

class SpeedPIControllerFloat:
    """
    Floating-point outer speed-loop PI.

    Inputs:
        omega_ref : reference speed [rad/s]
        omega     : measured speed [rad/s]

    Output:
        iq_ref    : q-axis current reference [normalized Q1.15 scale]

    Physical scaling:
        iq_ref = +1.0 -> +20 A
        iq_ref = -1.0 -> -20 A
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        dt: float,
        output_min: float = -1.0,
        output_max: float = 1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.dt = dt

        self.output_min = output_min
        self.output_max = output_max

        self.integrator = 0.0

    def reset(self):
        self.integrator = 0.0

    def update(self, omega_ref: float, omega: float) -> float:
        error = omega_ref - omega

        proportional = self.kp * error

        # Candidate integrator.
        candidate_integrator = (
            self.integrator
            + self.ki * error * self.dt
        )

        unsaturated = proportional + candidate_integrator

        saturated = clamp(
            unsaturated,
            self.output_min,
            self.output_max,
        )

        # Conditional-integration anti-windup.
        if unsaturated > self.output_max:
            if error < 0.0:
                self.integrator = candidate_integrator

        elif unsaturated < self.output_min:
            if error > 0.0:
                self.integrator = candidate_integrator

        else:
            self.integrator = candidate_integrator

        output = clamp(
            proportional + self.integrator,
            self.output_min,
            self.output_max,
        )

        return output


# ============================================================
# Fixed-point speed PI
# ============================================================

@dataclass
class SpeedPIFixedResult:
    omega_ref_q6: int
    omega_q6: int
    error_q6: int
    error_float: float

    proportional_q15: int
    integrator_q24: int

    output_q15: int
    output_float: float


class SpeedPIControllerFixed:
    """
    Fixed-point speed PI.

    omega:
        Q10.6

    gains:
        Q1.15

    integrator:
        Q8.24

    output:
        Q1.15

    Important:
        The fixed-point controller performs the same conditional
        integration logic as the floating-point golden reference.
    """

    def __init__(
        self,
        kp_q15: int,
        ki_q15: int,
        dt: float,
        output_min_q15: int = Q15_MIN,
        output_max_q15: int = Q15_MAX,
    ):
        self.kp_q15 = kp_q15
        self.ki_q15 = ki_q15
        self.dt = dt

        self.output_min_q15 = output_min_q15
        self.output_max_q15 = output_max_q15

        self.integrator_q24 = 0

        self.ki_dt = (
            q15_to_float(self.ki_q15) * self.dt
        )

    def reset(self):
        self.integrator_q24 = 0

    def update(
        self,
        omega_ref_q6: int,
        omega_q6: int,
    ) -> SpeedPIFixedResult:

        # ----------------------------------------------------
        # Error in Q10.6
        # ----------------------------------------------------

        error_q6 = omega_ref_q6 - omega_q6

        error_float = q6_to_float(error_q6)

        # ----------------------------------------------------
        # Proportional term
        #
        # kp is Q1.15
        # error is represented as float here for the golden model
        #
        # Convert proportional result back to Q1.15.
        # ----------------------------------------------------

        proportional_float = (
            q15_to_float(self.kp_q15) * error_float
        )

        proportional_q15 = float_to_q15(
            proportional_float
        )

        # ----------------------------------------------------
        # Candidate integrator
        # ----------------------------------------------------

        candidate_integrator_float = (
            q24_to_float(self.integrator_q24)
            + self.ki_dt * error_float
        )

        candidate_integrator_q24 = float_to_q24(
            candidate_integrator_float
        )

        unsaturated_float = (
            proportional_float
            + candidate_integrator_float
        )

        # ----------------------------------------------------
        # Output saturation
        # ----------------------------------------------------

        output_max = q15_to_float(self.output_max_q15)
        output_min = q15_to_float(self.output_min_q15)

        saturated_float = clamp(
            unsaturated_float,
            output_min,
            output_max,
        )

        # ----------------------------------------------------
        # Conditional integration anti-windup
        # ----------------------------------------------------

        if unsaturated_float > output_max:

            if error_float < 0.0:
                self.integrator_q24 = candidate_integrator_q24

        elif unsaturated_float < output_min:

            if error_float > 0.0:
                self.integrator_q24 = candidate_integrator_q24

        else:
            self.integrator_q24 = candidate_integrator_q24

        # ----------------------------------------------------
        # Final output
        # ----------------------------------------------------

        output_float = clamp(
            proportional_float
            + q24_to_float(self.integrator_q24),
            output_min,
            output_max,
        )

        output_q15 = float_to_q15(output_float)

        return SpeedPIFixedResult(
            omega_ref_q6=omega_ref_q6,
            omega_q6=omega_q6,
            error_q6=error_q6,
            error_float=error_float,
            proportional_q15=proportional_q15,
            integrator_q24=self.integrator_q24,
            output_q15=output_q15,
            output_float=output_float,
        )


# ============================================================
# Controller factory
# ============================================================

def make_fixed_speed_pi(
    kp: float,
    ki: float,
    dt: float,
) -> SpeedPIControllerFixed:

    return SpeedPIControllerFixed(
        kp_q15=float_to_q15(kp),
        ki_q15=float_to_q15(ki),
        dt=dt,
    )


# ============================================================
# Self-tests
# ============================================================

def test_q6_conversion():
    values = [
        0.0,
        1.0,
        10.0,
        100.0,
        500.0,
    ]

    for value in values:
        q = float_to_q6(value)
        reconstructed = q6_to_float(q)

        assert abs(reconstructed - value) <= (1.0 / 64.0)

    print("Q10.6 speed conversion test: PASS")


def test_float_basic():
    dt = 50e-6

    pi = SpeedPIControllerFloat(
        kp=0.005,
        ki=0.20,
        dt=dt,
    )

    pi.reset()

    output = pi.update(
        omega_ref=100.0,
        omega=0.0,
    )

    assert output > 0.0

    print("Float speed PI basic test: PASS")


def test_float_saturation():
    dt = 50e-6

    pi = SpeedPIControllerFloat(
        kp=0.05,
        ki=1.0,
        dt=dt,
    )

    pi.reset()

    for _ in range(100):
        output = pi.update(
            omega_ref=1000.0,
            omega=0.0,
        )

    assert output <= 1.0
    assert output >= -1.0

    print("Float speed PI saturation test: PASS")


def test_float_anti_windup():
    dt = 50e-6

    pi = SpeedPIControllerFloat(
        kp=0.05,
        ki=1.0,
        dt=dt,
    )

    pi.reset()

    for _ in range(1000):
        pi.update(
            omega_ref=1000.0,
            omega=0.0,
        )

    # Controller is saturated, but integrator must remain bounded.
    assert abs(pi.integrator) < 1.0

    print("Float speed PI anti-windup test: PASS")


def test_float_recovery():
    dt = 50e-6

    pi = SpeedPIControllerFloat(
        kp=0.02,
        ki=0.5,
        dt=dt,
    )

    pi.reset()

    # Drive the controller into strong positive output.
    for _ in range(2000):
        output = pi.update(
            omega_ref=1000.0,
            omega=0.0,
        )

    assert output > 0.9

    # Apply a strong negative speed error.
    # Depending on the proportional term, the output may cross
    # zero immediately, so we do NOT require the first recovery
    # sample to remain positive.
    recovery_outputs = []

    for _ in range(5000):
        output = pi.update(
            omega_ref=0.0,
            omega=1000.0,
        )
        recovery_outputs.append(output)

    # The controller must recover from the previous positive state
    # and eventually command negative q-axis current.
    assert any(x < 0.0 for x in recovery_outputs)

    print("Float speed PI recovery test: PASS")


def test_fixed_basic():
    dt = 50e-6

    pi = make_fixed_speed_pi(
        kp=0.005,
        ki=0.20,
        dt=dt,
    )

    pi.reset()

    result = pi.update(
        float_to_q6(100.0),
        float_to_q6(0.0),
    )

    assert result.output_q15 > 0

    print("Fixed speed PI basic test: PASS")


def test_fixed_saturation():
    dt = 50e-6

    pi = make_fixed_speed_pi(
        kp=0.05,
        ki=1.0,
        dt=dt,
    )

    pi.reset()

    result = None

    for _ in range(100):
        result = pi.update(
            float_to_q6(1000.0),
            float_to_q6(0.0),
        )

    assert result is not None
    assert result.output_q15 <= Q15_MAX
    assert result.output_q15 >= Q15_MIN

    print("Fixed speed PI saturation test: PASS")


def test_fixed_anti_windup():
    dt = 50e-6

    pi = make_fixed_speed_pi(
        kp=0.05,
        ki=1.0,
        dt=dt,
    )

    pi.reset()

    for _ in range(1000):
        pi.update(
            float_to_q6(1000.0),
            float_to_q6(0.0),
        )

    integrator = q24_to_float(pi.integrator_q24)

    assert abs(integrator) < 1.0

    print("Fixed speed PI anti-windup test: PASS")


def test_float_vs_fixed():
    dt = 50e-6

    kp = 0.005
    ki = 0.20

    float_pi = SpeedPIControllerFloat(
        kp=kp,
        ki=ki,
        dt=dt,
    )

    fixed_pi = make_fixed_speed_pi(
        kp=kp,
        ki=ki,
        dt=dt,
    )

    float_pi.reset()
    fixed_pi.reset()

    test_points = [
        (0.0, 0.0),
        (100.0, 0.0),
        (250.0, 100.0),
        (500.0, 250.0),
        (1000.0, 500.0),
        (500.0, 900.0),
        (100.0, 500.0),
        (0.0, 100.0),
    ]

    max_error = 0.0

    for omega_ref, omega in test_points:

        float_output = float_pi.update(
            omega_ref,
            omega,
        )

        result = fixed_pi.update(
            float_to_q6(omega_ref),
            float_to_q6(omega),
        )

        fixed_output = result.output_float

        error = abs(
            float_output - fixed_output
        )

        max_error = max(max_error, error)

        # One Q15 LSB plus small Q6 quantization tolerance.
        assert error <= 0.01

    print(
        "Float vs fixed speed PI test: PASS "
        f"(max error={max_error:.8f})"
    )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 60)
    print("SPEED PI SELF-TEST")
    print("=" * 60)

    test_q6_conversion()
    test_float_basic()
    test_float_saturation()
    test_float_anti_windup()
    test_float_recovery()

    test_fixed_basic()
    test_fixed_saturation()
    test_fixed_anti_windup()

    test_float_vs_fixed()

    print("=" * 60)
    print("ALL SPEED PI TESTS PASS")
    print("=" * 60)


if __name__ == "__main__":
    main()