"""
pi_controller.py
================

Golden-model PI controllers for the PMSM FOC project.

Fixed-point formats:
    Current signals : Q1.15
    Voltage signals : Q1.15
    iq_ref          : Q1.15
    PI integrator   : Q8.24
    Speed           : Q10.6

Includes:
    - Floating-point PI
    - Fixed-point PI
    - Output saturation
    - Conditional-integration anti-windup
    - Reset
    - Self-tests
"""

from dataclasses import dataclass


# ============================================================
# Fixed-point constants
# ============================================================

Q15_FRAC = 15
Q24_FRAC = 24
Q6_FRAC = 6

Q15_MAX = 32767
Q15_MIN = -32768

Q24_MAX = (1 << 31) - 1
Q24_MIN = -(1 << 31)


# ============================================================
# Fixed-point helper functions
# ============================================================

def sat(value, minimum, maximum):
    """Saturate integer value to [minimum, maximum]."""
    if value > maximum:
        return maximum
    if value < minimum:
        return minimum
    return value


def float_to_q15(value):
    """Convert float to signed Q1.15."""
    scaled = int(round(value * (1 << Q15_FRAC)))
    return sat(scaled, Q15_MIN, Q15_MAX)


def q15_to_float(value):
    """Convert signed Q1.15 to float."""
    return value / float(1 << Q15_FRAC)


def float_to_q24(value):
    """Convert float to signed Q8.24."""
    scaled = int(round(value * (1 << Q24_FRAC)))
    return sat(scaled, Q24_MIN, Q24_MAX)


def q24_to_float(value):
    """Convert signed Q8.24 to float."""
    return value / float(1 << Q24_FRAC)


def float_to_q6(value):
    """Convert speed in rad/s to unsigned Q10.6."""
    scaled = int(round(value * (1 << Q6_FRAC)))
    return max(0, min(scaled, (1 << 16) - 1))


def q6_to_float(value):
    """Convert unsigned Q10.6 to float."""
    return value / float(1 << Q6_FRAC)


def mul_q15(a, b):
    """
    Multiply two Q1.15 numbers.

    Q1.15 * Q1.15 -> Q2.30
    then return Q1.15 with rounding.
    """
    product = a * b

    if product >= 0:
        product += 1 << (Q15_FRAC - 1)
    else:
        product -= 1 << (Q15_FRAC - 1)

    return product >> Q15_FRAC


# ============================================================
# Floating-point PI controller
# ============================================================

@dataclass
class PIControllerFloat:
    """
    Floating-point reference PI controller.

    e = reference - feedback

    P = Kp * e

    I_candidate = I + Ki * e

    u_raw = P + I_candidate

    Conditional integration is used for anti-windup.
    """

    kp: float
    ki: float

    output_min: float = -1.0
    output_max: float = 1.0

    integrator: float = 0.0

    def reset(self):
        """Reset the PI integrator."""
        self.integrator = 0.0

    def update(self, reference, feedback):
        """Execute one PI update."""

        error = reference - feedback

        proportional = self.kp * error

        integrator_candidate = (
            self.integrator +
            self.ki * error
        )

        unsaturated = (
            proportional +
            integrator_candidate
        )

        output = max(
            self.output_min,
            min(unsaturated, self.output_max)
        )

        # Conditional integration anti-windup.
        saturated_high = (
            unsaturated > self.output_max
        )

        saturated_low = (
            unsaturated < self.output_min
        )

        allow_integrator = True

        if saturated_high and error > 0:
            allow_integrator = False

        if saturated_low and error < 0:
            allow_integrator = False

        if allow_integrator:
            self.integrator = integrator_candidate

        return output


# ============================================================
# Fixed-point PI controller
# ============================================================

@dataclass
class PIControllerFixed:
    """
    Fixed-point PI controller.

    Inputs:
        reference : Q1.15
        feedback  : Q1.15

    Output:
        Q1.15

    Integrator:
        Q8.24

    Gains:
        kp_q15 : Q1.15
        ki_q15 : Q1.15
    """

    kp_q15: int
    ki_q15: int

    output_min_q15: int = Q15_MIN
    output_max_q15: int = Q15_MAX

    integrator_q24: int = 0

    def reset(self):
        """Reset the integrator."""
        self.integrator_q24 = 0

    def update(self, reference_q15, feedback_q15):
        """Execute one fixed-point PI update."""

        # ----------------------------------------------------
        # Error
        # ----------------------------------------------------

        error_q15 = reference_q15 - feedback_q15

        error_q15 = sat(
            error_q15,
            Q15_MIN,
            Q15_MAX
        )

        # ----------------------------------------------------
        # Proportional term
        #
        # Q1.15 * Q1.15 -> Q1.15
        # ----------------------------------------------------

        proportional_q15 = mul_q15(
            self.kp_q15,
            error_q15
        )

        # ----------------------------------------------------
        # Integral increment
        #
        # Q1.15 * Q1.15 -> Q1.15
        #
        # Convert Q1.15 -> Q8.24
        # ----------------------------------------------------

        integral_increment_q15 = mul_q15(
            self.ki_q15,
            error_q15
        )

        integral_increment_q24 = (
            integral_increment_q15 <<
            (Q24_FRAC - Q15_FRAC)
        )

        # ----------------------------------------------------
        # Candidate integrator
        # ----------------------------------------------------

        integrator_candidate = (
            self.integrator_q24 +
            integral_increment_q24
        )

        integrator_candidate = sat(
            integrator_candidate,
            Q24_MIN,
            Q24_MAX
        )

        # ----------------------------------------------------
        # Convert integrator Q8.24 -> Q1.15
        # ----------------------------------------------------

        shift = Q24_FRAC - Q15_FRAC

        if integrator_candidate >= 0:
            integrator_q15 = (
                integrator_candidate +
                (1 << (shift - 1))
            ) >> shift
        else:
            integrator_q15 = (
                integrator_candidate -
                (1 << (shift - 1))
            ) >> shift

        integrator_q15 = sat(
            integrator_q15,
            Q15_MIN,
            Q15_MAX
        )

        # ----------------------------------------------------
        # Raw output
        # ----------------------------------------------------

        unsaturated_q15 = (
            proportional_q15 +
            integrator_q15
        )

        # ----------------------------------------------------
        # Output saturation
        # ----------------------------------------------------

        output_q15 = sat(
            unsaturated_q15,
            self.output_min_q15,
            self.output_max_q15
        )

        # ----------------------------------------------------
        # Conditional integration anti-windup
        # ----------------------------------------------------

        saturated_high = (
            unsaturated_q15 >
            self.output_max_q15
        )

        saturated_low = (
            unsaturated_q15 <
            self.output_min_q15
        )

        allow_integrator = True

        if saturated_high and error_q15 > 0:
            allow_integrator = False

        if saturated_low and error_q15 < 0:
            allow_integrator = False

        if allow_integrator:
            self.integrator_q24 = integrator_candidate

        return output_q15


# ============================================================
# Gain conversion helper
# ============================================================

def make_fixed_pi(
    kp,
    ki,
    output_min=-1.0,
    output_max=1.0
):
    """Create a fixed-point PI controller from float gains."""

    return PIControllerFixed(
        kp_q15=float_to_q15(kp),
        ki_q15=float_to_q15(ki),
        output_min_q15=float_to_q15(output_min),
        output_max_q15=float_to_q15(output_max)
    )


# ============================================================
# Self-tests
# ============================================================

def test_fixed_point_conversion():
    """Check Q1.15 conversion."""

    test_values = [
        -1.0,
        -0.75,
        -0.5,
        0.0,
        0.25,
        0.5,
        0.75,
        0.999
    ]

    for value in test_values:

        q = float_to_q15(value)
        recovered = q15_to_float(q)

        error = abs(value - recovered)

        assert error <= 1.0 / (1 << Q15_FRAC), (
            f"Q15 conversion failed: "
            f"{value} -> {q} -> {recovered}"
        )

    print("Q1.15 conversion test: PASS")


def test_float_pi_basic():
    """Basic floating-point PI response."""

    pi = PIControllerFloat(
        kp=0.5,
        ki=0.1,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    output = pi.update(
        reference=0.5,
        feedback=0.0
    )

    assert output > 0.0
    assert output <= 1.0

    print("Float PI basic test: PASS")


def test_float_pi_saturation():
    """Check floating-point output saturation."""

    pi = PIControllerFloat(
        kp=2.0,
        ki=1.0,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    output = pi.update(
        reference=10.0,
        feedback=0.0
    )

    assert output == 1.0

    print("Float PI saturation test: PASS")


def test_float_pi_anti_windup():
    """Check floating-point anti-windup."""

    pi = PIControllerFloat(
        kp=1.0,
        ki=0.1,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    for _ in range(100):
        output = pi.update(
            reference=10.0,
            feedback=0.0
        )

    assert output == 1.0

    # Integrator must not grow without bound.
    assert abs(pi.integrator) < 0.5

    print("Float PI anti-windup test: PASS")


def test_float_pi_recovery():
    """Check recovery after saturation."""

    pi = PIControllerFloat(
        kp=1.0,
        ki=0.1,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    # --------------------------------------------------------
    # Force positive saturation.
    # --------------------------------------------------------

    for _ in range(20):
        output = pi.update(
            reference=10.0,
            feedback=0.0
        )

    assert output == 1.0

    # The conditional anti-windup should have prevented
    # the integrator from growing during saturation.
    assert pi.integrator == 0.0

    # --------------------------------------------------------
    # Apply a moderate negative error.
    #
    # This should bring the output inside the limits and
    # therefore allow the integrator to update.
    # --------------------------------------------------------

    old_integrator = pi.integrator

    output = pi.update(
        reference=-0.2,
        feedback=0.0
    )

    assert output < 0.0
    assert output > -1.0

    # The integrator should now move in the negative direction.
    assert pi.integrator < old_integrator

    print("Float PI recovery test: PASS")

    
def test_fixed_pi_basic():
    """Basic fixed-point PI test."""

    pi = make_fixed_pi(
        kp=0.5,
        ki=0.1,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    output_q15 = pi.update(
        float_to_q15(0.5),
        float_to_q15(0.0)
    )

    output = q15_to_float(output_q15)

    assert output > 0.0
    assert output <= 1.0

    print("Fixed PI basic test: PASS")


def test_fixed_pi_saturation():
    """Check fixed-point output saturation."""

    pi = make_fixed_pi(
        kp=2.0,
        ki=1.0,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    output_q15 = pi.update(
        float_to_q15(1.0),
        float_to_q15(0.0)
    )

    assert output_q15 == Q15_MAX

    print("Fixed PI saturation test: PASS")


def test_fixed_pi_anti_windup():
    """Check fixed-point anti-windup."""

    pi = make_fixed_pi(
        kp=1.0,
        ki=0.1,
        output_min=-1.0,
        output_max=1.0
    )

    pi.reset()

    for _ in range(100):

        output_q15 = pi.update(
            float_to_q15(1.0),
            float_to_q15(-1.0)
        )

    assert output_q15 == Q15_MAX

    assert abs(pi.integrator_q24) < (1 << 23)

    print("Fixed PI anti-windup test: PASS")


def test_fixed_vs_float():
    """Compare fixed-point and floating-point PI."""

    pi_float = PIControllerFloat(
        kp=0.5,
        ki=0.05,
        output_min=-1.0,
        output_max=1.0
    )

    pi_fixed = make_fixed_pi(
        kp=0.5,
        ki=0.05,
        output_min=-1.0,
        output_max=1.0
    )

    reference = 0.5

    feedback_values = [
        0.0,
        0.05,
        0.10,
        0.15,
        0.20,
        0.25,
        0.30,
        0.35,
        0.40,
        0.45
    ]

    for feedback in feedback_values:

        out_float = pi_float.update(
            reference,
            feedback
        )

        out_fixed_q15 = pi_fixed.update(
            float_to_q15(reference),
            float_to_q15(feedback)
        )

        out_fixed = q15_to_float(
            out_fixed_q15
        )

        error = abs(
            out_float -
            out_fixed
        )

        assert error <= 4.0 / (1 << Q15_FRAC), (
            f"Float/fixed mismatch: "
            f"float={out_float}, "
            f"fixed={out_fixed}, "
            f"error={error}"
        )

    print("Float vs fixed PI test: PASS")


# ============================================================
# Run all tests
# ============================================================

def run_self_tests():

    print("=" * 60)
    print("PI CONTROLLER SELF-TEST")
    print("=" * 60)

    test_fixed_point_conversion()
    test_float_pi_basic()
    test_float_pi_saturation()
    test_float_pi_anti_windup()
    test_float_pi_recovery()
    test_fixed_pi_basic()
    test_fixed_pi_saturation()
    test_fixed_pi_anti_windup()
    test_fixed_vs_float()

    print("=" * 60)
    print("ALL PI TESTS PASS")
    print("=" * 60)


if __name__ == "__main__":
    run_self_tests()