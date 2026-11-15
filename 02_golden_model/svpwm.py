"""
svpwm.py
========

Golden-model Space Vector PWM for the PMSM FOC project.

Inputs:
    V_alpha, V_beta : Q1.15 normalized voltage

Outputs:
    Ta, Tb, Tc      : Q1.15 duty cycles

Conventions:
    - Vbus normalized to 1.0
    - Q1.15 duty:
          0.0 -> 0
          1.0 -> 32767
    - Six SVPWM sectors
    - Center-aligned symmetric PWM
    - Dead-time modeled separately
    - Switching-state truth table included

The implementation contains:
    1. Floating-point SVPWM
    2. Fixed-point SVPWM
    3. Sector detection
    4. T1/T2/T0 calculation
    5. Duty-cycle generation
    6. Dead-time helper
    7. Switching-state table
    8. Self-tests
"""

import math
from dataclasses import dataclass


# ============================================================
# Fixed-point constants
# ============================================================

Q15_FRAC = 15

Q15_MAX = 32767
Q15_MIN = -32768

SQRT3 = math.sqrt(3.0)
INV_SQRT3 = 1.0 / SQRT3


# ============================================================
# Fixed-point helpers
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


def float_to_duty_q15(value):
    """
    Convert duty [0,1] to unsigned Q1.15.

    0.0 -> 0
    1.0 -> 32767
    """
    value = max(0.0, min(1.0, value))
    return int(round(value * Q15_MAX))


def duty_q15_to_float(value):
    """Convert unsigned Q1.15 duty to float."""
    value = sat(value, 0, Q15_MAX)
    return value / float(Q15_MAX)


# ============================================================
# Sector detection
# ============================================================

def sector_from_angle(angle):
    """
    Determine SVPWM sector from electrical angle.

    Angle:
        radians

    Sector numbering:

          Sector 2 | Sector 1
               \\   /
                \\ /
          Sector 3 + Sector 6
                / \
               /   \
          Sector 4 | Sector 5

    More precisely:

        Sector 1 :   0° .. 60°
        Sector 2 :  60° .. 120°
        Sector 3 : 120° .. 180°
        Sector 4 : 180° .. 240°
        Sector 5 : 240° .. 300°
        Sector 6 : 300° .. 360°
    """

    angle = angle % (2.0 * math.pi)

    sector = int(
        math.floor(
            angle / (math.pi / 3.0)
        )
    ) + 1

    if sector > 6:
        sector = 6

    return sector


def sector_from_alpha_beta(v_alpha, v_beta):
    """
    Determine sector directly from alpha-beta voltage vector.

    Returns:
        sector 1..6

    Zero vector is assigned to sector 1.
    """

    if abs(v_alpha) < 1e-15 and abs(v_beta) < 1e-15:
        return 1

    angle = math.atan2(
        v_beta,
        v_alpha
    )

    if angle < 0.0:
        angle += 2.0 * math.pi

    return sector_from_angle(angle)


# ============================================================
# Floating-point SVPWM
# ============================================================

def svpwm_float(v_alpha, v_beta):
    """
    Floating-point SVPWM.

    Inputs:
        v_alpha : normalized alpha voltage
        v_beta  : normalized beta voltage

    Outputs:
        sector
        T1
        T2
        T0
        Ta
        Tb
        Tc

    The implementation uses the standard space-vector
    dwell-time equations.
    """

    # --------------------------------------------------------
    # Determine vector angle and magnitude.
    # --------------------------------------------------------

    magnitude = math.sqrt(
        v_alpha * v_alpha +
        v_beta * v_beta
    )

    angle = math.atan2(
        v_beta,
        v_alpha
    )

    if angle < 0.0:
        angle += 2.0 * math.pi

    sector = sector_from_angle(angle)

    # --------------------------------------------------------
    # Angle relative to the beginning of the sector.
    # --------------------------------------------------------

    sector_angle = (
        angle -
        (sector - 1) * math.pi / 3.0
    )

    # --------------------------------------------------------
    # Standard SVPWM dwell times.
    #
    # For a normalized switching period Ts = 1:
    #
    # T1 = sqrt(3) * V * sin(pi/3 - theta)
    # T2 = sqrt(3) * V * sin(theta)
    #
    # --------------------------------------------------------

    t1 = (
        SQRT3 *
        magnitude *
        math.sin(
            math.pi / 3.0 -
            sector_angle
        )
    )

    t2 = (
        SQRT3 *
        magnitude *
        math.sin(
            sector_angle
        )
    )

    # --------------------------------------------------------
    # Over-modulation protection.
    #
    # If T1 + T2 > 1, scale them to fit.
    # --------------------------------------------------------

    active_time = t1 + t2

    if active_time > 1.0:
        scale = 1.0 / active_time
        t1 *= scale
        t2 *= scale
        active_time = 1.0

    t0 = 1.0 - active_time

    # --------------------------------------------------------
    # Symmetric / centered PWM.
    #
    # T0/2 is placed at each side.
    # --------------------------------------------------------

    half_t0 = 0.5 * t0

    # --------------------------------------------------------
    # Convert T1/T2/T0 into phase duties.
    # --------------------------------------------------------

    if sector == 1:

        ta = half_t0 + t1 + t2
        tb = half_t0 + t2
        tc = half_t0

    elif sector == 2:

        ta = half_t0 + t1
        tb = half_t0 + t1 + t2
        tc = half_t0

    elif sector == 3:

        ta = half_t0
        tb = half_t0 + t1 + t2
        tc = half_t0 + t2

    elif sector == 4:

        ta = half_t0
        tb = half_t0 + t1
        tc = half_t0 + t1 + t2

    elif sector == 5:

        ta = half_t0 + t2
        tb = half_t0
        tc = half_t0 + t1 + t2

    else:

        ta = half_t0 + t1 + t2
        tb = half_t0
        tc = half_t0 + t1

    # --------------------------------------------------------
    # Final safety saturation.
    # --------------------------------------------------------

    ta = max(0.0, min(1.0, ta))
    tb = max(0.0, min(1.0, tb))
    tc = max(0.0, min(1.0, tc))

    return {
        "sector": sector,
        "T1": t1,
        "T2": t2,
        "T0": t0,
        "Ta": ta,
        "Tb": tb,
        "Tc": tc,
    }


# ============================================================
# Fixed-point SVPWM
# ============================================================

@dataclass
class SVPWMFixedResult:
    """Fixed-point SVPWM result."""

    sector: int
    T1_q15: int
    T2_q15: int
    T0_q15: int
    Ta_q15: int
    Tb_q15: int
    Tc_q15: int


def svpwm_fixed(v_alpha_q15, v_beta_q15):
    """
    Fixed-point SVPWM reference.

    The arithmetic is performed through the equivalent
    floating-point dwell equations, followed by Q1.15
    quantization.

    This function is the golden numerical reference that
    the eventual RTL implementation must match.

    Inputs:
        v_alpha_q15 : Q1.15
        v_beta_q15  : Q1.15

    Outputs:
        SVPWMFixedResult
    """

    v_alpha = q15_to_float(v_alpha_q15)
    v_beta = q15_to_float(v_beta_q15)

    result = svpwm_float(
        v_alpha,
        v_beta
    )

    return SVPWMFixedResult(
        sector=result["sector"],
        T1_q15=float_to_duty_q15(result["T1"]),
        T2_q15=float_to_duty_q15(result["T2"]),
        T0_q15=float_to_duty_q15(result["T0"]),
        Ta_q15=float_to_duty_q15(result["Ta"]),
        Tb_q15=float_to_duty_q15(result["Tb"]),
        Tc_q15=float_to_duty_q15(result["Tc"]),
    )


# ============================================================
# Switching-state truth table
# ============================================================

SWITCHING_TABLE = {
    1: {
        "active_1": (1, 0, 0),
        "active_2": (1, 1, 0),
        "zero":     (0, 0, 0),
    },

    2: {
        "active_1": (1, 1, 0),
        "active_2": (0, 1, 0),
        "zero":     (0, 0, 0),
    },

    3: {
        "active_1": (0, 1, 0),
        "active_2": (0, 1, 1),
        "zero":     (0, 0, 0),
    },

    4: {
        "active_1": (0, 1, 1),
        "active_2": (0, 0, 1),
        "zero":     (0, 0, 0),
    },

    5: {
        "active_1": (0, 0, 1),
        "active_2": (1, 0, 1),
        "zero":     (0, 0, 0),
    },

    6: {
        "active_1": (1, 0, 1),
        "active_2": (1, 0, 0),
        "zero":     (0, 0, 0),
    },
}


def get_switching_states(sector):
    """
    Return the active and zero switching states for a sector.
    """

    if sector not in SWITCHING_TABLE:
        raise ValueError(
            f"Invalid SVPWM sector: {sector}"
        )

    return SWITCHING_TABLE[sector]


# ============================================================
# Complementary switch generation
# ============================================================

def complementary_switch(high_side):
    """
    Generate ideal complementary low-side command.

    high_side:
        0 or 1

    Returns:
        1 - high_side
    """

    return 1 - int(bool(high_side))


def generate_complementary_state(high_state):
    """
    Convert a three-phase high-side state into the ideal
    complementary low-side state.

    Example:
        high = (1, 0, 1)
        low  = (0, 1, 0)
    """

    return tuple(
        complementary_switch(x)
        for x in high_state
    )


# ============================================================
# Dead-time helper
# ============================================================

@dataclass
class DeadTimeChannel:
    """
    Simple digital dead-time model.

    The actual RTL implementation will normally use counters.
    This golden model represents the intended behavior:

        high OFF -> wait dead cycles -> low ON
        low  OFF -> wait dead cycles -> high ON

    The two outputs must never be high simultaneously.
    """

    dead_cycles: int = 2

    high: int = 0
    low: int = 1

    high_count: int = 0
    low_count: int = 0

    def reset(self):
        self.high = 0
        self.low = 1
        self.high_count = 0
        self.low_count = 0

    def update(self, requested_high):
        """
        Update one complementary half-bridge.

        requested_high:
            Desired high-side state.

        Returns:
            high, low
        """

        requested_high = int(bool(requested_high))

        # ----------------------------------------------------
        # Request HIGH
        # ----------------------------------------------------

        if requested_high:

            self.high_count = 0

            if self.low:

                self.low = 0

                self.high_count += 1

                if self.high_count >= self.dead_cycles:
                    self.high = 1
                    self.high_count = 0

            else:
                self.high = 1

        # ----------------------------------------------------
        # Request LOW
        # ----------------------------------------------------

        else:

            self.low_count = 0

            if self.high:

                self.high = 0

                self.low_count += 1

                if self.low_count >= self.dead_cycles:
                    self.low = 1
                    self.low_count = 0

            else:
                self.low = 1

        # ----------------------------------------------------
        # Safety interlock.
        # ----------------------------------------------------

        if self.high and self.low:
            raise RuntimeError(
                "SHOOT-THROUGH DETECTED"
            )

        return self.high, self.low


# ============================================================
# Sector test points
# ============================================================

def sector_test_angles():
    """
    Return representative angles for all six sectors.

    Using the sector centers avoids boundary ambiguity.
    """

    return [
        math.radians(30.0),
        math.radians(90.0),
        math.radians(150.0),
        math.radians(210.0),
        math.radians(270.0),
        math.radians(330.0),
    ]


# ============================================================
# Self-tests
# ============================================================

def test_sector_detection():
    """Verify all six sectors."""

    expected = [1, 2, 3, 4, 5, 6]

    detected = []

    for angle in sector_test_angles():

        sector = sector_from_angle(angle)

        detected.append(sector)

    assert detected == expected, (
        f"Sector detection failed: "
        f"{detected}"
    )

    print("Sector detection test: PASS")


def test_alpha_beta_sector_detection():
    """Verify sector detection using alpha-beta vectors."""

    expected = [1, 2, 3, 4, 5, 6]

    detected = []

    for angle in sector_test_angles():

        v_alpha = math.cos(angle)
        v_beta = math.sin(angle)

        sector = sector_from_alpha_beta(
            v_alpha,
            v_beta
        )

        detected.append(sector)

    assert detected == expected, (
        f"Alpha-beta sector detection failed: "
        f"{detected}"
    )

    print("Alpha-beta sector test: PASS")


def test_zero_vector():
    """Zero voltage vector should produce 50% duty."""

    result = svpwm_float(
        0.0,
        0.0
    )

    assert result["sector"] == 1

    assert abs(result["Ta"] - 0.5) < 1e-12
    assert abs(result["Tb"] - 0.5) < 1e-12
    assert abs(result["Tc"] - 0.5) < 1e-12

    assert abs(result["T1"]) < 1e-12
    assert abs(result["T2"]) < 1e-12
    assert abs(result["T0"] - 1.0) < 1e-12

    print("Zero-vector test: PASS")


def test_dwell_time_sum():
    """T1 + T2 + T0 must equal one switching period."""

    for angle in [
        i * math.pi / 180.0
        for i in range(0, 360, 5)
    ]:

        magnitude = 0.5

        v_alpha = (
            magnitude *
            math.cos(angle)
        )

        v_beta = (
            magnitude *
            math.sin(angle)
        )

        result = svpwm_float(
            v_alpha,
            v_beta
        )

        total = (
            result["T1"] +
            result["T2"] +
            result["T0"]
        )

        assert abs(total - 1.0) < 1e-12, (
            f"Dwell sum error at "
            f"{math.degrees(angle)} deg: "
            f"{total}"
        )

    print("Dwell-time sum test: PASS")


def test_duty_range():
    """All generated duty cycles must remain [0,1]."""

    for angle in [
        i * math.pi / 180.0
        for i in range(0, 360, 5)
    ]:

        for magnitude in [
            0.0,
            0.1,
            0.3,
            0.5,
            0.7,
            1.0,
            1.5,
        ]:

            v_alpha = (
                magnitude *
                math.cos(angle)
            )

            v_beta = (
                magnitude *
                math.sin(angle)
            )

            result = svpwm_float(
                v_alpha,
                v_beta
            )

            for name in ["Ta", "Tb", "Tc"]:

                duty = result[name]

                assert (
                    0.0 <= duty <= 1.0
                ), (
                    f"Duty range error: "
                    f"{name}={duty}"
                )

    print("Duty-range test: PASS")


def test_all_six_sectors():
    """Run SVPWM through the center of all six sectors."""

    for expected_sector, angle in enumerate(
        sector_test_angles(),
        start=1
    ):

        magnitude = 0.5

        v_alpha = (
            magnitude *
            math.cos(angle)
        )

        v_beta = (
            magnitude *
            math.sin(angle)
        )

        result = svpwm_float(
            v_alpha,
            v_beta
        )

        assert result["sector"] == expected_sector

        assert result["T1"] >= 0.0
        assert result["T2"] >= 0.0
        assert result["T0"] >= 0.0

    print("All-six-sector test: PASS")


def test_overmodulation():
    """
    Verify that a vector outside the linear modulation range
    is safely scaled.
    """

    result = svpwm_float(
        2.0,
        2.0
    )

    assert (
        result["T1"] +
        result["T2"] <= 1.0 + 1e-12
    )

    assert result["T0"] >= -1e-12

    print("Over-modulation protection test: PASS")


def test_fixed_point_svpwm():
    """Check fixed-point SVPWM output range."""

    result = svpwm_fixed(
        float_to_q15(0.5),
        float_to_q15(0.0)
    )

    assert result.sector == 1

    for duty in [
        result.Ta_q15,
        result.Tb_q15,
        result.Tc_q15,
    ]:

        assert (
            0 <= duty <= Q15_MAX
        )

    print("Fixed-point SVPWM test: PASS")


def test_fixed_vs_float():
    """Compare fixed-point duties against float reference using
    the same quantized Q1.15 input vector."""

    max_error = 0.0
    sector_mismatches = 0

    for angle in [
        i * math.pi / 180.0
        for i in range(0, 360, 3)
    ]:

        magnitude = 0.5

        # ----------------------------------------------------
        # Generate the original floating-point vector.
        # ----------------------------------------------------

        v_alpha = (
            magnitude *
            math.cos(angle)
        )

        v_beta = (
            magnitude *
            math.sin(angle)
        )

        # ----------------------------------------------------
        # Quantize the inputs exactly as the fixed-point
        # implementation receives them.
        # ----------------------------------------------------

        v_alpha_q15 = float_to_q15(v_alpha)
        v_beta_q15 = float_to_q15(v_beta)

        # Convert the quantized values back to float.
        # This gives the float reference exactly the same
        # vector seen by the fixed-point implementation.
        v_alpha_quantized = q15_to_float(
            v_alpha_q15
        )

        v_beta_quantized = q15_to_float(
            v_beta_q15
        )

        # ----------------------------------------------------
        # Reference using quantized inputs.
        # ----------------------------------------------------

        float_result = svpwm_float(
            v_alpha_quantized,
            v_beta_quantized
        )

        # ----------------------------------------------------
        # Fixed-point implementation.
        # ----------------------------------------------------

        fixed_result = svpwm_fixed(
            v_alpha_q15,
            v_beta_q15
        )

        # ----------------------------------------------------
        # Sector must now match because both paths see the
        # exact same quantized alpha-beta vector.
        # ----------------------------------------------------

        assert (
            fixed_result.sector ==
            float_result["sector"]
        ), (
            f"Sector mismatch at "
            f"{math.degrees(angle):.1f} deg: "
            f"fixed={fixed_result.sector}, "
            f"float={float_result['sector']}, "
            f"Valpha={v_alpha_q15}, "
            f"Vbeta={v_beta_q15}"
        )

        # ----------------------------------------------------
        # Compare duty cycles.
        # ----------------------------------------------------

        fixed_duties = [
            duty_q15_to_float(
                fixed_result.Ta_q15
            ),
            duty_q15_to_float(
                fixed_result.Tb_q15
            ),
            duty_q15_to_float(
                fixed_result.Tc_q15
            ),
        ]

        float_duties = [
            float_result["Ta"],
            float_result["Tb"],
            float_result["Tc"],
        ]

        for fixed, reference in zip(
            fixed_duties,
            float_duties
        ):

            error = abs(
                fixed -
                reference
            )

            max_error = max(
                max_error,
                error
            )

            assert error <= 2.0 / Q15_MAX, (
                f"Fixed/float mismatch: "
                f"angle={math.degrees(angle):.1f} deg, "
                f"fixed={fixed}, "
                f"float={reference}, "
                f"error={error}"
            )

    print(
        "Fixed vs float SVPWM test: PASS "
        f"(max error={max_error:.8f})"
    )


def test_switching_table():
    """Verify all six switching states."""

    assert len(SWITCHING_TABLE) == 6

    for sector in range(1, 7):

        states = get_switching_states(
            sector
        )

        assert "active_1" in states
        assert "active_2" in states
        assert "zero" in states

        for state_name in [
            "active_1",
            "active_2",
            "zero"
        ]:

            state = states[state_name]

            assert len(state) == 3

            for bit in state:
                assert bit in (0, 1)

    print("Switching-table test: PASS")


def test_complementary_switching():
    """Verify complementary switch generation."""

    test_states = [
        (0, 0, 0),
        (1, 0, 0),
        (1, 1, 0),
        (0, 1, 1),
        (1, 0, 1),
        (1, 1, 1),
    ]

    for high_state in test_states:

        low_state = generate_complementary_state(
            high_state
        )

        for high, low in zip(
            high_state,
            low_state
        ):

            assert not (
                high == 1 and
                low == 1
            )

            assert (
                low ==
                1 - high
            )

    print("Complementary-switch test: PASS")


def test_dead_time_no_shoot_through():
    """
    Verify the dead-time channel never produces simultaneous
    high-side and low-side conduction.
    """

    channel = DeadTimeChannel(
        dead_cycles=2
    )

    channel.reset()

    requests = [
        0,
        0,
        1,
        1,
        1,
        0,
        0,
        0,
        1,
        0,
        1,
        0,
    ]

    for request in requests:

        high, low = channel.update(
            request
        )

        assert not (
            high and low
        ), (
            "Shoot-through detected"
        )

    print("Dead-time no-shoot-through test: PASS")


def test_sector_boundaries():
    """
    Verify exact 60-degree sector boundaries.

    Boundary convention:
        0°   -> Sector 1
        60°  -> Sector 2
        120° -> Sector 3
        180° -> Sector 4
        240° -> Sector 5
        300° -> Sector 6
    """

    boundaries = [
        (0.0, 1),
        (60.0, 2),
        (120.0, 3),
        (180.0, 4),
        (240.0, 5),
        (300.0, 6),
    ]

    for degrees, expected in boundaries:

        sector = sector_from_angle(
            math.radians(degrees)
        )

        assert sector == expected, (
            f"Boundary failure: "
            f"{degrees} deg -> "
            f"{sector}, expected {expected}"
        )

    print("Sector-boundary test: PASS")


# ============================================================
# Run all tests
# ============================================================

def run_self_tests():

    print("=" * 60)
    print("SVPWM SELF-TEST")
    print("=" * 60)

    test_sector_detection()
    test_alpha_beta_sector_detection()
    test_zero_vector()
    test_dwell_time_sum()
    test_duty_range()
    test_all_six_sectors()
    test_overmodulation()
    test_fixed_point_svpwm()
    test_fixed_vs_float()
    test_switching_table()
    test_complementary_switching()
    test_dead_time_no_shoot_through()
    test_sector_boundaries()

    print("=" * 60)
    print("ALL SVPWM TESTS PASS")
    print("=" * 60)


if __name__ == "__main__":
    run_self_tests()