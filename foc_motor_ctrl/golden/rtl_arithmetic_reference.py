
"""
Independent integer arithmetic reference for speed_pi.sv.

Input formats:
    omega_ref_q6 : unsigned Q10.6
    omega_meas_q6: unsigned Q10.6
    kp_q15       : signed Q1.15
    ki_dt_q31    : signed Q1.31

Internal formats:
    P term       : Q1.15
    Integrator   : Q8.24
    I increment  : Q8.24
    iq_ref       : Q1.15

Input CSV:
    Existing foc_pipeline_testvectors.csv
    13 integer columns, NO HEADER.

Columns used:
    0 = omega_ref_q6
    1 = omega_meas_q6

Output:
    Separate speed_pi_rtl_reference.csv

This file does not modify the frozen golden models or RTL.
"""

import csv
import argparse
from pathlib import Path


# ------------------------------------------------------------
# DEFAULT RTL CONSTANTS
# ------------------------------------------------------------

SPEED_KP_Q15 = 1311
SPEED_KI_DT_Q31 = 53687

OUTPUT_MIN = -32768
OUTPUT_MAX = 32767

INTEGRATOR_MIN = -(1 << 31)
INTEGRATOR_MAX = (1 << 31) - 1

# Signed 32-bit range
SIGNED32_MIN = -(1 << 31)
SIGNED32_MAX = (1 << 31) - 1


# ------------------------------------------------------------
# GENERAL INTEGER HELPERS
# ------------------------------------------------------------

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def saturate_signed32(value):
    return clamp(value, SIGNED32_MIN, SIGNED32_MAX)


def round_shift_signed(value, shift):
    """
    Sign-aware rounding followed by arithmetic right shift.

    Positive:
        (value + half) >>> shift

    Negative:
        (value - half) >>> shift

    This follows the signed-rounding convention recorded
    during the RTL arithmetic review.
    """

    if shift <= 0:
        return value << (-shift)

    half = 1 << (shift - 1)

    if value >= 0:
        value = value + half
    else:
        value = value - half

    return value >> shift


def q24_to_q15(value_q24):
    """
    Convert Q8.24 to Q1.15.

    Fractional bit difference = 24 - 15 = 9.
    """

    result = round_shift_signed(value_q24, 9)

    return clamp(result, OUTPUT_MIN, OUTPUT_MAX)


# ------------------------------------------------------------
# SPEED PI RTL INTEGER REFERENCE
# ------------------------------------------------------------

class SpeedPIRTLReference:

    def __init__(
        self,
        kp_q15=SPEED_KP_Q15,
        ki_dt_q31=SPEED_KI_DT_Q31,
        output_min=OUTPUT_MIN,
        output_max=OUTPUT_MAX,
    ):

        self.kp_q15 = int(kp_q15)
        self.ki_dt_q31 = int(ki_dt_q31)

        self.output_min = int(output_min)
        self.output_max = int(output_max)

        self.integrator_q24 = 0

    def reset(self):
        self.integrator_q24 = 0

    def step(self, omega_ref_q6, omega_meas_q6):

        omega_ref_q6 = int(omega_ref_q6)
        omega_meas_q6 = int(omega_meas_q6)

        # ----------------------------------------------------
        # 1. SPEED ERROR
        # ----------------------------------------------------

        # Both inputs are unsigned Q10.6.
        # Their difference is signed.

        error_q6 = omega_ref_q6 - omega_meas_q6

        # ----------------------------------------------------
        # 2. PROPORTIONAL PATH
        # ----------------------------------------------------

        # Q10.6 * Q1.15 = Q11.21
        p_product = error_q6 * self.kp_q15

        # RTL rounding and shift:
        # Q21 -> Q15, shift right by 6.

        p_rounded = round_shift_signed(p_product, 6)

        p_term_q15 = clamp(
            p_rounded,
            OUTPUT_MIN,
            OUTPUT_MAX,
        )

        # ----------------------------------------------------
        # 3. INTEGRAL PATH
        # ----------------------------------------------------

        # Q10.6 * Q1.31 = Q11.37
        i_product = error_q6 * self.ki_dt_q31

        # Q37 -> Q24, shift right by 13.

        i_increment_unclamped = round_shift_signed(
            i_product,
            13,
        )

        # Match signed 32-bit increment saturation.
        i_increment_q24 = saturate_signed32(
            i_increment_unclamped
        )

        # ----------------------------------------------------
        # 4. CANDIDATE INTEGRATOR
        # ----------------------------------------------------

        candidate_integrator_unclamped = (
            self.integrator_q24 + i_increment_q24
        )

        candidate_integrator_q24 = saturate_signed32(
            candidate_integrator_unclamped
        )

        # ----------------------------------------------------
        # 5. CONVERT INTEGRATOR TO Q1.15
        # ----------------------------------------------------

        candidate_i_q15 = q24_to_q15(
            candidate_integrator_q24
        )

        # ----------------------------------------------------
        # 6. UNSATURATED OUTPUT
        # ----------------------------------------------------

        # RTL output calculation uses candidate integrator.

        unsaturated_output_q15 = (
            p_term_q15 + candidate_i_q15
        )

        # ----------------------------------------------------
        # 7. OUTPUT SATURATION
        # ----------------------------------------------------

        output_q15 = clamp(
            unsaturated_output_q15,
            self.output_min,
            self.output_max,
        )

        # ----------------------------------------------------
        # 8. CONDITIONAL INTEGRATION / ANTI-WINDUP
        # ----------------------------------------------------

        upper_saturated = (
            unsaturated_output_q15 > self.output_max
        )

        lower_saturated = (
            unsaturated_output_q15 < self.output_min
        )

        integration_blocked = (
            (upper_saturated and error_q6 > 0)
            or
            (lower_saturated and error_q6 < 0)
        )

        # Store candidate only if integration is allowed.
        if not integration_blocked:
            self.integrator_q24 = candidate_integrator_q24

        # ----------------------------------------------------
        # 9. RETURN ALL INTERMEDIATE RESULTS
        # ----------------------------------------------------

        return {
            "error_q6": error_q6,

            "p_product": p_product,
            "p_term_q15": p_term_q15,

            "i_product": i_product,
            "i_increment_q24": i_increment_q24,

            "candidate_integrator_q24":
                candidate_integrator_q24,

            "candidate_i_q15": candidate_i_q15,

            "unsaturated_output_q15":
                unsaturated_output_q15,

            "iq_ref_q15": output_q15,

            "integration_blocked":
                int(integration_blocked),

            "stored_integrator_q24":
                self.integrator_q24,
        }


# ------------------------------------------------------------
# SVPWM RTL INTEGER REFERENCE
# ------------------------------------------------------------

SVPWM_F_FRAC = 16

SVPWM_ONE_F = 2147418112
SVPWM_K1P5 = 98301
SVPWM_KSQ3_2 = 56754
SVPWM_KSQ3 = 113508
SVPWM_SQ3_SEC = 29058991


def svpwm_duty_from_fixed(value):
    """
    Match duty_from_fixed() in svpwm.sv.

    RTL:
        rounded = (value + 2^(F_FRAC-1)) >>> F_FRAC

    Output range: 0..32767.
    """

    rounded = (
        value + (1 << (SVPWM_F_FRAC - 1))
    ) >> SVPWM_F_FRAC

    return clamp(rounded, 0, 32767)


def svpwm_rtl_reference(v_alpha_q15, v_beta_q15):
    """
    Independent integer implementation of svpwm.sv.

    Inputs:
        v_alpha_q15: signed Q1.15 integer
        v_beta_q15 : signed Q1.15 integer

    Returns:
        sector, duty_a, duty_b, duty_c,
        raw/scaled dwell values and intermediate diagnostics.
    """

    v_alpha_q15 = int(v_alpha_q15)
    v_beta_q15 = int(v_beta_q15)

    if not (-32768 <= v_alpha_q15 <= 32767):
        raise ValueError("v_alpha_q15 is outside signed 16-bit range")

    if not (-32768 <= v_beta_q15 <= 32767):
        raise ValueError("v_beta_q15 is outside signed 16-bit range")

    alpha = v_alpha_q15
    beta = v_beta_q15

    abs_alpha = abs(alpha)
    abs_beta = abs(beta)

    compare_left = abs_beta << 24
    compare_right = abs_alpha * SVPWM_SQ3_SEC

    # --------------------------------------------------------
    # 1. SECTOR DETECTION
    # Directly follows the RTL conditional ordering.
    # --------------------------------------------------------

    if alpha == 0 and beta == 0:
        sector = 1

    elif alpha < 0 and beta == 0:
        sector = 4

    elif alpha == 0 and beta < 0:
        sector = 5

    elif alpha >= 0 and beta >= 0:
        if compare_left < compare_right:
            sector = 1
        else:
            sector = 2

    elif alpha < 0 and beta > 0:
        if compare_left > compare_right:
            sector = 2
        else:
            sector = 3

    elif alpha < 0 and beta < 0:
        if compare_left < compare_right:
            sector = 4
        else:
            sector = 5

    else:
        if compare_left > compare_right:
            sector = 5
        else:
            sector = 6

    # --------------------------------------------------------
    # 2. DWELL CALCULATION
    # Exact integer equations from svpwm.sv.
    # --------------------------------------------------------

    t1_raw = 0
    t2_raw = 0

    if sector == 1:
        t1_raw = (
            alpha * SVPWM_K1P5
            - beta * SVPWM_KSQ3_2
        )
        t2_raw = beta * SVPWM_KSQ3

    elif sector == 2:
        t1_raw = (
            alpha * SVPWM_K1P5
            + beta * SVPWM_KSQ3_2
        )
        t2_raw = (
            beta * SVPWM_KSQ3_2
            - alpha * SVPWM_K1P5
        )

    elif sector == 3:
        t1_raw = beta * SVPWM_KSQ3
        t2_raw = (
            -alpha * SVPWM_K1P5
            - beta * SVPWM_KSQ3_2
        )

    elif sector == 4:
        t1_raw = (
            -alpha * SVPWM_K1P5
            + beta * SVPWM_KSQ3_2
        )
        t2_raw = -beta * SVPWM_KSQ3

    elif sector == 5:
        t1_raw = (
            -alpha * SVPWM_K1P5
            - beta * SVPWM_KSQ3_2
        )
        t2_raw = (
            alpha * SVPWM_K1P5
            - beta * SVPWM_KSQ3_2
        )

    else:  # Sector 6
        t1_raw = -beta * SVPWM_KSQ3
        t2_raw = (
            alpha * SVPWM_K1P5
            + beta * SVPWM_KSQ3_2
        )

    # RTL clamps negative dwell values to zero.
    if t1_raw < 0:
        t1_raw = 0

    if t2_raw < 0:
        t2_raw = 0

    # --------------------------------------------------------
    # 3. ACTIVE DWELL AND OVERMODULATION
    # --------------------------------------------------------

    active_raw = t1_raw + t2_raw

    if active_raw > SVPWM_ONE_F:

        # SystemVerilog signed division truncates toward zero.
        # All operands here are nonnegative.
        t1_scaled = (
            t1_raw * SVPWM_ONE_F
            + (active_raw >> 1)
        ) // active_raw

        t2_scaled = SVPWM_ONE_F - t1_scaled

        active_raw = SVPWM_ONE_F

    else:
        t1_scaled = t1_raw
        t2_scaled = t2_raw

    # --------------------------------------------------------
    # 4. ZERO VECTOR TIME
    # --------------------------------------------------------

    t0_value = SVPWM_ONE_F - active_raw
    half_t0 = (t0_value + 1) >> 1

    # --------------------------------------------------------
    # 5. SYMMETRIC SEVEN-SEGMENT DUTY ASSIGNMENT
    # --------------------------------------------------------

    if sector == 1:
        ta_value = half_t0 + t1_scaled + t2_scaled
        tb_value = half_t0 + t2_scaled
        tc_value = half_t0

    elif sector == 2:
        ta_value = half_t0 + t1_scaled
        tb_value = half_t0 + t1_scaled + t2_scaled
        tc_value = half_t0

    elif sector == 3:
        ta_value = half_t0
        tb_value = half_t0 + t1_scaled + t2_scaled
        tc_value = half_t0 + t2_scaled

    elif sector == 4:
        ta_value = half_t0
        tb_value = half_t0 + t1_scaled
        tc_value = half_t0 + t1_scaled + t2_scaled

    elif sector == 5:
        ta_value = half_t0 + t2_scaled
        tb_value = half_t0
        tc_value = half_t0 + t1_scaled + t2_scaled

    else:
        ta_value = half_t0 + t1_scaled + t2_scaled
        tb_value = half_t0
        tc_value = half_t0 + t1_scaled

    # --------------------------------------------------------
    # 6. FINAL DUTY CONVERSION
    # --------------------------------------------------------

    duty_a = svpwm_duty_from_fixed(ta_value)
    duty_b = svpwm_duty_from_fixed(tb_value)
    duty_c = svpwm_duty_from_fixed(tc_value)

    return {
        "sector": sector,

        "duty_a": duty_a,
        "duty_b": duty_b,
        "duty_c": duty_c,

        "t1_raw": t1_raw,
        "t2_raw": t2_raw,

        "t1_scaled": t1_scaled,
        "t2_scaled": t2_scaled,

        "t0_value": t0_value,
        "half_t0": half_t0,

        "active_raw": active_raw,
    }

# ------------------------------------------------------------
# SELF TEST
# ------------------------------------------------------------


def self_test():

    # --------------------------------------------------------
    # SPEED PI TESTS
    # --------------------------------------------------------

    controller = SpeedPIRTLReference()

    result = controller.step(640, 640)

    assert result["error_q6"] == 0
    assert result["p_term_q15"] == 0
    assert result["i_increment_q24"] == 0
    assert result["iq_ref_q15"] == 0
    assert result["stored_integrator_q24"] == 0

    controller.reset()
    result = controller.step(640, 0)

    assert result["error_q6"] == 640
    assert result["p_term_q15"] >= 0
    assert result["iq_ref_q15"] >= 0

    controller.reset()
    result = controller.step(0, 640)

    assert result["error_q6"] == -640
    assert result["p_term_q15"] <= 0
    assert result["iq_ref_q15"] <= 0

    assert clamp(40000, OUTPUT_MIN, OUTPUT_MAX) == OUTPUT_MAX
    assert clamp(-40000, OUTPUT_MIN, OUTPUT_MAX) == OUTPUT_MIN

    assert saturate_signed32(1 << 40) == SIGNED32_MAX
    assert saturate_signed32(-(1 << 40)) == SIGNED32_MIN

    # --------------------------------------------------------
    # SVPWM TESTS
    # --------------------------------------------------------

    sector_vectors = [
        (20000, 10000, 1),
        (10000, 20000, 2),
        (-20000, 10000, 3),
        (-20000, -10000, 4),
        (-10000, -20000, 5),
        (20000, -10000, 6),
    ]

    for alpha, beta, expected_sector in sector_vectors:

        result = svpwm_rtl_reference(alpha, beta)

        assert result["sector"] == expected_sector, (
            f"Sector mismatch for ({alpha}, {beta}): "
            f"expected {expected_sector}, got {result['sector']}"
        )

        assert 0 <= result["duty_a"] <= 32767
        assert 0 <= result["duty_b"] <= 32767
        assert 0 <= result["duty_c"] <= 32767

    # Zero vector: sector 1, centered duties.
    zero_result = svpwm_rtl_reference(0, 0)

    assert zero_result["sector"] == 1
    assert zero_result["duty_a"] == 16384
    assert zero_result["duty_b"] == 16384
    assert zero_result["duty_c"] == 16384

    # Verify duty range for several large vectors.
    large_vectors = [
        (32767, 0),
        (0, 32767),
        (-32768, 0),
        (0, -32768),
        (32767, 32767),
        (-32768, 32767),
        (-32768, -32768),
        (32767, -32768),
    ]

    for alpha, beta in large_vectors:

        result = svpwm_rtl_reference(alpha, beta)

        assert 1 <= result["sector"] <= 6

        assert 0 <= result["duty_a"] <= 32767
        assert 0 <= result["duty_b"] <= 32767
        assert 0 <= result["duty_c"] <= 32767

    print("SELF-TEST: PASS")
    print()
    print("SPEED PI")
    print("  Zero-error test       : PASS")
    print("  Positive-error test   : PASS")
    print("  Negative-error test   : PASS")
    print("  Output saturation     : PASS")
    print("  Integrator saturation : PASS")
    print()
    print("SVPWM")
    print("  Six-sector test       : PASS")
    print("  Zero-vector test      : PASS")
    print("  Duty-range tests      : PASS")
    print()
    print("Note: These are Python reference tests,")
    print("not an RTL simulation or RTL comparison.")


# ------------------------------------------------------------
# CSV GENERATION
# ------------------------------------------------------------

def run_csv(input_path, output_path, kp_q15, ki_dt_q31):

    input_path = Path(input_path)
    output_path = Path(output_path)

    controller = SpeedPIRTLReference(
        kp_q15=kp_q15,
        ki_dt_q31=ki_dt_q31,
    )

    # Read existing headerless 13-column test-vector CSV.

    with open(input_path, "r", newline="") as source:

        reader = csv.reader(source)
        rows = list(reader)

    if not rows:
        raise ValueError("Input CSV is empty.")

    # Validate all input rows before generating output.

    for index, row in enumerate(rows):

        if len(row) != 13:
            raise ValueError(
                f"Vector {index}: expected 13 columns, "
                f"but found {len(row)}."
            )

        try:
            int(row[0])
            int(row[1])
        except ValueError:
            raise ValueError(
                f"Vector {index}: speed columns must be integers."
            )

    output_fields = [
        "vector",
        "omega_ref_q6",
        "omega_meas_q6",
        "error_q6",
        "p_product",
        "p_term_q15",
        "i_product",
        "i_increment_q24",
        "candidate_integrator_q24",
        "candidate_i_q15",
        "unsaturated_output_q15",
        "iq_ref_q15",
        "integration_blocked",
        "stored_integrator_q24",
    ]

    controller.reset()

    with open(output_path, "w", newline="") as destination:

        writer = csv.DictWriter(
            destination,
            fieldnames=output_fields,
        )

        writer.writeheader()

        for index, row in enumerate(rows):

            omega_ref_q6 = int(row[0])
            omega_meas_q6 = int(row[1])

            result = controller.step(
                omega_ref_q6,
                omega_meas_q6,
            )

            writer.writerow({
                "vector": index,

                "omega_ref_q6": omega_ref_q6,
                "omega_meas_q6": omega_meas_q6,

                "error_q6": result["error_q6"],

                "p_product": result["p_product"],
                "p_term_q15": result["p_term_q15"],

                "i_product": result["i_product"],
                "i_increment_q24":
                    result["i_increment_q24"],

                "candidate_integrator_q24":
                    result["candidate_integrator_q24"],

                "candidate_i_q15":
                    result["candidate_i_q15"],

                "unsaturated_output_q15":
                    result["unsaturated_output_q15"],

                "iq_ref_q15":
                    result["iq_ref_q15"],

                "integration_blocked":
                    result["integration_blocked"],

                "stored_integrator_q24":
                    result["stored_integrator_q24"],
            })

    print()
    print("REFERENCE GENERATION COMPLETE")
    print("--------------------------------")
    print(f"Input vectors : {len(rows)}")
    print(f"Input file    : {input_path}")
    print(f"Output file   : {output_path}")
    print(f"Kp Q1.15      : {kp_q15}")
    print(f"Ki*dt Q1.31   : {ki_dt_q31}")
    print("--------------------------------")


# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Independent integer speed PI reference"
    )

    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Input 13-column headerless pipeline CSV",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output reference CSV",
    )

    parser.add_argument(
        "--kp",
        type=int,
        default=SPEED_KP_Q15,
        help="Speed Kp in Q1.15",
    )

    parser.add_argument(
        "--ki-dt",
        type=int,
        default=SPEED_KI_DT_Q31,
        help="Speed Ki*dt in Q1.31",
    )

    args = parser.parse_args()

    if args.input is None:

        self_test()

    else:

        if args.output is None:
            parser.error(
                "--output is required when --input is specified"
            )

        run_csv(
            args.input,
            args.output,
            args.kp,
            args.ki_dt,
        )
