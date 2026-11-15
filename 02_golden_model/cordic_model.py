"""
cordic_model.py
===============
Fixed-point CORDIC golden model for the FOC motor controller.

Project-wide external angle convention:
    unsigned 16-bit full-circle Q2.14

    0 deg   = 0x0000
    90 deg  = 0x2000 = 8192
    180 deg = 0x4000 = 16384
    270 deg = 0x6000 = 24576
    360 deg = 0x8000, which wraps to 0x0000

External sin/cos:
    signed Q1.15

Internal CORDIC angle representation remains signed
Q2.14 in [-pi, +pi).
"""

import csv
import math
import os

import numpy as np


# ============================================================================
# CONSTANTS
# ============================================================================

N_STAGES = 16

# ---------------------------------------------------------------------------
# External Q formats
# ---------------------------------------------------------------------------

Q15_FRAC = 15
Q15_ONE = 1 << Q15_FRAC

Q214_FRAC = 14
Q214_PI = 1 << Q214_FRAC              # 16384
Q214_HALF_PI = 1 << 13                # 8192
Q214_TWO_PI = 1 << 15                 # 32768


# ---------------------------------------------------------------------------
# Internal precision
# ---------------------------------------------------------------------------

XY_INTERNAL_FRAC = 20
ANGLE_INTERNAL_FRAC = 22

XY_INTERNAL_ONE = 1 << XY_INTERNAL_FRAC


# ============================================================================
# CORDIC GAIN
# ============================================================================

# CORDIC rotation gain:
#
#     G = product(sqrt(1 + 2^(-2i)))
#
# Its inverse is:
#
#     K = product(cos(atan(2^-i)))
#
#     K ~= 0.607252935...
#
# Rotation-mode CORDIC produces:
#
#     output magnitude = input magnitude / K
#
# Therefore the initial X value must be K.

_K = 1.0

for _i in range(N_STAGES):
    _K *= math.cos(math.atan(2.0 ** (-_i)))

K_CORDIC = _K
K_INV_FLOAT = 1.0 / K_CORDIC

K_INTERNAL = round(
    K_CORDIC * XY_INTERNAL_ONE
)


# ============================================================================
# ANGLE TABLES
# ============================================================================

# Internal high-precision angle table.
#
# Internal angle representation:
#
#     pi = 2^ANGLE_INTERNAL_FRAC

ATAN_TABLE_INTERNAL = [
    round(
        math.atan(2.0 ** (-i))
        / math.pi
        * (1 << ANGLE_INTERNAL_FRAC)
    )
    for i in range(N_STAGES)
]


# External Q2.14 table.
ATAN_TABLE_Q214 = [
    round(
        math.atan(2.0 ** (-i))
        / math.pi
        * Q214_PI
    )
    for i in range(N_STAGES)
]


# Floating-point table.
ATAN_TABLE_RAD = [
    math.atan(2.0 ** (-i))
    for i in range(N_STAGES)
]


# ============================================================================
# BASIC HELPERS
# ============================================================================

def _sat16(x: int) -> int:
    """
    Saturate to signed 16-bit range.

    Q1.15:
        -32768 -> -1.0
         32767 -> +0.999969...
    """

    if x > 32767:
        return 32767

    if x < -32768:
        return -32768

    return int(x)


def _sat32(x: int) -> int:
    """
    Saturate to signed 32-bit range.
    """

    if x > 2147483647:
        return 2147483647

    if x < -2147483648:
        return -2147483648

    return int(x)


def _round_shift(value: int, shift: int) -> int:
    """
    Arithmetic right shift with symmetric round-to-nearest.
    """

    value = int(value)

    if shift <= 0:
        return value << (-shift)

    half = 1 << (shift - 1)

    if value >= 0:
        return (value + half) >> shift

    return -((-value + half) >> shift)


def _q15_to_float(value: int) -> float:
    """
    Convert signed Q1.15 integer to float.
    """

    return int(value) / float(Q15_ONE)


def _float_to_q15(value: float) -> int:
    """
    Convert float to saturated signed Q1.15.
    """

    value = max(
        -1.0,
        min(1.0, value)
    )

    return _sat16(
        round(value * Q15_ONE)
    )


# ============================================================================
# ANGLE CONVERSION HELPERS
# ============================================================================

def _wrap_angle_q214(theta_q214: int) -> int:
    """
    Wrap an angle into the INTERNAL signed Q2.14 range:

        [-pi, +pi)

    This is an internal helper only.
    """

    x = int(theta_q214) % Q214_TWO_PI

    if x >= Q214_PI:
        x -= Q214_TWO_PI

    return x


def _unsigned_q214_to_signed(theta_uq214: int) -> int:
    """
    Convert EXTERNAL unsigned full-circle Q2.14
    into INTERNAL signed Q2.14.

    External:

        0x0000 ->   0 deg
        0x2000 ->  90 deg
        0x4000 -> 180 deg
        0x6000 -> 270 deg

    Internal:

        0x0000 ->     0
        0x2000 ->  +8192
        0x4000 -> -16384
        0x6000 ->  -8192
    """

    x = int(theta_uq214) & 0xFFFF

    x %= Q214_TWO_PI

    if x >= Q214_PI:
        x -= Q214_TWO_PI

    return int(x)


def _signed_q214_to_unsigned(theta_q214: int) -> int:
    """
    Convert INTERNAL signed Q2.14
    into EXTERNAL unsigned full-circle Q2.14.
    """

    x = _wrap_angle_q214(theta_q214)

    if x < 0:
        x += Q214_TWO_PI

    return int(x)


def _signed_q214_to_rad(theta_q214: int) -> float:
    """
    Convert INTERNAL signed Q2.14 angle to radians.
    """

    theta = _wrap_angle_q214(theta_q214)

    return (
        theta
        / float(Q214_PI)
        * math.pi
    )


def _uq214_to_rad(theta_uq214: int) -> float:
    """
    Convert EXTERNAL unsigned full-circle Q2.14
    to radians in [0, 2*pi).
    """

    theta = int(theta_uq214) & 0xFFFF

    theta %= Q214_TWO_PI

    return (
        theta
        / float(Q214_PI)
        * math.pi
    )


def _rad_to_signed_q214(theta_rad: float) -> int:
    """
    Convert radians to INTERNAL signed Q2.14.
    """

    theta = float(theta_rad) % (
        2.0 * math.pi
    )

    q = round(
        theta
        / (2.0 * math.pi)
        * Q214_TWO_PI
    )

    return _unsigned_q214_to_signed(q)


def _rad_to_uq214(theta_rad: float) -> int:
    """
    Convert radians to EXTERNAL unsigned
    full-circle Q2.14.
    """

    theta = float(theta_rad) % (
        2.0 * math.pi
    )

    q = round(
        theta
        / (2.0 * math.pi)
        * Q214_TWO_PI
    )

    return int(q % Q214_TWO_PI)


# ---------------------------------------------------------------------------
# Backward-compatible helper names
# ---------------------------------------------------------------------------

def _q214_to_rad(theta_q214: int) -> float:
    """
    Backward-compatible helper.

    Interprets the supplied value as INTERNAL signed Q2.14.
    """

    return _signed_q214_to_rad(theta_q214)


def _rad_to_q214(theta_rad: float) -> int:
    """
    Backward-compatible helper.

    Returns INTERNAL signed Q2.14.
    """

    return _rad_to_signed_q214(theta_rad)


def circular_angle_error_q214(a: int, b: int) -> int:
    """
    Circular signed angle error in Q2.14.

    Works for both unsigned external values and signed
    internal values because the subtraction is wrapped.
    """

    return _wrap_angle_q214(
        int(a) - int(b)
    )


def to_hex16(value: int) -> str:
    """
    Format a value as a 16-bit hexadecimal word.
    """

    return "{:04X}".format(
        int(value) & 0xFFFF
    )


# ============================================================================
# FLOATING-POINT CORDIC ROTATION
# ============================================================================

def cordic_rotation_float(theta: float):
    """
    Floating-point CORDIC rotation mode.

    Parameters
    ----------
    theta : float
        Angle in radians.

    Returns
    -------
    cos_theta, sin_theta
    """

    theta = (
        (float(theta) + math.pi)
        % (2.0 * math.pi)
    ) - math.pi

    z = theta

    negate = False

    # Quadrant pre-rotation.
    if z > math.pi / 2.0:
        z -= math.pi
        negate = True

    elif z < -math.pi / 2.0:
        z += math.pi
        negate = True

    # Gain-compensated initial vector.
    x = K_CORDIC
    y = 0.0

    for i in range(N_STAGES):

        if z >= 0.0:
            d = 1.0
        else:
            d = -1.0

        x_new = (
            x
            - d * y * (2.0 ** (-i))
        )

        y_new = (
            y
            + d * x * (2.0 ** (-i))
        )

        z_new = (
            z
            - d * ATAN_TABLE_RAD[i]
        )

        x = x_new
        y = y_new
        z = z_new

    if negate:
        x = -x
        y = -y

    return x, y


# ============================================================================
# FLOATING-POINT CORDIC ATAN2
# ============================================================================

def cordic_atan2_float(x: float, y: float) -> float:
    """
    Floating-point CORDIC vectoring-mode atan2.

    Returns:
        angle in radians in [-pi, +pi)
    """

    if x == 0.0 and y == 0.0:
        return 0.0

    angle_offset = 0.0

    if x < 0.0:

        x = -x
        y = -y

        if y >= 0.0:
            angle_offset = math.pi
        else:
            angle_offset = -math.pi

    z = 0.0

    for i in range(N_STAGES):

        if y >= 0.0:
            d = -1.0
        else:
            d = 1.0

        x_new = (
            x
            - d * y * (2.0 ** (-i))
        )

        y_new = (
            y
            + d * x * (2.0 ** (-i))
        )

        z_new = (
            z
            - d * ATAN_TABLE_RAD[i]
        )

        x = x_new
        y = y_new
        z = z_new

    result = z + angle_offset

    result = (
        (result + math.pi)
        % (2.0 * math.pi)
    ) - math.pi

    return result


# ============================================================================
# FIXED-POINT CORDIC ROTATION
# ============================================================================

def cordic_rotation_fixed(theta_q214: int):
    """
    Fixed-point 16-stage CORDIC rotation matched bit-for-bit to cordic.sv.

    EXTERNAL INPUT:
        theta_q214 : unsigned full-circle Q2.14

    EXTERNAL OUTPUT:
        cos_q15 : signed Q1.15
        sin_q15 : signed Q1.15

    RTL-matched arithmetic:
        * quadrant reduction is identical to cordic.sv
        * internal X/Y shifts use arithmetic truncation (>>> / >>)
        * final Q20->Q15 conversion is (+16) followed by arithmetic >> 5
        * quadrant signs are applied AFTER final magnitude quantization
    """

    theta_q214 = int(theta_q214) & 0xFFFF

    # ------------------------------------------------------------------------
    # Match the RTL quadrant reduction exactly.
    # ------------------------------------------------------------------------
    if theta_q214 == 0x8000:
        quadrant = 0
        reduced_angle_q214 = 0
    elif theta_q214 < 0x2000:
        quadrant = 0
        reduced_angle_q214 = theta_q214
    elif theta_q214 < 0x4000:
        quadrant = 1
        reduced_angle_q214 = 0x4000 - theta_q214
    elif theta_q214 < 0x6000:
        quadrant = 2
        reduced_angle_q214 = theta_q214 - 0x4000
    else:
        quadrant = 3
        reduced_angle_q214 = 0x8000 - theta_q214

    # Q2.14 -> internal angle Q?.22.
    z = reduced_angle_q214 << (ANGLE_INTERNAL_FRAC - Q214_FRAC)

    # Initial vector: K, 0.
    x = K_INTERNAL
    y = 0

    # ------------------------------------------------------------------------
    # 16 CORDIC stages. Python >> is an arithmetic right shift, matching
    # signed SystemVerilog >>> for these integer values.
    # ------------------------------------------------------------------------
    for i in range(N_STAGES):
        if z >= 0:
            x_shift = y >> i
            y_shift = x >> i
            x_new = x - x_shift
            y_new = y + y_shift
            z_new = z - ATAN_TABLE_INTERNAL[i]
        else:
            x_shift = y >> i
            y_shift = x >> i
            x_new = x + x_shift
            y_new = y - y_shift
            z_new = z + ATAN_TABLE_INTERNAL[i]

        x = x_new
        y = y_new
        z = z_new

    # ------------------------------------------------------------------------
    # Match RTL q20_to_q15_sat() exactly BEFORE applying quadrant signs.
    # ------------------------------------------------------------------------
    def rtl_q20_to_q15(value: int) -> int:
        shifted = (int(value) + 16) >> 5
        if shifted > 32767:
            return 32767
        if shifted < -32768:
            return -32768
        return shifted

    cos_mag = rtl_q20_to_q15(x)
    sin_mag = rtl_q20_to_q15(y)

    # Match the RTL output quadrant case statement.
    if quadrant == 0:
        cos_q15 = cos_mag
        sin_q15 = sin_mag
    elif quadrant == 1:
        cos_q15 = -cos_mag
        sin_q15 = sin_mag
    elif quadrant == 2:
        cos_q15 = -cos_mag
        sin_q15 = -sin_mag
    else:
        cos_q15 = cos_mag
        sin_q15 = -sin_mag

    return _sat16(cos_q15), _sat16(sin_q15)


# ============================================================================
# FIXED-POINT CORDIC ATAN2
# ============================================================================

def cordic_atan2_fixed(
    x_q15: int,
    y_q15: int
) -> int:
    """
    Fixed-point 16-stage vectoring-mode CORDIC.

    Inputs:
        x_q15 : signed Q1.15
        y_q15 : signed Q1.15

    Output:
        angle_q214 : unsigned full-circle Q2.14
    """

    x = int(x_q15)
    y = int(y_q15)

    if x == 0 and y == 0:
        return 0

    # ------------------------------------------------------------------------
    # Convert X/Y to internal precision.
    # ------------------------------------------------------------------------

    xy_shift = (
        XY_INTERNAL_FRAC
        - Q15_FRAC
    )

    x <<= xy_shift
    y <<= xy_shift

    # ------------------------------------------------------------------------
    # Quadrant correction.
    # ------------------------------------------------------------------------

    angle_offset = 0

    if x < 0:

        x = -x
        y = -y

        if y >= 0:
            angle_offset = Q214_PI
        else:
            angle_offset = -Q214_PI

    # ------------------------------------------------------------------------
    # Convert angle offset to internal units.
    # ------------------------------------------------------------------------

    angle_shift = (
        ANGLE_INTERNAL_FRAC
        - Q214_FRAC
    )

    z = 0

    # ------------------------------------------------------------------------
    # Vectoring-mode CORDIC.
    # ------------------------------------------------------------------------

    for i in range(N_STAGES):

        if y >= 0:
            d = -1
        else:
            d = 1

        x_shift = _round_shift(
            y,
            i
        )

        y_shift = _round_shift(
            x,
            i
        )

        x_new = (
            x
            - d * x_shift
        )

        y_new = (
            y
            + d * y_shift
        )

        z_new = (
            z
            - d * ATAN_TABLE_INTERNAL[i]
        )

        x = x_new
        y = y_new
        z = z_new

    # ------------------------------------------------------------------------
    # Add quadrant offset.
    # ------------------------------------------------------------------------

    z += (
        angle_offset
        << angle_shift
    )

    # ------------------------------------------------------------------------
    # Convert internal angle back to signed Q2.14.
    # ------------------------------------------------------------------------

    angle_q214_signed = _round_shift(
        z,
        angle_shift
    )

    angle_q214_signed = _wrap_angle_q214(
        angle_q214_signed
    )

    # ------------------------------------------------------------------------
    # Convert signed internal representation to the
    # project-wide unsigned full-circle representation.
    # ------------------------------------------------------------------------

    return _signed_q214_to_unsigned(
        angle_q214_signed
    )


# ============================================================================
# ERROR ANALYSIS
# ============================================================================

def cordic_error_analysis():
    """
    Perform 1024-point CORDIC accuracy analysis.

    Reports:

        A) Continuous-angle error
        B) Quantized-angle implementation error
    """

    N_TEST = 1024

    continuous_cos_errors = []
    continuous_sin_errors = []

    quantized_cos_errors = []
    quantized_sin_errors = []

    for k in range(N_TEST):

        # --------------------------------------------------------------------
        # Original continuous angle.
        # --------------------------------------------------------------------

        theta_rad = (
            -math.pi
            + 2.0 * math.pi * k / N_TEST
        )

        # --------------------------------------------------------------------
        # Quantize into EXTERNAL unsigned Q2.14.
        # --------------------------------------------------------------------

        theta_uq214 = _rad_to_uq214(
            theta_rad
        )

        # --------------------------------------------------------------------
        # Convert the actual quantized external angle back to radians.
        # --------------------------------------------------------------------

        theta_quantized_rad = _uq214_to_rad(
            theta_uq214
        )

        # --------------------------------------------------------------------
        # CORDIC.
        # --------------------------------------------------------------------

        cos_q15, sin_q15 = (
            cordic_rotation_fixed(
                theta_uq214
            )
        )

        # --------------------------------------------------------------------
        # Continuous reference.
        # --------------------------------------------------------------------

        cos_cont_ref = (
            math.cos(theta_rad)
            * Q15_ONE
        )

        sin_cont_ref = (
            math.sin(theta_rad)
            * Q15_ONE
        )

        continuous_cos_errors.append(
            abs(
                cos_q15
                - cos_cont_ref
            )
        )

        continuous_sin_errors.append(
            abs(
                sin_q15
                - sin_cont_ref
            )
        )

        # --------------------------------------------------------------------
        # Quantized-angle reference.
        # --------------------------------------------------------------------

        cos_quant_ref = _float_to_q15(
            math.cos(theta_quantized_rad)
        )

        sin_quant_ref = _float_to_q15(
            math.sin(theta_quantized_rad)
        )

        quantized_cos_errors.append(
            abs(
                cos_q15
                - cos_quant_ref
            )
        )

        quantized_sin_errors.append(
            abs(
                sin_q15
                - sin_quant_ref
            )
        )

    continuous_cos_errors = np.asarray(
        continuous_cos_errors,
        dtype=float
    )

    continuous_sin_errors = np.asarray(
        continuous_sin_errors,
        dtype=float
    )

    quantized_cos_errors = np.asarray(
        quantized_cos_errors,
        dtype=float
    )

    quantized_sin_errors = np.asarray(
        quantized_sin_errors,
        dtype=float
    )

    results = {

        "continuous_max_cos_err_lsb":
            float(
                np.max(
                    continuous_cos_errors
                )
            ),

        "continuous_max_sin_err_lsb":
            float(
                np.max(
                    continuous_sin_errors
                )
            ),

        "continuous_rms_cos_err_lsb":
            float(
                np.sqrt(
                    np.mean(
                        continuous_cos_errors ** 2
                    )
                )
            ),

        "continuous_rms_sin_err_lsb":
            float(
                np.sqrt(
                    np.mean(
                        continuous_sin_errors ** 2
                    )
                )
            ),

        "cordic_max_cos_err_lsb":
            float(
                np.max(
                    quantized_cos_errors
                )
            ),

        "cordic_max_sin_err_lsb":
            float(
                np.max(
                    quantized_sin_errors
                )
            ),

        "cordic_rms_cos_err_lsb":
            float(
                np.sqrt(
                    np.mean(
                        quantized_cos_errors ** 2
                    )
                )
            ),

        "cordic_rms_sin_err_lsb":
            float(
                np.sqrt(
                    np.mean(
                        quantized_sin_errors ** 2
                    )
                )
            ),
    }

    print()
    print("=" * 72)

    print(
        "CORDIC FIXED-POINT ERROR ANALYSIS "
        "(16 stages, {} test points)".format(
            N_TEST
        )
    )

    print("=" * 72)

    print()
    print("A) CONTINUOUS-ANGLE COMPARISON")
    print("--------------------------------")

    print(
        "Max |cos error| : {:.4f} LSB".format(
            results[
                "continuous_max_cos_err_lsb"
            ]
        )
    )

    print(
        "Max |sin error| : {:.4f} LSB".format(
            results[
                "continuous_max_sin_err_lsb"
            ]
        )
    )

    print(
        "RMS |cos error| : {:.4f} LSB".format(
            results[
                "continuous_rms_cos_err_lsb"
            ]
        )
    )

    print(
        "RMS |sin error| : {:.4f} LSB".format(
            results[
                "continuous_rms_sin_err_lsb"
            ]
        )
    )

    print()
    print("B) CORDIC IMPLEMENTATION ERROR")
    print("--------------------------------")

    print(
        "Max |cos error| : {:.4f} LSB".format(
            results[
                "cordic_max_cos_err_lsb"
            ]
        )
    )

    print(
        "Max |sin error| : {:.4f} LSB".format(
            results[
                "cordic_max_sin_err_lsb"
            ]
        )
    )

    print(
        "RMS |cos error| : {:.4f} LSB".format(
            results[
                "cordic_rms_cos_err_lsb"
            ]
        )
    )

    print(
        "RMS |sin error| : {:.4f} LSB".format(
            results[
                "cordic_rms_sin_err_lsb"
            ]
        )
    )

    max_cordic_error = max(
        results[
            "cordic_max_cos_err_lsb"
        ],
        results[
            "cordic_max_sin_err_lsb"
        ]
    )

    print()

    if max_cordic_error <= 1.0:
        print(
            "TARGET (<= 1 LSB): PASS"
        )
    else:
        print(
            "TARGET (<= 1 LSB): FAIL"
        )

    print("=" * 72)

    return results


# ============================================================================
# 360-DEGREE TEST VECTOR GENERATION
# ============================================================================

def generate_testvectors(
    filename: str = "golden/cordic_testvectors.csv"
):
    """
    Generate 360 integer-degree test vectors.

    CSV columns:

        theta_deg
        theta_q214_hex
        cos_q15_hex
        sin_q15_hex
        cos_ref
        sin_ref
    """

    directory = os.path.dirname(
        filename
    )

    if directory:
        os.makedirs(
            directory,
            exist_ok=True
        )

    with open(
        filename,
        "w",
        newline=""
    ) as csvfile:

        writer = csv.writer(
            csvfile
        )

        writer.writerow([
            "theta_deg",
            "theta_q214_hex",
            "cos_q15_hex",
            "sin_q15_hex",
            "cos_ref",
            "sin_ref",
        ])

        for deg in range(360):

            theta_rad = math.radians(
                deg
            )

            # EXTERNAL unsigned Q2.14.
            theta_uq214 = _rad_to_uq214(
                theta_rad
            )

            # Actual quantized angle.
            theta_quantized_rad = _uq214_to_rad(
                theta_uq214
            )

            cos_q15, sin_q15 = (
                cordic_rotation_fixed(
                    theta_uq214
                )
            )

            cos_ref = math.cos(
                theta_quantized_rad
            )

            sin_ref = math.sin(
                theta_quantized_rad
            )

            writer.writerow([
                deg,
                to_hex16(theta_uq214),
                to_hex16(cos_q15),
                to_hex16(sin_q15),
                "{:.8f}".format(cos_ref),
                "{:.8f}".format(sin_ref),
            ])

    print()

    print(
        "Test vectors written to: {}".format(
            filename
        )
    )

    print(
        "  360 vectors, 0-359 degrees"
    )

    print(
        "  unsigned Q2.14 angle / signed Q1.15 cos+sin"
    )


# ============================================================================
# CONSTANT DISPLAY
# ============================================================================

def _print_constants():

    print()
    print("=" * 72)
    print("CORDIC CONSTANTS")
    print("=" * 72)

    print(
        "Stages              : {}".format(
            N_STAGES
        )
    )

    print(
        "External angle      : unsigned full-circle Q2.14"
    )

    print(
        "External sin/cos    : signed Q1.15"
    )

    print(
        "Internal X/Y frac   : {} bits".format(
            XY_INTERNAL_FRAC
        )
    )

    print(
        "Internal angle frac : {} bits".format(
            ANGLE_INTERNAL_FRAC
        )
    )

    print(
        "K                   : {:.15f}".format(
            K_CORDIC
        )
    )

    print(
        "1/K                 : {:.15f}".format(
            K_INV_FLOAT
        )
    )

    print(
        "K internal          : {}".format(
            K_INTERNAL
        )
    )

    print(
        "K Q15 equivalent    : {}".format(
            round(
                K_CORDIC * Q15_ONE
            )
        )
    )

    print()
    print("ATAN TABLE:")

    print(
        "{:>4} {:>12} {:>12} {:>16}".format(
            "i",
            "Q2.14",
            "internal",
            "radians"
        )
    )

    for i in range(N_STAGES):

        print(
            "{:4d} {:12d} {:12d} {:16.10f}".format(
                i,
                ATAN_TABLE_Q214[i],
                ATAN_TABLE_INTERNAL[i],
                ATAN_TABLE_RAD[i]
            )
        )


# ============================================================================
# SELF TEST
# ============================================================================

def _self_test():

    print()
    print("=" * 72)
    print(
        "SELF-TEST: FIXED-POINT CORDIC ROTATION"
    )
    print("=" * 72)

    test_cases = [

        (
            0,
            1.0,
            0.0
        ),

        (
            45,
            math.sqrt(2) / 2,
            math.sqrt(2) / 2
        ),

        (
            90,
            0.0,
            1.0
        ),

        (
            135,
            -math.sqrt(2) / 2,
            math.sqrt(2) / 2
        ),

        (
            180,
            -1.0,
            0.0
        ),

        (
            225,
            -math.sqrt(2) / 2,
            -math.sqrt(2) / 2
        ),

        (
            270,
            0.0,
            -1.0
        ),

        (
            315,
            math.sqrt(2) / 2,
            -math.sqrt(2) / 2
        ),
    ]

    rotation_pass = True

    print(
        "{:>6} {:>12} {:>12} {:>12} {:>12} {:>12}".format(
            "deg",
            "angle",
            "cos",
            "sin",
            "err LSB",
            "status"
        )
    )

    for deg, cos_ref, sin_ref in test_cases:

        theta_rad = math.radians(
            deg
        )

        theta_uq214 = _rad_to_uq214(
            theta_rad
        )

        cos_q15, sin_q15 = (
            cordic_rotation_fixed(
                theta_uq214
            )
        )

        cos_got = _q15_to_float(
            cos_q15
        )

        sin_got = _q15_to_float(
            sin_q15
        )

        err = max(
            abs(
                cos_q15
                - cos_ref * Q15_ONE
            ),
            abs(
                sin_q15
                - sin_ref * Q15_ONE
            )
        )

        status = (
            "PASS"
            if err <= 2.0
            else "FAIL"
        )

        print(
            "{:6d} 0x{:04X} {:12.6f} {:12.6f} {:12.3f} {:>12}".format(
                deg,
                theta_uq214,
                cos_got,
                sin_got,
                err,
                status
            )
        )

        if err > 2.0:
            rotation_pass = False

    print()

    print(
        "Rotation spot-check: {}".format(
            "PASS"
            if rotation_pass
            else "FAIL"
        )
    )

    # ------------------------------------------------------------------------
    # ATAN2 TEST
    # ------------------------------------------------------------------------

    print()
    print("=" * 72)
    print(
        "SELF-TEST: FIXED-POINT CORDIC ATAN2"
    )
    print("=" * 72)

    print(
        "{:>6} {:>12} {:>12} {:>12}".format(
            "deg",
            "ref deg",
            "got deg",
            "err deg"
        )
    )

    atan_pass = True

    for deg, _, _ in test_cases:

        theta_rad = math.radians(
            deg
        )

        x_q15 = _float_to_q15(
            math.cos(theta_rad)
        )

        y_q15 = _float_to_q15(
            math.sin(theta_rad)
        )

        angle_uq214 = (
            cordic_atan2_fixed(
                x_q15,
                y_q15
            )
        )

        angle_got = (
            angle_uq214
            / float(Q214_TWO_PI)
            * 360.0
        )

        ref_uq214 = _rad_to_uq214(
            theta_rad
        )

        err_q214 = abs(
            circular_angle_error_q214(
                angle_uq214,
                ref_uq214
            )
        )

        err_deg = (
            err_q214
            / Q214_PI
            * 180.0
        )

        print(
            "{:6d} {:12.4f} {:12.4f} {:12.4f}".format(
                deg,
                _uq214_to_rad(
                    ref_uq214
                )
                * 180.0
                / math.pi,
                angle_got,
                err_deg
            )
        )

        if err_q214 > 2:
            atan_pass = False

    print()

    print(
        "atan2 spot-check: {}".format(
            "PASS"
            if atan_pass
            else "FAIL"
        )
    )

    # ------------------------------------------------------------------------
    # ARCHITECTURAL BOUNDARY TEST
    # ------------------------------------------------------------------------

    boundary_pass = True

    boundary_cases = [

        (
            0,
            1.0,
            0.0
        ),

        (
            8192,
            0.0,
            1.0
        ),

        (
            16384,
            -1.0,
            0.0
        ),

        (
            24576,
            0.0,
            -1.0
        ),
    ]

    for theta_uq214, cos_ref, sin_ref in boundary_cases:

        cos_q15, sin_q15 = (
            cordic_rotation_fixed(
                theta_uq214
            )
        )

        err = max(
            abs(
                cos_q15
                - cos_ref * Q15_ONE
            ),
            abs(
                sin_q15
                - sin_ref * Q15_ONE
            )
        )

        if err > 2.0:
            boundary_pass = False

    # atan2 cardinal directions.
    if cordic_atan2_fixed(
        Q15_ONE - 1,
        0
    ) != 0:
        boundary_pass = False

    if cordic_atan2_fixed(
        0,
        Q15_ONE - 1
    ) != 8192:
        boundary_pass = False

    if cordic_atan2_fixed(
        -Q15_ONE,
        0
    ) != 16384:
        boundary_pass = False

    if cordic_atan2_fixed(
        0,
        -Q15_ONE
    ) != 24576:
        boundary_pass = False

    print()

    print(
        "Unsigned Q2.14 boundary test: {}".format(
            "PASS"
            if boundary_pass
            else "FAIL"
        )
    )

    return (
        rotation_pass
        and atan_pass
        and boundary_pass
    )


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":

    print()
    print("=" * 72)
    print(
        "FOC MOTOR CONTROLLER - CORDIC GOLDEN MODEL"
    )
    print("=" * 72)

    _print_constants()

    self_test_pass = _self_test()

    results = cordic_error_analysis()

    generate_testvectors(
        "golden/cordic_testvectors.csv"
    )

    print()
    print("=" * 72)

    if self_test_pass:
        print(
            "SELF-TEST STATUS: PASS"
        )
    else:
        print(
            "SELF-TEST STATUS: FAIL"
        )

    max_impl_error = max(
        results[
            "cordic_max_cos_err_lsb"
        ],
        results[
            "cordic_max_sin_err_lsb"
        ]
    )

    if max_impl_error <= 1.0:

        print(
            "CORDIC IMPLEMENTATION ACCURACY: "
            "PASS (<= 1 LSB)"
        )

    else:

        print(
            "CORDIC IMPLEMENTATION ACCURACY: "
            "NOT YET <= 1 LSB"
        )

    print("=" * 72)