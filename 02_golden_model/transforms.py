"""
foc_motor_ctrl/golden/transforms.py
=====================================
Golden-reference floating-point and fixed-point (Q1.15) implementations of the
coordinate transforms used in Field-Oriented Control (FOC) of AC motors.

Transform chain (forward path – current sensing):
    abc  ──[Clarke]──▶  αβ (stationary)  ──[Park]──▶  dq (rotating)

Transform chain (inverse path – voltage output):
    dq  ──[InvPark]──▶  αβ  ──[InvClarke]──▶  abc  ──▶  SVPWM

Theory background
-----------------
* Clarke transform: Projects the three phase quantities (Ia, Ib, Ic) onto a
  two-dimensional stationary orthogonal frame (α, β).  Because the three phases
  are balanced (Ia + Ib + Ic = 0) the system is fully described by only two
  independent quantities.  The *amplitude-invariant* scaling is used here, so
  peak values in αβ equal peak values in abc.

* Park transform: Rotates the stationary αβ frame by the electrical rotor angle
  θₑ so that the resulting d-axis is aligned with the rotor flux.  In steady
  state, Id and Iq become DC values, which greatly simplifies the current
  control loops.

References
----------
* Holmes & Lipo, "Pulse Width Modulation for Power Converters", IEEE Press, 2003
* Texas Instruments, Application Report SPRA524, "Clarke & Park Transforms on
  the TMS320C2xx"
"""

import math


# ---------------------------------------------------------------------------
# Floating-point transforms
# ---------------------------------------------------------------------------

def clarke(Ia: float, Ib: float, Ic: float) -> tuple[float, float]:
    """Amplitude-invariant Clarke transform (abc → αβ stationary frame).

    Converts three balanced phase currents into a two-component orthogonal
    representation.  'Amplitude-invariant' means the peak magnitude is
    preserved (unlike the power-invariant variant which scales by √(2/3)).

    The derivation uses the constraint Ia + Ib + Ic = 0, which lets us
    eliminate Ic and write:

        Iα =  Ia
        Iβ = (Ia + 2·Ib) / √3

    Geometrically, α is collinear with the a-phase axis and β leads by 90°.

    Parameters
    ----------
    Ia, Ib, Ic : float
        Instantaneous phase currents [A].  Must satisfy Ia + Ib + Ic ≈ 0.

    Returns
    -------
    Ialpha, Ibeta : float
        Stationary αβ frame components.
    """
    # α component is simply the a-phase current (a-axis aligned with α-axis)
    Ialpha = Ia

    # β component — derived by projecting b and c phases onto the β-axis.
    # Using Ic = -Ia - Ib the standard 2-row Clarke matrix reduces to:
    #   Iβ = (1/√3)·Ia + (2/√3)·Ib  =  (Ia + 2·Ib) / √3
    Ibeta = (Ia + 2.0 * Ib) / math.sqrt(3.0)

    return Ialpha, Ibeta


def park(Ialpha: float, Ibeta: float, theta_e: float) -> tuple[float, float]:
    """Park transform (αβ stationary → dq rotating frame).

    Rotates the αβ vector by the electrical angle θₑ so that the d-axis
    tracks the rotor flux linkage direction.  This is equivalent to
    multiplying by a 2-D rotation matrix:

        ⎡ Id ⎤   ⎡  cos θ   sin θ ⎤ ⎡ Iα ⎤
        ⎣ Iq ⎦ = ⎣ -sin θ   cos θ ⎦ ⎣ Iβ ⎦

    Parameters
    ----------
    Ialpha, Ibeta : float
        Stationary αβ frame currents (output of Clarke transform).
    theta_e : float
        Electrical rotor angle [radians].  Typically from encoder/observer.

    Returns
    -------
    Id, Iq : float
        Rotating dq frame currents.
        Id – magnetising (flux) component.
        Iq – torque-producing component.
    """
    cos_t = math.cos(theta_e)
    sin_t = math.sin(theta_e)

    # d-axis: projection of αβ vector onto the rotor flux direction
    Id =  Ialpha * cos_t + Ibeta * sin_t

    # q-axis: projection perpendicular to the rotor flux (torque component)
    Iq = -Ialpha * sin_t + Ibeta * cos_t

    return Id, Iq


def inv_park(Vd: float, Vq: float, theta_e: float) -> tuple[float, float]:
    """Inverse Park transform (dq rotating → αβ stationary frame).

    Rotates the dq voltage commands back into the stationary αβ frame so
    they can be fed to the Space-Vector PWM modulator.  This is the
    transpose (= inverse for orthogonal matrices) of the Park rotation:

        ⎡ Vα ⎤   ⎡ cos θ  -sin θ ⎤ ⎡ Vd ⎤
        ⎣ Vβ ⎦ = ⎣ sin θ   cos θ ⎦ ⎣ Vq ⎦

    Parameters
    ----------
    Vd, Vq : float
        Voltage commands in the rotating dq frame [V or p.u.].
    theta_e : float
        Electrical rotor angle [radians].

    Returns
    -------
    Valpha, Vbeta : float
        Voltage commands in the stationary αβ frame.
    """
    cos_t = math.cos(theta_e)
    sin_t = math.sin(theta_e)

    # α component: rotate dq back to stationary frame
    Valpha = Vd * cos_t - Vq * sin_t

    # β component
    Vbeta  = Vd * sin_t + Vq * cos_t

    return Valpha, Vbeta


def inv_clarke(Valpha: float, Vbeta: float) -> tuple[float, float, float]:
    """Inverse Clarke transform (αβ stationary → abc three-phase).

    Reconstructs balanced three-phase voltage references from the αβ frame.
    Used to convert SVPWM modulator αβ outputs into per-phase duty cycles.

    The amplitude-invariant inverse Clarke matrix is:

        ⎡ Va ⎤   ⎡  1       0     ⎤ ⎡ Vα ⎤
        ⎢ Vb ⎥ = ⎢ -1/2   √3/2   ⎥ ⎣ Vβ ⎦
        ⎣ Vc ⎦   ⎣ -1/2  -√3/2   ⎦

    Parameters
    ----------
    Valpha, Vbeta : float
        Voltage commands in the stationary αβ frame.

    Returns
    -------
    Va, Vb, Vc : float
        Balanced three-phase voltage references.
    """
    sqrt3_over_2 = math.sqrt(3.0) / 2.0

    # a-phase is collinear with the α-axis
    Va =  Valpha

    # b and c phases are at ±120° from the a-phase
    Vb = -0.5 * Valpha + sqrt3_over_2 * Vbeta
    Vc = -0.5 * Valpha - sqrt3_over_2 * Vbeta

    return Va, Vb, Vc


# ---------------------------------------------------------------------------
# Fixed-point (Q1.15) transforms
# ---------------------------------------------------------------------------
# Q1.15 format
# ------------
#   • 1 sign bit, 15 fractional bits
#   • Range: -32768 to +32767  (representing -1.0 to +32767/32768 ≈ +1.0)
#   • Scale factor: Q15 = 32768  (integer value for 1.0)
#   • Physical value = integer / 32768
#
# Multiplication: two Q1.15 values multiplied give a Q2.30 product.
#   To return to Q1.15 we right-shift by 15, which truncates.
#   We add 0x4000 (= 2^14) before the shift to get round-to-nearest.
#
# Overflow: results are clipped to [-32768, 32767].

_Q15 = 32768          # 1.0 in Q1.15
_Q15_MAX =  32767     # maximum representable value
_Q15_MIN = -32768     # minimum representable value


def _q15_mul(a: int, b: int) -> int:
    """Multiply two Q1.15 integers, returning a Q1.15 integer.

    The raw product is Q2.30 (64-bit to avoid overflow).  We add the
    round-to-nearest constant (2^14) before right-shifting by 15.
    Result is clipped to the Q1.15 range [-32768, 32767].
    """
    # 32-bit × 32-bit → 48-bit product, stored as Python int (no overflow)
    product = a * b                    # Q2.30

    # Round-to-nearest then rescale back to Q1.15
    rounded = product + 0x4000        # add 0.5 LSB in Q2.30
    result  = rounded >> 15           # arithmetic right-shift to Q1.15

    # Clip to representable range
    return max(_Q15_MIN, min(_Q15_MAX, result))


def _q15_add(a: int, b: int) -> int:
    """Add two Q1.15 integers with saturation clipping."""
    result = a + b
    return max(_Q15_MIN, min(_Q15_MAX, result))


def _q15_sub(a: int, b: int) -> int:
    """Subtract two Q1.15 integers with saturation clipping."""
    result = a - b
    return max(_Q15_MIN, min(_Q15_MAX, result))


# Precomputed constant: 1/√3 in Q1.15
# 1/√3 ≈ 0.57735  →  round(0.57735 * 32768) = 18919
_INV_SQRT3_Q15 = round((1.0 / math.sqrt(3.0)) * _Q15)   # 18919

# Precomputed constant: 2/√3 in Q1.15
# 2/√3 ≈ 1.15470  →  clips to 32767 (> 1.0, just outside range!)
# We handle this by computing (Ia + 2*Ib) then multiplying by 1/√3.
_TWO_INV_SQRT3_Q15 = round((2.0 / math.sqrt(3.0)) * _Q15)  # 37837, clips!

# Constant: 0.5 in Q1.15
_HALF_Q15 = round(0.5 * _Q15)   # 16384

# Constant: √3/2 in Q1.15
_SQRT3_OVER_2_Q15 = round((math.sqrt(3.0) / 2.0) * _Q15)  # 28378


def clarke_fixed(Ia_q15: int, Ib_q15: int, Ic_q15: int) -> tuple[int, int]:
    """Fixed-point amplitude-invariant Clarke transform (Q1.15).

    Implements:
        Iα = Ia
        Iβ = (Ia + 2·Ib) / √3

    To keep intermediate values in range we split the division:
        Iβ = Ia·(1/√3)  +  Ib·(2/√3)
           = Ia·(1/√3)  +  (Ib + Ib)·(1/√3)

    Both (1/√3) ≈ 0.5774 and each term fit within Q1.15.

    Parameters
    ----------
    Ia_q15, Ib_q15, Ic_q15 : int
        Q1.15 phase currents.  Ic is accepted but not used (balanced system).

    Returns
    -------
    Ialpha_q15, Ibeta_q15 : int
        Q1.15 stationary frame currents.
    """
    # α = Ia  (trivial — just pass through, clip for safety)
    Ialpha_q15 = max(_Q15_MIN, min(_Q15_MAX, Ia_q15))

    # β = Ia/√3 + 2·Ib/√3
    #   = Ia · (1/√3)  +  Ib · (2/√3)
    # (2/√3 ≈ 1.1547 overflows Q1.15; compute as two separate 1/√3 terms)
    term_Ia  = _q15_mul(Ia_q15, _INV_SQRT3_Q15)         # Ia/√3
    term_Ib1 = _q15_mul(Ib_q15, _INV_SQRT3_Q15)         # Ib/√3
    term_Ib2 = _q15_mul(Ib_q15, _INV_SQRT3_Q15)         # Ib/√3  (second copy)
    Ibeta_q15 = _q15_add(_q15_add(term_Ia, term_Ib1), term_Ib2)

    return Ialpha_q15, Ibeta_q15


def park_fixed(
    Ialpha_q15: int,
    Ibeta_q15: int,
    cos_q15: int,
    sin_q15: int,
) -> tuple[int, int]:
    """Fixed-point Park transform (αβ → dq), Q1.15.

    cos_q15 and sin_q15 should be precomputed from the electrical angle θₑ,
    typically via a CORDIC unit or a sine look-up table.

    Implements:
        Id =  Iα·cos(θ) + Iβ·sin(θ)
        Iq = -Iα·sin(θ) + Iβ·cos(θ)

    Parameters
    ----------
    Ialpha_q15, Ibeta_q15 : int
        Q1.15 stationary frame currents.
    cos_q15, sin_q15 : int
        Q1.15 cosine and sine of the electrical angle.

    Returns
    -------
    Id_q15, Iq_q15 : int
        Q1.15 rotating dq frame currents.
    """
    # Id =  Iα·cos θ + Iβ·sin θ
    Id_q15 = _q15_add(
        _q15_mul(Ialpha_q15,  cos_q15),
        _q15_mul(Ibeta_q15,   sin_q15),
    )

    # Iq = -Iα·sin θ + Iβ·cos θ
    Iq_q15 = _q15_add(
        _q15_mul(Ialpha_q15, -sin_q15),   # negate sin for −Iα·sin θ
        _q15_mul(Ibeta_q15,   cos_q15),
    )

    return Id_q15, Iq_q15


def inv_park_fixed(
    Vd_q15: int,
    Vq_q15: int,
    cos_q15: int,
    sin_q15: int,
) -> tuple[int, int]:
    """Fixed-point Inverse Park transform (dq → αβ), Q1.15.

    Implements:
        Vα = Vd·cos(θ) - Vq·sin(θ)
        Vβ = Vd·sin(θ) + Vq·cos(θ)

    Parameters
    ----------
    Vd_q15, Vq_q15 : int
        Q1.15 rotating dq frame voltage commands.
    cos_q15, sin_q15 : int
        Q1.15 cosine and sine of the electrical angle (same as used in Park).

    Returns
    -------
    Valpha_q15, Vbeta_q15 : int
        Q1.15 stationary αβ frame voltage commands.
    """
    # Vα = Vd·cos θ - Vq·sin θ
    Valpha_q15 = _q15_sub(
        _q15_mul(Vd_q15,  cos_q15),
        _q15_mul(Vq_q15,  sin_q15),
    )

    # Vβ = Vd·sin θ + Vq·cos θ
    Vbeta_q15 = _q15_add(
        _q15_mul(Vd_q15,  sin_q15),
        _q15_mul(Vq_q15,  cos_q15),
    )

    return Valpha_q15, Vbeta_q15


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    """Quick smoke tests for floating-point transforms.

    Run with:
        python transforms.py
    """
    import sys
    # Force UTF-8 output on Windows so Greek letters print correctly
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')

    PASS = "PASS"
    FAIL = "FAIL"
    TOL  = 1e-10   # acceptable floating-point round-trip error

    print("=" * 60)
    print("FOC Transform Golden Reference – Self-Test")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Test 1: Clarke transform with balanced input
    # Input:  Ia=1.0, Ib=-0.5, Ic=-0.5  (balanced, Ia+Ib+Ic=0)
    # Expected: Iα=1.0, Iβ=0.0
    # ------------------------------------------------------------------
    Ia, Ib, Ic = 1.0, -0.5, -0.5
    Ialpha, Ibeta = clarke(Ia, Ib, Ic)

    clarke_ok = (abs(Ialpha - 1.0) < TOL) and (abs(Ibeta - 0.0) < TOL)
    print(f"\nTest 1 – Clarke transform")
    print(f"  Input : Ia={Ia}, Ib={Ib}, Ic={Ic}")
    print(f"  Output: Iα={Ialpha:.10f}, Iβ={Ibeta:.10f}")
    print(f"  Expect: Iα=1.0000000000, Iβ=0.0000000000")
    print(f"  Result: {PASS if clarke_ok else FAIL}")

    # ------------------------------------------------------------------
    # Test 2: Park → InvPark round-trip
    # Apply Park then immediately InvPark — the result must equal the input.
    # Tests several angles to catch sign / quadrant errors.
    # ------------------------------------------------------------------
    print(f"\nTest 2 – Park → InvPark round-trip")
    roundtrip_ok = True
    test_vectors = [
        # (Iα,   Iβ,   θ_e)
        (1.0,   0.0,   0.0),
        (0.0,   1.0,   math.pi / 4),
        (0.707, 0.707, math.pi / 3),
        (1.0,  -1.0,   math.pi),
        (0.5,   0.866, 5 * math.pi / 6),
    ]
    for Ia_t, Ib_t, theta in test_vectors:
        Id, Iq = park(Ia_t, Ib_t, theta)
        Va_rt, Vb_rt = inv_park(Id, Iq, theta)
        err_alpha = abs(Va_rt - Ia_t)
        err_beta  = abs(Vb_rt - Ib_t)
        ok = (err_alpha < TOL) and (err_beta < TOL)
        if not ok:
            roundtrip_ok = False
        print(f"  θ={theta:.4f} rad | err_α={err_alpha:.2e} err_β={err_beta:.2e} | {'OK' if ok else 'FAIL'}")

    print(f"  Result: {PASS if roundtrip_ok else FAIL}")

    # ------------------------------------------------------------------
    # Test 3: Clarke → Park → InvPark → InvClarke round-trip
    # Full chain with a rotating angle.
    # ------------------------------------------------------------------
    print(f"\nTest 3 – Full chain round-trip (Clarke→Park→InvPark→InvClarke)")
    Ia_in, Ib_in, Ic_in = 1.0, -0.5, -0.5
    theta_test = math.pi / 6  # 30°

    Ialpha_c, Ibeta_c   = clarke(Ia_in, Ib_in, Ic_in)
    Id_p, Iq_p          = park(Ialpha_c, Ibeta_c, theta_test)
    Valpha_ip, Vbeta_ip = inv_park(Id_p, Iq_p, theta_test)
    Va_ic, Vb_ic, Vc_ic = inv_clarke(Valpha_ip, Vbeta_ip)

    err_a = abs(Va_ic - Ia_in)
    err_b = abs(Vb_ic - Ib_in)
    err_c = abs(Vc_ic - Ic_in)
    chain_ok = (err_a < TOL) and (err_b < TOL) and (err_c < TOL)

    print(f"  Input : Ia={Ia_in}, Ib={Ib_in}, Ic={Ic_in}  θ={theta_test:.4f} rad")
    print(f"  Output: Va={Va_ic:.10f}, Vb={Vb_ic:.10f}, Vc={Vc_ic:.10f}")
    print(f"  Errors: ΔVa={err_a:.2e}, ΔVb={err_b:.2e}, ΔVc={err_c:.2e}")
    print(f"  Result: {PASS if chain_ok else FAIL}")

    # ------------------------------------------------------------------
    # Test 4: Fixed-point Clarke sanity check (Ia=1.0, Ib=-0.5, Ic=-0.5)
    # Expected: Ialpha_q15 = 32767, Ibeta_q15 ≈ 0
    # (Ia=1.0 clips to Q15_MAX=32767; Ibeta should be ≈0)
    # ------------------------------------------------------------------
    print(f"\nTest 4 – Fixed-point Clarke (Q1.15)")
    Ia_q  = _Q15_MAX         # 1.0 in Q1.15 (clipped)
    Ib_q  = round(-0.5 * _Q15)  # -16384
    Ic_q  = round(-0.5 * _Q15)  # -16384
    Ialpha_q, Ibeta_q = clarke_fixed(Ia_q, Ib_q, Ic_q)
    # Accept ±2 LSB quantisation error for Ibeta
    fixed_clarke_ok = (Ialpha_q == _Q15_MAX) and (abs(Ibeta_q) <= 2)
    print(f"  Input : Ia_q15={Ia_q}, Ib_q15={Ib_q}, Ic_q15={Ic_q}")
    print(f"  Output: Ialpha_q15={Ialpha_q}, Ibeta_q15={Ibeta_q}")
    print(f"  Expect: Ialpha_q15=32767, Ibeta_q15≈0 (±2 LSB)")
    print(f"  Result: {PASS if fixed_clarke_ok else FAIL}")

    # ------------------------------------------------------------------
    # Test 5: Fixed-point Park → InvPark round-trip at θ = π/4
    # ------------------------------------------------------------------
    print(f"\nTest 5 – Fixed-point Park → InvPark round-trip (θ=π/4)")
    theta_fp = math.pi / 4
    cos_q = round(math.cos(theta_fp) * _Q15)   # ≈23170
    sin_q = round(math.sin(theta_fp) * _Q15)   # ≈23170
    Ia_fp_q  = round(0.8 * _Q15)               # 0.8 in Q1.15 = 26214
    Ib_fp_q  = round(0.3 * _Q15)               # 0.3 in Q1.15 = 9830

    Id_fp, Iq_fp         = park_fixed(Ia_fp_q, Ib_fp_q, cos_q, sin_q)
    Va_fp, Vb_fp         = inv_park_fixed(Id_fp, Iq_fp, cos_q, sin_q)

    err_fp_a = abs(Va_fp - Ia_fp_q)
    err_fp_b = abs(Vb_fp - Ib_fp_q)
    # Allow up to 4 LSB error due to cumulative Q1.15 rounding
    fixed_rt_ok = (err_fp_a <= 4) and (err_fp_b <= 4)
    print(f"  Iα_in={Ia_fp_q} ({Ia_fp_q/_Q15:.4f}), Iβ_in={Ib_fp_q} ({Ib_fp_q/_Q15:.4f})")
    print(f"  cos_q15={cos_q}, sin_q15={sin_q}")
    print(f"  Id={Id_fp}, Iq={Iq_fp}")
    print(f"  Vα_out={Va_fp}, Vβ_out={Vb_fp}  (err: {err_fp_a} LSB, {err_fp_b} LSB)")
    print(f"  Result: {PASS if fixed_rt_ok else FAIL}")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    all_passed = clarke_ok and roundtrip_ok and chain_ok and fixed_clarke_ok and fixed_rt_ok
    print("\n" + "=" * 60)
    print(f"Overall: {PASS if all_passed else FAIL}")
    print("=" * 60)
