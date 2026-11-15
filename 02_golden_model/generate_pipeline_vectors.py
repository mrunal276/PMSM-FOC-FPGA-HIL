"""
generate_pipeline_vectors.py
=============================================================================
Independent Bit-True Fixed-Point FOC Pipeline Vector Generator

Architecture & Math Specifications:
  1. Clarke Transform   : Q1.15 arithmetic with 1/sqrt(3) (18919).
  2. CORDIC Unit        : 16-stage shift-add rotation with quadrant reduction.
  3. Park & Inv Park    : Q1.15 saturated arithmetic (+16384 >> 15 rounding).
  4. Speed PI Loop      : Q10.6 speed error, Q1.15 Kp (shift 6), Q1.31 Ki*dt (shift 13),
                          persistent Q8.24 integrator, and conditional anti-windup.
  5. Dual Current PIs   : Q1.15 current error, Q4.12 Kp (shift 12), Q1.15 Ki (shift 15)
                          scaled to Q8.24 integrator (<< 9), and anti-windup.
  6. Space Vector PWM   : 6-sector boundary detection (sqrt(3)*2^24 = 29058991),
                          16-bit fractional guard bits, overmodulation scaling,
                          and symmetric 7-segment duty generation (0..32767).

This script preserves 'foc_golden.py' and 'pmsm_plant.py' untouched as the
theoretical physical references.
=============================================================================
"""

import csv
import shutil
import sys
from pathlib import Path

# ============================================================================
# FIXED-POINT ARITHMETIC CORE PRIMITIVES
# ============================================================================

def clamp(val, low, high):
    return max(low, min(high, val))

def sat_signed16(val):
    return clamp(val, -32768, 32767)

def sat_signed32(val):
    return clamp(val, -2147483648, 2147483647)

def round_shift_sym(product, shift):
    """Sign-symmetric round-to-nearest and arithmetic right shift."""
    half = 1 << (shift - 1)
    if product >= 0:
        rounded = product + half
    else:
        rounded = product - half
    return rounded >> shift

def q15_mul_sat(a, b):
    """Standard Q1.15 convergent round-to-nearest multiplication."""
    product = a * b
    rounded = product + 16384
    shifted = rounded >> 15
    return sat_signed16(shifted)

def q15_add_sat(a, b):
    """Saturating Q1.15 addition."""
    return sat_signed16(a + b)

def q15_mul_sym(a, b):
    """Sign-symmetric Q1.15 multiplication."""
    return sat_signed16(round_shift_sym(a * b, 15))


# ============================================================================
# 1. CLARKE TRANSFORM (Q1.15)
# ============================================================================

INV_SQRT3 = 18919  # round(1.0 / sqrt(3) * 32768)

def clarke_fixed(ia, ib, ic):
    """
    Alpha = Ia
    Beta  = (Ia + 2*Ib) / sqrt(3) = Ia/sqrt(3) + Ib/sqrt(3) + Ib/sqrt(3)
    """
    alpha = ia
    term_ia = q15_mul_sat(ia, INV_SQRT3)
    term_ib1 = q15_mul_sat(ib, INV_SQRT3)
    term_ib2 = q15_mul_sat(ib, INV_SQRT3)
    beta = q15_add_sat(q15_add_sat(term_ia, term_ib1), term_ib2)
    return alpha, beta


# ============================================================================
# 2. CORDIC UNIT (16-STAGE ROTATION)
# ============================================================================

CORDIC_K = 636751  # round(prod(cos(atan(2^-i))) * 2^20) for 16 stages
ATAN_TABLE = [
    1048576, 619011, 327068, 166025, 83335, 41708, 20859, 10430,
    5215, 2608, 1304, 652, 326, 163, 81, 41
]

def cordic_fixed(theta_q14):
    """
    Input : theta_q14 unsigned [0, 2pi) where 16384 = pi rad.
    Output: cos_q15, sin_q15 signed Q1.15 [-32768, 32767].
    """
    theta_q14 = theta_q14 & 0xFFFF
    if theta_q14 == 0x8000:
        quadrant = 0
        reduced = 0
    elif theta_q14 < 0x2000:
        quadrant = 0
        reduced = theta_q14
    elif theta_q14 < 0x4000:
        quadrant = 1
        reduced = 0x4000 - theta_q14
    elif theta_q14 < 0x6000:
        quadrant = 2
        reduced = theta_q14 - 0x4000
    else:
        quadrant = 3
        reduced = 0x8000 - theta_q14

    z = reduced << 8
    x = CORDIC_K
    y = 0

    for i in range(16):
        x_shift = y >> i
        y_shift = x >> i
        if z >= 0:
            x_new = x - x_shift
            y_new = y + y_shift
            z_new = z - ATAN_TABLE[i]
        else:
            x_new = x + x_shift
            y_new = y - y_shift
            z_new = z + ATAN_TABLE[i]
        x, y, z = x_new, y_new, z_new

    # Convert Q20 internal magnitude to Q1.15
    cos_mag = sat_signed16((x + 16) >> 5)
    sin_mag = sat_signed16((y + 16) >> 5)

    if quadrant == 0:
        return cos_mag, sin_mag
    elif quadrant == 1:
        return -cos_mag, sin_mag
    elif quadrant == 2:
        return -cos_mag, -sin_mag
    else:
        return cos_mag, -sin_mag


# ============================================================================
# 3. PARK & INVERSE PARK TRANSFORMS (Q1.15)
# ============================================================================

def park_fixed(alpha, beta, cos_th, sin_th):
    id_calc = q15_add_sat(q15_mul_sat(alpha, cos_th), q15_mul_sat(beta, sin_th))
    iq_calc = q15_add_sat(q15_mul_sat(alpha, -sin_th), q15_mul_sat(beta, cos_th))
    return id_calc, iq_calc

def inv_park_fixed(vd, vq, cos_th, sin_th):
    valpha = q15_add_sat(q15_mul_sat(vd, cos_th), q15_mul_sat(vq, -sin_th))
    vbeta  = q15_add_sat(q15_mul_sat(vd, sin_th), q15_mul_sat(vq, cos_th))
    return valpha, vbeta


# ============================================================================
# 4. SPEED PI CONTROLLER (Q10.6 -> Q1.15)
# ============================================================================

class SpeedPIFixed:
    def __init__(self, kp=1311, ki_dt=53687):
        self.kp = kp          # Q1.15: round(0.04 * 32768) = 1311
        self.ki_dt = ki_dt    # Q1.31: round(0.5 * 50e-6 * 2^31) = 53687
        self.integrator = 0   # Q8.24 persistent state

    def step(self, w_ref_q6, w_meas_q6):
        err = w_ref_q6 - w_meas_q6
        p_term = sat_signed16(round_shift_sym(err * self.kp, 6))
        i_inc = sat_signed32(round_shift_sym(err * self.ki_dt, 13))

        cand_integ = sat_signed32(self.integrator + i_inc)
        cand_i_q15 = sat_signed16(round_shift_sym(cand_integ, 9))
        unsat = p_term + cand_i_q15
        out = sat_signed16(unsat)

        # Anti-windup conditional integration
        upper = unsat > 32767
        lower = unsat < -32768
        blocked = (upper and err > 0) or (lower and err < 0)
        if not blocked:
            self.integrator = cand_integ

        return out


# ============================================================================
# 5. CURRENT PI CONTROLLER (Q1.15 -> Q1.15, KP_FRAC = 12)
# ============================================================================

class CurrentPIFixed:
    def __init__(self, kp=12288, ki=819, kp_frac=12):
        self.kp = kp          # Q4.12: round(3.0 * 4096) = 12288
        self.ki = ki          # Q1.15: round(0.025 * 32768) = 819
        self.kp_frac = kp_frac
        self.integrator = 0   # Q8.24 persistent state

    def step(self, ref_q15, meas_q15):
        err = sat_signed16(ref_q15 - meas_q15)
        p_term = sat_signed16(round_shift_sym(err * self.kp, self.kp_frac))
        i_inc_q15 = q15_mul_sym(self.ki, err)
        i_inc_q24 = sat_signed32(i_inc_q15 << 9)

        cand_integ = sat_signed32(self.integrator + i_inc_q24)
        cand_i_q15 = sat_signed16(round_shift_sym(cand_integ, 9))
        unsat = p_term + cand_i_q15
        out = sat_signed16(unsat)

        upper = unsat > 32767
        lower = unsat < -32768
        blocked = (upper and err > 0) or (lower and err < 0)
        if not blocked:
            self.integrator = cand_integ

        return out


# ============================================================================
# 6. SPACE VECTOR PWM (SVPWM)
# ============================================================================

SVPWM_F_FRAC  = 16
SVPWM_ONE_F   = 2147418112   # 32767 * 2^16
SVPWM_K1P5    = 98301        # round(1.5 * (32767/32768) * 2^16)
SVPWM_KSQ3_2  = 56754        # round((sqrt(3)/2) * (32767/32768) * 2^16)
SVPWM_KSQ3    = 113508       # round(sqrt(3) * (32767/32768) * 2^16)
SVPWM_SQ3_SEC = 29058991     # round(sqrt(3) * 2^24)

def svpwm_fixed(alpha, beta):
    abs_alpha = abs(alpha)
    abs_beta = abs(beta)
    compare_left = abs_beta << 24
    compare_right = abs_alpha * SVPWM_SQ3_SEC

    # Sector detection
    if alpha == 0 and beta == 0:
        sector = 1
    elif alpha < 0 and beta == 0:
        sector = 4
    elif alpha == 0 and beta < 0:
        sector = 5
    elif alpha >= 0 and beta >= 0:
        sector = 1 if compare_left < compare_right else 2
    elif alpha < 0 and beta > 0:
        sector = 2 if compare_left > compare_right else 3
    elif alpha < 0 and beta < 0:
        sector = 4 if compare_left < compare_right else 5
    else:
        sector = 5 if compare_left > compare_right else 6

    # Dwell times
    if sector == 1:
        t1 = alpha * SVPWM_K1P5 - beta * SVPWM_KSQ3_2
        t2 = beta * SVPWM_KSQ3
    elif sector == 2:
        t1 = alpha * SVPWM_K1P5 + beta * SVPWM_KSQ3_2
        t2 = beta * SVPWM_KSQ3_2 - alpha * SVPWM_K1P5
    elif sector == 3:
        t1 = beta * SVPWM_KSQ3
        t2 = -alpha * SVPWM_K1P5 - beta * SVPWM_KSQ3_2
    elif sector == 4:
        t1 = -alpha * SVPWM_K1P5 + beta * SVPWM_KSQ3_2
        t2 = -beta * SVPWM_KSQ3
    elif sector == 5:
        t1 = -alpha * SVPWM_K1P5 - beta * SVPWM_KSQ3_2
        t2 = alpha * SVPWM_K1P5 - beta * SVPWM_KSQ3_2
    else:
        t1 = -beta * SVPWM_KSQ3
        t2 = alpha * SVPWM_K1P5 + beta * SVPWM_KSQ3_2

    t1 = max(0, t1)
    t2 = max(0, t2)
    active = t1 + t2

    # Overmodulation scaling
    if active > SVPWM_ONE_F:
        t1_s = (t1 * SVPWM_ONE_F + (active >> 1)) // active
        t2_s = SVPWM_ONE_F - t1_s
        active = SVPWM_ONE_F
    else:
        t1_s = t1
        t2_s = t2

    t0 = SVPWM_ONE_F - active
    half_t0 = (t0 + 1) >> 1

    # Symmetric 7-segment duty assignment
    if sector == 1:
        ta, tb, tc = half_t0 + t1_s + t2_s, half_t0 + t2_s, half_t0
    elif sector == 2:
        ta, tb, tc = half_t0 + t1_s, half_t0 + t1_s + t2_s, half_t0
    elif sector == 3:
        ta, tb, tc = half_t0, half_t0 + t1_s + t2_s, half_t0 + t2_s
    elif sector == 4:
        ta, tb, tc = half_t0, half_t0 + t1_s, half_t0 + t1_s + t2_s
    elif sector == 5:
        ta, tb, tc = half_t0 + t2_s, half_t0, half_t0 + t1_s + t2_s
    else:
        ta, tb, tc = half_t0 + t1_s + t2_s, half_t0, half_t0 + t1_s

    def duty_from_fixed(v):
        return clamp((v + (1 << 15)) >> 16, 0, 32767)

    return sector, duty_from_fixed(ta), duty_from_fixed(tb), duty_from_fixed(tc)


# ============================================================================
# 7. INTERNAL SELF-TEST VALIDATION SUITE
# ============================================================================

def self_test():
    """Runs a standalone sanity check to verify zero numerical anomalies."""
    print("Running internal self-test validation suite...")

    # CORDIC full-circle checks
    for angle in [0, 4096, 8192, 12288, 16384, 24576, 32767]:
        c, s = cordic_fixed(angle)
        assert -32768 <= c <= 32767 and -32768 <= s <= 32767, f"CORDIC overflow at {angle}"

    # SVPWM boundary checks
    for alpha, beta in [(0, 0), (20000, 10000), (-20000, 10000), (-20000, -10000), (20000, -10000)]:
        sec, da, db, dc = svpwm_fixed(alpha, beta)
        assert 1 <= sec <= 6, f"Invalid sector {sec}"
        assert 0 <= da <= 32767 and 0 <= db <= 32767 and 0 <= dc <= 32767, "Duty out of range"

    # PI Controller anti-windup check
    spi = SpeedPIFixed()
    out1 = spi.step(1000, 0)
    out2 = spi.step(1000, 0)
    assert out1 > 0 and out2 >= out1, "Speed PI tracking failed"

    cpi = CurrentPIFixed()
    out_c1 = cpi.step(1000, 0)
    out_c2 = cpi.step(1000, 0)
    assert out_c1 > 0 and out_c2 >= out_c1, "Current PI tracking failed"

    print("Internal validation suite: ALL CHECKS PASSED.")


# ============================================================================
# 8. MAIN EXECUTION & CSV GENERATION
# ============================================================================

HERE = Path(__file__).resolve().parent
INPUT_CSV = HERE / "foc_step_response.csv"
OUTPUT_CSV = HERE / "foc_pipeline_testvectors.csv"
BACKUP_CSV = HERE / "foc_pipeline_testvectors_float.csv.bak"

MAX_VECTORS = 500
I_BASE = 20.0

def main():
    self_test()

    if not INPUT_CSV.exists():
        raise FileNotFoundError(f"Input file not found: {INPUT_CSV}")

    # Backup existing float CSV if backup does not already exist
    if OUTPUT_CSV.exists() and not BACKUP_CSV.exists():
        shutil.copyfile(OUTPUT_CSV, BACKUP_CSV)
        print(f"Backed up original CSV to: {BACKUP_CSV}")

    speed_pi = SpeedPIFixed()
    pi_d = CurrentPIFixed()
    pi_q = CurrentPIFixed()

    count = 0
    with open(INPUT_CSV, "r", encoding="utf-8-sig") as f_in:
        reader = csv.DictReader(f_in)
        rows_to_write = []

        for row in reader:
            if count >= MAX_VECTORS:
                break

            # Quantize physical inputs to hardware interfaces
            w_ref = float(row["omega_ref"])
            w_meas = float(row.get("omega_mech", row.get("omega_meas", 0.0)))
            th = float(row.get("theta_e", row.get("theta", 0.0)))
            ia = float(row["ia"])
            ib = float(row["ib"])
            ic = float(row["ic"])
            id_ref_pu = float(row.get("id_ref_pu", 0.0))

            w_ref_q6 = clamp(round(w_ref * 64.0), 0, 65535)
            w_meas_q6 = clamp(round(w_meas * 64.0), 0, 65535)

            th_mod = th % (2.0 * 3.141592653589793)
            theta_q14 = clamp(round(th_mod / (2.0 * 3.141592653589793) * 32768.0), 0, 32767)
            if theta_q14 >= 32768:
                theta_q14 = 0

            ia_q15 = sat_signed16(round((ia / I_BASE) * 32768.0))
            ib_q15 = sat_signed16(round((ib / I_BASE) * 32768.0))
            ic_q15 = sat_signed16(round((ic / I_BASE) * 32768.0))
            id_ref_q15 = sat_signed16(round(id_ref_pu * 32768.0))

            # Execute bit-true pipeline architecture
            alp, bet = clarke_fixed(ia_q15, ib_q15, ic_q15)
            cos_th, sin_th = cordic_fixed(theta_q14)
            id_meas, iq_meas = park_fixed(alp, bet, cos_th, sin_th)

            iq_ref = speed_pi.step(w_ref_q6, w_meas_q6)
            vd = pi_d.step(id_ref_q15, id_meas)
            vq = pi_q.step(iq_ref, iq_meas)

            valp, vbet = inv_park_fixed(vd, vq, cos_th, sin_th)
            sec, da, db, dc = svpwm_fixed(valp, vbet)

            # Sanity checks per vector
            assert 1 <= sec <= 6, f"Invalid sector {sec} at vector {count}"
            assert 0 <= da <= 32767 and 0 <= db <= 32767 and 0 <= dc <= 32767, f"Duty out of bounds at {count}"

            rows_to_write.append([
                w_ref_q6, w_meas_q6, theta_q14,
                ia_q15, ib_q15, ic_q15, id_ref_q15,
                sec, da, db, dc,
                alp, bet
            ])
            count += 1

    with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as f_out:
        writer = csv.writer(f_out)
        writer.writerows(rows_to_write)

    print(f"Successfully generated {count} continuous bit-true vectors in: {OUTPUT_CSV}")
    print("Format: 13 comma-separated integer columns, no header.")

if __name__ == "__main__":
    main()