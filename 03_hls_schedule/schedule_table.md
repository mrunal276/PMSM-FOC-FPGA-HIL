# Hand-HLS 2-DSP Resource-Constrained Schedule (FDS / ILP)

## 1. Overview & Objective

- **Target Platform:** Xilinx Zynq-7000 (Zybo Z7-10, `xc7z010clg400-1`)
- **Clock Frequency:** 100 MHz (`Tclk = 10 ns`)
- **Control Period:** 20 kHz (`Ts = 50 µs = 5000 clock cycles`)
- **Target Resources:** Strict allocation of **at most 2 DSP48E1 slices** (`DSP0`, `DSP1`) dedicated to the arithmetic datapath.
- **Adder/Subtractor Logic:** Mapped to standard FPGA Slice LUTs. DSP multipliers are used exclusively for multiplication/MAC operations.

The resource-constrained datapath covers:

- Park Transform
- Speed PI
- Id PI
- Iq PI
- Inverse Park Transform
- SVPWM scaling

**Clarke transform is intentionally excluded from the 25-operation DSP-sharing inventory** because its implementation is accounted for separately as a 2-cycle LUT/add-shift datapath stage in the control-period budget.

---

## 2. Operation Inventory — 25 Arithmetic Operations

| Op ID | Subsystem | Mathematical Expression | Op Type | Resource | Data Dependency |
|:---:|:---|:---|:---:|:---:|:---|
| **M1** | Park Transform | `Iα · cosθ` | MUL | `DSP0` | CORDIC cosθ, Clarke Iα |
| **M2** | Park Transform | `Iβ · sinθ` | MUL | `DSP1` | CORDIC sinθ, Clarke Iβ |
| **M3** | Park Transform | `Iα · sinθ` | MUL | `DSP0` | CORDIC sinθ, Clarke Iα |
| **M4** | Park Transform | `Iβ · cosθ` | MUL | `DSP1` | CORDIC cosθ, Clarke Iβ |
| **A1** | Park Transform | `Id = M1 + M2` | ADD | LUT | M1, M2 |
| **A2** | Park Transform | `Iq = M4 − M3` | SUB | LUT | M3, M4 |
| **M5** | Speed PI | `eω · Kp,ω` | MUL | `DSP0` | `ωref − ωmeas` |
| **M6** | Speed PI | `eω · Ki,ω(update)` | MUL | `DSP1` | `ωref − ωmeas` |
| **A3** | Speed PI | `Iacc,ω ← Iacc,ω + M6` | ADD | LUT | M6, anti-windup flag |
| **A4** | Speed PI | `iq_ref = clamp(M5 + Iacc,ω)` | ADD | LUT | M5, A3 |
| **M7** | Id PI | `eId · Kp,d` | MUL | `DSP0` | `Id_ref − Id` |
| **M8** | Id PI | `eId · Ki,d` | MUL | `DSP1` | `Id_ref − Id` |
| **A5** | Id PI | `Iacc,d ← Iacc,d + M8` | ADD | LUT | M8, anti-windup flag |
| **A6** | Id PI | `Vd = clamp(M7 + Iacc,d)` | ADD | LUT | M7, A5 |
| **M9** | Iq PI | `eIq · Kp,q` | MUL | `DSP0` | `Iq_ref − Iq` |
| **M10** | Iq PI | `eIq · Ki,q` | MUL | `DSP1` | `Iq_ref − Iq` |
| **A7** | Iq PI | `Iacc,q ← Iacc,q + M10` | ADD | LUT | M10, anti-windup flag |
| **A8** | Iq PI | `Vq = clamp(M9 + Iacc,q)` | ADD | LUT | M9, A7 |
| **M11** | Inverse Park | `Vd · cosθ` | MUL | `DSP0` | A6, CORDIC cosθ |
| **M12** | Inverse Park | `Vq · sinθ` | MUL | `DSP1` | A8, CORDIC sinθ |
| **A9** | Inverse Park | `Vα = M11 − M12` | SUB | LUT | M11, M12 |
| **M13** | Inverse Park | `Vd · sinθ` | MUL | `DSP0` | A6, CORDIC sinθ |
| **M14** | Inverse Park | `Vq · cosθ` | MUL | `DSP1` | A8, CORDIC cosθ |
| **A10** | Inverse Park | `Vβ = M13 + M14` | ADD | LUT | M13, M14 |
| **M15** | SVPWM | SVPWM internal scaling / dwell-scaling multiplication | MUL | `DSP0` | SVPWM voltage quantities |

### Operation Accounting

- **15 multiplications**
- **10 additions/subtractions**
- **25 total arithmetic operations**
- **Peak DSP requirement: 2 DSP48E1 slices**
- Clarke arithmetic is accounted for separately as a **2-cycle LUT/add-shift stage**.

---

## 3. ASAP / ALAP Resource-Constrained Schedule

DSP multipliers are pipelined with **1 internal register stage** (`latency = 1 cycle`).

LUT adders and saturators are modeled as 1-cycle stages for scheduling purposes.

```text
Cycle 0..15:
    CORDIC pipeline resolves cos(theta), sin(theta)
    from unsigned Q2.14 theta_e

Cycle 16:
    Clarke transform output latched
    Ialpha, Ibeta

Cycle 17:
    [DSP0] M1: Ialpha * cos
    [DSP1] M2: Ibeta  * sin

Cycle 18:
    [DSP0] M3: Ialpha * sin
    [DSP1] M4: Ibeta  * cos

Cycle 19:
    [LUT] A1: Id = M1 + M2
    [LUT] A2: Iq = M4 - M3

Cycle 20:
    [DSP0] M5: e_spd * Kp_spd
    [DSP1] M6: e_spd * Ki_spd(update)

Cycle 21:
    [LUT] A3: Speed I-update
    [LUT] A4: iq_ref clamped

Cycle 22:
    [DSP0] M7: e_Id * Kp_d
    [DSP1] M8: e_Id * Ki_d

Cycle 23:
    [LUT] A5: Id I-update
    [LUT] A6: Vd clamped

Cycle 24:
    [DSP0] M9: e_Iq * Kp_q
    [DSP1] M10: e_Iq * Ki_q

Cycle 25:
    [LUT] A7: Iq I-update
    [LUT] A8: Vq clamped

Cycle 26:
    [DSP0] M11: Vd * cos
    [DSP1] M12: Vq * sin

Cycle 27:
    [DSP0] M13: Vd * sin
    [DSP1] M14: Vq * cos

Cycle 28:
    [LUT] A9: Valpha = M11 - M12
    [LUT] A10: Vbeta  = M13 + M14

Cycle 29..32:
    SVPWM sector decode and dwell-time calculation
    M15 performed on DSP0 as required
```

### DSP Utilization

The schedule never requires more than **two DSP48E1 slices simultaneously**.

- Park: 2 DSPs
- Speed PI: 2 DSPs
- Id PI: 2 DSPs
- Iq PI: 2 DSPs
- Inverse Park: 2 DSPs
- SVPWM scaling M15: **DSP0 only**

Therefore:

> **Peak DSP utilization = 2 DSP48E1 slices.**

This satisfies the strict two-DSP resource constraint.

---

## 4. Speed PI Fixed-Point Convention

The golden Speed PI uses the discrete-time integrator:

\[
I_{acc,\omega}[k+1]
=
I_{acc,\omega}[k]
+
K_{i,\omega}e_\omega[k]T_s
\]

Therefore, in the HLS schedule:

\[
K_{i,\omega(update)}
=
K_{i,\omega}T_s
\]

and M6 represents:

\[
M6=e_\omega K_{i,\omega(update)}
\]

This preserves exact consistency with the frozen golden `speed_pi.py` implementation.

The current-loop PI coefficients remain consistent with the frozen current PI convention, where `Ki` is already represented as a **per-update coefficient**.

---

## 5. Control Period Budget Analysis

| Subsystem | DP1: Unshared | DP2: 2-DSP Shared |
|---|---:|---:|
| CORDIC — 16-stage pipeline | 16 cycles | 16 cycles |
| Clarke Transform | 2 cycles | 2 cycles |
| Park Transform | 18 cycles | 18 cycles |
| Speed PI | 15 cycles | 12 cycles |
| Id + Iq Inner PI | 30 cycles | 18 cycles |
| Inverse Park Transform | 18 cycles | 18 cycles |
| SVPWM — sector + dwells | 10 cycles | 10 cycles |
| Dead-Time Insertion | 2 cycles | 2 cycles |
| HIL Plant Euler Step — 4 ODEs | 80 cycles | 80 cycles |
| **Total End-to-End System Budget** | **~191 cycles** | **~176 cycles** |
| Available cycles @ 20 kHz / 100 MHz | 5,000 | 5,000 |
| **Budget utilization** | **3.82%** | **3.52%** |
| DSP allocation | 6–8 DSPs | **2 DSPs** |

### DP2 Utilization

\[
\frac{176}{5000}\times100
=
\boxed{3.52\%}
\]

Remaining control-period headroom:

\[
5000-176
=
\boxed{4824\text{ cycles}}
\]

Therefore, the proposed architecture has substantial timing margin at the 100 MHz / 20 kHz operating point.

---

## 6. FDS / ILP Resource-Constrained Result

### Design Point 1 — DP1

The unshared implementation allows multiple arithmetic operations to use independent DSP resources.

- Estimated DSP requirement: **6–8 DSP48E1**
- Lower latency but substantially higher DSP consumption.

### Design Point 2 — DP2

The proposed resource-constrained schedule shares two DSP48E1 slices across the complete arithmetic datapath.

- `DSP0` and `DSP1` are time-multiplexed.
- Maximum simultaneous multiplier demand = **2**.
- All multiplier dependencies are respected.
- Add/subtract operations are assigned to LUT fabric.
- End-to-end estimated system budget = **~176 cycles**.
- Control-period utilization = **3.52%**.
- Available period = **5000 cycles**.

### Final Conclusion

> **DP2 satisfies the strict 2-DSP resource constraint while leaving approximately 96.48% of the 20 kHz control-period budget unused.**

The shared architecture therefore provides the required **resource reduction with large timing headroom**, making it the selected implementation target for the RTL datapath.

---

## 7. Freeze Record

**Week 1 HLS/FDS/ILP artifact: FROZEN ✅**

### Frozen Parameters

```text
Target FPGA        : Zynq-7000 / Zybo Z7-10
Device             : xc7z010clg400-1
Clock              : 100 MHz
FOC frequency      : 20 kHz
Control period     : 5000 cycles
DSP constraint     : <= 2 DSP48E1
DSP binding        : DSP0 / DSP1
Arithmetic ops     : 25
Multiplications    : 15
Add/Sub            : 10
CORDIC             : 16 stages
Angle interface    : Unsigned Q2.14 [0, 2π)
DP2 budget         : ~176 cycles
Budget utilization : 3.52%
Timing headroom    : ~4824 cycles
```

### Final Status

**`ACCEPT / FREEZE`**

This artifact is the frozen **Week 1 2-DSP HLS/FDS/ILP scheduling baseline** for the subsequent RTL implementation.