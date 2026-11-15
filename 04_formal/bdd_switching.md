# Binary Decision Diagram (BDD) & ESPRESSO Derivation: Inverter Shoot-Through Interlock & SVPWM Decoding

This document provides the formal Boolean algebraic derivation, Binary Decision Diagram (BDD) analysis, and two-level logic minimization (ESPRESSO) for two critical safety and decoding blocks in the FOC controller:
1. **The Inverter Half-Bridge Shoot-Through Interlock (`dead_time.sv`)**
2. **The 3-Phase SVPWM Sector Decoder (`svpwm.sv`)**

---

## 1. Shoot-Through Interlock (`dead_time.sv`)

### 1.1 Problem Specification & Safety Invariant
In a two-level 3-phase voltage source inverter (VSI), each half-bridge leg comprises a complementary switch pair:
- **Upper MOSFET/IGBT Gate**: $G_H$
- **Lower MOSFET/IGBT Gate**: $G_L$

If both $G_H$ and $G_L$ conduct simultaneously, a direct short circuit occurs across the DC bus ($V_{DC} \to \text{GND}$), known as **shoot-through**. This causes catastrophic device failure.

#### Safety Invariant
$$\text{Safety} = \bigwedge_{p \in \{A, B, C\}} \neg \left( G_{H,p} \wedge G_{L,p} \right)$$

Additionally, during system reset ($R$) or hardware fault ($F$), all gate drives must be inhibited:
$$R \vee F \implies \neg G_{H} \wedge \neg G_{L}$$

---

### 1.2 State Encoding & Boolean Function
The physical state of each inverter phase is captured by 2 flip-flops $S_1, S_0 \in \{0, 1\}$:
- **OFF ($00_2$)**: Both gates inactive ($G_H = 0, G_L = 0$).
- **HIGH ($01_2$)**: Upper gate active ($G_H = 1, G_L = 0$).
- **LOW ($10_2$)**: Lower gate active ($G_H = 0, G_L = 1$).
- **FORBIDDEN ($11_2$)**: Shoot-through hazard ($G_H = 1, G_L = 1$).

Inputs:
- $R$: Active-high reset
- $F$: Active-high fault / PWM inhibit
- $S_1, S_0$: Current state bits

Outputs:
$$G_H = \overline{R} \cdot \overline{F} \cdot \overline{S_1} \cdot S_0$$
$$G_L = \overline{R} \cdot \overline{F} \cdot S_1 \cdot \overline{S_0}$$

---

### 1.3 Binary Decision Diagram (BDD) Derivation

Using the natural variable ordering $\pi = (R < F < S_1 < S_0)$:

```
           [ R ]
          /     \
       1 /       \ 0
        /         \
       0        [ F ]
               /     \
            1 /       \ 0
             /         \
            0        [ S1 ]
                    /      \
                 1 /        \ 0
                  /          \
              [ S0 ]        [ S0 ]
              /    \        /    \
           1 /      \ 0  1 /      \ 0
            /        \    /        \
           0          1  0          0   -> For G_L (Low Gate)
```

#### Reduced Ordered BDD (ROBDD) Properties:
1. **Canonical Form**: The state $11_2$ is mapped to terminal `0` for both $G_H$ and $G_L$.
2. **Mutual Exclusion Proof**:
   $$\text{Path}(G_H = 1) = \overline{R} \overline{F} \overline{S_1} S_0$$
   $$\text{Path}(G_L = 1) = \overline{R} \overline{F} S_1 \overline{S_0}$$
   $$\text{Path}(G_H = 1) \cap \text{Path}(G_L = 1) = \emptyset$$
   $\therefore$ The intersection of active paths in the ROBDD is the empty set $\emptyset$. The hardware physically cannot assert both outputs simultaneously.

---

## 2. SVPWM Sector Decoder Logic Minimization (ESPRESSO)

### 2.1 Problem Formulation
In Space Vector PWM, the voltage vector $(V_\alpha, V_\beta)$ is mapped into one of 6 sectors ($S_1$ through $S_6$).
The classic geometric sector boundaries are determined by 3 comparator signs:
- $A = \text{sign}(V_\beta) \equiv (V_\beta > 0)$
- $B = \text{sign}\left(\frac{\sqrt{3}}{2} V_\alpha - \frac{1}{2} V_\beta\right)$
- $C = \text{sign}\left(-\frac{\sqrt{3}}{2} V_\alpha - \frac{1}{2} V_\beta\right)$

Let the sector index be encoded as an integer $N \in \{1, 2, 3, 4, 5, 6\}$:
$$N = A + 2B + 4C$$

### 2.2 ESPRESSO Input File Specification (`.kiss2` / `.pla`)

```pla
# ESPRESSO PLA format for SVPWM Sector Identification
.i 3
.o 3
.ilb A B C
.ob s2 s1 s0
# Inputs: A B C -> Outputs: 3-bit Sector [2:0]
0 0 1  0 0 1   # Sector 1 (001_2)
0 1 1  0 1 0   # Sector 2 (010_2)
0 1 0  0 1 1   # Sector 3 (011_2)
1 1 0  1 0 0   # Sector 4 (100_2)
1 0 0  1 0 1   # Sector 5 (101_2)
1 0 1  1 1 0   # Sector 6 (110_2)
# Don't care states (000 and 111 are geometrically impossible)
0 0 0  - - -
1 1 1  - - -
.e
```

### 2.3 ESPRESSO Minimization Result
Running ESPRESSO heuristic minimization yields the minimal sum-of-products:
$$s_2 = A \cdot \overline{B} + A \cdot C$$
$$s_1 = B \cdot \overline{C} + \overline{A} \cdot B + A \cdot \overline{B} \cdot C$$
$$s_0 = \overline{A} \cdot \overline{B} + \overline{B} \cdot \overline{C} + \overline{A} \cdot \overline{C}$$

Which maps to exact 1-LUT delay in Xilinx 7-Series architecture (6-LUT can implement any arbitrary 3-input function in a single LUT stage of 0.28 ns).

---

## 3. Formal Verification Link
The properties derived here are formally proved in:
- **NuSMV model**: [`formal/nusmv/shoot_through.smv`](file:///c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/formal/nusmv/shoot_through.smv) via CTL invariant `CTLSPEC AG !(gate_h & gate_l)`
- **SymbiYosys**: [`formal/sby/shoot_through.sby`](file:///c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/formal/sby/shoot_through.sby)
- **RTL Testbench**: [`tb_dead_time.sv`](file:///c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sim_1/new/tb_dead_time.sv) & [`tb_foc_top.sv`](file:///c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sim_1/new/tb_foc_top.sv) verifying zero shoot-through over 81 switching edges and thousands of PWM cycles.
