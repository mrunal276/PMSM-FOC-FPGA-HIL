# Reference Paper Comparison & Hardware Benchmark

## 1. Selected Reference Paper
- **Citation**: Burak Tufekci, Bugra Onal, Hamza Dere, H. Fatih Ugurdag, *"Efficient FPGA Implementation of Field Oriented Control for 3-Phase Machine Drives,"* IEEE / University of North Texas / Ozyegin University, 2020.
- **Reference URL**: [Paper PDF Link](https://engineering.unt.edu/cse/research/labs/nsl/sites/default/files/people/documents/efficient_fpga_implementation_of_field_oriented_control_for_3-phase_machine_drives.pdf)
- **Target Device**: Xilinx Zynq-7020 FPGA

---

## 2. Comparative Metrics Table

| Metric | Reference Paper (Tufekci et al., 2020) | This Project (P11 FOC Controller + HIL) | Notes / Comparison Analysis |
|---|:---:|:---:|---|
| **Architecture** | Pipelined + Resource Shared (4 MAS modules) | Hand-HLS 2-DSP FDS/ILP Shared Datapath | Our design constrains multipliers strictly to 2 DSPs. |
| **Trigonometry** | CORDIC | 16-Stage Pipelined CORDIC | Both achieve $\le 1$ LSB precision. |
| **Control Loops** | Dual Current PI | Triple Loop: Speed PI + Dual Current PI | Our design includes on-chip outer-loop speed regulation. |
| **Modulation** | Space Vector PWM (SVM) | SVPWM with Dead-Time Insertion | Our design incorporates shoot-through interlocks. |
| **FOC Core Latency** | 84 cycles (II = 72 cycles) | ~96 cycles (FOC only) / ~176 cycles (FOC + HIL) | Our total budget accounts for on-chip HIL digital twin plant. |
| **Clock Frequency** | 100 MHz | 100 MHz target | Identical operating frequency. |
| **FPGA Platform** | Xilinx Zynq-7020 | Digilent Zybo Z7-10 (Zynq-7010) | We target a smaller, cost-effective FPGA slice budget. |
| **Resource Usage** | 1245 LUTs, 1014 FFs | Target: $\approx$ 1500 LUTs, 1200 FFs, 2 DSPs | Synthesis benchmark report to follow in Week 4. |
| **Verification** | RTL Simulation | Python Golden + HIL + Formal Proofs (NuSMV, SBY) | We add formal safety proofs against bridge shoot-through. |
| **Telemetry / Host** | Dedicated interface | RV32I Processor + APB CSR + UART Logger | RV32I soft core coordinates setpoints & fault interrupts. |

---

## 3. Engineering Analysis: Field-Weakening & High-Speed Saturation

During the 1000 RPM high-speed golden simulation evaluation, an important practical constraint was identified and documented:
- **Observed Operating Point**: At 1000 RPM with load, current reaches $I_d \approx -14\text{ A}$ and $I_q \approx 0.952\text{ A}$.
- **Voltage Assessment**: Using the SPMSM stator voltage equations at 1000 RPM ($104.7\text{ rad/s}$ mechanical, $\omega_e = 209.4\text{ rad/s}$):
  $$V_d = R_s I_d - \omega_e L_q I_q$$
  $$V_q = R_s I_q + \omega_e (L_d I_d + \lambda_m)$$
  The resultant vector magnitude $V_{mag} = \sqrt{V_d^2 + V_q^2}$ requires approximately $26.2\text{ V}$.
- **Hardware Bus Limit**: The base DC bus voltage $V_{bus}$ is normalized to $24.0\text{ V}$.
- **Design Finding**: Because the required terminal voltage exceeds the $24\text{ V}$ bus rail, the inverter operates in overmodulation/saturation rather than maintaining an ideal 90% linear modulation margin at this specific load.
- **Reporting Takeaway**: This is recorded as a physical plant boundary condition for the project documentation and comparative evaluation.
