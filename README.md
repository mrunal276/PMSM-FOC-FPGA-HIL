# P11 — Field-Oriented Motor Controller with Hardware-in-the-Loop (HIL)
> **Target:** Digilent Zybo Z7-10 (`xc7z010clg400-1`, Speed Grade -1) · Vivado 2025.1 · 125 MHz  
> **Status: Timing closed (WNS = +0.321 ns, 0 failing endpoints). Bitstream generated and hardware-validated via ILA.**

---

## Project Structure

```
rtl_design_verif_project/
│
├── 01_rtl/                         ← All SystemVerilog RTL (21 modules + 20 testbenches)
│   ├── sources/                    ← Production RTL modules
│   │   ├── clarke.sv               Clarke transform (4-stage pipeline, 125 MHz closed)
│   │   ├── cordic.sv               16-stage pipelined CORDIC sin/cos
│   │   ├── park.sv                 Park transform (Iα·cos + Iβ·sin)
│   │   ├── inv_park.sv             Inverse Park transform
│   │   ├── foc_shared_2dsp.sv      Hand-HLS 2-DSP FDS/ILP scheduler (21-state FSM)
│   │   ├── foc_pipeline.sv         Full FOC pipeline (41-cycle latency)
│   │   ├── current_pi.sv           Id/Iq PI controllers with anti-windup
│   │   ├── speed_pi.sv             Speed PI controller
│   │   ├── svpwm.sv                Space-Vector PWM (6-sector, 2-stage pipelined)
│   │   ├── dead_time.sv            Shoot-through interlock + dead-time blanking
│   │   ├── pwm_generator.sv        Triangle-carrier PWM generator
│   │   ├── startup_fsm.sv          Startup FSM: Align → Ramp → Handover → Running
│   │   ├── fault_fsm.sv            Fault FSM: over-current/stall → 1-cycle gate trip
│   │   ├── pmsm_plant.sv           Fixed-point PMSM digital twin (HIL plant)
│   │   ├── rv32i_core.sv           RV32I RISC-V soft-core processor
│   │   ├── rv32i_alu.sv            RV32I ALU
│   │   ├── rv32i_decode.sv         RV32I instruction decoder
│   │   ├── rv32i_regfile.sv        RV32I register file
│   │   ├── apb_csr.sv              APB CSR (speed ref, id/iq readback, fault IRQ)
│   │   ├── uart_logger.sv          UART telemetry logger (115200 baud)
│   │   └── foc_top.sv              Top-level integration + ILA probes
│   │
│   ├── testbenches/                ← Self-checking testbenches (20 files)
│   │   ├── tb_foc_top.sv           System integration: 7/7 PASS
│   │   ├── tb_foc_pipeline.sv      Pipeline bit-accuracy: 500/500 PASS
│   │   ├── tb_foc_shared_2dsp.sv   2-DSP FSM verification
│   │   ├── tb_cordic.sv / tb_cordic_golden.sv
│   │   ├── tb_clarke.sv / tb_park.sv / tb_inv_park.sv
│   │   ├── tb_svpwm.sv / tb_svpwm_random.sv / tb_switching_path.sv
│   │   ├── tb_dead_time.sv
│   │   ├── tb_current_pi.sv / tb_speed_pi.sv
│   │   ├── tb_startup_fsm.sv / tb_fault_fsm.sv
│   │   ├── tb_pmsm_plant.sv
│   │   ├── tb_rv32i_core.sv
│   │   ├── tb_apb_csr.sv
│   │   └── tb_pwm_generator.sv
│   │
│   └── constraints/
│       └── zybo_z7_10.xdc          Pin assignments + 125 MHz clock constraint
│
├── 02_golden_model/                ← Bit-true Python fixed-point golden model
│   ├── foc_golden.py               Full system simulator (0→500→1000 RPM step)
│   ├── transforms.py               Clarke, Park, Inverse Park
│   ├── cordic_model.py             16-iteration CORDIC model
│   ├── pi_controller.py            Fixed-point PI with anti-windup
│   ├── svpwm.py                    SVPWM 6-sector model
│   ├── pmsm_plant.py               PMSM motor ODE plant model
│   ├── speed_pi.py                 Speed PI model
│   ├── generate_pipeline_vectors.py  Generates 500 test vectors for tb_foc_pipeline
│   ├── generate_svpwm_vectors.py   Generates SVPWM test vectors
│   ├── rtl_arithmetic_reference.py Fixed-point arithmetic reference
│   ├── foc_golden_results.csv      Full 180 ms simulation data
│   ├── foc_step_response.csv       Step-response trace
│   ├── foc_speed_response.png      Speed step response plot
│   ├── foc_current_response.png    Id/Iq step response plot
│   └── foc_svpwm_duties.png        SVPWM duty cycle plot
│
├── 03_hls_schedule/                ← Hand-HLS FDS/ILP 2-DSP schedule
│   ├── schedule_table.md           25-op operation table (15 MUL + 10 ADD) on DSP0/DSP1
│   └── hand_hls_2dsp_fds_ilp_schedule.pdf  Gantt chart + ILP formulation
│
├── 04_formal/                      ← Formal verification proofs
│   ├── nusmv/
│   │   └── shoot_through.smv       CTL: AG !(gate_h & gate_l) — no shoot-through
│   ├── sby/
│   │   ├── shoot_through.sby       SymbiYosys BMC config (depth 20, boolector)
│   │   └── shoot_through.sv        SVA assertions on all 3 phases
│   └── bdd_switching.md            ESPRESSO minimization + BDD derivation of SVPWM interlock
│
├── 05_paper_impl/                  ← Reference paper implementation (Tufekci et al., 2020)
│   ├── 00_READ_ME_FIRST_File_Guide.md
│   ├── paper/
│   │   └── Tufekci2020_EWDTS_FOC_FPGA.pdf
│   ├── rtl/
│   │   ├── cordic_rotate.sv        CORDIC vector rotator (0 DSP, 18-clk latency)
│   │   ├── foc_current_cordic.sv   Paper FOC loop: Clarke+Park+PI+InvPark on 1 DSP
│   │   └── foc_fixed_pkg.svh       Q3.13 / Q4.14 / 48-bit accumulator helpers
│   ├── tb/
│   │   └── tb_paper_cordic.sv      Self-checking: 4096 CORDIC + 2000 FOC + anti-windup
│   ├── sim_out/
│   │   ├── tb_paper_cordic.log     PASS: max error 1.07 LSB, latency 51 clk
│   │   ├── system_arch2.log        PASS: 0 shoot-through, fault in 1 clk
│   │   ├── p11_arch2.csv           Full 180 ms HIL run data
│   │   └── p11_arch2_fast.csv      High-resolution current ripple data
│   ├── results/
│   │   ├── metrics_arch2.json      All benchmark metrics (latency, rise time, ripple)
│   │   ├── overview_arch2.png      Full HIL run waveform
│   │   ├── steps_arch2.png         Step response zoom
│   │   └── ripple_arch2.png        Current ripple plot
│   └── vivado_synth_paper.tcl      OOC synthesis script (1 DSP48, WNS = +0.161 ns)
│
├── 06_reports/                     ← Final deliverable reports
│   ├── foc_top.bit                 ← FPGA bitstream (2.08 MB, Zybo Z7-10)
│   ├── foc_top.ltx                 Probe file for Vivado Hardware Manager
│   ├── paper_comparison.md         Benchmark: Paper vs P11 (latency, DSP, LUT, accuracy)
│   ├── Field-Weakening Limitation.md  Notes on 1000 RPM voltage saturation
│   ├── timing/
│   │   ├── foc_top_impl_timing.rpt     Final timing: WNS=+0.321 ns, 0 failing endpoints
│   │   ├── foc_top_impl_utilization.rpt  LUTs=11777, DSP=44, FF=6822
│   │   └── hierarchical_utilization.rpt  Per-module breakdown (u_foc_shared_2dsp = 2 DSP)
│   ├── figures/
│   │   ├── ila_hardware_capture.png   Full 1024-sample ILA waveform
│   │   └── ila_transition_zoom.png    State transition + shoot-through zoom
│   └── ila_capture/
│       ├── waveform.csv             Raw ILA data (1024 samples, all probes)
│       ├── hw_ila_data_1.wcfg       ILA window config
│       ├── waveform.vcd             VCD export
│       └── waveform.dmp             Binary dump
│
├── 07_vivado_project/              ← Vivado project file (open to rebuild)
│   └── foc_motor_ctrl.xpr          Open in Vivado 2025.1 to access full project
│
├── 08_scripts/                     ← Useful run scripts
│   ├── run_project_implementation.tcl  Full synth+impl+bitstream (batch mode)
│   ├── run_tb_foc_top.tcl              Run tb_foc_top simulation
│   ├── run_sim_project.tcl             Run all simulations
│   └── run_svpwm_verification.tcl      Run SVPWM random verification
│
├── foc_motor_ctrl/                 ← Full Vivado project (source of truth)
│   └── ...                         (Vivado-managed, do not edit directly)
│
├── Tufekci2020_Paper_Implementation-main/  ← Original paper impl download
│   └── ...
│
└── README.md                       ← This file
```

---

## Key Results Summary

| Metric | Value |
|---|---|
| **Clock** | 125.000 MHz (`sys_clk_pin`, Zybo Z7-10 K17) |
| **WNS** | **+0.321 ns** (setup timing fully closed) |
| **TNS** | **0.000 ns** (0 / 14,718 endpoints failing) |
| **LUTs** | 11,777 / 17,600 (66.9%) |
| **DSP48E1** | 44 / 80 (55%) — hand-HLS 2-DSP inner loop uses exactly **2** |
| **FOC loop latency** | 96 cycles / 5,000 available (1.92%) |
| **Paper arch latency** | 51 cycles / 5,000 available (1.02%), 1 DSP48 |
| **Golden model accuracy** | 500/500 vectors, max error ≤ 1 LSB |
| **System integration** | 7/7 tests PASS |
| **Formal proof** | NuSMV `AG !(gH & gL)` + SymbiYosys BMC depth 20 |
| **Hardware demo** | Ramp→Handover→Closed-loop→Fault trip in 1 clock (ILA verified) |

---

## How to Rebuild

### 1. Run Full Synthesis + Implementation + Bitstream
```tcl
vivado -mode batch -source 08_scripts/run_project_implementation.tcl
```

### 2. Run System Integration Testbench
```tcl
vivado -mode batch -source 08_scripts/run_tb_foc_top.tcl
```

### 3. Run Paper Architecture Simulation (Xsim)
Open `05_paper_impl/vivado_synth_paper.tcl` in Vivado or launch with:
```tcl
vivado -mode batch -source 05_paper_impl/vivado_synth_paper.tcl
```

### 4. Regenerate Golden Model Vectors
```bash
cd 02_golden_model
python foc_golden.py
python generate_pipeline_vectors.py
```

---

## References
- **Tufekci B. et al.**, "Efficient FPGA Implementation of Field Oriented Control for 3-Phase Machine Drives," IEEE EWDTS 2020. DOI: [10.1109/EWDTS50664.2020.9224884](https://doi.org/10.1109/EWDTS50664.2020.9224884)
- **Target Board:** Digilent Zybo Z7-10. [Reference Manual](https://digilent.com/reference/programmable-logic/zybo-z7/reference-manual)
- **Vivado:** AMD/Xilinx Vivado 2025.1
