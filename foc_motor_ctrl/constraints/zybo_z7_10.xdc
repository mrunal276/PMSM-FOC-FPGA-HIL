## Master Constraints for Digilent Zybo Z7-10 (XC7Z010CLG400-1)
## PMSM Field-Oriented Control (FOC) Top-Level
## Pin assignments verified against:
##   https://github.com/Digilent/digilent-xdc/blob/master/Zybo-Z7-Master.xdc

## ============================================================
## 125 MHz System Clock (Sch=sysclk)
## ============================================================
set_property -dict { PACKAGE_PIN K17   IOSTANDARD LVCMOS33 } [get_ports { clk }];
create_clock -add -name sys_clk_pin -period 8.00 -waveform {0 4} [get_ports { clk }];

## ============================================================
## User Push Buttons (Active-HIGH, Sch=btn[0..2])
## At rest: logic 0 (10K pull-down). Pressed: logic 1 (3.3V).
## ============================================================
set_property -dict { PACKAGE_PIN K18   IOSTANDARD LVCMOS33 } [get_ports { btn_rst }];   # btn[0]
set_property -dict { PACKAGE_PIN P16   IOSTANDARD LVCMOS33 } [get_ports { start_btn }]; # btn[1]
set_property -dict { PACKAGE_PIN K19   IOSTANDARD LVCMOS33 } [get_ports { ext_irq }];   # btn[2]

## ============================================================
## User LEDs (Active-HIGH, Sch=led[0..3])
## ============================================================
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { led_alive }];   # led[0] LD0 (Heartbeat)
set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { led_startup }]; # led[1] LD1 (Align/Ramp open-loop)
set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { led_running }]; # led[2] LD2 (Closed-loop FOC)
set_property -dict { PACKAGE_PIN D18   IOSTANDARD LVCMOS33 } [get_ports { led_fault }];   # led[3] LD3 (Fault active)

## ============================================================
## Pmod JB — Inverter Gate Drive Outputs (Sch=jb_p[1..3], jb_n[1..3])
## ============================================================
set_property -dict { PACKAGE_PIN V15   IOSTANDARD LVCMOS33 } [get_ports { gate_ah }]; # jb_p[1]
set_property -dict { PACKAGE_PIN W15   IOSTANDARD LVCMOS33 } [get_ports { gate_al }]; # jb_n[1]
set_property -dict { PACKAGE_PIN T11   IOSTANDARD LVCMOS33 } [get_ports { gate_bh }]; # jb_p[2]
set_property -dict { PACKAGE_PIN T10   IOSTANDARD LVCMOS33 } [get_ports { gate_bl }]; # jb_n[2]
set_property -dict { PACKAGE_PIN W14   IOSTANDARD LVCMOS33 } [get_ports { gate_ch }]; # jb_p[3]
set_property -dict { PACKAGE_PIN Y14   IOSTANDARD LVCMOS33 } [get_ports { gate_cl }]; # jb_n[3]

## ============================================================
## Pmod JA — UART Telemetry TX (Sch=ja_p[1])
## ============================================================
set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports { uart_tx }]; # ja_p[1]

## ============================================================
## Timing Exceptions: False Paths for Asynchronous Inputs & Debug
## ============================================================
set_false_path -from [get_ports btn_rst]
set_false_path -from [get_ports start_btn]
set_false_path -from [get_ports ext_irq]
set_false_path -through [get_cells -hierarchical -filter {NAME =~ *gen_ila*U_XSDB_SLAVE*}]

## ============================================================
## Multicycle Path: SVPWM Stage-1 (64-bit overmodulation divide)
## SVPWM runs at 25 kHz (carrier rate, once per 5000 cycles).
## The stage-1 divide path has 40 µs to settle.
## Covers all internal SVPWM registers including phys_opt *psdsp*.
## ============================================================
set_multicycle_path 35 -setup -to [get_cells -hierarchical -filter {NAME =~ *u_pipeline/u_svpwm/*}]
set_multicycle_path 34 -hold  -to [get_cells -hierarchical -filter {NAME =~ *u_pipeline/u_svpwm/*}]

## ============================================================
## Multicycle Path: PMSM Plant Model (HIL Digital Twin)
## The plant updates once per control cycle (20/25 kHz carrier rate,
## once every 5,000 clock cycles = 40 µs).
## The combinational state increment cone has 40 µs to settle.
## Covers all internal plant registers including phys_opt *psdsp*.
## ============================================================
set_multicycle_path 5 -setup -to [get_cells -hierarchical -filter {NAME =~ *u_plant/*}]
set_multicycle_path 4 -hold  -to [get_cells -hierarchical -filter {NAME =~ *u_plant/*}]
