# Tcl script to run complete Synthesis and Implementation flow for foc_top on Zybo Z7-10
set_param general.maxThreads 8

# Create in-memory project
create_project -in_memory -part xc7z010clg400-1

# Read all RTL sources
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/clarke.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/cordic.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/park.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/speed_pi.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/current_pi.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/inv_park.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/svpwm.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/foc_pipeline.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/pwm_generator.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/dead_time.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/startup_fsm.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/fault_fsm.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/rv32i_regfile.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/rv32i_alu.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/rv32i_decode.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/rv32i_core.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/apb_csr.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/uart_logger.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/pmsm_plant.sv
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sources_1/new/foc_top.sv

# Read XDC Constraints
read_xdc c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/constraints/zybo_z7_10.xdc

# Run Synthesis
puts "=== Starting Synthesis ==="
synth_design -top foc_top -part xc7z010clg400-1

# Run Implementation Steps
puts "=== Starting Logic Optimization (opt_design) ==="
opt_design

puts "=== Starting Placement (place_design) ==="
place_design

puts "=== Starting Routing (route_design) ==="
route_design

# Reports
puts "=== Generating Implementation Reports ==="
report_utilization -file c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top_impl_utilization.rpt
report_timing_summary -file c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top_impl_timing.rpt

puts "=== Writing Checkpoint ==="
write_checkpoint -force c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top_routed.dcp

puts "=== Generating Bitstream ==="
write_bitstream -force c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top.bit

puts "=== Implementation & Bitstream Complete! ==="
exit
