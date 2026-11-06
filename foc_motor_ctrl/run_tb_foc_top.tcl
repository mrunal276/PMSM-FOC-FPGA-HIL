# Tcl script to simulate tb_foc_top
set_param general.maxThreads 8
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

# Read Testbench
read_verilog -sv c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.srcs/sim_1/new/tb_foc_top.sv

# Launch simulation
set_property top tb_foc_top [current_fileset -simset]
launch_simulation -mode behavioral
run all
exit
