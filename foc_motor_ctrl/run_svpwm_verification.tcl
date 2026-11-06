# run_svpwm_verification.tcl
# Run the three testbenches affected by the 2-stage SVPWM pipeline change:
#   1. tb_svpwm          (8 hand-crafted golden vectors, 2-cycle latency)
#   2. tb_svpwm_random   (2175 CSV-driven golden vectors)
#   3. tb_foc_pipeline   (22-cycle latency check)
# Usage: vivado -mode batch -source run_svpwm_verification.tcl -log run_svpwm_verification.log

open_project c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.xpr

# ---- tb_svpwm ----
puts "\n========== SIM 1: tb_svpwm =========="
set_property top tb_svpwm [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -simset sim_1 -mode behavioral
run 10us
close_sim

# ---- tb_svpwm_random ----
puts "\n========== SIM 2: tb_svpwm_random =========="
set_property top tb_svpwm_random [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -simset sim_1 -mode behavioral
run 500us
close_sim

# ---- tb_foc_pipeline ----
puts "\n========== SIM 3: tb_foc_pipeline =========="
set_property top tb_foc_pipeline [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1
launch_simulation -simset sim_1 -mode behavioral
run 20ms
close_sim

close_project
puts "\n========== All 3 simulations complete =========="
exit
