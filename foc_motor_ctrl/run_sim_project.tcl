# Open existing project
open_project c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.xpr

# Set top to tb_foc_top
set_property top tb_foc_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1

# Launch simulation
launch_simulation -simset sim_1 -mode behavioral
run 100us
close_sim
close_project
exit
