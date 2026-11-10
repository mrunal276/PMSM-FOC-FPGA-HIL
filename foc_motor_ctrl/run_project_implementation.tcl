# run_project_implementation.tcl
# Runs synthesis and implementation on the Vivado project
open_project c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.xpr

# Reset previous failed runs
reset_run synth_1
reset_run impl_1

# Launch Synthesis
puts "=== Launching Synthesis ==="
launch_runs synth_1 -jobs 4
wait_on_run synth_1

if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

# Launch Implementation
puts "=== Launching Implementation ==="
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    puts "ERROR: Implementation failed!"
    exit 1
}

# Open routed design and export updated reports
open_run impl_1
report_utilization -file c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top_impl_utilization.rpt
report_timing_summary -file c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top_impl_timing.rpt

# Copy fresh bitstream to reports directory
file copy -force c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/foc_motor_ctrl.runs/impl_1/foc_top.bit c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/reports/foc_top.bit

puts "=== SUCCESS: Project implementation & bitstream complete ==="
close_project
exit 0
