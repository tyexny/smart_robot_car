# Rebuild the submitted source with the audited implementation settings.
set project_root [file normalize [file dirname [info script]]]
open_project [file join $project_root ov5640_ddr3_hdmi.gprj]
set_option -top_module gao_ov5640_ddr_hdmi
set_option -output_base_name ov5640_ddr3_hdmi
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -convert_sdp32_36_to_sdp16_18 1
set_option -bit_compress 0
set_option -place_option 3
set_option -route_option 0
# Export the same routed implementation for the RGMII port skew audit.
set_option -gen_sdf 1
set_option -gen_verilog_sim_netlist 1
set_option -inc_pnr 0
set_option -inc_place 0
set_option -clock_route_order 0
set_option -route_maxfan 23
set_option -timing_driven 1
set_option -correct_hold_violation 1
run all
puts "SMART_ROBOT_CAR_BUILD_FINISHED"
exit
