# Board reference clock and the complete PLL/CLKDIV clock tree.
# Explicit generated clocks preserve their relationships during placement too.
create_clock -name telemetry_sys50 -period 20.000 [get_ports {clk50M}]
create_generated_clock -name ddr_memory200 -source [get_ports {clk50M}] -multiply_by 4 [get_pins {Gowin_PLL/PLL_inst/CLKOUT2}]
create_generated_clock -name local_sys50 -source [get_ports {clk50M}] -divide_by 1 [get_pins {Gowin_PLL/PLL_inst/CLKOUT1}]
create_generated_clock -name video_serial375 -source [get_pins {Gowin_PLL/PLL_inst/CLKOUT1}] -multiply_by 15 -divide_by 2 [get_pins {camera_pll/u_pll/PLL_inst/CLKOUT0}]
create_generated_clock -name camera_xclk25 -source [get_pins {Gowin_PLL/PLL_inst/CLKOUT1}] -divide_by 2 [get_pins {camera_pll/u_pll/PLL_inst/CLKOUT1}]
create_generated_clock -name telemetry_video75 -source [get_pins {camera_pll/u_pll/PLL_inst/CLKOUT0}] -divide_by 5 [get_pins {u_clkdiv/CLKOUT}]
create_generated_clock -name ddr_ui50 -source [get_pins {Gowin_PLL/PLL_inst/CLKOUT2}] -divide_by 4 [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]

# DVP clock is returned by the sensor, which has its own PLL and auto divider.
# 100 MHz is a conservative timing budget, NOT a measured PCLK frequency.
# With 25 MHz XCLK, 3037=13, 3036=69, 3035=21, 3034=1a,
# the sensor clock before the DVP scaler is 87.5 MHz; the scaler divides it.
create_clock -name camera_pclk -period 10.000 [get_ports {camera_pclk}]

# CDC exceptions apply only to the first synchronizer stage, never whole clocks.
# Bound pointer routing to less than the fastest FIFO clock period (10 ns).
# The second stage and all DDR PHY data/calibration paths stay timed normally.
set_max_delay -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/wr_data_fifo/fifo_inst/Big.rq1_wptr_*_s0/D}] 10.000
set_false_path -hold -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/wr_data_fifo/fifo_inst/Big.rq1_wptr_*_s0/D}]
set_max_delay -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/wr_data_fifo/fifo_inst/Big.wq1_rptr_*_s0/D}] 10.000
set_false_path -hold -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/wr_data_fifo/fifo_inst/Big.wq1_rptr_*_s0/D}]
set_max_delay -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.rq1_wptr_*_s0/D}] 10.000
set_false_path -hold -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.rq1_wptr_*_s0/D}]
set_max_delay -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.wq1_rptr_*_s0/D}] 10.000
set_false_path -hold -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/Small.wq1_rptr_*_s0/D}]

# Mailbox request/acknowledge each cross through a two-flop synchronizer.
set_max_delay -to [get_pins {u_telemetry_cdc/req_meta_s0/D u_telemetry_cdc/ack_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_telemetry_cdc/req_meta_s0/D u_telemetry_cdc/ack_meta_s0/D}]
# Held payload is captured only after two destination cycles (26.667 ns),
# and stays unchanged until the acknowledgement returns. Keep a 20 ns bound.
set_max_delay -from [get_pins {u_telemetry_cdc/held*_s0/Q}] -to [get_pins {u_telemetry_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_telemetry_cdc/held*_s0/Q}] -to [get_pins {u_telemetry_cdc/dst_data*_s0/D}]

# Scan metadata uses the same held-payload mailbox; map RAM is dual-clock.
set_max_delay -to [get_pins {u_scan_metadata_cdc/req_meta_s0/D u_scan_metadata_cdc/ack_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_scan_metadata_cdc/req_meta_s0/D u_scan_metadata_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_scan_metadata_cdc/held*_s0/Q}] -to [get_pins {u_scan_metadata_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_scan_metadata_cdc/held*_s0/Q}] -to [get_pins {u_scan_metadata_cdc/dst_data*_s0/D}]

# Existing two-flop DDR-ready synchronizers in the system/video domains.
# Navigation uses the same synchronized toggle/held-payload protocol.
# Only its first synchronizer stages and held payload get CDC exceptions.
set_max_delay -to [get_pins {u_nav_metadata_cdc/req_meta_s0/D u_nav_metadata_cdc/ack_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_nav_metadata_cdc/req_meta_s0/D u_nav_metadata_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_nav_metadata_cdc/held*_s0/Q}] -to [get_pins {u_nav_metadata_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_nav_metadata_cdc/held*_s0/Q}] -to [get_pins {u_nav_metadata_cdc/dst_data*_s0/D}]

set_max_delay -to [get_pins {u_ddr_reset_retry/calib_meta_s0/D image_ready_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_ddr_reset_retry/calib_meta_s0/D image_ready_meta_s0/D}]

# ACG720 RTL8211F, Gigabit RGMII with PHY internal RX/TX 2 ns delays.
# Keep clock creation in this single SDC; Gowin sorts separate SDC filenames.
# No whole-clock CDC cuts are used.
# Regenerated PLL output: see docs/PLL_CLOCK_MODEL.md; retain all CDC checks.
create_clock -name ethernet_tx125 -period 8.000 -waveform {0 4} [get_pins {u_vision_ethernet/u_eth_pll/PLL_inst/CLKOUT0}]
create_generated_clock -name ethernet_tx125_90 -source [get_pins {u_vision_ethernet/u_eth_pll/PLL_inst/CLKOUT0}] -divide_by 1 -phase 90 [get_pins {u_vision_ethernet/u_eth_pll/PLL_inst/CLKOUT1}]
create_clock -name ethernet_rx125 -period 8.000 [get_ports {eth_rxc}]
create_clock -name audio_i2s_bclk -period 488.421 [get_ports {audio_bclk}]
# ES8388 launches ADC data/LRCK on BCLK falling edges; allow 20 ns for board skew.
set_input_delay -clock audio_i2s_bclk -clock_fall -max 20.000 [get_ports {audio_adc_data audio_lrclk}]
set_input_delay -clock audio_i2s_bclk -clock_fall -min 0.000 [get_ports {audio_adc_data audio_lrclk}]
# Codec initialization changes once after boot; synchronize it before allowing FIFO writes.
set_max_delay -to [get_pins {u_vision_ethernet/audio_ready_meta_bclk_s0/D}] 20.000
set_false_path -hold -to [get_pins {u_vision_ethernet/audio_ready_meta_bclk_s0/D}]
# Bound only the Gray FIFO's asynchronous first captures; second stages remain timed.
set_max_delay -to [get_pins {u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_0_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_1_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_2_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_3_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_4_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_5_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_6_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_7_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_8_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_0_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_1_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_2_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_3_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_4_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_5_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_6_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_7_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_8_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_0_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_1_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_2_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_3_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_4_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_5_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_6_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_7_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/rdside_wr_pntr_gray_8_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_0_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_1_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_2_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_3_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_4_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_5_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_6_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_7_s0/D u_vision_ethernet/u_audio_rx/adc_fifo/async_fifo_ctrl_inst/wrside_rd_pntr_gray_8_s0/D}]

# RX delays describe the NEXT data transition after the delayed PHY RXC.
# +1.2/+2.8 ns correspond to the same valid window as -2.8/-1.2 ns
# relative to the preceding transition, but retain the correct hold edge in
# Gowin (which has no per-edge false-path constraints).
# PHY RX timing budgets are supplied on BOTH clock edges.
set_input_delay -clock ethernet_rx125 -max 2.800 [get_ports {eth_rxd[*] eth_rxdv}]
set_input_delay -clock ethernet_rx125 -min 1.200 [get_ports {eth_rxd[*] eth_rxdv}]
set_input_delay -clock ethernet_rx125 -clock_fall -add_delay -max 2.800 [get_ports {eth_rxd[*] eth_rxdv}]
set_input_delay -clock ethernet_rx125 -clock_fall -add_delay -min 1.200 [get_ports {eth_rxd[*] eth_rxdv}]
# Keep all six TX outputs timed and exported as ordinary output paths.
# Gowin 1.9.12.04 omits the forwarding OBUF SDF arc when eth_gtxc itself
# carries a generated clock. Zero delays here are MEASUREMENT constraints,
# not the PHY setup/hold model; their absolute slack is not the PHY verdict.
# PHY +/-2 ns DDR windows are checked from the SAME full routed SDF/VO/FS
# by tests/audit_rgmii_tx_sdf.py, including all clock/data route and buffer arcs.
# Missing arcs or a negative window MUST fail that audit. No TX false paths.
set_output_delay -clock ethernet_tx125 -max 0.000 [get_ports {eth_gtxc eth_txd[*] eth_txen}]
set_output_delay -clock ethernet_tx125 -min 0.000 [get_ports {eth_gtxc eth_txd[*] eth_txen}]
set_output_delay -clock ethernet_tx125 -clock_fall -add_delay -max 0.000 [get_ports {eth_gtxc eth_txd[*] eth_txen}]
set_output_delay -clock ethernet_tx125 -clock_fall -add_delay -min 0.000 [get_ports {eth_gtxc eth_txd[*] eth_txen}]

# Row RAM ownership toggles cross camera/125 MHz domains through two FFs.
set_max_delay -to [get_pins {u_vision_ethernet/u_video_packets/ack_meta*_s0/D u_vision_ethernet/u_video_packets/req_meta*_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_video_packets/ack_meta*_s0/D u_vision_ethernet/u_video_packets/req_meta*_s0/D}]
# Frame/line metadata is held with its row until network acknowledgement.
set_max_delay -from [get_pins {u_vision_ethernet/u_video_packets/frame0*_s0/Q u_vision_ethernet/u_video_packets/frame1*_s0/Q u_vision_ethernet/u_video_packets/line0*_s0/Q u_vision_ethernet/u_video_packets/line1*_s0/Q}] -to [get_pins {u_vision_ethernet/u_video_packets/active_frame*_s0/D u_vision_ethernet/u_video_packets/active_line*_s0/D u_vision_ethernet/u_video_packets/latest_frame_id*_s0/D u_vision_ethernet/u_video_packets/active_bank_s0/D u_vision_ethernet/u_video_packets/latest_frame_id*_s0/CE u_vision_ethernet/u_video_packets/snapshot_frame0*_s0/D u_vision_ethernet/u_video_packets/snapshot_frame1*_s0/D u_vision_ethernet/u_video_packets/snapshot_line0*_s0/D u_vision_ethernet/u_video_packets/snapshot_line1*_s0/D}] 12.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_video_packets/frame0*_s0/Q u_vision_ethernet/u_video_packets/frame1*_s0/Q u_vision_ethernet/u_video_packets/line0*_s0/Q u_vision_ethernet/u_video_packets/line1*_s0/Q}] -to [get_pins {u_vision_ethernet/u_video_packets/active_frame*_s0/D u_vision_ethernet/u_video_packets/active_line*_s0/D u_vision_ethernet/u_video_packets/latest_frame_id*_s0/D u_vision_ethernet/u_video_packets/active_bank_s0/D u_vision_ethernet/u_video_packets/latest_frame_id*_s0/CE u_vision_ethernet/u_video_packets/snapshot_frame0*_s0/D u_vision_ethernet/u_video_packets/snapshot_frame1*_s0/D u_vision_ethernet/u_video_packets/snapshot_line0*_s0/D u_vision_ethernet/u_video_packets/snapshot_line1*_s0/D}]
# Capture-ready is a slow level crossing into the conservative 100 MHz camera
# clock. It changes only at initialization; the first-stage route may take
# two camera cycles, followed by the existing two-FF synchronizer.
set_max_delay -to [get_pins {u_vision_ethernet/u_video_packets/enable_meta_s0/D}] 20.000
set_max_delay -to [get_pins {u_vision_ethernet/u_eth/tx_reset_sync_0_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_video_packets/enable_meta_s0/D u_vision_ethernet/u_eth/tx_reset_sync_0_s0/D}]

# SW4 is a slow asynchronous board input. Debounce after its first two FFs;
# SW3 is also a maintained level switch; time both second synchronizer stages.
# SW2: constrain only asynchronous first capture; second stages remain timed.
# SW1: 20 ms debounce and two-FF transfer into the video domain.
set_max_delay -from [get_ports {imu_panel_enable}] -to [get_pins {u_imu_panel_switch/switch_meta_s0/D}] 20.000
set_false_path -hold -from [get_ports {imu_panel_enable}] -to [get_pins {u_imu_panel_switch/switch_meta_s0/D}]
set_max_delay -to [get_pins {imu_panel_meta_s0/D}] 20.000
set_false_path -hold -to [get_pins {imu_panel_meta_s0/D}]
set_max_delay -from [get_ports {obstacle_page_enable}] -to [get_pins {u_obstacle_page_switch/switch_meta_s0/D}] 20.000
set_false_path -hold -from [get_ports {obstacle_page_enable}] -to [get_pins {u_obstacle_page_switch/switch_meta_s0/D}]
set_max_delay -to [get_pins {obstacle_page_meta_s0/D}] 20.000
set_false_path -hold -to [get_pins {obstacle_page_meta_s0/D}]
set_max_delay -from [get_ports {speed_page_enable}] -to [get_pins {u_speed_page_switch/switch_meta_s0/D}] 20.000
set_false_path -hold -from [get_ports {speed_page_enable}] -to [get_pins {u_speed_page_switch/switch_meta_s0/D}]
set_max_delay -to [get_pins {speed_page_meta_s0/D}] 20.000
set_false_path -hold -to [get_pins {speed_page_meta_s0/D}]

# distribute the stable recognition level through separate two-FF pairs.
# Keep only first-capture paths asynchronous; all second stages stay timed.
set_max_delay -from [get_ports {recognition_enable}] -to [get_pins {u_recognition_switch/switch_meta_s0/D}] 20.000
set_false_path -hold -from [get_ports {recognition_enable}] -to [get_pins {u_recognition_switch/switch_meta_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/recognition_meta_camera_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_vision_ethernet/recognition_meta_camera_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/recognition_meta_tx_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/recognition_meta_tx_s0/D}]
# RX/video consume the local reset-release chains below; their old redundant
# recognition_meta data chains have no consumers and are optimized away.

# Async assertion / synchronous release for the SW4 session resets.
# Only the reset-release synchronizers themselves take raw asynchronous PRESET.
# Synthesis may merge identical per-clock chains across the mailbox instances;
# these two patterns select only the meta/sync PRESET ports of those chains.
# Audit the actual matches after synthesis. Stage-to-stage D and final Q to
# every ordinary logic CLEAR/PRESET remain timed for recovery and removal.
# Gowin SUG940 5.4.1: omit the analysis qualifier to cover all four
# analyses on these same asynchronous synchronizer PRESET pins. Ordinary
# reset consumers and both D stages remain outside this exception.
set_false_path -to [get_pins {u_vision_ethernet/*reset*meta*_s*/PRESET u_vision_ethernet/*reset*sync*_s*/PRESET}]

# Coherent latest-frame mailbox TX -> RX and boxes mailbox RX -> video.
set_max_delay -to [get_pins {u_vision_ethernet/u_video_id_cdc/req_meta_s0/D u_vision_ethernet/u_video_id_cdc/ack_meta_s0/D u_vision_ethernet/u_boxes_cdc/req_meta_s0/D u_vision_ethernet/u_boxes_cdc/ack_meta_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_video_id_cdc/req_meta_s0/D u_vision_ethernet/u_video_id_cdc/ack_meta_s0/D u_vision_ethernet/u_boxes_cdc/req_meta_s0/D u_vision_ethernet/u_boxes_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_vision_ethernet/u_video_id_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_video_id_cdc/dst_data*_s0/D}] 12.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_video_id_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_video_id_cdc/dst_data*_s0/D}]
set_max_delay -from [get_pins {u_vision_ethernet/u_boxes_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_boxes_cdc/dst_frame_id*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_count*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_coords*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_classes*_s0/D u_vision_ethernet/u_boxes_cdc/dst_tracker_session*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_track_ids*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_confidences*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_observed*_s0/D u_vision_ethernet/u_boxes_cdc/dst_label_text*_s0/D u_vision_ethernet/u_boxes_cdc/dst_label_length*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_boxes_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_boxes_cdc/dst_frame_id*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_count*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_coords*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_classes*_s0/D u_vision_ethernet/u_boxes_cdc/dst_tracker_session*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_track_ids*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_confidences*_s0/D u_vision_ethernet/u_boxes_cdc/dst_box_observed*_s0/D u_vision_ethernet/u_boxes_cdc/dst_label_text*_s0/D u_vision_ethernet/u_boxes_cdc/dst_label_length*_s0/D}]

# SYS50 -> TX125 selected tracker identity. Only the first req/ack captures
# and the complete acknowledged held-payload capture receive exceptions.
set_max_delay -to [get_pins {u_vision_ethernet/u_follow_control_cdc/req_meta_s0/D u_vision_ethernet/u_follow_control_cdc/ack_meta_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_follow_control_cdc/req_meta_s0/D u_vision_ethernet/u_follow_control_cdc/ack_meta_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/u_task_command_cdc/req_meta_s0/D u_vision_ethernet/u_task_command_cdc/ack_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_task_command_cdc/req_meta_s0/D u_vision_ethernet/u_task_command_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_vision_ethernet/u_task_command_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_task_command_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_task_command_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_task_command_cdc/dst_data*_s0/D}]
# MLX90640 temperature uses the same acknowledged held-payload protocol:
# Ethernet RX -> SYS50, followed by SYS50 -> final HDMI overlay.
set_max_delay -to [get_pins {u_vision_ethernet/u_thermal_receive_cdc/req_meta_s0/D u_vision_ethernet/u_thermal_receive_cdc/ack_meta_s0/D u_thermal_display_cdc/req_meta_s0/D u_thermal_display_cdc/ack_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_thermal_receive_cdc/req_meta_s0/D u_vision_ethernet/u_thermal_receive_cdc/ack_meta_s0/D u_thermal_display_cdc/req_meta_s0/D u_thermal_display_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_vision_ethernet/u_thermal_receive_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_thermal_receive_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_thermal_receive_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_thermal_receive_cdc/dst_data*_s0/D}]
set_max_delay -from [get_pins {u_thermal_display_cdc/held*_s0/Q}] -to [get_pins {u_thermal_display_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_thermal_display_cdc/held*_s0/Q}] -to [get_pins {u_thermal_display_cdc/dst_data*_s0/D}]
# Only asynchronous assertion on the reset-release synchronizer itself.
set_false_path -to [get_pins {u_thermal_display_cdc/*reset*meta*_s*/PRESET u_thermal_display_cdc/*reset*sync*_s*/PRESET}]
set_max_delay -from [get_pins {u_vision_ethernet/u_follow_control_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_follow_control_cdc/dst_data*_s0/D}] 12.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_follow_control_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_follow_control_cdc/dst_data*_s0/D}]

# ARP/peer MAC mailboxes use the same stable-payload handshake.
set_max_delay -to [get_pins {u_vision_ethernet/u_arrival_cdc/req_meta_s0/D u_vision_ethernet/u_arrival_cdc/ack_meta_s0/D u_vision_ethernet/u_thermal_rpc/u_request_cdc/req_meta_s0/D u_vision_ethernet/u_thermal_rpc/u_request_cdc/ack_meta_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_arrival_cdc/req_meta_s0/D u_vision_ethernet/u_arrival_cdc/ack_meta_s0/D u_vision_ethernet/u_thermal_rpc/u_request_cdc/req_meta_s0/D u_vision_ethernet/u_thermal_rpc/u_request_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_vision_ethernet/u_arrival_cdc/held*_s0/Q u_vision_ethernet/u_thermal_rpc/u_request_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_arrival_cdc/dst_data*_s0/D u_vision_ethernet/u_thermal_rpc/u_request_cdc/dst_data*_s0/D}] 12.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_arrival_cdc/held*_s0/Q u_vision_ethernet/u_thermal_rpc/u_request_cdc/held*_s0/Q}] -to [get_pins {u_vision_ethernet/u_arrival_cdc/dst_data*_s0/D u_vision_ethernet/u_thermal_rpc/u_request_cdc/dst_data*_s0/D}]
set_false_path -to [get_pins {u_vision_ethernet/u_arrival_cdc/*reset*meta*_s*/PRESET u_vision_ethernet/u_arrival_cdc/*reset*sync*_s*/PRESET u_vision_ethernet/u_thermal_rpc/u_request_cdc/*reset*meta*_s*/PRESET u_vision_ethernet/u_thermal_rpc/u_request_cdc/*reset*sync*_s*/PRESET}]
# Only the asynchronous I2C first captures are excepted; synchronizer second
# stages and the complete master/transport datapath remain fully timed.
set_false_path -from [get_ports {thermal_scl}] -to [get_pins {u_vision_ethernet/u_thermal_rpc/u_i2c/scl_meta_s0/D}]
set_false_path -from [get_ports {thermal_sda}] -to [get_pins {u_vision_ethernet/u_thermal_rpc/u_i2c/sda_meta_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/u_eth/peer_req_tx_0_s0/D u_vision_ethernet/u_eth/arp_req_tx_0_s0/D u_vision_ethernet/u_eth/peer_ack_rx_0_s0/D u_vision_ethernet/u_eth/arp_ack_rx_0_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_eth/peer_req_tx_0_s0/D u_vision_ethernet/u_eth/arp_req_tx_0_s0/D u_vision_ethernet/u_eth/peer_ack_rx_0_s0/D u_vision_ethernet/u_eth/arp_ack_rx_0_s0/D}]
set_max_delay -from [get_pins {u_vision_ethernet/u_eth/peer_mailbox*_s0/Q u_vision_ethernet/u_eth/arp_mailbox*_s0/Q}] -to [get_pins {u_vision_ethernet/u_eth/tx_peer_mac*_s0/D u_vision_ethernet/u_eth/tx_mac/destination_mac*_s0/D}] 12.000
set_false_path -hold -from [get_pins {u_vision_ethernet/u_eth/peer_mailbox*_s0/Q u_vision_ethernet/u_eth/arp_mailbox*_s0/Q}] -to [get_pins {u_vision_ethernet/u_eth/tx_peer_mac*_s0/D u_vision_ethernet/u_eth/tx_mac/destination_mac*_s0/D}]

# YOLO follow video -> 50MHz full detection mailbox and 50MHz -> video UI status.
# Cut only first request/ack stages and bound the held payload before acknowledged capture.
set_max_delay -to [get_pins {u_follow_input_cdc/req_meta_s0/D u_follow_input_cdc/ack_meta_s0/D u_follow_status_cdc/req_meta_s0/D u_follow_status_cdc/ack_meta_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_follow_input_cdc/req_meta_s0/D u_follow_input_cdc/ack_meta_s0/D u_follow_status_cdc/req_meta_s0/D u_follow_status_cdc/ack_meta_s0/D}]
set_max_delay -from [get_pins {u_follow_input_cdc/held*_s0/Q}] -to [get_pins {u_follow_input_cdc/dst_frame_id*_s0/D u_follow_input_cdc/dst_box_count*_s0/D u_follow_input_cdc/dst_box_coords*_s0/D u_follow_input_cdc/dst_box_classes*_s0/D u_follow_input_cdc/dst_tracker_session*_s0/D u_follow_input_cdc/dst_box_track_ids*_s0/D u_follow_input_cdc/dst_box_confidences*_s0/D u_follow_input_cdc/dst_box_observed*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_follow_input_cdc/held*_s0/Q}] -to [get_pins {u_follow_input_cdc/dst_frame_id*_s0/D u_follow_input_cdc/dst_box_count*_s0/D u_follow_input_cdc/dst_box_coords*_s0/D u_follow_input_cdc/dst_box_classes*_s0/D u_follow_input_cdc/dst_tracker_session*_s0/D u_follow_input_cdc/dst_box_track_ids*_s0/D u_follow_input_cdc/dst_box_confidences*_s0/D u_follow_input_cdc/dst_box_observed*_s0/D}]
set_max_delay -from [get_pins {u_follow_status_cdc/held*_s0/Q}] -to [get_pins {u_follow_status_cdc/dst_data*_s0/D}] 20.000
set_false_path -hold -from [get_pins {u_follow_status_cdc/held*_s0/Q}] -to [get_pins {u_follow_status_cdc/dst_data*_s0/D}]

# Diagnostic reports only: retain all periods, clock relationships and path bounds.
# Reporting directives only; append to an authorized next PNR SDC.
# They do not change the clock periods, timing exceptions, placement, or routing.
report_timing -setup -from_clock [get_clocks {telemetry_sys50}] -to_clock [get_clocks {telemetry_sys50}] -max_paths 100 -max_common_paths 5
report_timing -setup -from_clock [get_clocks {ethernet_rx125}] -to_clock [get_clocks {ethernet_rx125}] -max_paths 100 -max_common_paths 5
report_timing -setup -from_clock [get_clocks {telemetry_video75}] -to_clock [get_clocks {telemetry_video75}] -max_paths 100 -max_common_paths 5


# Reporting only: classify TX IO, internal and cross-clock endpoints.
report_timing -setup -from_clock [get_clocks {ethernet_tx125}] -to_clock [get_clocks {ethernet_tx125}] -max_paths 100 -max_common_paths 5
report_timing -setup -to_clock [get_clocks {ethernet_tx125}] -max_paths 100 -max_common_paths 5

# HDMI screen row ownership and held frame/line metadata: same protocol as DVP.
set_max_delay -to [get_pins {u_vision_ethernet/u_screen_packets/enable_meta_s0/D}] 20.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_screen_packets/enable_meta_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/screen_meta_camera_s0/D}] 10.000
set_false_path -hold -to [get_pins {u_vision_ethernet/screen_meta_camera_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/screen_meta_tx_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/screen_meta_tx_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/recognition_meta_video_s0/D}] 13.333
set_false_path -hold -to [get_pins {u_vision_ethernet/recognition_meta_video_s0/D}]
# RGB565 row FIFO: bound only Gray-code first captures, never the second stages.
set_max_delay -to [get_pins {u_vision_ethernet/u_screen_packets/write_gray_meta*_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_screen_packets/write_gray_meta*_s0/D}]
set_max_delay -to [get_pins {u_vision_ethernet/u_screen_packets/read_gray_meta*_s0/D}] 8.000
set_false_path -hold -to [get_pins {u_vision_ethernet/u_screen_packets/read_gray_meta*_s0/D}]

# BEGIN AUDITED VENDOR DDR FIRST CAPTURES
# Applied after vendor/reference and flat-netlist pin audit; see docs/DDR_TIMING_CONSTRAINTS.md.
# Supplied vendor .vo has a two-FF recalib_s chain (first capture below).
# wloadn_1 is the UI->continuous-clk first level capture feeding DQS.WLOADN;
# IPUG281 permits an independent low-speed clk for synchronization/control.
# No memory/controller data paths or entire clock pairs are excepted.
set_max_delay -to [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/ddr_sync/recalib_s_0_s0/D ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/wloadn_1_s0/D}] 20.000
set_false_path -hold -to [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/ddr_sync/recalib_s_0_s0/D ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/wloadn_1_s0/D}]
# Vendor gw5a prim_sim.v IDES8_MEM has calib_0 -> calib_1 -> calib_2 on FCLK,
# with pulse detection only from calib_1/calib_2. Bound the asynchronous first
# capture route; hard-macro internal stages/data clocks are unchanged.
set_max_delay -to [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/data_lane_gen*.u_ddr_phy_data_lane/u_ddr_phy_data_io/iserdes_gen*.u_ides8_mem/CALIB}] 5.000
set_false_path -hold -to [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/data_lane_gen*.u_ddr_phy_data_lane/u_ddr_phy_data_io/iserdes_gen*.u_ides8_mem/CALIB}]
# Vendor FIFO reset_w/reset_r two-FF chains asynchronously preset to1 and
# shift GND through their clock domain on release. Only their own PRESET pins
# are asynchronous, not the subsequent ordinary logic CLEAR/PRESET consumers.
set_false_path -to [get_pins {ddr3_ctrl_2port/fifo_ddr3_adapter/wr_data_fifo/fifo_inst/reset_w_*_s0/PRESET ddr3_ctrl_2port/fifo_ddr3_adapter/wr_data_fifo/fifo_inst/reset_r_*_s0/PRESET ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/reset_w_*_s0/PRESET ddr3_ctrl_2port/fifo_ddr3_adapter/rd_data_fifo/fifo_inst/reset_r_*_s0/PRESET}]
# Same-family manufacturer reference (DDR3_MC_PHY_1vs4_5a138k, 5.8E)
# explicitly groups its continuous control, divided UI and memory clocks as
# asynchronous. See vendor_ddr_constraints/0005.sdc.txt and hashed manifest.
# Retain our full clock tree and ordinary data checks; narrow the vendor's
# whole-domain prescription to confirmed internal initialization reset pins.
# Match count: 62 RESET pins = 2 DQS + 16 IDES8_MEM + 20 OSER8_MEM + 24 OSER8.
# Source is ONLY the IP's own ddr_rst register. No ordinary fabric FF reset,
# user data, clock pair, or normal read/write path is excepted by this rule.
set_false_path -from [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/ddr_rsti_reg_2_s0/Q}] -to [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/data_lane_gen*.u_ddr_phy_data_lane/u_ddr_phy_data_io/u_dqs/RESET ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/data_lane_gen*.u_ddr_phy_data_lane/u_ddr_phy_data_io/u_dqs_gen/RESET ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/data_lane_gen*.u_ddr_phy_data_lane/u_ddr_phy_data_io/iserdes_gen*.u_ides8_mem/RESET ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/data_lane_gen*.u_ddr_phy_data_lane/u_ddr_phy_data_io/oserdes_gen*.u_oser8_mem/RESET ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/u_ddr_phy_cmd_lane/u_ddr_phy_cmd_io/cmd_oserdes_gen*.u_cmd_gen/RESET ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/u_ddr_phy_wds/u_ddr_phy_cmd_lane/u_ddr_phy_cmd_io/u_ck_gen/RESET}]
# END AUDITED VENDOR DDR FIRST CAPTURES

# Adjacent-edge PLL jitter; do not decorrelate a shared physical hold edge.
set_clock_uncertainty 0.500 -setup -from [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -setup -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {telemetry_sys50}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {telemetry_sys50}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ddr_memory200}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {ddr_memory200}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {local_sys50}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {local_sys50}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {video_serial375}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {video_serial375}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {camera_xclk25}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {camera_xclk25}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {telemetry_video75}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {telemetry_video75}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ddr_ui50}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {ddr_ui50}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {camera_pclk}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {camera_pclk}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125_90}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {ethernet_tx125_90}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_rx125}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {ethernet_rx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {audio_i2s_bclk}] -to [get_clocks {ethernet_tx125}]
set_clock_uncertainty 0.500 -hold -from [get_clocks {ethernet_tx125}] -to [get_clocks {audio_i2s_bclk}]
report_timing -hold -from_clock [get_clocks {telemetry_sys50}] -to_clock [get_clocks {ethernet_tx125}] -max_paths 20
report_timing -hold -from_clock [get_clocks {ethernet_tx125}] -to_clock [get_clocks {telemetry_sys50}] -max_paths 20
report_timing -hold -from_clock [get_clocks {ethernet_rx125}] -to_clock [get_clocks {ethernet_tx125}] -max_paths 20
report_timing -hold -from_clock [get_clocks {ethernet_tx125}] -to_clock [get_clocks {ethernet_rx125}] -max_paths 20
report_timing -hold -from_clock [get_clocks {telemetry_video75}] -to_clock [get_clocks {ethernet_tx125}] -max_paths 20
report_timing -hold -from_clock [get_clocks {ethernet_tx125}] -to_clock [get_clocks {telemetry_video75}] -max_paths 20
report_timing -hold -from_clock [get_clocks {camera_pclk}] -to_clock [get_clocks {ethernet_tx125}] -max_paths 20
report_timing -hold -from_clock [get_clocks {ethernet_tx125}] -to_clock [get_clocks {camera_pclk}] -max_paths 20
