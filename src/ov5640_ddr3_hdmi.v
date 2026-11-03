module gao_ov5640_ddr_hdmi(
    input       iIR           , // IR receiver, active low pulses
    input       motor_uart_rx , // car UART reply, 115200 8N1
    input       imu_uart_rx   , // IMU TX -> F20, 115200 8N1
    input       page_select   , // SW7 low: dashboard, high: full data
    input       avoidance_enable, // SW6 C22: low remote, high avoidance
    input       navigation_enable, // SW5 D21: raw scan map; SW6 has priority
    input       recognition_enable, // SW4 E21: screen stream ON/OFF in this revision
    input       speed_page_enable, // SW3 G21: high four-wheel speed curves
    input       obstacle_page_enable, // SW2 E22: high front-distance / speed curves
    input       imu_panel_enable, // SW1 D22: high nine-value IMU panel on main page
    inout       thermal_sda, // E13, 3.3 V open-drain MLX90640 SDA
    inout       thermal_scl, // E14, 3.3 V open-drain MLX90640 SCL
    input       lidar_uart_rx , // GPIO0_6/F13 <- X3PRO Tx, 3.3V UART, 115200 8N1
    output      lidar_motor_pwm, // GPIO0_7/F14 -> X3PRO M_CTR, 10kHz / IR-adjustable duty
    output      motor_uart_tx , // F18 -> car RX, 115200 8N1
    input       front_echo    , // A14, fixed front ultrasonic probe
    output      front_trig    , // A13
    input       rear_echo     , // A16, fixed rear ultrasonic probe
    output      rear_trig     , // A15
    input       left_echo     , // H14, front-left ultrasonic probe
    output      left_trig     , // H15
    input       right_echo    , // A19, front-right ultrasonic probe
    output      right_trig    , // A18
    //System clock reset
    input       clk50M        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入

    // ACG720 onboard Gigabit Ethernet PHY (RGMII).
    input       eth_rxc       ,
    input [3:0] eth_rxd       ,
    input       eth_rxdv      ,
    output      eth_gtxc      ,
    output [3:0] eth_txd      ,
    output      eth_txen      ,
    output      eth_rst_n     ,
    output      eth_mdc       ,
    inout       eth_mdio      ,
    input       audio_bclk    ,
    input       audio_lrclk   ,
    input       audio_adc_data,
    output      audio_mclk    ,
    output      audio_i2c_scl,
    inout       audio_i2c_sda,

    //camera interface
    output      camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    output      camera_pwdn   ,
    //led
    output [3:0]led           ,

    //hdmi output
	output       	tmds_clk_p ,
	output       	tmds_clk_n ,
	output 	[2:0]   tmds_data_p,    //rgb
	output 	[2:0]   tmds_data_n,     //rgb

    //ddr
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        

);
    //Set IMAGE Size  
    parameter IMAGE_WIDTH  = 1280;
    parameter IMAGE_HEIGHT = 720;
    // Board map geometry. Keep every address, cell and CDC field consistent.
    // +X is screen-up (4.2 m); +Y is screen-left (5.2 m).
    localparam integer MAP_CELL_MM=40, MAP_GRID_SIZE=105, MAP_GRID_X=105, MAP_GRID_Y=130;
    localparam integer MAP_GRID_BITS=$clog2(MAP_GRID_X>MAP_GRID_Y?MAP_GRID_X:MAP_GRID_Y);
    // Send every camera frame; Pi always processes the newest complete frame.
    parameter ETHERNET_FRAME_STRIDE = 1;
    // Runtime camera/HDMI selection reuses the SAME two line RAMs/MAC.
    // This legacy parameter can force HDMI-only capture.
    parameter ETHERNET_SCREEN_RECORD = 0; // camera recognition + annotated Pi recording
    parameter CAMERA_FULL_RES = 1; // 1280x720 RGB565, two datagrams per row
    parameter SCREEN_RECORD_FRAME_STRIDE = 2; // 75MHz/(1650*750*2) = 30.303 fps
    parameter SCREEN_RECORD_FULL_RES = 1; // legacy compatibility; dynamic stream uses RGB565
    // RTL8211F is configured with internal TX/RX delays; avoid double delay.
    parameter ETHERNET_TX_CLOCK_DELAY = 0;

    // Declare shared buses before their first use (strict Verilog elaboration).
    wire signed [15:0] rx_speed_a, rx_speed_b, rx_speed_c, rx_speed_d;
    wire [3:0] source_motion, display_motion;
    wire detection_valid;
    wire detection_reset_n;
    wire [31:0] detection_frame_id;
    wire [3:0] detection_count;
    wire [511:0] detection_coords;
wire [55:0] detection_classes;
    wire [119:0] detection_label_text;
    wire [3:0] detection_label_length;
    wire [31:0] detection_tracker_session;
    wire [255:0] detection_track_ids;
    wire [79:0] detection_confidences;
    wire [7:0] detection_observed;

    wire          loc_clk24m;
    wire          clk_200M;
    wire          pll_lock;
    wire          loc_clk50m;
    //camera
    wire          camera_init_done;
    wire          pclk_bufg_o;

    //ddr
	wire fifo_aclr;
	wire fifo_wrreq;
	wire [15:0] fifo_wrdata;
    wire init_calib_complete;
    wire ddr_reset_n;
    wire ddr_pll_enable; // DDR PHY pll_stop, connected directly to PLL ENCLK2.
    
    //hdmi
    wire clk_vga;   // original display pixel clock
	wire clk_vgax5; // original display serial clock
    wire hdmi_pll_lock;

	wire [11:0] hcount;
	wire [11:0] vcount;
	wire [23:0] disp_data;
 
	wire video_hs;
	wire video_vs;
	wire video_de;
	wire [7:0] video_r;
	wire [7:0] video_g;
	wire [7:0] video_b;

    wire video_frame_begin;
    wire [23:0] osd_rgb;
    wire osd_hs, osd_vs, osd_de;

    // X3 Pro rotates internally. Parked scans share one encoder/IMU map origin.
    // Re-entering the effective SW5 mode always starts a fresh local map.
    wire mapping_active, scan_avoidance_active;
    scan_mode_control u_scan_mode(
        .clk(clk50M),.reset_n(reset_n),.sw5_async(navigation_enable),
        .sw6_async(avoidance_enable),.mapping_active(mapping_active),
        .avoidance_active(scan_avoidance_active));
    wire [15:0] tof_distance_mm;
    wire tof_valid,tof_update,tof_online;
    wire [7:0] tof_fault;
    // X3PRO is a single-channel continuous transmitter. No A5 start command.
    // The direct GPIO link receives laser data and controls M_CTR independently
    // of the map/avoidance mode. The USB adapter must not also drive M_CTR.
    wire lidar_point_ready,lidar_point_valid,lidar_range_valid,lidar_scan_start,lidar_no_return;
    wire lidar_map_ready,lidar_obstacle_ready;
    assign lidar_point_ready=lidar_map_ready && lidar_obstacle_ready;
    wire [15:0] lidar_distance_mm;
    wire [11:0] lidar_angle_tenths;
    wire ir_command_valid;
    wire [7:0] ir_command;
    wire [7:0] lidar_duty_percent;
    // Number 1 (0C) raises LiDAR duty; number 3 (5E) lowers it. Car speed
    // keys 45/47 remain independent. Only accepted complete NEC frames step.
    ir_speed_control #(.IR_SPEED_UP(8'h0C),.IR_SPEED_DOWN(8'h5E),
        .SPEED_INITIAL(10),.SPEED_MIN(10),.SPEED_MAX(100),.SPEED_STEP(10)) u_lidar_speed(
        .clk(clk50M),.reset_n(reset_n),.dec_done(ir_command_valid),
        .command(ir_command),.requested_speed(lidar_duty_percent));
    wire [31:0] lidar_point_timestamp;
    wire lidar_point_timestamp_valid;
    x3pro_driver u_lidar(
        .clk(clk50M),.reset_n(reset_n),.uart_rx(lidar_uart_rx),.motor_pwm(lidar_motor_pwm),
        .motor_duty_percent(lidar_duty_percent),
        .point_ready(lidar_point_ready),.point_valid(lidar_point_valid),
        .distance_mm(lidar_distance_mm),.angle_tenths(lidar_angle_tenths),
        .range_valid(lidar_range_valid),.no_return(lidar_no_return),.scan_start(lidar_scan_start),.point_timestamp(lidar_point_timestamp),.point_timestamp_valid(lidar_point_timestamp_valid),.online(tof_online),
        .fault(tof_fault),.good_packets(),.bad_packets(),.dropped_packets());
    lidar_front_sector u_lidar_front(
        .clk(clk50M),.reset_n(reset_n),.online(tof_online),.scan_start(lidar_scan_start),
        .point_valid(lidar_point_valid),.point_ready(lidar_point_ready),.range_valid(lidar_range_valid),
        .distance_mm(lidar_distance_mm),.angle_tenths(lidar_angle_tenths),
        .front_distance_mm(tof_distance_mm),.front_valid(tof_valid),.front_update(tof_update));
    wire scan_map_clear,scan_ray_update,scan_ray_valid;
    wire scan_map_busy,scan_map_clear_done,scan_map_update;
    wire signed [31:0] scan_ray_x_mm,scan_ray_y_mm,scan_heading;
    wire [3:0] scan_status;
    wire [15:0] scan_samples;
    wire scan_sample_allow,scan_continuous_capture;
    wire [2:0] scan_auto_state;
    wire [11:0] scan_auto_progress;
    wire [3:0] scan_auto_fault;
    wire [3:0] scan_stop_wait_reason;
    wire scan_resume_event;
    wire scan_stop_confirmed;
    wire nav_owner,nav_map_sample_allow,nav_scan_restart,motor_stop_confirmed;
    wire nav_live_mapping_active,nav_localization_replan,nav_pose_transform_pending;
    wire [3:0] nav_motion,nav_status,nav_fault;
    wire nav_manual_takeover;
    wire [7:0] nav_speed;
    wire signed [2:0] nav_yaw_trim;
    wire signed [12:0] planner_body_heading;
    wire [MAP_GRID_BITS-1:0] nav_goal_x,nav_goal_y,nav_pose_cell_x,nav_pose_cell_y;
    wire nav_goal_valid,nav_path_valid,nav_pose_valid,nav_pose_fault,nav_pose_moved;
    wire signed [15:0] nav_pose_x_mm,nav_pose_y_mm;
    wire scan_heading_valid;
    wire odom_pose_valid,odom_pose_fault,odom_pose_moved,odom_heading_valid;
    wire signed [15:0] odom_pose_x_mm,odom_pose_y_mm;
    wire signed [31:0] odom_heading;
    wire capture_healthy,capture_ray_update,capture_frame_start;
    wire signed [31:0] capture_body_heading;
    wire [3:0] capture_status,mapping_status;
    wire [6:0] capture_bin;
    wire [3:0] capture_bin_error;
    wire [31:0] capture_timestamp;
    wire signed [31:0] capture_start_x,capture_start_y,capture_end_x,capture_end_y;
    wire slam_busy,slam_transform_update;
    wire slam_rebuild_clear,slam_history_full;
    wire [6:0] slam_keyframe_count;
    wire [15:0] slam_loop_closures,slam_optimized_loops,slam_dropped_frames;
    wire mapping_scan_running,mapping_paused_ready;
    // 42 alone selects acquisition. Navigation grants permission to sample
    // while executing, but never forces the user's acquisition switch on.
    wire map_acquisition_running=mapping_scan_running;
    wire [3:0] mapping_reject_reason;
    wire [7:0] mapping_diagnostic_points,mapping_diagnostic_hits,mapping_diagnostic_support;
    wire signed [15:0] slam_transform_x,slam_transform_y;
    wire signed [12:0] slam_transform_yaw;
    wire [7:0] slam_map_epoch;
    wire [15:0] slam_accepted_frames,slam_rejected_frames,slam_match_score;
    wire slam_map_req,slam_map_ready,slam_map_rsp_valid;
    wire slam_loop_reference,loop_map_busy,loop_map_clear_done,loop_map_ready,loop_map_rsp;
    wire global_matcher_ready,global_matcher_rsp;
    wire [1:0] loop_map_data;
    wire [2*MAP_GRID_BITS-1:0] slam_map_addr,map_read_addr;
    wire map_read_req,map_read_ready,map_read_rsp_valid;
    wire [1:0] map_read_data;
    wire navigation_map_busy=scan_map_busy || slam_busy || nav_pose_transform_pending || slam_transform_update;
    function mapping_speed_quiet;
        input signed [15:0] speed;
        begin mapping_speed_quiet=speed>=-2 && speed<=2;end
    endfunction
    wire mapping_stationary=source_motion==0 &&
        mapping_speed_quiet(rx_speed_a) && mapping_speed_quiet(rx_speed_b) &&
        mapping_speed_quiet(rx_speed_c) && mapping_speed_quiet(rx_speed_d);
    wire scan_resume_allowed=1'b0;
    wire signed [31:0] scan_ray_start_x_mm,scan_ray_start_y_mm;
    wire planner_start,planner_abort,planner_busy,planner_done,planner_success;
    wire planner_route_uses_unknown;
    wire [3:0] planner_fail;
    wire planner_map_req,planner_map_ready,planner_map_rsp_valid;
    wire [2*MAP_GRID_BITS-1:0] planner_map_addr;
    wire [1:0] planner_map_data;
    wire path_rewind,waypoint_next,waypoint_busy,waypoint_valid,waypoint_last;
    wire [MAP_GRID_BITS-1:0] waypoint_x,waypoint_y;
    wire path_clear,path_cell_valid,path_clear_busy,path_write_bank;
    wire [2*MAP_GRID_BITS-1:0] path_cell_addr;
    wire path_display_data;
    wire [44+2*MAP_GRID_BITS:0] nav_display_metadata;
    wire nav_display_stop_confirmed;
    wire nav_metadata_valid;
    wire [3:0] nav_display_status,nav_display_fault;
    wire [MAP_GRID_BITS-1:0] nav_display_goal_x,nav_display_goal_y;
    wire nav_display_goal_valid,nav_display_pose_valid,nav_display_path_valid,nav_display_path_bank;
    wire signed [15:0] nav_display_pose_x_mm,nav_display_pose_y_mm;
    reg path_frame_bank;
    wire scan_ray_accepted=scan_ray_update && scan_ray_valid && !scan_map_busy;
    wire [3:0] scan_status_for_display=scan_status;
    wire [2*MAP_GRID_BITS-1:0] scan_read_address;
    wire [2*MAP_GRID_BITS-1:0] scan_read_address_raw;
    wire [1:0] scan_read_data;
    wire [90:0] scan_display_metadata;
    wire [3:0] scan_display_reject_reason;
    wire [7:0] scan_display_diagnostic_points,scan_display_diagnostic_hits,scan_display_diagnostic_support;
    wire scan_metadata_valid;
    wire scan_display_active,scan_display_range_valid;
    wire [3:0] scan_display_status;
    wire [3:0] scan_display_fault;
    wire [3:0] scan_display_stop_wait_reason;
    wire scan_display_stop_confirmed;
    wire signed [31:0] scan_display_heading;
    wire [15:0] scan_display_distance;
    wire [23:0] scan_rgb;
    wire scan_hs,scan_vs,scan_de,scan_page_active;
    wire [23:0] speed_rgb;
    wire speed_hs,speed_vs,speed_de,speed_page_active;
    wire [23:0] obstacle_rgb;
    wire obstacle_hs,obstacle_vs,obstacle_de,obstacle_page_active;
    wire [23:0] page_rgb=obstacle_page_active ? obstacle_rgb : speed_page_active ? speed_rgb : scan_page_active ? scan_rgb : osd_rgb;
    wire page_hs=obstacle_page_active ? obstacle_hs : speed_page_active ? speed_hs : scan_page_active ? scan_hs : osd_hs;
    wire page_vs=obstacle_page_active ? obstacle_vs : speed_page_active ? speed_vs : scan_page_active ? scan_vs : osd_vs;
    wire page_de=obstacle_page_active ? obstacle_de : speed_page_active ? speed_de : scan_page_active ? scan_de : osd_de;
    wire [23:0] hdmi_rgb;
    wire hdmi_hs,hdmi_vs,hdmi_de;
    wire thermal_arrived;
    wire [31:0] thermal_arrival_session,thermal_arrival_id;
    wire thermal_sample_valid,thermal_sensor_valid,thermal_task_active;
    wire signed [15:0] thermal_sample_maximum,thermal_maximum;
    wire [6:0] thermal_sample_threshold,thermal_threshold;
    wire thermal_task_present,thermal_sensor_ok,thermal_alarm;
    thermal_alert_control u_thermal_control(
        .clk(clk50M),.reset_n(reset_n),.sample_valid(thermal_sample_valid),
        .sample_sensor_valid(thermal_sensor_valid),.sample_task_active(thermal_task_active),
        .sample_maximum_centi(thermal_sample_maximum),.sample_threshold_c(thermal_sample_threshold),
        .task_present(thermal_task_present),.sensor_ok(thermal_sensor_ok),.alarm(thermal_alarm),
        .maximum_centi(thermal_maximum),.threshold_c(thermal_threshold));
    wire [25:0] thermal_display;
    reg [21:0] thermal_display_tick;
    reg thermal_sample_delayed;
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin thermal_display_tick<=0;thermal_sample_delayed<=0;end
        else begin
            thermal_sample_delayed<=thermal_sample_valid;
            if(thermal_display_tick==2499999) thermal_display_tick<=0;
            else thermal_display_tick<=thermal_display_tick+1'b1;
        end
    end
    telemetry_mailbox #(.WIDTH(26),.RESET_SYNC_RELEASE(1)) u_thermal_display_cdc(
        .src_clk(clk50M),.dst_clk(clk_vga),.reset_n(reset_n),
        .src_valid(thermal_sample_delayed || thermal_display_tick==2499999),
        .src_data({thermal_task_present,thermal_sensor_ok,thermal_alarm,thermal_threshold,thermal_maximum}),
        .dst_valid(),.dst_data(thermal_display));
    thermal_warning_overlay u_thermal_overlay(
        .clk(clk_vga),.reset_n(reset_n),.rgb_in(page_rgb),
        .hs_in(page_hs),.vs_in(page_vs),.de_in(page_de),
        .task_present(thermal_display[25]),.sensor_ok(thermal_display[24]),.alarm(thermal_display[23]),
        .threshold_c(thermal_display[22:16]),.maximum_centi(thermal_display[15:0]),
        .rgb_out(hdmi_rgb),.hs_out(hdmi_hs),.vs_out(hdmi_vs),.de_out(hdmi_de));

    // All telemetry starts in the 50 MHz domain and crosses as one mailbox.
    wire [671:0] imu_source_values, imu_display_values;
    wire [3:0] imu_source_known, imu_display_known;
    wire imu_ready, imu_update, imu_yaw_update;
    wire rx_motor_valid;
    wire [63:0] rx_counts, display_speeds, display_counts;
    wire display_motor_online, display_telemetry_valid;
    wire [976:0] display_telemetry;
    wire [15:0] display_obstacle_front_cm;
    wire display_obstacle_front_valid;
    wire [63:0] source_wheel_targets,display_wheel_targets;
    wire [15:0] display_front_distance_cm, display_rear_distance_cm;
    wire display_front_distance_valid, display_rear_distance_valid;
    wire [15:0] display_left_distance_cm, display_right_distance_cm;
    wire display_left_distance_valid, display_right_distance_valid;
    wire display_left_distance_measured_valid, display_right_distance_measured_valid;
    wire [1:0] display_left_distance_status, display_right_distance_status;
    wire [5:0] source_avoidance_state, display_avoidance_state;
    wire [1:0] source_avoidance_side_clear,display_avoidance_side_clear;
    wire [3:0] source_avoidance_fault, display_avoidance_fault;
    wire display_avoidance_enabled;
    reg [31:0] motor_age;
    reg [19:0] telemetry_tick;
    // Cache the comparison of the NEXT age in the same clock that publishes
    // that age. Packet priority and the exact 25,000,000-cycle expiry stay
    // unchanged; navigation no longer decodes a wide timer on its state path.
    reg source_motor_online;
    wire [15:0] front_distance_cm, rear_distance_cm;
    wire [15:0] left_distance_cm, right_distance_cm;
    wire left_valid, right_valid, left_update, right_update;
    wire front_valid, rear_valid, front_update, rear_update;
    wire front_measured_valid, rear_measured_valid;
    wire left_measured_valid, right_measured_valid;
    wire [1:0] left_status, right_status;
    // ready means the shared IMU converter is idle, not that an IMU is online.
    // Hold off snapshots during conversion to preserve the complete IMU group.
    // With no IMU attached it stays ready; heartbeat also propagates stale sonar.
    wire send_telemetry=imu_ready && (imu_update || rx_motor_valid || front_update ||
                                    rear_update || left_update || right_update || telemetry_tick==0);
    wire [3:0] motor_status_led;
    wire motor_avoidance_hold;

    // Ported live LiDAR avoidance. Distances are gaps from the real body edge,
    // independent of the frozen SLAM map. Radar is centred in the 28 cm base;
    // the rear monitor extends that edge to 26 cm behind the radar.
    wire [15:0] obstacle_front_mm,obstacle_rear_mm,obstacle_left_mm,obstacle_right_mm,obstacle_rotation_mm;
    wire obstacle_healthy_raw,obstacle_front_valid_raw,obstacle_rear_valid_raw;
    wire obstacle_left_valid_raw,obstacle_right_valid_raw,obstacle_rotation_valid_raw;
    wire obstacle_front_update_raw,obstacle_rear_update_raw,obstacle_left_update_raw,obstacle_right_update_raw;
    reg obstacle_healthy,obstacle_front_valid,obstacle_rear_valid,obstacle_left_valid,obstacle_right_valid,obstacle_rotation_valid;
    reg obstacle_front_update,obstacle_rear_update,obstacle_left_update,obstacle_right_update;
    reg [15:0] obstacle_front_cm,obstacle_rear_cm,obstacle_left_cm,obstacle_right_cm,obstacle_rotation_cm;
    lidar_obstacle_monitor #(.BODY_FRONT_MM(140),.BODY_REAR_MM(260),.BODY_HALF_WIDTH_MM(140),
        .SENSOR_FORWARD_MM(0),.SENSOR_LEFT_MM(0),.SENSOR_YAW_TENTHS(0),
        .SIDE_SAFETY_MARGIN_MM(30),.ROTATION_RADIUS_MM(311),.CORRIDOR_MARGIN_MM(0),
        .NO_RETURN_IS_CLEAR(1),.MISSING_SAMPLE_IS_CLEAR(1),.ENABLE_GAP_PROFILE(0)) u_lidar_obstacles(
        .clk(clk50M),.reset_n(reset_n),.online(tof_online),.scan_start(lidar_scan_start),
        .point_valid(lidar_point_valid && lidar_map_ready),.range_valid(lidar_range_valid),.no_return(lidar_no_return),
        .distance_mm(lidar_distance_mm),.angle_tenths(lidar_angle_tenths),.point_ready(lidar_obstacle_ready),
        .scan_healthy(obstacle_healthy_raw),.front_mm(obstacle_front_mm),.rear_mm(obstacle_rear_mm),
        .left_mm(obstacle_left_mm),.right_mm(obstacle_right_mm),.rotation_mm(obstacle_rotation_mm),
        .front_valid(obstacle_front_valid_raw),.rear_valid(obstacle_rear_valid_raw),
        .left_valid(obstacle_left_valid_raw),.right_valid(obstacle_right_valid_raw),.rotation_valid(obstacle_rotation_valid_raw),
        .front_update(obstacle_front_update_raw),.rear_update(obstacle_rear_update_raw),
        .left_update(obstacle_left_update_raw),.right_update(obstacle_right_update_raw),
        .coverage(),.gap_clearance_mm(),.gap_direction_valid());
    always @(posedge clk50M or negedge reset_n)begin
        if(!reset_n)begin
            obstacle_front_cm<=0;obstacle_rear_cm<=0;obstacle_left_cm<=0;obstacle_right_cm<=0;obstacle_rotation_cm<=0;
            obstacle_healthy<=0;obstacle_front_valid<=0;obstacle_rear_valid<=0;obstacle_left_valid<=0;obstacle_right_valid<=0;obstacle_rotation_valid<=0;
            obstacle_front_update<=0;obstacle_rear_update<=0;obstacle_left_update<=0;obstacle_right_update<=0;
        end else begin
            obstacle_front_cm<=obstacle_front_mm/16'd10;obstacle_rear_cm<=obstacle_rear_mm/16'd10;
            obstacle_left_cm<=obstacle_left_mm/16'd10;obstacle_right_cm<=obstacle_right_mm/16'd10;
            obstacle_rotation_cm<=obstacle_rotation_mm/16'd10;
            obstacle_healthy<=obstacle_healthy_raw;obstacle_front_valid<=obstacle_front_valid_raw;
            obstacle_rear_valid<=obstacle_rear_valid_raw;obstacle_left_valid<=obstacle_left_valid_raw;
            obstacle_right_valid<=obstacle_right_valid_raw;obstacle_rotation_valid<=obstacle_rotation_valid_raw;
            obstacle_front_update<=obstacle_front_update_raw;obstacle_rear_update<=obstacle_rear_update_raw;
            obstacle_left_update<=obstacle_left_update_raw;obstacle_right_update<=obstacle_right_update_raw;
        end
    end

    quad_ultrasonic u_ultrasonic(
        .clk(clk50M),.reset_n(reset_n),
        .front_echo(front_echo),.rear_echo(rear_echo),
        .left_echo(left_echo),.right_echo(right_echo),
        .front_trig(front_trig),.rear_trig(rear_trig),
        .left_trig(left_trig),.right_trig(right_trig),
        .front_distance_cm(front_distance_cm),.rear_distance_cm(rear_distance_cm),
        .left_distance_cm(left_distance_cm),.right_distance_cm(right_distance_cm),
        .front_valid(front_valid),.rear_valid(rear_valid),
        .left_valid(left_valid),.right_valid(right_valid),
        .front_measured_valid(front_measured_valid),.rear_measured_valid(rear_measured_valid),
        .front_update(front_update),.rear_update(rear_update),
        .left_update(left_update),.right_update(right_update),
        .front_status(),.rear_status(),.front_busy(),.rear_busy(),
        .left_measured_valid(left_measured_valid),.right_measured_valid(right_measured_valid),
        .left_status(left_status),.right_status(right_status),.left_busy(),.right_busy());

    imu_telemetry u_imu(
        .clk(clk50M),.reset_n(reset_n),.uart_rx(imu_uart_rx),
        .values(imu_source_values),.known(imu_source_known),
        .ready(imu_ready),.update(imu_update),.yaw_update(imu_yaw_update));
    // YOLO follow is explicit and mutually exclusive with SW5 mapping/SW6 avoidance.
    // Detection coordinates/classes/frame are transferred as one acknowledged mailbox.
    wire recognition_enabled;
    wire network_video_enabled;
    assign recognition_enabled=ETHERNET_SCREEN_RECORD ? 1'b0 : network_video_enabled;
    wire follow_mode_available=recognition_enabled && !mapping_active && !scan_avoidance_active;
    wire follow_boxes_valid;
    wire [31:0] follow_boxes_frame_id;
    wire [3:0] follow_boxes_count;
    wire [511:0] follow_boxes_coords;
    wire [55:0] follow_boxes_classes;
    wire [31:0] follow_boxes_tracker_session;
    wire [255:0] follow_boxes_track_ids;
    wire [79:0] follow_boxes_confidences;
    wire [7:0] follow_boxes_observed;
    wire task_command_valid;
    wire [7:0] task_command;
    wire [5:0] task_command_actions;
    wire follow_owner;
    wire motor_control_ready,motor_motion_committed;
    wire [3:0] motor_transmitted_motion;
    wire map_ir_valid=ir_command_valid && mapping_active && !follow_owner;
    wire map_scan_toggle=map_ir_valid && ir_command==8'h42;
    wire map_scan_clear_key=map_ir_valid && ir_command==8'h4A;
    wire [3:0] follow_motion,follow_selected_slot;
    wire [7:0] follow_speed,source_requested_speed;
    wire signed [15:0] follow_linear,follow_turn;
    wire follow_request_nonzero;
    wire [2:0] follow_status;
    wire [31:0] follow_selected_frame_id;
    wire [31:0] follow_target_id,follow_target_session;
    wire [3:0] follow_reason;
    wire follow_lock_valid;
    wire [15:0] follow_reference_height;
    // Coherent owner/request boundary. Mode and vector combinational decode
    // must not extend into the UART serializer's STOP/obsolete-frame cone.
    // Physical STOP and the motor's own SW6 filter retain their direct paths.
    reg motor_follow_available,motor_follow_owner,motor_follow_nonzero,motor_nav_owner,motor_mapping_active;
    reg [3:0] motor_follow_motion,motor_nav_motion;
    reg [7:0] motor_follow_speed,motor_nav_speed;
    reg signed [15:0] motor_follow_linear,motor_follow_turn;
    reg signed [2:0] motor_nav_yaw_trim;
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            {motor_follow_available,motor_follow_owner,motor_follow_motion,motor_follow_speed,
             motor_follow_linear,motor_follow_turn,motor_follow_nonzero}<=0;
            {motor_nav_owner,motor_nav_motion,motor_nav_speed}<=0;
            motor_nav_yaw_trim<=0;
            motor_mapping_active<=0;
        end else begin
            {motor_follow_available,motor_follow_owner,motor_follow_motion,motor_follow_speed,
             motor_follow_linear,motor_follow_turn,motor_follow_nonzero}<=
                {follow_mode_available,follow_owner,follow_motion,follow_speed,
                 follow_linear,follow_turn,follow_request_nonzero};
            {motor_nav_owner,motor_nav_motion,motor_nav_speed}<=
                {nav_owner,nav_motion,nav_speed};
            motor_nav_yaw_trim<=nav_yaw_trim;
            motor_mapping_active<=mapping_active;
        end
    end
    wire [74:0] follow_display_data;
    wire [31:0] follow_display_target_id=follow_display_data[70:39];
    wire [3:0] follow_display_reason=follow_display_data[74:71];
    wire [2:0] follow_display_status=follow_display_data[38:36];
    wire [3:0] follow_display_slot=follow_display_data[35:32];
    wire [31:0] follow_display_frame_id=follow_display_data[31:0];
    // Parser publishes only bounds-checked 640x360 boxes scaled to 1280x720.
    // Zero-extend their legal x11/y10 fields so following does not reactivate
    // unused high coordinate bits in the RX banks or either detection mailbox.
    // Malformed full16 coordinates are rejected by the unchanged parser first.
    wire [511:0] follow_input_coords;
    genvar follow_coord_slot;
    generate for(follow_coord_slot=0;follow_coord_slot<8;follow_coord_slot=follow_coord_slot+1) begin:g_follow_coord_width
        assign follow_input_coords[follow_coord_slot*64 +: 64]={
            6'd0,detection_coords[follow_coord_slot*64+48 +: 10],
            5'd0,detection_coords[follow_coord_slot*64+32 +: 11],
            6'd0,detection_coords[follow_coord_slot*64+16 +: 10],
            5'd0,detection_coords[follow_coord_slot*64 +: 11]};
    end endgenerate
    detection_box_cdc #(.TRACK_METADATA_ENABLE(1)) u_follow_input_cdc(
        .src_clk(clk_vga),.dst_clk(clk50M),.reset_n(reset_n),
        .src_valid(detection_valid),.src_ready(),
        .src_frame_id(detection_frame_id),.src_box_count(detection_count),
        .src_box_coords(follow_input_coords),.src_box_classes(detection_classes),
        .src_label_text(120'd0),.src_label_length(4'd0),
        .src_tracker_session(detection_tracker_session),.src_box_track_ids(detection_track_ids),
        .src_box_confidences(detection_confidences),.src_box_observed(detection_observed),
        .dst_valid(follow_boxes_valid),.dst_frame_id(follow_boxes_frame_id),
        .dst_box_count(follow_boxes_count),.dst_box_coords(follow_boxes_coords),
        .dst_box_classes(follow_boxes_classes),.dst_tracker_session(follow_boxes_tracker_session),
        .dst_box_track_ids(follow_boxes_track_ids),.dst_box_confidences(follow_boxes_confidences),
        .dst_box_observed(follow_boxes_observed));
    vision_follow_control #(.REGISTER_IR_DECODE(1),.TRACK_ID_ENABLE(1),.APPROACH_STOP_CM(30),
        .FOLLOW_SPEED_MAX(10),.FOLLOW_TURN_ENABLE(1),
        .FOLLOW_TURN_SPEED_MAX(5),.FOLLOW_CURVE_ENABLE(1),.FOLLOW_ARC_SPEED_MAX(4),
        .ALIGN_ENTER_PX(320),.ALIGN_EXIT_PX(256),.FOLLOW_SERVO_ENABLE(1),.FOLLOW_REVERSE_ENABLE(1),
        .FOLLOW_PROXIMITY_ENABLE(0),.FOLLOW_WIDTH_FALLBACK_ENABLE(1),.TARGET_EDGE_MARGIN_PX(8),
        .REF_HEIGHT_MAX(720),.CENTER_ENTER_PX(120),.CENTER_EXIT_PX(80),
        .TURN_PULSE_MS(80),.TURN_NEAR_PULSE_MS(40),
        .TURN_TIMER_FROM_TX(1),.TURN_TX_WAIT_MS(200),.FOLLOW_VECTOR_ENABLE(1),
        .VECTOR_SPEED_SCALE(2),.VECTOR_MIN_LINEAR(4),.VECTOR_MIN_TURN(2),.VECTOR_MIN_PIVOT(4),
        // Hold the locked ID across the Pi's own re-acquisition window. The
        // module default stays at 600 ms for the testbenches.
        .VECTOR_ACCEL_STEP(2),.VECTOR_OBSERVATION_MS(200),.TARGET_RETAIN_MS(2000)) u_vision_follow(
        .clk(clk50M),.reset_n(reset_n),.mode_available(follow_mode_available),
        .ir_valid(ir_command_valid),.ir_command(ir_command),
        .boxes_valid(follow_boxes_valid),.frame_id(follow_boxes_frame_id),
        .count(follow_boxes_count),.coords(follow_boxes_coords),.classes(follow_boxes_classes),
        .tracker_session(follow_boxes_tracker_session),.track_ids(follow_boxes_track_ids),
        .confidences(follow_boxes_confidences),.observed(follow_boxes_observed),
        .motor_ready(motor_stop_confirmed),.motor_online(source_motor_online && motor_control_ready),
        .motor_motion_committed(motor_motion_committed),.motor_transmitted_motion(motor_transmitted_motion),
        .requested_speed(source_requested_speed),
        .front_cm(front_distance_cm),.rear_cm(rear_distance_cm),
        .left_cm(left_distance_cm),.right_cm(right_distance_cm),
        .front_valid(front_valid),.rear_valid(rear_valid),
        .left_valid(left_valid),.right_valid(right_valid),
        .owner(follow_owner),.motion(follow_motion),.speed(follow_speed),
        .follow_linear(follow_linear),.follow_turn(follow_turn),.follow_request_nonzero(follow_request_nonzero),
        .status(follow_status),.selected_slot(follow_selected_slot),
        .selected_frame_id(follow_selected_frame_id),.reference_height(follow_reference_height),
        .arrived(thermal_arrived),.arrival_session(thermal_arrival_session),.arrival_id(thermal_arrival_id),
        .target_track_id(follow_target_id),.target_tracker_session(follow_target_session),
        .lock_valid(follow_lock_valid),.reason(follow_reason));
    // Repeat current control status; UI highlights only a matching detection frame.
    telemetry_mailbox #(.WIDTH(75)) u_follow_status_cdc(
        .src_clk(clk50M),.dst_clk(clk_vga),.reset_n(reset_n),.src_valid(1'b1),
        .src_data({follow_reason,follow_target_id,follow_status,follow_selected_slot,follow_selected_frame_id}),
        .dst_valid(),.dst_data(follow_display_data));
    motor_motion_top #(.ENABLE_NAVIGATION(1), .ENABLE_NAV_ARC(0),.ENABLE_NAV_HOLONOMIC(0),.TASK_ACTIONS_PREDECODED(1),
        .ENABLE_NAV_FORWARD_TRIM(1),.NAV_YAW_TRIM_BITS(3),.NAV_YAW_TRIM_MAX(2),.ENABLE_NAV_MAP_TOGGLE(1),
        .ENABLE_LIDAR_SAFETY(1),.LIDAR_GUARD_WHEN_DISABLED(0),.ENABLE_GAP_AVOIDANCE(0),
        .LIDAR_DYNAMIC_BYPASS(1),.AVOIDANCE_SETTLE_MS(50),.ENABLE_FLOW_BYPASS(1),.CANCEL_REMOTE_LOCK(1),
        // SW6 obstacle travel holds one IMU heading through forward/strafe.
        // Navigation and visual follow retain their own steering controllers.
        .ENABLE_HEADING_HOLD(1),.HEADING_RIGHT_YAW_POSITIVE(0),
        .ENABLE_RETURN_TO_LINE(1),.SPEED_INITIAL(9),
        .AVOID_SPEED_MAX(9),.AVOID_RETURN_SPEED(9),.AVOID_FINE_RETURN_SPEED(9),
        .AVOID_TRIGGER_CM(38),.AVOID_SLOW_CM(50),.AVOID_CLEAR_CM(50),
        .AVOID_FRONT_RELIEF_CM(10),.AVOID_RETURN_REDUCTION_MM(100),
        .AVOID_SLEW_UNITS(6),.AVOID_RETURN_BLEND(1),
        .LIDAR_STOP_CM(8),.LIDAR_SLOW_CM(22),.LIDAR_EXTRA_STOP_CM(2),
        .ENABLE_FOLLOW(1), .MOTOR_SPEED_SCALE(2), .FOLLOW_SPEED_MAX(10),.FOLLOW_TURN_ENABLE(1),
        .FOLLOW_TURN_SPEED_MAX(5),.FOLLOW_CURVE_ENABLE(1),.FOLLOW_ARC_SPEED_MAX(4),.FOLLOW_REVERSE_ENABLE(1),
        .OFFSET_TARGET_COUNTS(631),.MOTOR_OFFICIAL_PROTOCOL(1),.FOLLOW_VECTOR_ENABLE(1),.FOLLOW_VECTOR_STATUS_ENABLE(1),
        .OFFICIAL_STARTUP_MS(50),.OFFICIAL_SETTLE_MS(50),.OFFICIAL_REFRESH_MS(50)) u_motor_control(
        .clk50M(clk50M),.reset_n(reset_n),.iIR(iIR),
        .task_command_valid(task_command_valid),.task_command(task_command),
        .task_command_actions(task_command_actions),
        .avoidance_enable(avoidance_enable),
        .imu_yaw(imu_source_values[15*32+:32]),
        .imu_yaw_valid(imu_source_known[2]),.imu_yaw_update(imu_yaw_update),
        .motor_counts(rx_counts),.motor_feedback_update(rx_motor_valid),
        .mapping_enable(motor_mapping_active),.motor_online(source_motor_online),
        .motor_speeds({rx_speed_a,rx_speed_b,rx_speed_c,rx_speed_d}),
        .scan_capture_status(scan_status),.scan_map_busy(scan_map_busy),
        .scan_ray_accepted(scan_ray_accepted),.scan_sample_allow(scan_sample_allow),
        .scan_continuous_capture(scan_continuous_capture),
        .scan_auto_state(scan_auto_state),.scan_auto_progress(scan_auto_progress),
        .scan_auto_fault(scan_auto_fault),
        .scan_resume_allowed(scan_resume_allowed),.stop_wait_reason(scan_stop_wait_reason),
        .scan_resume_event(scan_resume_event),
        .scan_stop_confirmed(scan_stop_confirmed),
        .follow_mode_available(motor_follow_available),.follow_owner(motor_follow_owner),
        .follow_motion(motor_follow_motion),.follow_speed(motor_follow_speed),
        .follow_linear(motor_follow_linear),.follow_turn(motor_follow_turn),.follow_request_nonzero(motor_follow_nonzero),
        .nav_owner(motor_nav_owner),.nav_motion(motor_nav_motion),.nav_speed(motor_nav_speed),
        .nav_yaw_trim(motor_nav_yaw_trim),
        .lidar_healthy(obstacle_healthy),.lidar_rotation_valid(obstacle_rotation_valid),.lidar_rotation_cm(obstacle_rotation_cm),
        .gap_clearance_mm(144'd0),.gap_direction_valid(9'd0),
        .ir_command_valid(ir_command_valid),.ir_command(ir_command),
        .motor_stop_confirmed(motor_stop_confirmed),
        .motor_control_ready(motor_control_ready),.transmitted_motion(motor_transmitted_motion),
        .motion_frame_committed(motor_motion_committed),
        .committed_wheel_targets(source_wheel_targets),
        .front_distance_cm(obstacle_front_cm),.rear_distance_cm(obstacle_rear_cm),
        .left_distance_cm(obstacle_left_cm),.right_distance_cm(obstacle_right_cm),
        .front_valid(obstacle_front_valid),.rear_valid(obstacle_rear_valid),
        .left_valid(obstacle_left_valid),.right_valid(obstacle_right_valid),
        .front_update(obstacle_front_update),.rear_update(obstacle_rear_update),
        .left_update(obstacle_left_update),.right_update(obstacle_right_update),
        .motor_uart_tx(motor_uart_tx),.led(motor_status_led),.motion_command(source_motion),
        .requested_speed(source_requested_speed),.applied_speed(),.avoidance_state(source_avoidance_state),
        .avoidance_side_clear(source_avoidance_side_clear),
        .avoidance_fault(source_avoidance_fault),
        .avoidance_hold(motor_avoidance_hold));

    motor_telemetry_rx u_motor_rx(
        .clk(clk50M),.reset_n(reset_n),.uart_rx(motor_uart_rx),
        .data_valid(rx_motor_valid),
        .speed_a(rx_speed_a),.speed_b(rx_speed_b),
        .speed_c(rx_speed_c),.speed_d(rx_speed_d),.counts(rx_counts));
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin motor_age<=50000000; telemetry_tick<=0; source_motor_online<=0; end
        else begin
            source_motor_online<=rx_motor_valid || (motor_age<32'd24999999);
            if(rx_motor_valid) motor_age<=0;
            else if(motor_age<50000000) motor_age<=motor_age+1'b1;
            if(telemetry_tick==499999) telemetry_tick<=0;
            else telemetry_tick<=telemetry_tick+1'b1;
        end
    end
    // Stage the entire display-only snapshot, including its strobe. This
    // removes motor command arithmetic from the wide mailbox capture path
    // without delaying motor control, odometry or mapping admission.
    wire [976:0] source_telemetry_bundle={imu_source_values,imu_source_known,
                   rx_speed_a,rx_speed_b,rx_speed_c,rx_speed_d,
                   rx_counts,source_motor_online,source_motion,
                   front_distance_cm,rear_distance_cm,
                   front_measured_valid,rear_measured_valid,
                   left_distance_cm,right_distance_cm,left_valid,right_valid,
                   left_measured_valid,right_measured_valid,left_status,right_status,
                   source_avoidance_state,source_avoidance_fault,scan_avoidance_active,
                   source_wheel_targets,obstacle_front_cm,obstacle_front_valid && obstacle_healthy,source_avoidance_side_clear};
    reg [976:0] telemetry_bundle_q;
    reg telemetry_valid_q;
    always @(posedge clk50M or negedge reset_n)
        if(!reset_n)begin telemetry_bundle_q<=0;telemetry_valid_q<=0;end
        else begin telemetry_bundle_q<=source_telemetry_bundle;telemetry_valid_q<=send_telemetry;end
    telemetry_mailbox #(.WIDTH(977)) u_telemetry_cdc(
        .src_clk(clk50M),.dst_clk(clk_vga),.reset_n(reset_n),
        .src_valid(telemetry_valid_q),.src_data(telemetry_bundle_q),
        .dst_valid(display_telemetry_valid),.dst_data(display_telemetry));
    assign {imu_display_values,imu_display_known,display_speeds,
            display_counts,display_motor_online,display_motion,
            display_front_distance_cm,display_rear_distance_cm,
            display_front_distance_valid,display_rear_distance_valid,
            display_left_distance_cm,display_right_distance_cm,
            display_left_distance_valid,display_right_distance_valid,
            display_left_distance_measured_valid,display_right_distance_measured_valid,
            display_left_distance_status,display_right_distance_status,
            display_avoidance_state,display_avoidance_fault,display_avoidance_enabled,
            display_wheel_targets,display_obstacle_front_cm,display_obstacle_front_valid,display_avoidance_side_clear}=display_telemetry;

    // The acquisition switch never disables map RAM or either pose module.
    // 42 toggles acquisition; during navigation it preserves route ownership.
    // 4A clears/rebases through navigation and leaves acquisition paused.
    scan_run_control u_scan_run_control(
        .clk(clk50M),.reset_n(reset_n),.enable(mapping_active),
        .toggle_event(map_scan_toggle),.clear_event(map_scan_clear_key || nav_scan_restart),
        .engine_busy(slam_busy),.map_busy(scan_map_busy),
        .running(mapping_scan_running),.paused_ready(mapping_paused_ready));

    // FPGA-only local SLAM: raw odometry and corrected map pose stay separate.
    continuous_lidar_capture #(.SENSOR_FORWARD_MM(0),.SENSOR_LEFT_MM(0),.SENSOR_YAW_TENTHS(0)) u_scan_capture(
        .clk(clk50M),.reset_n(reset_n),.enable(mapping_active),
        .sample_allow(nav_map_sample_allow && map_acquisition_running),.restart(nav_scan_restart),
        .rescan(map_scan_toggle),
        .yaw_tenths(imu_source_values[15*32+:32]),
        .yaw_valid(imu_source_known[2]),.yaw_update(imu_yaw_update),
        .motor_update(rx_motor_valid),.motor_online(source_motor_online),
        .motor_speeds({rx_speed_a,rx_speed_b,rx_speed_c,rx_speed_d}),
        .stop_confirmed(motor_stop_confirmed),.motion(source_motion),
        .pose_valid(odom_pose_valid),.pose_fault(odom_pose_fault),
        .pose_x_mm(odom_pose_x_mm),.pose_y_mm(odom_pose_y_mm),
        .projection_heading_tenths(odom_heading),
        .point_timestamp(lidar_point_timestamp),.point_timestamp_valid(lidar_point_timestamp_valid),
        .lidar_online(tof_online),.point_valid(lidar_point_valid && lidar_obstacle_ready),.point_ready(lidar_map_ready),
        .distance_mm(lidar_distance_mm),.angle_tenths(lidar_angle_tenths),
        .range_valid(lidar_range_valid),.scan_start(lidar_scan_start),
        .map_clear_done(scan_map_clear_done),.map_clear(scan_map_clear),
        .odom_heading_tenths(odom_heading),.odom_heading_valid(odom_heading_valid),.capture_healthy(capture_healthy),
        .ray_update(capture_ray_update),.ray_frame_start(capture_frame_start),
        .ray_end_x_mm(capture_end_x),.ray_end_y_mm(capture_end_y),
        .ray_start_x_mm(capture_start_x),.ray_start_y_mm(capture_start_y),
        .ray_bin(capture_bin),.ray_bin_error(capture_bin_error),.ray_timestamp(capture_timestamp),
        .ray_body_heading_tenths(capture_body_heading),.status(capture_status));
    slam_pose_transform #(.CELL_SHIFT(6),.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y)) u_slam_pose(
        .clk(clk50M),.reset_n(reset_n),.enable(mapping_active),.clear(scan_map_clear),
        .odom_valid(odom_pose_valid),.odom_fault(odom_pose_fault),.odom_x_mm(odom_pose_x_mm),.odom_y_mm(odom_pose_y_mm),
        .odom_heading_tenths(odom_heading),.transform_update(slam_transform_update),
        .transform_x_mm(slam_transform_x),.transform_y_mm(slam_transform_y),.transform_yaw_tenths(slam_transform_yaw),
        .valid(nav_pose_valid),.fault(nav_pose_fault),.moved(nav_pose_moved),
        .transform_pending(nav_pose_transform_pending),
        .x_mm(nav_pose_x_mm),.y_mm(nav_pose_y_mm),.heading_tenths(scan_heading),
        .cell_x(nav_pose_cell_x),.cell_y(nav_pose_cell_y));
    assign scan_heading_valid=odom_heading_valid && nav_pose_valid;
    slam_mapping_pipeline #(.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.INPUT_RAY_PIPELINE(1),
        .DEFER_LOOPS_DURING_NAV(1)) u_slam_mapping(
        .clk(clk50M),.reset_n(reset_n),.enable(mapping_active),.clear(scan_map_clear),
        .rescan(map_scan_toggle),.allow(nav_map_sample_allow && map_acquisition_running),
        .navigation_active(nav_live_mapping_active),
        .healthy(capture_healthy && odom_pose_valid && !odom_pose_fault),.stationary(mapping_stationary),
        .ray_update(capture_ray_update),.ray_frame_start(capture_frame_start),.ray_bin(capture_bin),.ray_bin_error(capture_bin_error),
        .ray_timestamp(capture_timestamp),.ray_start_x_mm(capture_start_x),.ray_start_y_mm(capture_start_y),
        .ray_end_x_mm(capture_end_x),.ray_end_y_mm(capture_end_y),.current_heading_tenths(capture_body_heading),
        .map_busy(slam_loop_reference?loop_map_busy:scan_map_busy),
        .map_clear_done(slam_loop_reference?loop_map_clear_done:scan_map_clear_done),
        .rebuild_clear(slam_rebuild_clear),.loop_reference(slam_loop_reference),
        .loop_closures(slam_loop_closures),.optimized_loops(slam_optimized_loops),.dropped_frames(slam_dropped_frames),
        .history_full(slam_history_full),.keyframe_count(slam_keyframe_count),
        .map_ray_update(scan_ray_update),.map_start_x_mm(scan_ray_start_x_mm),.map_start_y_mm(scan_ray_start_y_mm),
        .map_end_x_mm(scan_ray_x_mm),.map_end_y_mm(scan_ray_y_mm),.map_epoch(slam_map_epoch),
        .map_req(slam_map_req),.map_addr(slam_map_addr),.map_req_ready(slam_map_ready),
        .map_rsp_valid(slam_map_rsp_valid),.map_rsp_data(slam_loop_reference?loop_map_data:map_read_data),
        .busy(slam_busy),.status(mapping_status),.accepted_samples(scan_samples),
        .accepted_frames(slam_accepted_frames),.rejected_frames(slam_rejected_frames),.match_score(slam_match_score),
        .reject_reason(mapping_reject_reason),.diagnostic_points(mapping_diagnostic_points),
        .diagnostic_hits(mapping_diagnostic_hits),
        .diagnostic_support(mapping_diagnostic_support),
        .transform_update(slam_transform_update),.transform_x_mm(slam_transform_x),
        .transform_y_mm(slam_transform_y),.transform_yaw_tenths(slam_transform_yaw));
    assign scan_ray_valid=scan_ray_update;
    assign scan_status=!mapping_active ? 4'd0 : !map_acquisition_running ?
        (mapping_paused_ready ? 4'd6 : 4'd7) : capture_status==5 ? mapping_status : capture_status;
    scan_map_read_mux #(.GRID_BITS(MAP_GRID_BITS)) u_map_read_mux(
        .clk(clk50M),.reset_n(reset_n),.planner_req(planner_map_req),.planner_addr(planner_map_addr),
        .planner_ready(planner_map_ready),.planner_rsp_valid(planner_map_rsp_valid),
        .matcher_req(slam_map_req&&!slam_loop_reference),.matcher_addr(slam_map_addr),.matcher_ready(global_matcher_ready),
        .matcher_rsp_valid(global_matcher_rsp),.map_req(map_read_req),.map_addr(map_read_addr),
        .map_ready(map_read_ready),.map_rsp_valid(map_read_rsp_valid));
    assign slam_map_ready=slam_loop_reference?loop_map_ready:global_matcher_ready;
    assign slam_map_rsp_valid=loop_map_rsp||global_matcher_rsp;
    // A small binary scratch map verifies historical candidates without
    // erasing the live map. Only an accepted graph optimization rebuilds it.
    scan_map_memory #(.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.ENDPOINT_LIMIT_MM(16384),
        .ENABLE_RAY_START(1),.ENABLE_PLANNER(1)) u_loop_reference_memory(
        .clk(clk50M),.reset_n(reset_n),.clear(scan_map_clear||(slam_rebuild_clear&&slam_loop_reference)),
        .rotation_sweep(1'b0),.scan_epoch(8'd0),.ray_update(scan_ray_update&&slam_loop_reference),
        .ray_valid(scan_ray_valid),.ray_end_x_mm(scan_ray_x_mm),.ray_end_y_mm(scan_ray_y_mm),
        .ray_start_x_mm(scan_ray_start_x_mm),.ray_start_y_mm(scan_ray_start_y_mm),
        .planner_req(slam_map_req&&slam_loop_reference),.planner_addr(slam_map_addr),
        .planner_req_ready(loop_map_ready),.planner_rsp_valid(loop_map_rsp),.planner_rsp_data(loop_map_data),
        .busy(loop_map_busy),.map_update(),.clear_done(loop_map_clear_done),
        .rd_clk(clk50M),.rd_addr({2*MAP_GRID_BITS{1'b0}}),.rd_data());
    assign planner_map_data=map_read_data;
    // The car no longer sweeps a rotation disk; only measured laser rays and
    // the initial stationary rectangular body footprint establish free cells.
    scan_map_memory #(.CELL_SHIFT(6),.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.ENDPOINT_LIMIT_MM(16384),
        .ENABLE_RAY_START(1),.ENABLE_PLANNER(1),.ENABLE_VEHICLE_FOOTPRINT(1),
        .BODY_FRONT_MM(140),.BODY_REAR_MM(260),.BODY_HALF_WIDTH_MM(140),
        .ENABLE_CONFIDENCE(1),.ENABLE_SCAN_EPOCH(1),
        .CONFIDENCE_BANK_BITS(2),.REGISTER_RD_ADDRESS(0),.DISPLAY_ADDRESS_COMPACT(1)) u_scan_memory(
        .clk(clk50M),.reset_n(reset_n),.clear(scan_map_clear||(slam_rebuild_clear&&!slam_loop_reference)),
        .rotation_sweep(1'b0),.scan_epoch(slam_map_epoch),
        .ray_update(scan_ray_update&&!slam_loop_reference),.ray_valid(scan_ray_valid),
        .ray_end_x_mm(scan_ray_x_mm),.ray_end_y_mm(scan_ray_y_mm),
        .ray_start_x_mm(scan_ray_start_x_mm),.ray_start_y_mm(scan_ray_start_y_mm),
        .planner_req(map_read_req),.planner_addr(map_read_addr),
        .planner_req_ready(map_read_ready),.planner_rsp_valid(map_read_rsp_valid),
        .planner_rsp_data(map_read_data),
        .busy(scan_map_busy),.map_update(scan_map_update),.clear_done(scan_map_clear_done),
        .rd_clk(clk_vga),.rd_addr(scan_read_address),.rd_data(scan_read_data));
    telemetry_mailbox #(.WIDTH(91)) u_scan_metadata_cdc(
        .src_clk(clk50M),.dst_clk(clk_vga),.reset_n(reset_n),
        .src_valid(telemetry_tick==0 || tof_update || imu_yaw_update || rx_motor_valid),
        .src_data({(slam_history_full ? 4'd14 : mapping_reject_reason),mapping_diagnostic_points,mapping_diagnostic_hits,mapping_diagnostic_support,
                    mapping_active,scan_status_for_display,scan_heading,tof_distance_mm,tof_valid,
                    scan_auto_fault,motor_stop_confirmed,scan_stop_wait_reason}),
        .dst_valid(scan_metadata_valid),.dst_data(scan_display_metadata));
    assign {scan_display_reject_reason,scan_display_diagnostic_points,scan_display_diagnostic_hits,scan_display_diagnostic_support,
             scan_display_active,scan_display_status,scan_display_heading,
            scan_display_distance,scan_display_range_valid,scan_display_fault,
             scan_display_stop_confirmed,scan_display_stop_wait_reason}=scan_display_metadata;

    // Rebase flushes odometry state; its decoded IR/mode cone must not also
    // drive the odometry fault register's CE in the same cycle.
    reg odom_rebase_r,odom_mapping_r;
    always @(posedge clk50M or negedge reset_n)
        if(!reset_n) begin odom_rebase_r<=0;odom_mapping_r<=0;end
        else begin odom_rebase_r<=scan_map_clear;odom_mapping_r<=mapping_active;end
    navigation_pose #(.CELL_SHIFT(6),.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.ODOM_LIMIT_MM(16000),
        // Measured straight 600 mm: average 720 counts/wheel, total 2880.
        // 600*4096/(4*720)=853.333; integer Q12 scale rounds to853.
        .MM_PER_SUM_COUNT_Q12(853),
        // Measured lateral 200 mm: mean sum(|wheel counts|)=1084.333.
        // 200*4096/1084.333=755.487; integer Q12 scale rounds to755.
        .STRAFE_MM_PER_SUM_COUNT_Q12(755),
        .ENABLE_ARC_MIDPOINT(1),.ENABLE_TRANSLATION_MIDPOINT(1)) u_nav_pose(
        .clk(clk50M),.reset_n(reset_n),.enable(odom_mapping_r),.rebase(odom_rebase_r),
        .heading_tenths(odom_heading),.heading_valid(odom_heading_valid),.yaw_update(imu_yaw_update),
        .counts(rx_counts),.speeds({rx_speed_a,rx_speed_b,rx_speed_c,rx_speed_d}),
        .motor_update(rx_motor_valid),.motor_online(source_motor_online),.motion(source_motion),
        .valid(odom_pose_valid),.fault(odom_pose_fault),.moved(odom_pose_moved),
        .x_mm(odom_pose_x_mm),.y_mm(odom_pose_y_mm),.cell_x(),.cell_y());
    navigation_localization_guard #(.USE_POSE_DELTA(1),.CLK_FREQ_HZ(50000000),
        .POSITION_MM(2*MAP_CELL_MM),.YAW_TENTHS(50),.COOLDOWN_MS(3000),
        .EMERGENCY_POSITION_MM(4*MAP_CELL_MM),.EMERGENCY_YAW_TENTHS(100)) u_nav_localization_guard(
        .clk(clk50M),.reset_n(reset_n),.enable(mapping_active),.clear(scan_map_clear),
        .active(nav_live_mapping_active),.transform_update(slam_transform_update),
        .transform_x_mm(slam_transform_x),.transform_y_mm(slam_transform_y),
        .transform_yaw_tenths(slam_transform_yaw),
        .pose_valid(nav_pose_valid),.pose_transform_pending(nav_pose_transform_pending),
        .pose_x_mm(nav_pose_x_mm),.pose_y_mm(nav_pose_y_mm),.pose_heading_tenths(scan_heading),
        .replan(nav_localization_replan));
    navigation_control #(.CELL_SHIFT(6),.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.MULTI_POSE_MAPPING(1),.GOAL_STEP_CELLS(4),
        .ENABLE_LIVE_MAPPING(1),
        .AUTO_REPLAN_OFF_PATH(1),.OFF_PATH_RETRY_MS(1000),.OFF_PATH_MAX_RETRIES(3),
        .USE_RAW_ODOMETRY_PROGRESS(1),
        .NAV_SPEED(4),.NAV_TURN_SPEED(2),.ENABLE_HOLONOMIC(0),.ENABLE_FORWARD_TRIM(1),
        .NAV_START_SPEED(5),.NAV_START_BOOST_MS(150),.NAV_RECOVERY_SPEED(3),
        .ENABLE_HOLONOMIC_TRACKING(0),.NAV_YAW_TRIM_BITS(3),.NAV_YAW_TRIM_MAX(2),
        .ENABLE_CONTINUOUS_STEER(0),.NAV_ARC_SPEED(3),
        .STEER_ENTER_TENTHS(50),.STEER_EXIT_TENTHS(20),.CROSS_TRACK_DEADBAND_MM(30),
        .TURN_TOLERANCE_TENTHS(50),.MOVE_YAW_TOLERANCE_TENTHS(100),.STOP_EARLY_MM(5),
        .CROSS_TRACK_MM(30),.ENABLE_CROSS_TRACK_STOP(0),.GOAL_TOLERANCE_MM(30),.ENABLE_FRONT_STOP(0),.FRONT_STOP_MM(250),
        .STOP_ON_MOVE_YAW_DEVIATION(0)) u_navigation(
        .clk(clk50M),.reset_n(reset_n),.enable(mapping_active),
        .ir_valid(map_ir_valid),.ir_command(ir_command),.scan_state(scan_auto_state),
        .scan_resume_event(scan_resume_event),
        .scan_samples(scan_samples),.map_busy(navigation_map_busy),.map_clear_done(scan_map_clear_done),
        .localization_replan(nav_localization_replan),
        .pose_valid(nav_pose_valid),.pose_fault(nav_pose_fault),.pose_moved(nav_pose_moved),
        .pose_x_mm(nav_pose_x_mm),.pose_y_mm(nav_pose_y_mm),
        .raw_pose_x_mm(odom_pose_x_mm),.raw_pose_y_mm(odom_pose_y_mm),
        .pose_cell_x(nav_pose_cell_x),.pose_cell_y(nav_pose_cell_y),
        .heading_tenths(scan_heading),.heading_valid(scan_heading_valid),.yaw_update(imu_yaw_update),
        .motor_online(source_motor_online),.stop_confirmed(motor_stop_confirmed),
        .motor_speeds({rx_speed_a,rx_speed_b,rx_speed_c,rx_speed_d}),
        .distance_mm(tof_distance_mm),.range_valid(tof_valid),.range_update(tof_update),.tof_online(tof_online),
        .planner_busy(planner_busy),.planner_done(planner_done),.planner_success(planner_success),
        .planner_fail(planner_fail),.waypoint_busy(waypoint_busy),.waypoint_valid(waypoint_valid),
        .planner_body_heading(planner_body_heading),
        .waypoint_last(waypoint_last),.waypoint_x(waypoint_x),.waypoint_y(waypoint_y),
        .planner_start(planner_start),.planner_abort(planner_abort),.path_rewind(path_rewind),
        .waypoint_next(waypoint_next),.scan_restart(nav_scan_restart),.map_sample_allow(nav_map_sample_allow),
        .nav_owner(nav_owner),.manual_takeover(nav_manual_takeover),.nav_motion(nav_motion),.nav_speed(nav_speed),
        .nav_live_mapping_active(nav_live_mapping_active),
        .nav_yaw_trim(nav_yaw_trim),
        .nav_status(nav_status),.nav_fault(nav_fault),.goal_x_cell(nav_goal_x),.goal_y_cell(nav_goal_y),
        .goal_valid(nav_goal_valid),.path_valid(nav_path_valid));
    // Radar-relative 14 cm front, 26 cm rear (monitor), 14 cm each side.
    // Keep the existing translation planner and its corridor-centre costs.
    // Reduce its padding to 2 cm. Execution turns then drives forward; do
    // not reject or reshape this route using a full-turn clearance mask.
    body_path_planner #(.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.CELL_MM(MAP_CELL_MM),
        .FRONT_MM(140),.REAR_MM(260),.HALF_WIDTH_MM(140),.MARGIN_MM(20),.ALLOW_UNKNOWN(1),
        .CHECK_START_CLEARANCE(0),.CHECK_GOAL_CLEARANCE(0),
        .TURN_IN_PLACE(0),.TRACK_YAW_TENTHS(100)) u_nav_planner(
        .clk(clk50M),.reset_n(reset_n),.start_plan(planner_start),.abort(planner_abort),
        .start_x(nav_pose_cell_x),.start_y(nav_pose_cell_y),.goal_x(nav_goal_x),.goal_y(nav_goal_y),
        .start_heading_tenths(scan_heading),
        .map_req(planner_map_req),.map_addr(planner_map_addr),.map_req_ready(planner_map_ready),
        .map_rsp_valid(planner_map_rsp_valid),.map_rsp_data(planner_map_data),
        .busy(planner_busy),.done(planner_done),.success(planner_success),.fail_code(planner_fail),
        .route_uses_unknown(planner_route_uses_unknown),
        .body_heading(planner_body_heading),
        .path_rewind(path_rewind),.waypoint_next(waypoint_next),.waypoint_busy(waypoint_busy),
        .waypoint_valid(waypoint_valid),.waypoint_x(waypoint_x),.waypoint_y(waypoint_y),.waypoint_last(waypoint_last),
        .path_clear(path_clear),.path_cell_valid(path_cell_valid),.path_cell_addr(path_cell_addr));
    wire retained_auto_path=nav_manual_takeover && nav_path_valid;
    navigation_path_memory #(.GRID_BITS(MAP_GRID_BITS),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.RD_ADDRESS_COMPACT(1),.RETAIN_ENABLE(1)) u_nav_path_memory(
        .clk(clk50M),.reset_n(reset_n),.clear(scan_map_clear || path_clear || slam_rebuild_clear),.retain_path(retained_auto_path),
        .write_valid(path_cell_valid),.write_addr(path_cell_addr),.clear_busy(path_clear_busy),
        .write_bank(path_write_bank),.rd_clk(clk_vga),.rd_addr(scan_read_address),
        .rd_bank(path_frame_bank),.rd_data(path_display_data));
    // This warning is display-only. It never feeds navigation_control's fault
    // state or motor command: a route through unmeasured cells may still run.
    wire [3:0] nav_page_fault=(nav_fault==0 && nav_path_valid && planner_route_uses_unknown) ?
        4'd12 : nav_fault;
    telemetry_mailbox #(.WIDTH(45+2*MAP_GRID_BITS)) u_nav_metadata_cdc(
        .src_clk(clk50M),.dst_clk(clk_vga),.reset_n(reset_n),
        .src_valid(telemetry_tick==0 || ir_command_valid || rx_motor_valid || planner_done),
        .src_data({nav_status,nav_page_fault,nav_goal_x,nav_goal_y,nav_goal_valid,nav_pose_valid,
                   nav_pose_x_mm,nav_pose_y_mm,nav_path_valid && !path_clear_busy,path_write_bank,motor_stop_confirmed}),
        .dst_valid(nav_metadata_valid),.dst_data(nav_display_metadata));
    assign {nav_display_status,nav_display_fault,nav_display_goal_x,nav_display_goal_y,
            nav_display_goal_valid,nav_display_pose_valid,nav_display_pose_x_mm,nav_display_pose_y_mm,
            nav_display_path_valid,nav_display_path_bank,nav_display_stop_confirmed}=nav_display_metadata;
    always @(posedge clk_vga or negedge reset_n)
        if(!reset_n) path_frame_bank<=0;
        else if(video_frame_begin) path_frame_bank<=nav_display_path_bank;


    // Normal mode keeps the original PLL/camera/DDR status lamps. Avoidance
    // HOLD exposes its captured fault on the actual four board LED pins.
    assign led = motor_avoidance_hold ? motor_status_led :
                 {hdmi_pll_lock,camera_init_done,init_calib_complete,pll_lock};
    
    
    camera_pll camera_pll(
        .clkout0(clk_vgax5), //output clkout0
        .clkout1(loc_clk24m), //output clkout1
        .clkin(loc_clk50m), //input clkin
        .init_clk(clk50M), // PLL initialization runs from the stable board clock
        .lock(hdmi_pll_lock)
    );

    Gowin_PLL Gowin_PLL(
        .lock(pll_lock), //output lock
        .clkout2(clk_200M), //GW5AT-138 DDR memory clock must use CLKOUT2
        .enclk2(ddr_pll_enable),
        .clkout1(loc_clk50m), //output clkout1
        .clkin(clk50M), //input clkin
        .reset(~reset_n) //input reset
    );


    CLKDIV u_clkdiv
    (.RESETN(hdmi_pll_lock)
    ,.HCLKIN(clk_vgax5) //clk  x5
    ,.CLKOUT(clk_vga)    //clk  x1
    ,.CALIB (1'b1)
    );
    defparam u_clkdiv.DIV_MODE="5";


    assign camera_xclk = loc_clk24m;
    //摄像头
    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ),//系统时钟采用50MHz
        .SCL_CLOCK      ( 400_000      ),//SCL总线时钟采用400kHz
        .CAMERA_TYPE    ( "ov5640"     ),//"ov5640" or "ov7725"
        .IMAGE_TYPE     ( 0            ),// 0: RGB; 1: JPEG
        .IMAGE_WIDTH    ( IMAGE_WIDTH  ),// 图片宽度
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ),// 图片高度
        .IMAGE_FLIP_EN  ( 1            ),// 0: 不翻转，1: 上下翻转（本车摄像头倒装）
        .IMAGE_MIRROR_EN( 1            ) // 0: 不镜像，1: 左右镜像（与 FLIP 同时打开即旋转 180°）
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (reset_n         ),
        .Init_Done   (camera_init_done ),
        .camera_rst_n(camera_rst_n     ),
        .camera_pwdn (camera_pwdn      ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );
    
    assign pclk_bufg_o = camera_pclk;

    DVP_Capture DVP_Capture(
        .Rst_n      (reset_n         ),//input
        .PCLK       (pclk_bufg_o      ),//input
        .Vsync      (camera_vsync     ),//input
        .Href       (camera_href      ),//input
        .Data       (camera_data      ),//input     [7:0]

        .ImageState (fifo_aclr        ),//output reg
        .DataValid  (fifo_wrreq       ),//output
        .DataPixel  (fifo_wrdata      ),//output    [15:0]
        .DataHs     (                 ),//output
        .DataVs     (wr_load          ),//output
        .Xaddr      (                 ),//output    [11:0],start is 1
        .Yaddr      (                 )   //output    [11:0],start is 1
    );
    

    wire rdfifo_rden;
    wire wrfifo_clr;
    wire [15:0] wrfifo_din;
    wire wrfifo_clk;
    wire rdfifo_clr;
    wire rdfifo_clk;
    wire [15:0] rdfifo_dout;
    
    
    assign wrfifo_clr = fifo_aclr;
    assign wrfifo_clk = pclk_bufg_o;
    assign rdfifo_clr = 0;
    assign rdfifo_clk = clk_vga;
    assign wrfifo_wren = fifo_wrreq;
    assign wrfifo_din = fifo_wrdata;
    assign disp_data = {rdfifo_dout[15:11],3'd0,rdfifo_dout[10:5],2'd0,rdfifo_dout[4:0],3'd0}; 

    wire [27:0] app_addr_max = IMAGE_WIDTH * IMAGE_HEIGHT;
    wire [7:0] burst_len = IMAGE_WIDTH[10:3];

    ddr_reset_retry u_ddr_reset_retry (
        .clk(loc_clk50m),
        .reset_n(reset_n),
        .pll_lock(pll_lock),
        .init_calib_complete(init_calib_complete),
        .ddr_reset_n(ddr_reset_n)
    );

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
        .pll_lock(pll_lock)            ,
        .clk_200m(clk_200M)            ,      //DDR3参考时钟信号
        .pll_stop(ddr_pll_enable),
        .sys_rst_n(ddr_reset_n)       ,      //wait for PLL and retry DDR calibration
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

        //用户接口
        .rd_load(video_vs)             ,   //输出源更新信号
        .wr_load(wr_load)             ,   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(wrfifo_clk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wrfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wrfifo_din)           , //写入到wr_fifo中的数据
        .rd_clk(rdfifo_clk)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 

        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
   

    //HDMI显示
    disp_driver disp_driver(
		.ClkDisp(clk_vga),
		.Rst_n(reset_n),
		.Data(disp_data),
		.DataReq(rdfifo_rden),
		.H_Addr(hcount),
		.V_Addr(vcount),
		.Disp_HS(video_hs),
		.Disp_VS(video_vs),
		.Disp_Red(video_r),
		.Disp_Green(video_g),
		.Disp_Blue(video_b),
        .Frame_Begin(video_frame_begin),
		.Disp_DE(video_de),
		.Disp_PCLK()
	);

    // disp_driver 的有效区 H_Addr 为 1..1280，V_Addr 为 0..719。
    // OSD 使用从 0 开始的坐标；RGB/HS/VS/DE 在模块内统一延迟 5 拍。
    // Keep a valid HDMI picture while DDR3 or the camera is initializing.
    // Switch to camera pixels only at a frame boundary after both are ready.
    reg image_ready_meta, image_ready_sync, video_image_ready;
    always @(posedge clk_vga or negedge reset_n) begin
        if (!reset_n) begin
            image_ready_meta <= 1'b0;
            image_ready_sync <= 1'b0;
            video_image_ready <= 1'b0;
        end else begin
            image_ready_meta <= init_calib_complete && camera_init_done;
            image_ready_sync <= image_ready_meta;
            if (!image_ready_sync)
                video_image_ready <= 1'b0;
            else if (video_frame_begin)
                video_image_ready <= 1'b1;
        end
    end
    wire [23:0] fallback_rgb =
        (hcount < 12'd320) ? 24'hff0000 :
        (hcount < 12'd640) ? 24'h00ff00 :
        (hcount < 12'd960) ? 24'h0000ff : 24'hffffff;
    wire [23:0] video_source_rgb = video_image_ready ?
                                   {video_r, video_g, video_b} : fallback_rgb;

    // SW4 is a maintained level switch. Reset starts with streaming off;
    // the existing two-flop synchronizer requires 20 ms of a stable level.
    avoidance_switch u_recognition_switch(
        .clk(clk50M),.reset_n(reset_n),.switch_async(recognition_enable),
        .avoidance_enabled(network_video_enabled));
    wire [15:0] screen_rgb565;
    wire screen_pixel_valid,screen_vsync,telemetry_record_screen;
    wire record_screen_page=ETHERNET_SCREEN_RECORD || obstacle_page_active ||
        speed_page_active || scan_page_active || telemetry_record_screen;
    display_record_tap u_screen_record_tap(
        .pixel_clk(clk_vga),.reset_n(reset_n),
        .hdmi_rgb(hdmi_rgb),.hdmi_de(hdmi_de),.hdmi_vs(hdmi_vs),
        .pixel_rgb565(screen_rgb565),.pixel_valid(screen_pixel_valid),
        .capture_vsync(screen_vsync));
    ethernet_vision_bridge #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .FRAME_STRIDE(ETHERNET_FRAME_STRIDE),.SCREEN_FRAME_STRIDE(SCREEN_RECORD_FRAME_STRIDE),
        .TX_CLOCK_DELAY(ETHERNET_TX_CLOCK_DELAY),.SCREEN_RECORD(0),.DYNAMIC_SCREEN(1),
        .SCREEN_FULL_RES(SCREEN_RECORD_FULL_RES),
        .CAMERA_FULL_RES(CAMERA_FULL_RES),
        .FOLLOW_TRACK_ENABLE(ETHERNET_SCREEN_RECORD ? 0 : 1)) u_vision_ethernet(
        .clk50m(clk50M),.reset_n(reset_n),
        // Independent DVP / HDMI row buffers; packet selection never switches clocks.
        .camera_clk(pclk_bufg_o),
        .recognition_enable(network_video_enabled),
        .camera_vsync(camera_vsync),.screen_record(record_screen_page),
        .screen_vsync(screen_vsync),.screen_capture_enable(hdmi_pll_lock),
        .screen_pixel_valid(screen_pixel_valid),.screen_pixel_rgb565(screen_rgb565),
        .capture_enable(camera_init_done),
        .pixel_valid(fifo_wrreq),
        .pixel_rgb565(fifo_wrdata),.display_clk(clk_vga),
        .audio_bclk(audio_bclk),.audio_lrclk(audio_lrclk),.audio_adc_data(audio_adc_data),
        .audio_mclk(audio_mclk),.audio_i2c_scl(audio_i2c_scl),.audio_i2c_sda(audio_i2c_sda),
        .boxes_valid(detection_valid),.boxes_frame_id(detection_frame_id),
        .boxes_count(detection_count),.boxes_coords(detection_coords),
    .boxes_classes(detection_classes),
        .boxes_label_text(detection_label_text),.boxes_label_length(detection_label_length),
        .boxes_tracker_session(detection_tracker_session),.boxes_track_ids(detection_track_ids),
        .boxes_confidences(detection_confidences),.boxes_observed(detection_observed),
        .task_command_valid(task_command_valid),.task_command(task_command),
        .task_command_actions(task_command_actions),
        .thermal_scl(thermal_scl),.thermal_sda(thermal_sda),
        .arrival_status({thermal_arrived && motor_stop_confirmed,thermal_arrival_session,thermal_arrival_id}),
        .thermal_sample_valid(thermal_sample_valid),.thermal_sensor_valid(thermal_sensor_valid),
        .thermal_task_active(thermal_task_active),.thermal_maximum_centi(thermal_sample_maximum),
        .thermal_threshold_c(thermal_sample_threshold),
        .follow_lock_session(follow_target_session),.follow_lock_id(follow_target_id),
        .follow_lock_valid(follow_lock_valid),
        .boxes_reset_n(detection_reset_n),
        .eth_rxc(eth_rxc),.eth_rxd(eth_rxd),.eth_rxdv(eth_rxdv),
        .eth_gtxc(eth_gtxc),.eth_txd(eth_txd),.eth_txen(eth_txen),
        .eth_rst_n(eth_rst_n),.eth_mdc(eth_mdc),.eth_mdio(eth_mdio),.phy_link_up());
    wire imu_panel_debounced;
    reg imu_panel_meta,imu_panel_sync;
    wire [215:0] display_imu_nine_bcd;
    wire [8:0] display_imu_nine_negative,display_imu_nine_valid;
    avoidance_switch u_imu_panel_switch(
        .clk(clk50M),.reset_n(reset_n),.switch_async(imu_panel_enable),
        .avoidance_enabled(imu_panel_debounced));
    always @(posedge clk_vga or negedge reset_n) begin
        if(!reset_n) begin imu_panel_meta<=0;imu_panel_sync<=0;end
        else begin imu_panel_meta<=imu_panel_debounced;imu_panel_sync<=imu_panel_meta;end
    end
    // Object-box comparisons occupy the OSD's existing first-to-second
    // pipeline stage. All RGB/sync outputs retain their six-cycle alignment.
    telemetry_pages #(.PERSON_BOXES_ENABLE(1),.TRACK_ID_LABEL_ENABLE(1)) u_telemetry_osd (
        .clk(clk_vga),
        .rst_n(reset_n),
        .frame_begin(video_frame_begin),
        .pixel_x(video_de ? (hcount - 12'd1) : 12'd0),
        .pixel_y(vcount),
        .rgb_in(video_source_rgb),
        .recognition_active_video(detection_reset_n),
        .person_boxes_reset_n(detection_reset_n),.person_boxes_enable(video_image_ready),
        .person_boxes_valid(detection_valid),.person_boxes_frame_id(detection_frame_id),
        .person_boxes_count(detection_count),.person_boxes_coords(detection_coords),
    .person_boxes_classes(detection_classes),
        .person_boxes_label_text(detection_label_text),.person_boxes_label_length(detection_label_length),
        .follow_status(follow_display_status),.follow_selected_slot(follow_display_slot),
        .follow_frame_id(follow_display_frame_id),
        .follow_target_id(follow_display_target_id),.follow_reason(follow_display_reason),
        .hs_in(video_hs),
        .vs_in(video_vs),
        .de_in(video_de),
        .page_select(page_select),.telemetry_valid(display_telemetry_valid),
        .imu_panel_select(imu_panel_sync),.record_screen_page(telemetry_record_screen),
        .imu_nine_bcd(display_imu_nine_bcd),.imu_nine_negative(display_imu_nine_negative),
        .imu_nine_valid(display_imu_nine_valid),
        .imu_values(imu_display_values),.imu_known(imu_display_known),
        .motor_speeds(display_speeds),.motor_counts(display_counts),
        .motor_online(display_motor_online),.motion_command(display_motion),
        .front_distance_cm(display_front_distance_cm),.rear_distance_cm(display_rear_distance_cm),
        .front_distance_valid(display_front_distance_valid),.rear_distance_valid(display_rear_distance_valid),
        .left_distance_cm(display_left_distance_cm),.right_distance_cm(display_right_distance_cm),
        .left_distance_valid(display_left_distance_valid),.right_distance_valid(display_right_distance_valid),
        .left_distance_measured_valid(display_left_distance_measured_valid),
        .right_distance_measured_valid(display_right_distance_measured_valid),
        .left_distance_status(display_left_distance_status),.right_distance_status(display_right_distance_status),
        .avoidance_state(display_avoidance_state),.avoidance_fault(display_avoidance_fault),
        .avoidance_enabled(display_avoidance_enabled),
        .rgb_out(osd_rgb),
        .hs_out(osd_hs), .vs_out(osd_vs), .de_out(osd_de)
    );

    // SW2 has display priority over SW3, maps and telemetry. Like the other
    // board switches it is a maintained high level, not a pushbutton toggle.
    wire obstacle_page_debounced;
    reg obstacle_page_meta,obstacle_page_sync;
    avoidance_switch u_obstacle_page_switch(
        .clk(clk50M),.reset_n(reset_n),.switch_async(obstacle_page_enable),
        .avoidance_enabled(obstacle_page_debounced));
    always @(posedge clk_vga or negedge reset_n) begin
        if(!reset_n) begin obstacle_page_meta<=0;obstacle_page_sync<=0;end
        else begin obstacle_page_meta<=obstacle_page_debounced;obstacle_page_sync<=obstacle_page_meta;end
    end
    obstacle_curve_page u_obstacle_page(
        .clk(clk_vga),.reset_n(reset_n),.frame_begin(video_frame_begin),.active(obstacle_page_sync),
        .pixel_x(video_de ? (hcount-12'd1) : 12'd0),.pixel_y(vcount),
        .hs_in(video_hs),.vs_in(video_vs),.de_in(video_de),
        .front_cm(display_obstacle_front_cm),.front_valid(display_obstacle_front_valid),
        .actual_speeds(display_speeds),.motor_online(display_motor_online),
        .avoidance_enabled(display_avoidance_enabled),
        .motion_command(display_motion),.avoidance_state(display_avoidance_state),
        .avoidance_fault(display_avoidance_fault),.side_clear(display_avoidance_side_clear),
        .rgb_out(obstacle_rgb),.hs_out(obstacle_hs),.vs_out(obstacle_vs),.de_out(obstacle_de),
        .active_out(obstacle_page_active));

    // SW3 changes only the HDMI view. Capture and all motion owners keep
    // running; page selection is debounced and applied at vertical blank.
    wire speed_page_debounced;
    reg speed_page_meta,speed_page_sync;
    avoidance_switch u_speed_page_switch(
        .clk(clk50M),.reset_n(reset_n),.switch_async(speed_page_enable),
        .avoidance_enabled(speed_page_debounced));
    always @(posedge clk_vga or negedge reset_n) begin
        if(!reset_n) begin speed_page_meta<=0;speed_page_sync<=0;end
        else begin speed_page_meta<=speed_page_debounced;speed_page_sync<=speed_page_meta;end
    end
    wheel_speed_page u_speed_page(
        .clk(clk_vga),.reset_n(reset_n),.frame_begin(video_frame_begin),
        .active(speed_page_sync),
        .pixel_x(video_de ? (hcount-12'd1) : 12'd0),.pixel_y(vcount),
        .hs_in(video_hs),.vs_in(video_vs),.de_in(video_de),
        .actual_speeds(display_speeds),.target_speeds(display_wheel_targets),
        .imu_bcd(display_imu_nine_bcd),.imu_negative(display_imu_nine_negative),.imu_valid(display_imu_nine_valid),
        .motor_online(display_motor_online),
        .rgb_out(speed_rgb),.hs_out(speed_hs),.vs_out(speed_vs),.de_out(speed_de),
        .active_out(speed_page_active));

    scan_map_page #(.CELL_SHIFT(6),.CELL_MM(MAP_CELL_MM),.GRID_SIZE(MAP_GRID_SIZE),.GRID_X(MAP_GRID_X),.GRID_Y(MAP_GRID_Y),.ALLOW_UNKNOWN_PATH(1),.ENABLE_MAPPING_DIAGNOSTICS(1),.COMPACT_ADDRESS(1)) u_scan_page(
        .clk(clk_vga),.reset_n(reset_n),.frame_begin(video_frame_begin),
        .pixel_x(video_de ? (hcount-12'd1) : 12'd0),.pixel_y(vcount),
        .hs_in(video_hs),.vs_in(video_vs),.de_in(video_de),
        .active(scan_display_active),.status(scan_display_status),
        .auto_fault(scan_display_fault),
        .reject_reason(scan_display_reject_reason),
        .diagnostic_points(scan_display_diagnostic_points),.diagnostic_hits(scan_display_diagnostic_hits),
        .diagnostic_support(scan_display_diagnostic_support),
        .motor_speeds(display_speeds),.motor_online(display_motor_online),
        .stop_wait_reason(scan_display_stop_wait_reason),
        .stop_confirmed(nav_display_status!=0 ? nav_display_stop_confirmed : scan_display_stop_confirmed),
        .heading_tenths(scan_display_heading),.distance_mm(scan_display_distance),
        .distance_valid(scan_display_range_valid),
        .nav_status(nav_display_status),.nav_fault(nav_display_fault),
        .goal_x_cell(nav_display_goal_x),.goal_y_cell(nav_display_goal_y),
        .goal_valid(nav_display_goal_valid),.pose_valid(nav_display_pose_valid),
        .pose_x_mm(nav_display_pose_x_mm),.pose_y_mm(nav_display_pose_y_mm),
        .path_valid(nav_display_path_valid),.path_rd_data(path_display_data),
        .map_rd_data(scan_read_data),.map_rd_addr(scan_read_address),.map_rd_addr_raw(scan_read_address_raw),
        .rgb_out(scan_rgb),.hs_out(scan_hs),.vs_out(scan_vs),.de_out(scan_de),
        .active_out(scan_page_active));
	dvi_encoder u_dvi_encoder(
		.pixelclk(clk_vga),// system clock
		.pixelclk5x(clk_vgax5),// system clock x5
		.rst_n(reset_n),// match the board-verified OSER10 color-bar startup
		.blue_din(hdmi_rgb[7:0]),// Blue data in
		.green_din(hdmi_rgb[15:8]),// Green data in
		.red_din(hdmi_rgb[23:16]),// Red data in
		.hsync(hdmi_hs),// hsync data
		.vsync(hdmi_vs),// vsync data
		.de(hdmi_de),// data enable
		.tmds_clk_p(tmds_clk_p),
		.tmds_clk_n(tmds_clk_n),
		.tmds_data_p(tmds_data_p),//rgb
		.tmds_data_n(tmds_data_n) //rgb
	);

endmodule
