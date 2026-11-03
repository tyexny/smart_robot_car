`timescale 1ns/1ps

// Remote requests pass through obstacle control before becoming UART frames.
// Every transmitted frame owns its motion/speed snapshot until its final byte.
module motor_motion_top #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer BAUD_RATE=115_200,
    parameter integer FRAME_GAP_MS=20,
    parameter integer MODE_DEBOUNCE_MS=20,
    parameter [7:0] IR_SPEED_UP=8'h45,
    parameter [7:0] IR_SPEED_DOWN=8'h47,
    parameter integer SPEED_INITIAL=5,
    parameter integer SPEED_MIN=1,
    parameter integer SPEED_MAX=10,
    parameter integer SPEED_STEP=1,
    // Policies and IR keys use grades; convert to motor units only in a frame.
    parameter integer MOTOR_SPEED_SCALE=1,
    parameter integer FOLLOW_SPEED_MAX=2,
    parameter integer FOLLOW_TURN_ENABLE=0,
    parameter integer FOLLOW_TURN_SPEED_MAX=FOLLOW_SPEED_MAX,
    parameter integer FOLLOW_CURVE_ENABLE=0, FOLLOW_ARC_SPEED_MAX=FOLLOW_SPEED_MAX,
    parameter integer FOLLOW_REVERSE_ENABLE=0,
    parameter integer FOLLOW_VECTOR_ENABLE=0,
    parameter integer FOLLOW_VECTOR_STATUS_ENABLE=0,
    // Legacy callers retain an always-on guard. Production explicitly opts
    // out below SW6 so remote/follow/navigation recover their original policy.
    parameter integer ENABLE_LIDAR_SAFETY=0,
    parameter integer LIDAR_GUARD_WHEN_DISABLED=1,
    parameter integer ENABLE_GAP_AVOIDANCE=0,
    parameter integer LIDAR_DYNAMIC_BYPASS=0, AVOIDANCE_SETTLE_MS=200,
    parameter integer ENABLE_FLOW_BYPASS=0, CANCEL_REMOTE_LOCK=0,
    parameter integer ENABLE_HEADING_HOLD=0, HEADING_RIGHT_YAW_POSITIVE=0,
    parameter integer LIDAR_STOP_CM=15,LIDAR_SLOW_CM=35,LIDAR_EXTRA_STOP_CM=3,
    parameter integer CONTROL_REQUEST_PIPELINE=0,
    parameter integer TASK_ACTIONS_PREDECODED=0,
    parameter integer OFFSET_TARGET_COUNTS=631,
    parameter integer SCAN_ANGLE=600, ENABLE_AUTO_SCAN=0,
    parameter integer ENABLE_REVERSE=1, REVERSE_MS=300,
    parameter integer ENABLE_RETURN_TO_LINE=0,
    parameter integer AVOID_SPEED_MAX=3,AVOID_RETURN_SPEED=2,AVOID_FINE_RETURN_SPEED=1,
    parameter integer AVOID_TRIGGER_CM=24,AVOID_SLOW_CM=32,AVOID_CLEAR_CM=36,
    parameter integer AVOID_FRONT_RELIEF_CM=0,AVOID_RETURN_REDUCTION_MM=0,
    parameter integer AVOID_SLEW_UNITS=2,AVOID_RETURN_BLEND=0,
    parameter integer ENABLE_YAW_DEVIATION_STOP=0, ENABLE_ENCODER_STOP=0,
    parameter integer ENABLE_MAP_SCAN=0, ENABLE_NAVIGATION=0, ENABLE_FOLLOW=0,
    parameter integer ENABLE_NAV_ARC=0, ENABLE_NAV_HOLONOMIC=0,
    parameter integer ENABLE_NAV_FORWARD_TRIM=0,
    parameter integer NAV_YAW_TRIM_BITS=2,NAV_YAW_TRIM_MAX=1,
    parameter integer ENABLE_NAV_MAP_TOGGLE=0,
    parameter integer MAP_SCAN_STEP_TENTHS=50, MAP_SCAN_ROTATE_SPEED=2,
    parameter integer MAP_SCAN_TOTAL_MS=180000,
    parameter integer MAP_SCAN_CONTINUOUS=0, MAP_SCAN_START_SPEED=2, MAP_SCAN_START_BOOST_MS=150,
    parameter integer SCAN_MOTION_REFRESH_MS=100,
    parameter integer UART_FRAME_SILENCE_MS=2, STOP_RETRY_MS=100,
    parameter integer MOTOR_OFFICIAL_PROTOCOL=0,
    parameter integer OFFICIAL_STARTUP_MS=50, OFFICIAL_SETTLE_MS=50,
    parameter integer OFFICIAL_REFRESH_MS=50
)(
    input wire clk50M, reset_n,
    input wire iIR,
    input wire task_command_valid,
    input wire [7:0] task_command,
    input wire [5:0] task_command_actions,
    input wire avoidance_enable, // SW6 raw level: 0 remote, 1 avoidance
    input wire lidar_healthy,lidar_rotation_valid,
    input wire [15:0] lidar_rotation_cm,
    input wire [143:0] gap_clearance_mm,
    input wire [8:0] gap_direction_valid,
    input wire [15:0] front_distance_cm, rear_distance_cm,
    input wire front_valid, rear_valid, front_update, rear_update,
    input wire [15:0] left_distance_cm, right_distance_cm,
    input wire left_valid, right_valid, left_update, right_update,
    input wire signed [31:0] imu_yaw,
    input wire imu_yaw_valid, imu_yaw_update,
    input wire [63:0] motor_counts,
    input wire motor_feedback_update,
    input wire mapping_enable,motor_online,
    input wire [63:0] motor_speeds,
    input wire [3:0] scan_capture_status,
    input wire scan_map_busy,scan_ray_accepted,scan_resume_allowed,
    // Follow owns motion only after explicit selection; default disabled keeps legacy callers.
    input wire follow_mode_available,follow_owner,
    input wire [3:0] follow_motion,
    input wire [7:0] follow_speed,
    // Motor command units: positive linear is forward; positive turn is right.
    // Ignored unless both vector following and the official protocol are enabled.
    input wire signed [15:0] follow_linear,follow_turn,
    input wire follow_request_nonzero,
    input wire nav_owner,
    input wire [3:0] nav_motion,
    input wire [7:0] nav_speed,
    input wire signed [NAV_YAW_TRIM_BITS-1:0] nav_yaw_trim,
    output reg ir_command_valid,
    output reg [7:0] ir_command,
    output wire motor_stop_confirmed,
    output wire scan_resume_event,
    output wire scan_sample_allow,scan_continuous_capture,
    output wire [2:0] scan_auto_state,
    output wire [11:0] scan_auto_progress,
    output wire [3:0] scan_auto_fault,
    output wire scan_stop_confirmed,
    output wire [3:0] stop_wait_reason,
    output wire motor_uart_tx,
    output wire [3:0] led,
    output wire [3:0] motion_command,
    output wire [7:0] requested_speed,
    output wire [7:0] applied_speed,
    output wire [1:0] avoidance_side_clear,
    output wire [5:0] avoidance_state,
    output wire avoidance_hold,
    output wire [3:0] avoidance_fault,
    output wire motor_control_ready,
    output wire [3:0] transmitted_motion,
    output wire motion_frame_committed,
    output reg [63:0] committed_wheel_targets
);
    // Isolate cross-module command/owner decoding from UART stop arbitration.
    // The complete tuple is sampled together. Raw remote STOP/mode changes
    // still use urgent_stop directly and never wait for this pipeline.
    wire follow_mode_available_r;
    wire follow_owner_r;
    wire [3:0] follow_motion_r;
    wire [7:0] follow_speed_r;
    wire signed [15:0] follow_linear_r;
    wire signed [15:0] follow_turn_r;
    wire follow_request_nonzero_r;
    wire nav_owner_r;
    wire [3:0] nav_motion_r;
    wire [7:0] nav_speed_r;
    wire signed [NAV_YAW_TRIM_BITS-1:0] nav_yaw_trim_r;
    generate if(CONTROL_REQUEST_PIPELINE) begin: request_pipeline
        reg follow_mode_available_q;
        assign follow_mode_available_r=follow_mode_available_q;
        reg follow_owner_q;
        assign follow_owner_r=follow_owner_q;
        reg [3:0] follow_motion_q;
        assign follow_motion_r=follow_motion_q;
        reg [7:0] follow_speed_q;
        assign follow_speed_r=follow_speed_q;
        reg signed [15:0] follow_linear_q;
        assign follow_linear_r=follow_linear_q;
        reg signed [15:0] follow_turn_q;
        assign follow_turn_r=follow_turn_q;
        reg follow_request_nonzero_q;
        assign follow_request_nonzero_r=follow_request_nonzero_q;
        reg nav_owner_q;
        assign nav_owner_r=nav_owner_q;
        reg [3:0] nav_motion_q;
        assign nav_motion_r=nav_motion_q;
        reg [7:0] nav_speed_q;
        reg signed [NAV_YAW_TRIM_BITS-1:0] nav_yaw_trim_q;
        assign nav_yaw_trim_r=nav_yaw_trim_q;
        assign nav_speed_r=nav_speed_q;
        always @(posedge clk50M or negedge reset_n) begin
            if(!reset_n) begin
                follow_mode_available_q<=0;
                follow_owner_q<=0;
                follow_motion_q<=0;
                follow_speed_q<=0;
                follow_linear_q<=0;
                follow_turn_q<=0;
                follow_request_nonzero_q<=0;
                nav_owner_q<=0;
                nav_motion_q<=0;
                nav_speed_q<=0;nav_yaw_trim_q<=0;
            end else begin
                follow_mode_available_q<=follow_mode_available;
                follow_owner_q<=follow_owner;
                follow_motion_q<=follow_motion;
                follow_speed_q<=follow_speed;
                follow_linear_q<=follow_linear;
                follow_turn_q<=follow_turn;
                follow_request_nonzero_q<=follow_request_nonzero;
                nav_owner_q<=nav_owner;
                nav_motion_q<=nav_motion;
                nav_speed_q<=nav_speed;nav_yaw_trim_q<=nav_yaw_trim;
            end
        end
    end else begin: request_passthrough
        assign follow_mode_available_r=follow_mode_available;
        assign follow_owner_r=follow_owner;
        assign follow_motion_r=follow_motion;
        assign follow_speed_r=follow_speed;
        assign follow_linear_r=follow_linear;
        assign follow_turn_r=follow_turn;
        assign follow_request_nonzero_r=follow_request_nonzero;
        assign nav_owner_r=nav_owner;
        assign nav_motion_r=nav_motion;
        assign nav_speed_r=nav_speed;
        assign nav_yaw_trim_r=nav_yaw_trim;
    end endgenerate

    localparam integer RAW_GAP_CYCLES=(CLK_FREQ_HZ/1000)*FRAME_GAP_MS;
    localparam integer GAP_CYCLES=(RAW_GAP_CYCLES<1)?1:RAW_GAP_CYCLES;
    // Modbus RTU V1.02: at high baud use at least 1.75ms between frames.
    // At low baud retain >=3.5 11-bit characters. Never add gaps inside a frame.
    localparam [63:0] RTU_SILENCE_CLKS=(64'd77*CLK_FREQ_HZ+2*BAUD_RATE-1)/(2*BAUD_RATE);
    localparam [63:0] CONFIG_SILENCE_CLKS=(64'd1*CLK_FREQ_HZ/1000)*UART_FRAME_SILENCE_MS;
    localparam [63:0] MIN_SILENCE_CLKS=CONFIG_SILENCE_CLKS>RTU_SILENCE_CLKS ? CONFIG_SILENCE_CLKS : RTU_SILENCE_CLKS;
    localparam integer SILENCE_BITS=$clog2(MIN_SILENCE_CLKS+1);
    localparam integer SCAN_REFRESH_CLKS=(CLK_FREQ_HZ/1000)*SCAN_MOTION_REFRESH_MS;
    localparam integer SCAN_REFRESH_BITS=SCAN_REFRESH_CLKS<1 ? 1 : $clog2(SCAN_REFRESH_CLKS+1);
    localparam integer STOP_RETRY_CLKS=(CLK_FREQ_HZ/1000)*STOP_RETRY_MS;
    localparam integer STOP_FEEDBACK_CLKS=(CLK_FREQ_HZ/1000)*500;
    localparam integer OFFICIAL_STARTUP_CLKS=(CLK_FREQ_HZ/1000)*OFFICIAL_STARTUP_MS<1 ? 1 :
        (CLK_FREQ_HZ/1000)*OFFICIAL_STARTUP_MS;
    localparam integer OFFICIAL_SETTLE_CLKS=(CLK_FREQ_HZ/1000)*OFFICIAL_SETTLE_MS<1 ? 1 :
        (CLK_FREQ_HZ/1000)*OFFICIAL_SETTLE_MS;
    localparam integer OFFICIAL_REFRESH_CLKS=(CLK_FREQ_HZ/1000)*OFFICIAL_REFRESH_MS<1 ? 1 :
        (CLK_FREQ_HZ/1000)*OFFICIAL_REFRESH_MS;
    localparam [1:0] OFFICIAL_BOOT_WAIT=0, OFFICIAL_SET_MODE=1,
                     OFFICIAL_SETTLE=2, OFFICIAL_READY=3;
    reg [1:0] official_phase;
    reg [31:0] official_wait_count,official_refresh_count;
    reg official_boot_stop_sent,official_previous_online,official_boot_mode_attempted;
    reg official_reinit_pending,official_reinit_authorized,official_reinit_started;
    reg official_recovery_zero_sent,official_previous_target_stop;
    reg official_stop_epoch_pending,official_frame_starts_stop_epoch;
    wire official_ready_rearm;
    wire official_reinit_authorization_event=MOTOR_OFFICIAL_PROTOCOL &&
        official_reinit_pending && !official_reinit_authorized &&
        motor_online && motor_feedback_update;
    wire official_link_lost=MOTOR_OFFICIAL_PROTOCOL && official_previous_online && !motor_online;
    wire official_settle_complete=MOTOR_OFFICIAL_PROTOCOL &&
        official_phase==OFFICIAL_SETTLE && official_wait_count>=OFFICIAL_SETTLE_CLKS-1;
    assign motor_control_ready=!MOTOR_OFFICIAL_PROTOCOL ||
        (official_phase==OFFICIAL_READY &&
         !official_reinit_pending && motor_online);
    localparam [3:0] MOT_NONE=0, MOT_FORWARD=1, MOT_BACKWARD=2,
                     MOT_LEFT_STRAFE=3, MOT_RIGHT_STRAFE=4,
                     MOT_ROTATE_LEFT=5, MOT_ROTATE_RIGHT=6,
                     MOT_LEFT_ARC=7, MOT_RIGHT_ARC=8,MOT_DIAGONAL_LEFT=9,MOT_DIAGONAL_RIGHT=10;
    localparam [3:0] FRAME_ENABLE=0, FRAME_STOP=15;
    localparam [2:0] SEQ_IDLE=0, SEQ_ENABLE=1, SEQ_GAP=2,
                     SEQ_MOTION=3, SEQ_STOP=4;
    localparam [7:0] IR_FORWARD=8'h40, IR_BACKWARD=8'h19,
                     IR_LEFT_STRAFE=8'h07, IR_RIGHT_STRAFE=8'h09,
                     IR_LEFT_ARC=8'h44, IR_RIGHT_ARC=8'h43,
                     IR_ROTATE_LEFT=8'h16, IR_ROTATE_RIGHT=8'h0D,
                     IR_STOP=8'h15, TASK_KEEPALIVE=8'hF0;

    wire avoidance_enabled;
    wire lidar_safety_active=ENABLE_LIDAR_SAFETY &&
        (LIDAR_GUARD_WHEN_DISABLED || avoidance_enabled);
    reg previous_avoidance_enabled;
    reg previous_mapping_enabled;
    wire mapping_enabled=(ENABLE_MAP_SCAN || ENABLE_NAVIGATION) ? mapping_enable && !avoidance_enabled : 1'b0;
    avoidance_switch #(.CLK_FREQ_HZ(CLK_FREQ_HZ),
                       .DEBOUNCE_MS(MODE_DEBOUNCE_MS)) u_avoidance_switch (
        .clk(clk50M),.reset_n(reset_n),.switch_async(avoidance_enable),
        .avoidance_enabled(avoidance_enabled)
    );
    wire mode_changed=(avoidance_enabled!=previous_avoidance_enabled) ||
                      (mapping_enabled!=previous_mapping_enabled);

    wire ir_done;
    wire [15:0] ir_data, ir_addr;
    wire physical_command_accept=ir_done && (ir_data[15:8]==~ir_data[7:0]);
    // Physical IR wins a simultaneous task packet, especially STOP.
    wire task_keepalive=(task_command_valid===1'b1) &&
        task_command==TASK_KEEPALIVE && !physical_command_accept;
    // A lease refresh only resets the one-second task timer. It must not look
    // like a new direction key to obstacle, follow or speed control.
    wire task_command_accept=(task_command_valid===1'b1) &&
        task_command!=TASK_KEEPALIVE && !physical_command_accept;
    wire [7:0] ir_cmd=physical_command_accept ? ir_data[7:0] : task_command;
    wire command_accept=physical_command_accept || task_command_accept;
    // Decode each byte before source arbitration. All six actions exclude
    // KEEPALIVE, so their accepted predicates need no general "not F0" chain.
    // Physical IR retains priority and no clock of command latency is added.
    function [5:0] command_action_bits;
        input [7:0] value;
        begin
            command_action_bits[0]=value==IR_STOP;
            command_action_bits[1]=value==IR_FORWARD||value==IR_BACKWARD||
                value==IR_LEFT_STRAFE||value==IR_RIGHT_STRAFE||value==IR_LEFT_ARC||
                value==IR_RIGHT_ARC||value==IR_ROTATE_LEFT||value==IR_ROTATE_RIGHT;
            command_action_bits[2]=value==8'h46;
            command_action_bits[3]=value==8'h4A;
            command_action_bits[4]=value==8'h42;
            command_action_bits[5]=value==8'h1C;
        end
    endfunction
    (* syn_keep=1 *) wire [5:0] physical_action_bits=command_action_bits(ir_data[7:0]);
    (* syn_keep=1 *) wire [5:0] task_action_bits=TASK_ACTIONS_PREDECODED ?
        task_command_actions : command_action_bits(task_command);
    wire [5:0] accepted_action_bits=physical_command_accept ? physical_action_bits :
        ({6{task_command_valid===1'b1}} & task_action_bits);
    wire remote_stop=accepted_action_bits[0];
    wire accepted_motion=accepted_action_bits[1];
    // A spoken manual direction is one bounded motion step. A lost Pi or
    // delayed follow-up utterance cannot leave the car driving indefinitely.
    localparam integer TASK_TICK_CLKS=CLK_FREQ_HZ/1000<1 ? 1 : CLK_FREQ_HZ/1000;
    localparam integer TASK_TICK_BITS=TASK_TICK_CLKS<2 ? 1 : $clog2(TASK_TICK_CLKS);
    reg [TASK_TICK_BITS-1:0] task_tick_age;
    reg [9:0] task_motion_age_ms;
    reg task_motion_active;
    wire task_motion_timeout=task_motion_active && task_motion_age_ms>=10'd999;
    reg [3:0] cmd_motion;
    wire scan_auto_owner,scan_auto_busy;
    wire [3:0] scan_auto_motion;
    wire [7:0] scan_auto_speed;
    wire scan_auto_paused=scan_auto_state==3'd7;
    reg follow_blocked,previous_follow_selected,follow_boundary_pending;
    wire follow_requested=ENABLE_FOLLOW && follow_owner_r && follow_mode_available_r;
    wire follow_selected=follow_requested && !follow_blocked;
    wire follow_enter=follow_selected && !previous_follow_selected;
    wire follow_exit=!follow_selected && previous_follow_selected;
    wire follow_entry_request=ENABLE_FOLLOW && follow_mode_available_r && accepted_action_bits[2];
    wire follow_cancel_event=remote_stop || accepted_motion || accepted_action_bits[3];
    wire follow_interrupt=follow_selected && follow_cancel_event;
    wire follow_wait_stop=ENABLE_FOLLOW && follow_boundary_pending;
    wire follow_motion_valid=follow_motion_r==MOT_FORWARD ||
        (FOLLOW_REVERSE_ENABLE && follow_motion_r==MOT_BACKWARD) ||
        (FOLLOW_CURVE_ENABLE && (follow_motion_r==MOT_LEFT_ARC || follow_motion_r==MOT_RIGHT_ARC)) ||
        (FOLLOW_TURN_ENABLE ?
         (follow_motion_r==MOT_ROTATE_LEFT || follow_motion_r==MOT_ROTATE_RIGHT) :
         (follow_motion_r==MOT_LEFT_STRAFE || follow_motion_r==MOT_RIGHT_STRAFE));
    localparam integer FOLLOW_TURN_LIMIT=FOLLOW_TURN_SPEED_MAX<FOLLOW_SPEED_MAX ?
        FOLLOW_TURN_SPEED_MAX : FOLLOW_SPEED_MAX;
    localparam integer FOLLOW_ARC_LIMIT=FOLLOW_ARC_SPEED_MAX<1 ? 1 :
        FOLLOW_ARC_SPEED_MAX<FOLLOW_SPEED_MAX ? FOLLOW_ARC_SPEED_MAX : FOLLOW_SPEED_MAX;
    wire [7:0] follow_speed_limit=(follow_motion_r==MOT_ROTATE_LEFT || follow_motion_r==MOT_ROTATE_RIGHT) ?
        FOLLOW_TURN_LIMIT : (follow_motion_r==MOT_LEFT_ARC || follow_motion_r==MOT_RIGHT_ARC) ?
        FOLLOW_ARC_LIMIT : FOLLOW_SPEED_MAX;
    wire nav_map_toggle=ENABLE_NAV_MAP_TOGGLE && ENABLE_NAVIGATION && mapping_enabled && nav_owner_r &&
        accepted_action_bits[4];
    wire park_request=mapping_enabled && ((accepted_action_bits[4] && !nav_map_toggle) || accepted_action_bits[3]);
    wire scan_auto_request=1'b0; // X3 Pro rotates itself; no car scan owner.
    wire scan_resume_request=scan_auto_request && scan_auto_paused && scan_resume_allowed;
    reg scan_start_pending,scan_resume_pending;
    wire scan_auto_start=scan_start_pending && mapping_enabled && !scan_auto_busy;
    wire scan_auto_resume=scan_resume_pending && mapping_enabled && !scan_auto_busy;
    assign scan_resume_event=scan_auto_resume;
    // Register public IR events to separate decoder data from navigation/map
    // control. Keep the command snapshot only for an accepted complete frame.
    // Mapping consumes these events for manual 42 capture and 4A rebase.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin ir_command_valid<=0;ir_command<=0;scan_start_pending<=0;scan_resume_pending<=0;end
        else begin
            ir_command_valid<=command_accept;
            if(command_accept) ir_command<=ir_cmd;
            scan_start_pending<=scan_auto_request && !scan_resume_request;
            scan_resume_pending<=scan_resume_request;
        end
    end
    reg nav_blocked,previous_nav_selected,nav_boundary_pending;
    wire nav_requested=ENABLE_NAVIGATION && mapping_enabled && nav_owner_r;
    wire nav_interrupt=ENABLE_NAVIGATION && nav_requested &&
        (remote_stop || accepted_motion || (accepted_action_bits[4] && !ENABLE_NAV_MAP_TOGGLE) || accepted_action_bits[3]);
    wire nav_selected=nav_requested && !nav_blocked && !scan_auto_busy && !scan_auto_paused &&
        !scan_auto_request && !scan_auto_start && !scan_auto_resume;
    wire nav_enter=nav_selected && !previous_nav_selected;
    wire nav_exit=!nav_selected && previous_nav_selected;
    wire nav_begin_event=ENABLE_NAVIGATION && mapping_enabled && !scan_auto_busy && accepted_action_bits[5];
    wire scan_auto_pause=remote_stop && (scan_auto_busy || scan_auto_paused);
    wire ownership_stop_event=park_request || nav_enter || nav_exit || nav_interrupt || nav_begin_event || scan_auto_pause || scan_auto_resume ||
        follow_entry_request || follow_enter || follow_exit || follow_interrupt;
    wire nav_wait_stop=ENABLE_NAVIGATION && mapping_enabled && nav_boundary_pending && !scan_auto_busy;
    wire nav_motion_valid=nav_motion_r==MOT_FORWARD || nav_motion_r==MOT_ROTATE_LEFT || nav_motion_r==MOT_ROTATE_RIGHT ||
        (ENABLE_NAV_HOLONOMIC && (nav_motion_r==MOT_BACKWARD || nav_motion_r==MOT_LEFT_STRAFE || nav_motion_r==MOT_RIGHT_STRAFE)) ||
        (ENABLE_NAV_ARC && (nav_motion_r==MOT_LEFT_ARC || nav_motion_r==MOT_RIGHT_ARC));
    wire scan_auto_cancel=accepted_motion || nav_begin_event || nav_selected ||
        (remote_stop && !scan_auto_busy && !scan_auto_paused);
    wire urgent_stop=remote_stop || task_motion_timeout || mode_changed || scan_auto_request || scan_auto_start || scan_auto_resume || ownership_stop_event;

    // Entering or leaving navigation owns a complete STOP handshake. A manual
    // direction received during navigation is retained below, but cannot run
    // until that STOP has been completely transmitted. In official mode the
    // handoff does not wait for measured wheel speeds to become exactly zero.
    // A STOP or manual key also blocks a still-asserted external owner: delayed
    // controller cancellation must never resurrect its previous motion.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin nav_blocked<=0;previous_nav_selected<=0;nav_boundary_pending<=0;end
        else begin
            previous_nav_selected<=nav_selected;
            // A fresh explicit 1C may restart after 15 while the controller's
            // STOPPED owner remains asserted. The ownership STOP handshake
            // below still waits for complete STOP transmission.
            if(!ENABLE_NAVIGATION || !nav_owner_r ||
               (nav_begin_event && (!MOTOR_OFFICIAL_PROTOCOL ||
                official_ready_rearm))) nav_blocked<=0;
            else if(MOTOR_OFFICIAL_PROTOCOL && !motor_control_ready) nav_blocked<=1;
            else if(nav_interrupt || scan_auto_request || scan_auto_start || scan_auto_resume || scan_auto_busy || mode_changed) nav_blocked<=1;
            if(!ENABLE_NAVIGATION) nav_boundary_pending<=0;
            else if(ownership_stop_event) nav_boundary_pending<=1;
            else if(motor_stop_confirmed) nav_boundary_pending<=0;
        end
    end
    // A delayed follow owner cannot restart motion after manual/STOP takeover.
    // Entry and exit each require a complete STOP transmission.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin follow_blocked<=0;previous_follow_selected<=0;follow_boundary_pending<=0;end
        else begin
            previous_follow_selected<=follow_selected;
            if(!ENABLE_FOLLOW || !follow_owner_r) follow_blocked<=0;
            // VLM approach confirms the selected target with A1. It needs the
            // same explicit rearm as ordinary 1C follow after the STOP handoff.
            else if(MOTOR_OFFICIAL_PROTOCOL && command_accept &&
                    (ir_cmd==8'h1C || ir_cmd==8'hA1) &&
                    official_ready_rearm) follow_blocked<=0;
            else if(MOTOR_OFFICIAL_PROTOCOL && !motor_control_ready) follow_blocked<=1;
            else if(follow_cancel_event || mode_changed || !follow_mode_available_r) follow_blocked<=1;
            if(!ENABLE_FOLLOW) follow_boundary_pending<=0;
            else if(follow_entry_request || follow_enter || follow_exit || follow_interrupt) follow_boundary_pending<=1;
            else if(motor_stop_confirmed) follow_boundary_pending<=0;
        end
    end
    ir_decode u_ir_decode (
        .clk(clk50M), .reset_n(reset_n), .iIR(iIR),
        .dec_done(ir_done), .ir_data(ir_data), .ir_addr(ir_addr)
    );
    ir_speed_control #(
        .IR_SPEED_UP(IR_SPEED_UP), .IR_SPEED_DOWN(IR_SPEED_DOWN),
        .SPEED_INITIAL(SPEED_INITIAL), .SPEED_MIN(SPEED_MIN),
        .SPEED_MAX(SPEED_MAX), .SPEED_STEP(SPEED_STEP)
    ) u_ir_speed_control (
        .clk(clk50M), .reset_n(reset_n), .dec_done(command_accept),
        .command(ir_cmd), .requested_speed(requested_speed)
    );

    always @(*) begin
        case(ir_cmd)
            IR_FORWARD:      cmd_motion=MOT_FORWARD;
            IR_BACKWARD:     cmd_motion=MOT_BACKWARD;
            IR_LEFT_STRAFE:  cmd_motion=MOT_LEFT_STRAFE;
            IR_RIGHT_STRAFE: cmd_motion=MOT_RIGHT_STRAFE;
            IR_LEFT_ARC:     cmd_motion=MOT_LEFT_ARC;
            IR_RIGHT_ARC:    cmd_motion=MOT_RIGHT_ARC;
            IR_ROTATE_LEFT:  cmd_motion=MOT_ROTATE_LEFT;
            IR_ROTATE_RIGHT: cmd_motion=MOT_ROTATE_RIGHT;
            default:         cmd_motion=MOT_NONE;
        endcase
    end

    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            task_motion_active<=0;task_tick_age<=0;task_motion_age_ms<=0;
        end else if(physical_command_accept || mode_changed || remote_stop ||
                    task_motion_timeout || follow_enter || nav_enter || park_request) begin
            task_motion_active<=0;task_tick_age<=0;task_motion_age_ms<=0;
        end else if(task_keepalive && task_motion_active) begin
            task_tick_age<=0;task_motion_age_ms<=0;
        end else if(task_command_accept && cmd_motion!=MOT_NONE) begin
            task_motion_active<=1;task_tick_age<=0;task_motion_age_ms<=0;
        end else if(task_motion_active) begin
            if(task_tick_age==TASK_TICK_CLKS-1) begin
                task_tick_age<=0;
                task_motion_age_ms<=task_motion_age_ms+1'b1;
            end else task_tick_age<=task_tick_age+1'b1;
        end
    end
    // Direction and speed stay independent: changing speed never starts a car.
    reg [3:0] requested_motion;
    reg command_update;
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            requested_motion<=MOT_NONE;
            command_update<=0;
            previous_avoidance_enabled<=0;
            previous_mapping_enabled<=0;
        end else begin
            command_update<=0;
            previous_avoidance_enabled<=avoidance_enabled;
            previous_mapping_enabled<=mapping_enabled;
            if((lidar_safety_active && !lidar_healthy) || official_link_lost || (MOTOR_OFFICIAL_PROTOCOL && command_accept &&
               cmd_motion!=MOT_NONE && !motor_online)) begin
                requested_motion<=MOT_NONE;
                command_update<=1;
            end else if(mode_changed || park_request || scan_auto_request || scan_auto_start || scan_auto_resume || nav_begin_event || follow_entry_request || follow_enter) begin
                // Stop at either mode transition; a fresh direction key restarts.
                requested_motion<=MOT_NONE;
                command_update<=1;
            end else if(task_motion_timeout) begin
                requested_motion<=MOT_NONE;
                command_update<=1;
            end else if(command_accept && (remote_stop || cmd_motion!=MOT_NONE)) begin
                requested_motion<=remote_stop ? MOT_NONE : cmd_motion;
                command_update<=1;
            end else if(nav_enter || (follow_exit && !follow_blocked)) begin
                // A cancellation already stored its manual request above. Preserve
                // that first direction key while the follow STOP boundary completes.
                requested_motion<=MOT_NONE;
                command_update<=1;
            end
        end
    end

    wire [3:0] policy_motion;
    wire [7:0] policy_speed;
    reg [3:0] candidate_motion;
    reg [7:0] candidate_speed;
    wire [3:0] effective_motion;
    wire [7:0] effective_speed;
    wire travel_valid, travel_update, travel_fault;
    wire [17:0] travel_step;
    wire [3:0] actual_motion;
    wire follow_vector_mode=FOLLOW_VECTOR_ENABLE && MOTOR_OFFICIAL_PROTOCOL;
    wire vector_authorized=follow_vector_mode && follow_selected && motor_online &&
        motor_control_ready && !urgent_stop && !nav_wait_stop && !follow_wait_stop;
    // The board supplies the equivalent one-bit request predicate directly
    // from the follow policy, before the component clamp arithmetic. Older
    // callers retain the original word comparison through the default option.
    wire vector_input_stop=FOLLOW_VECTOR_STATUS_ENABLE ? !follow_request_nonzero_r :
        (follow_linear_r==16'sd0 && follow_turn_r==16'sd0);
    wire nav_vector_authorized=(ENABLE_NAV_HOLONOMIC||ENABLE_NAV_FORWARD_TRIM) && MOTOR_OFFICIAL_PROTOCOL && nav_selected &&
        !follow_selected && motor_online && motor_control_ready && !urgent_stop && !nav_wait_stop && !follow_wait_stop;
    wire nav_vector_stop=!nav_motion_valid || nav_motion_r<1 || nav_motion_r>4 || nav_speed_r==0;
    wire signed [15:0] nav_base=(nav_speed_r>SPEED_MAX?SPEED_MAX:nav_speed_r)*MOTOR_SPEED_SCALE;
    // Trim is in motor units, never a speed grade, and cannot reverse a wheel.
    wire signed [15:0] nav_trim=nav_yaw_trim_r>NAV_YAW_TRIM_MAX?NAV_YAW_TRIM_MAX:
        nav_yaw_trim_r< -NAV_YAW_TRIM_MAX?-NAV_YAW_TRIM_MAX:$signed(nav_yaw_trim_r);
    reg candidate_nav_vector;
    // Grade caps for the vector (follow) mixer. Must track the follow limits so
    // the clamp here and the request cap inside vision_follow_control agree.
    localparam [7:0] VECTOR_LINEAR_GRADE_MAX=FOLLOW_SPEED_MAX;
    localparam [7:0] VECTOR_TURN_GRADE_MAX=FOLLOW_TURN_SPEED_MAX;
    wire [7:0] vector_linear_grade=requested_speed>VECTOR_LINEAR_GRADE_MAX ?
        VECTOR_LINEAR_GRADE_MAX : requested_speed;
    wire [7:0] vector_turn_grade=requested_speed>VECTOR_TURN_GRADE_MAX ?
        VECTOR_TURN_GRADE_MAX : requested_speed;
    wire signed [16:0] vector_linear_limit=vector_linear_grade*MOTOR_SPEED_SCALE;
    wire signed [16:0] vector_turn_limit=vector_turn_grade*MOTOR_SPEED_SCALE;
    function [16:0] wheel_limit_at_grade;
        input [7:0] grade;
        reg [31:0] numerator;
        begin
            numerator=grade*MOTOR_SPEED_SCALE*7+2;
            wheel_limit_at_grade=numerator/5;
        end
    endfunction
    function [16:0] wheel_limit_lookup;
        input [7:0] grade;
        begin
            if(VECTOR_LINEAR_GRADE_MAX<=10) begin
                case(grade)
                    0:wheel_limit_lookup=wheel_limit_at_grade(0);
                    1:wheel_limit_lookup=wheel_limit_at_grade(1);
                    2:wheel_limit_lookup=wheel_limit_at_grade(2);
                    3:wheel_limit_lookup=wheel_limit_at_grade(3);
                    4:wheel_limit_lookup=wheel_limit_at_grade(4);
                    5:wheel_limit_lookup=wheel_limit_at_grade(5);
                    6:wheel_limit_lookup=wheel_limit_at_grade(6);
                    7:wheel_limit_lookup=wheel_limit_at_grade(7);
                    8:wheel_limit_lookup=wheel_limit_at_grade(8);
                    9:wheel_limit_lookup=wheel_limit_at_grade(9);
                    10:wheel_limit_lookup=wheel_limit_at_grade(10);
                    default:wheel_limit_lookup=wheel_limit_at_grade(VECTOR_LINEAR_GRADE_MAX);
                endcase
            end else wheel_limit_lookup=wheel_limit_at_grade(grade);
        end
    endfunction
    wire [16:0] vector_wheel_limit=wheel_limit_lookup(vector_linear_grade);
    reg signed [15:0] vector_linear_clamped,vector_turn_clamped;
    reg [16:0] vector_clamped_wheel_limit;
    reg vector_components_valid,candidate_vector;
    reg signed [15:0] candidate_wheel0,candidate_wheel1,candidate_wheel2,candidate_wheel3;
    reg signed [16:0] vector_mix_linear,vector_mix_turn;
    reg [16:0] vector_mix_turn_abs,vector_mix_linear_abs,vector_mix_linear_room;
    reg [3:0] vector_mix_motion;
    localparam integer VECTOR_COMPACT_ENABLE=MOTOR_SPEED_SCALE>=1 && MOTOR_SPEED_SCALE<=8191;
    // Widest wheel magnitude is the 140% arc limit at the linear grade cap.
    localparam integer VECTOR_COMPACT_BITS=VECTOR_COMPACT_ENABLE ?
        $clog2((VECTOR_LINEAR_GRADE_MAX*MOTOR_SPEED_SCALE*7+2)/5+1) : 1;
    reg [VECTOR_COMPACT_BITS-1:0] vector_small_linear_abs,vector_small_turn_abs;
    reg [VECTOR_COMPACT_BITS-1:0] vector_small_turn,vector_small_room,vector_small_linear;
    reg signed [VECTOR_COMPACT_BITS:0] vector_small_linear_signed,vector_small_turn_signed;
    // Bound each independent component before the wheel mixer. The next stage
    // preserves turn authority and reduces translation if a wheel would exceed
    // the existing 140% arc limit. Neither component is a direction enum.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            vector_linear_clamped<=0;vector_turn_clamped<=0;
            vector_clamped_wheel_limit<=0;vector_components_valid<=0;
        end else if(!vector_authorized || vector_input_stop) begin
            vector_linear_clamped<=0;vector_turn_clamped<=0;
            vector_clamped_wheel_limit<=0;vector_components_valid<=0;
        end else begin
            vector_linear_clamped<=$signed(follow_linear_r)>vector_linear_limit ? vector_linear_limit :
                $signed(follow_linear_r)<-vector_linear_limit ? -vector_linear_limit : follow_linear_r;
            vector_turn_clamped<=$signed(follow_turn_r)>vector_turn_limit ? vector_turn_limit :
                $signed(follow_turn_r)<-vector_turn_limit ? -vector_turn_limit : follow_turn_r;
            vector_clamped_wheel_limit<=vector_wheel_limit;
            vector_components_valid<=1;
        end
    end
    always @(*) begin
        vector_small_linear_abs=0;vector_small_turn_abs=0;
        vector_small_turn=0;vector_small_room=0;vector_small_linear=0;
        vector_small_linear_signed=0;vector_small_turn_signed=0;
        if(VECTOR_COMPACT_ENABLE) begin
            // Component registers are bounded by min(IR grade,FOLLOW limits)*scale.
            // Work at the minimum unsigned magnitude width, then sign extend
            // once, avoiding serial 17-bit signed absolute/clamp comparisons.
            vector_small_linear_abs=vector_linear_clamped[15] ?
                -vector_linear_clamped[VECTOR_COMPACT_BITS-1:0] :
                 vector_linear_clamped[VECTOR_COMPACT_BITS-1:0];
            vector_small_turn_abs=vector_turn_clamped[15] ?
                -vector_turn_clamped[VECTOR_COMPACT_BITS-1:0] :
                 vector_turn_clamped[VECTOR_COMPACT_BITS-1:0];
            vector_small_turn=vector_small_turn_abs>vector_clamped_wheel_limit[VECTOR_COMPACT_BITS-1:0] ?
                vector_clamped_wheel_limit[VECTOR_COMPACT_BITS-1:0] : vector_small_turn_abs;
            vector_small_room=vector_clamped_wheel_limit[VECTOR_COMPACT_BITS-1:0]-vector_small_turn;
            vector_small_linear=vector_small_linear_abs>vector_small_room ?
                vector_small_room : vector_small_linear_abs;
            vector_small_linear_signed=vector_linear_clamped[15] ?
                -$signed({1'b0,vector_small_linear}) : $signed({1'b0,vector_small_linear});
            vector_small_turn_signed=vector_turn_clamped[15] ?
                -$signed({1'b0,vector_small_turn}) : $signed({1'b0,vector_small_turn});
            vector_mix_linear=vector_small_linear_signed;
            vector_mix_turn=vector_small_turn_signed;
            vector_mix_turn_abs=vector_small_turn;
            vector_mix_linear_abs=vector_small_linear_abs;
            vector_mix_linear_room=vector_small_room;
            if(vector_small_linear!=0)
                vector_mix_motion=vector_linear_clamped[15] ? MOT_BACKWARD :
                    vector_small_turn==0 ? MOT_FORWARD :
                    vector_turn_clamped[15] ? MOT_LEFT_ARC : MOT_RIGHT_ARC;
            else vector_mix_motion=vector_small_turn==0 ? MOT_NONE :
                vector_turn_clamped[15] ? MOT_ROTATE_LEFT : MOT_ROTATE_RIGHT;
        end else begin
        vector_mix_linear={vector_linear_clamped[15],vector_linear_clamped};
        vector_mix_turn={vector_turn_clamped[15],vector_turn_clamped};
        vector_mix_turn_abs=vector_mix_turn<0 ? -vector_mix_turn : vector_mix_turn;
        if(vector_mix_turn_abs>vector_clamped_wheel_limit) begin
            vector_mix_turn=vector_mix_turn<0 ? -$signed(vector_clamped_wheel_limit) :
                $signed(vector_clamped_wheel_limit);
            vector_mix_turn_abs=vector_clamped_wheel_limit;
        end
        vector_mix_linear_room=vector_clamped_wheel_limit-vector_mix_turn_abs;
        vector_mix_linear_abs=vector_mix_linear<0 ? -vector_mix_linear : vector_mix_linear;
        if(vector_mix_linear_abs>vector_mix_linear_room)
            vector_mix_linear=vector_mix_linear<0 ? -$signed(vector_mix_linear_room) :
                $signed(vector_mix_linear_room);
        if(vector_mix_linear>0)
            vector_mix_motion=vector_mix_turn<0 ? MOT_LEFT_ARC :
                vector_mix_turn>0 ? MOT_RIGHT_ARC : MOT_FORWARD;
        else if(vector_mix_linear<0) vector_mix_motion=MOT_BACKWARD;
        else vector_mix_motion=vector_mix_turn<0 ? MOT_ROTATE_LEFT :
            vector_mix_turn>0 ? MOT_ROTATE_RIGHT : MOT_NONE;
        end
    end
    wire effective_vector,effective_nav_vector;
    wire signed [15:0] effective_wheel0,effective_wheel1,effective_wheel2,effective_wheel3;
    // Removed automatic chassis rotation: scanning is entirely in X3 Pro.
    assign scan_auto_owner=0;assign scan_auto_busy=0;
    assign scan_auto_motion=0;assign scan_auto_speed=0;
    assign scan_sample_allow=1;assign scan_continuous_capture=0;assign scan_auto_state=0;
    assign scan_auto_progress=0;assign scan_auto_fault=0;
    encoder_travel #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_encoder_travel (
        .clk(clk50M),.reset_n(reset_n),.counts(motor_counts),
        .counts_update(motor_feedback_update),.valid(travel_valid),
        .step_valid(travel_update),.step_sum(travel_step),.fault(travel_fault)
    );
    wire [3:0] policy_fault;
    reg avoidance_manual_override;
    wire gap_auto_active;
    wire [3:0] gap_index;
    wire [3:0] legacy_policy_motion,legacy_policy_fault;
    wire [7:0] legacy_policy_speed;
    wire route_position_valid,route_position_fault;
    wire signed [15:0] route_x_mm,route_y_mm,held_heading_error,held_heading_reference;
    wire [7:0] flow_requested_speed=ENABLE_RETURN_TO_LINE && avoidance_enabled && requested_motion==MOT_FORWARD &&
        requested_speed>AVOID_SPEED_MAX?AVOID_SPEED_MAX:requested_speed;
    wire [5:0] legacy_avoidance_state;
    wire gap_authorized=ENABLE_GAP_AVOIDANCE && avoidance_enabled && !avoidance_manual_override &&
        requested_motion==MOT_FORWARD && motor_online && motor_control_ready && !urgent_stop;
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) avoidance_manual_override<=0;
        else if(CANCEL_REMOTE_LOCK || !avoidance_enabled) avoidance_manual_override<=0;
        else if(ENABLE_LIDAR_SAFETY && (avoidance_state!=0 ||
                (LIDAR_DYNAMIC_BYPASS && requested_motion==MOT_FORWARD)) && command_accept &&
                (remote_stop || cmd_motion!=MOT_NONE)) avoidance_manual_override<=1;
    end
    generate if(ENABLE_GAP_AVOIDANCE)begin: local_gap_policy
        lidar_gap_planner #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.MOTOR_SPEED_SCALE(MOTOR_SPEED_SCALE)) planner(
            .clk(clk50M),.reset_n(reset_n),.enable(avoidance_enabled && !avoidance_manual_override),
            .healthy(lidar_healthy),.requested_motion(requested_motion),.requested_speed(requested_speed),
            .clearances_mm(gap_clearance_mm),.direction_valid(gap_direction_valid),
            .motion(policy_motion),.speed(policy_speed),.auto_active(gap_auto_active),
            .selected_index(gap_index),.state(avoidance_state),.fault(policy_fault));
    end else if(ENABLE_FLOW_BYPASS)begin: flow_obstacle_policy
        assign gap_auto_active=0;assign gap_index=4;
        lidar_flow_control #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.STOP_CM(LIDAR_STOP_CM),
            .ENABLE_RETURN_TO_LINE(ENABLE_RETURN_TO_LINE),.SLOW_SPEED(AVOID_SPEED_MAX),
            .RETURN_SPEED(AVOID_RETURN_SPEED),.FINE_RETURN_SPEED(AVOID_FINE_RETURN_SPEED),
            .RETURN_ADVANCE_REDUCTION_MM(AVOID_RETURN_REDUCTION_MM),
            .TRIGGER_CM(AVOID_TRIGGER_CM),.SLOW_CM(AVOID_SLOW_CM),.CLEAR_CM(AVOID_CLEAR_CM)) controller(
            .clk(clk50M),.reset_n(reset_n),.enable(avoidance_enabled),.command_update(command_update),
            .requested_motion(requested_motion),.requested_speed(flow_requested_speed),.healthy(lidar_healthy),
            .front_valid(front_valid),.rear_valid(rear_valid),.left_valid(left_valid),.right_valid(right_valid),
            .front_cm(front_distance_cm),.rear_cm(rear_distance_cm),.left_cm(left_distance_cm),.right_cm(right_distance_cm),
            .front_update(front_update),.left_update(left_update),.right_update(right_update),
            .route_valid(route_position_valid),.route_fault(route_position_fault),
            .route_x_mm(route_x_mm),.route_y_mm(route_y_mm),
            .motion(policy_motion),.speed(policy_speed),.state(avoidance_state),.fault(policy_fault));
    end else begin: legacy_obstacle_policy
        assign gap_auto_active=0;assign gap_index=4;
        assign policy_motion=legacy_policy_motion;assign policy_speed=legacy_policy_speed;
        assign avoidance_state=legacy_avoidance_state;assign policy_fault=legacy_policy_fault;
    end endgenerate
    // Retain the historical instance name for legacy test/config compatibility.
    // Its disabled, unconsumed production logic is removed by synthesis.
    car_obstacle_control #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.LIDAR_POLICY(ENABLE_LIDAR_SAFETY),
        .LIDAR_DYNAMIC_FINISH(LIDAR_DYNAMIC_BYPASS),.SIDE_START_CM(30),.SETTLE_MS(AVOIDANCE_SETTLE_MS),
        .STOP_CM(ENABLE_LIDAR_SAFETY?15:12),.AVOID_CM(ENABLE_LIDAR_SAFETY?25:20),
        .SLOW_CM(ENABLE_LIDAR_SAFETY?45:25),.CLEAR_CM(ENABLE_LIDAR_SAFETY?50:30),
        .SCAN_ANGLE(SCAN_ANGLE), .ENABLE_AUTO_SCAN(ENABLE_AUTO_SCAN),
        .ENABLE_REVERSE(ENABLE_REVERSE), .REVERSE_MS(REVERSE_MS),
        .ENABLE_RETURN_TO_LINE(ENABLE_RETURN_TO_LINE),
        .ENABLE_YAW_DEVIATION_STOP(ENABLE_YAW_DEVIATION_STOP),
        .ENABLE_ENCODER_STOP(ENABLE_ENCODER_STOP),
        .OFFSET_TARGET_COUNTS(OFFSET_TARGET_COUNTS)) u_obstacle_control (
        .clk(clk50M), .reset_n(reset_n),
        .avoidance_enable(!ENABLE_FLOW_BYPASS && !ENABLE_GAP_AVOIDANCE && avoidance_enabled && !avoidance_manual_override),
        .requested_motion(requested_motion), .requested_speed(requested_speed),
        .command_update(command_update),
        .front_distance_cm(front_distance_cm), .rear_distance_cm(rear_distance_cm),
        .front_valid(front_valid), .rear_valid(rear_valid),
        .front_update(front_update), .rear_update(rear_update),
        .left_distance_cm(left_distance_cm), .right_distance_cm(right_distance_cm),
        .left_valid(left_valid), .right_valid(right_valid),
        .left_update(left_update), .right_update(right_update),
        .imu_yaw(imu_yaw),.imu_yaw_valid(imu_yaw_valid),.imu_yaw_update(imu_yaw_update),
        .travel_valid(travel_valid),.travel_update(travel_update),
        .travel_step(travel_step),.travel_fault(travel_fault),.actual_motion(actual_motion),
        .maneuver_stop_confirmed(motor_stop_confirmed && motor_online && motor_speeds==64'd0),
        .effective_motion(legacy_policy_motion), .effective_speed(legacy_policy_speed),
        .avoidance_state(legacy_avoidance_state), .avoidance_fault(legacy_policy_fault)
    );
    reg candidate_gap_active;
    reg [3:0] candidate_gap_index;
    wire effective_gap_active;
    wire [15:0] gap_budget=policy_speed*MOTOR_SPEED_SCALE;
    reg signed [15:0] gap_forward,gap_left;
    always @*begin
        gap_forward=0;gap_left=0;
        case(gap_index)
            0:gap_left=-$signed(gap_budget);
            1:begin gap_forward=gap_budget/3;gap_left=-$signed(gap_budget/3)*2;end
            2:begin gap_forward=gap_budget/2;gap_left=-$signed(gap_budget/2);end
            3:begin gap_forward=(gap_budget/3)*2;gap_left=-$signed(gap_budget/3);end
            4:gap_forward=gap_budget;
            5:begin gap_forward=(gap_budget/3)*2;gap_left=gap_budget/3;end
            6:begin gap_forward=gap_budget/2;gap_left=gap_budget/2;end
            7:begin gap_forward=gap_budget/3;gap_left=(gap_budget/3)*2;end
            8:gap_left=gap_budget;
            default:begin end
        endcase
    end
    reg [3:0] policy_motion_sample;
    reg [7:0] policy_speed_sample;
    reg candidate_heading_active;
    reg signed [15:0] candidate_heading_turn;
    wire effective_heading_active;
    wire heading_authorized=ENABLE_HEADING_HOLD && MOTOR_OFFICIAL_PROTOCOL && avoidance_enabled &&
        !follow_selected && !nav_selected && !scan_auto_owner &&
        !urgent_stop && !nav_wait_stop && !follow_wait_stop && motor_online && motor_control_ready &&
        requested_motion>=MOT_FORWARD && requested_motion<=MOT_RIGHT_STRAFE && requested_speed!=0;
    // Capture the policy result before heading blending. The route crossing
    // comparators must not feed wheel mixing/slew in the same 20ns cycle.
    // Explicit STOP/readiness vetoes retain their direct final priority.
    wire [3:0] heading_input_motion=policy_motion_sample;
    wire [7:0] heading_input_speed=policy_speed_sample;
    reg heading_return_sample,heading_turn_allowed_sample;
    function [15:0] lidar_stop_at_grade;
        input [7:0] grade;integer k;
        begin
            if(SPEED_MAX<=10)begin
                lidar_stop_at_grade=16'hffff;
                for(k=0;k<=10;k=k+1)if(k<=SPEED_MAX&&grade==k)
                    lidar_stop_at_grade=LIDAR_STOP_CM+(k>3?(k-3)*LIDAR_EXTRA_STOP_CM:0);
            end else lidar_stop_at_grade=LIDAR_STOP_CM+(grade>3?(grade-3)*LIDAR_EXTRA_STOP_CM:0);
        end
    endfunction
    wire [15:0] heading_stop_limit=lidar_stop_at_grade(heading_input_speed);
    // Display passability uses the same side-start and speed-dependent guard
    // thresholds as motion. Front/rear tightening never changes side margins.
    wire [15:0] display_side_stop=lidar_stop_at_grade(flow_requested_speed);
    assign avoidance_side_clear={lidar_healthy && right_valid && right_distance_cm>=18 && right_distance_cm>display_side_stop,
                                 lidar_healthy && left_valid && left_distance_cm>=18 && left_distance_cm>display_side_stop};
    wire heading_turn_allowed=!lidar_safety_active ||
        (lidar_rotation_valid && lidar_rotation_cm>heading_stop_limit);
    always @(posedge clk50M or negedge reset_n)begin
        if(!reset_n)begin heading_return_sample<=0;heading_turn_allowed_sample<=0;end
        else begin
            heading_return_sample<=avoidance_state==6'd39;
            heading_turn_allowed_sample<=heading_turn_allowed;
        end
    end
    wire heading_active,heading_turning,heading_ready;
    wire [3:0] heading_motion;
    wire [7:0] heading_speed;
    wire signed [15:0] heading_wheel0,heading_wheel1,heading_wheel2,heading_wheel3,heading_turn;
    wire heading_required=heading_authorized && avoidance_enabled;
    generate if(ENABLE_HEADING_HOLD)begin: heading_hold
        imu_heading_hold #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.MOTOR_SPEED_SCALE(MOTOR_SPEED_SCALE),
            .RIGHT_YAW_POSITIVE(HEADING_RIGHT_YAW_POSITIVE),
            .DEADBAND_TENTHS(ENABLE_RETURN_TO_LINE?2:5),.MEDIUM_TENTHS(20),.MAX_TURN_UNITS(2),
            .FRACTIONAL_CONTROL(ENABLE_RETURN_TO_LINE),.SLEW_PER_FRAME(AVOID_SLEW_UNITS),
            .RETURN_LINE_BLEND(AVOID_RETURN_BLEND)) controller(
            .clk(clk50M),.reset_n(reset_n),.enable(heading_authorized),.turn_allowed(heading_turn_allowed_sample),
            .motion(heading_input_motion),.speed(heading_input_speed),
            .yaw(imu_yaw),.yaw_valid(imu_yaw_valid),.yaw_update(imu_yaw_update),
            .frame_committed(motion_frame_committed),
            .returning(heading_return_sample),.lateral_offset_mm(route_y_mm),
            .active(heading_active),.turning(heading_turning),.out_motion(heading_motion),.out_speed(heading_speed),
            .wheel0(heading_wheel0),.wheel1(heading_wheel1),.wheel2(heading_wheel2),.wheel3(heading_wheel3),
            .out_turn(heading_turn),.ready(heading_ready),.error_tenths(held_heading_error),.reference_yaw(held_heading_reference));
    end else begin: no_heading_hold
        assign heading_active=0;assign heading_turning=0;assign heading_ready=0;
        assign heading_motion=0;assign heading_speed=0;assign heading_turn=0;
        assign heading_wheel0=0;assign heading_wheel1=0;assign heading_wheel2=0;assign heading_wheel3=0;
        assign held_heading_error=0;assign held_heading_reference=0;
    end endgenerate
    generate if(ENABLE_FLOW_BYPASS && ENABLE_RETURN_TO_LINE)begin: route_odometry
        wire requested=avoidance_enabled && requested_motion==MOT_FORWARD && requested_speed!=0 &&
            !follow_selected && !nav_selected && !scan_auto_owner && !urgent_stop;
        reg started;
        always @(posedge clk50M or negedge reset_n)
            if(!reset_n)started<=0;
            else if(!requested)started<=0;
            else if(heading_ready)started<=1;
        avoidance_route_odometry #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.FORWARD_Q12(853),.STRAFE_Q12(755)) position(
            .clk(clk50M),.reset_n(reset_n),.enable(requested && (started||heading_ready)),
            .motor_online(motor_online),.heading_valid(heading_ready),.counts(motor_counts),
            .counts_update(motor_feedback_update),.relative_heading(-held_heading_error),
            .valid(route_position_valid),.fault(route_position_fault),.x_mm(route_x_mm),.y_mm(route_y_mm));
    end else begin: no_route_odometry
        assign route_position_valid=1'b1;assign route_position_fault=1'b0;
        assign route_x_mm=0;assign route_y_mm=0;
    end endgenerate
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin policy_motion_sample<=MOT_NONE;policy_speed_sample<=0;end
        else if(urgent_stop || nav_wait_stop || follow_wait_stop ||
                (MOTOR_OFFICIAL_PROTOCOL && !motor_control_ready)) begin
            // Clear the pair at the same priority boundary as applied motion.
            // Releasing STOP must not replay the older sampled command.
            policy_motion_sample<=MOT_NONE;policy_speed_sample<=0;
        end else begin policy_motion_sample<=policy_motion;policy_speed_sample<=policy_speed;end
    end
    // Separate the yaw/crossing/motion mux from UART wheel-speed and CRC
    // arithmetic. One 50MHz clock (20ns); explicit STOP and mode changes also
    // keep their direct priority below, independent of this pipeline.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            candidate_motion<=MOT_NONE;candidate_speed<=0;candidate_vector<=0;
            candidate_gap_active<=0;candidate_gap_index<=4;
            candidate_heading_active<=0;candidate_heading_turn<=0;
            candidate_wheel0<=0;candidate_wheel1<=0;candidate_wheel2<=0;candidate_wheel3<=0;
            candidate_nav_vector<=0;
        end else begin
        candidate_vector<=0;candidate_gap_active<=0;candidate_heading_active<=0;candidate_heading_turn<=0;
        candidate_nav_vector<=0;
        if(urgent_stop || nav_wait_stop || follow_wait_stop ||
                (MOTOR_OFFICIAL_PROTOCOL && !motor_control_ready)) begin
            candidate_motion<=MOT_NONE; candidate_speed<=0;
        end else if(follow_selected) begin
            // Follow has its own per-direction obstacle/target watchdog. Never
            // enter the old reverse/side-bypass route or reissue IR unlock events.
            if(follow_vector_mode) begin
                candidate_vector<=1;
                candidate_motion<=vector_authorized && !vector_input_stop && vector_components_valid ?
                    vector_mix_motion : MOT_NONE;
                candidate_speed<=vector_authorized && !vector_input_stop && vector_components_valid &&
                    vector_mix_motion!=MOT_NONE ? vector_linear_grade : 8'd0;
                candidate_wheel0<=-vector_mix_linear-vector_mix_turn;
                candidate_wheel1<=vector_mix_linear-vector_mix_turn;
                candidate_wheel2<=vector_mix_linear+vector_mix_turn;
                candidate_wheel3<=vector_mix_linear-vector_mix_turn;
            end else begin
            candidate_motion<=(follow_motion_valid && follow_speed_r!=0 && motor_online) ? follow_motion_r : MOT_NONE;
            candidate_speed<=(follow_motion_valid && motor_online && follow_speed_r!=0) ?
                (follow_speed_r>follow_speed_limit ? follow_speed_limit : follow_speed_r) : 8'd0;
            end
        end else if(nav_selected) begin
            candidate_motion<=(nav_motion_valid && nav_speed_r!=0) ? nav_motion_r : MOT_NONE;
            candidate_speed<=!nav_motion_valid ? 8'd0 : nav_speed_r>SPEED_MAX ? SPEED_MAX : nav_speed_r;
            if(MOTOR_OFFICIAL_PROTOCOL && ((ENABLE_NAV_HOLONOMIC && nav_motion_r>=1 && nav_motion_r<=4)||
                    (ENABLE_NAV_FORWARD_TRIM && nav_motion_r==MOT_FORWARD)))begin
                candidate_vector<=1;candidate_nav_vector<=1;
                if(!nav_vector_authorized||nav_vector_stop)begin candidate_motion<=MOT_NONE;candidate_speed<=0;end
                case(nav_motion_r)
                    MOT_FORWARD:begin candidate_wheel0<=-nav_base-nav_trim;candidate_wheel1<=nav_base-nav_trim;
                        candidate_wheel2<=nav_base+nav_trim;candidate_wheel3<=nav_base-nav_trim;end
                    MOT_BACKWARD:begin candidate_wheel0<=nav_base-nav_trim;candidate_wheel1<=-nav_base-nav_trim;
                        candidate_wheel2<=-nav_base+nav_trim;candidate_wheel3<=-nav_base-nav_trim;end
                    MOT_LEFT_STRAFE:begin candidate_wheel0<=nav_base-nav_trim;candidate_wheel1<=nav_base-nav_trim;
                        candidate_wheel2<=nav_base+nav_trim;candidate_wheel3<=-nav_base-nav_trim;end
                    default:begin candidate_wheel0<=-nav_base-nav_trim;candidate_wheel1<=-nav_base-nav_trim;
                        candidate_wheel2<=-nav_base+nav_trim;candidate_wheel3<=nav_base-nav_trim;end
                endcase
            end
        end else if(gap_auto_active)begin
            candidate_gap_active<=1;candidate_gap_index<=gap_index;candidate_vector<=1;
            candidate_motion<=gap_authorized?policy_motion:MOT_NONE;
            candidate_speed<=gap_authorized?policy_speed:8'd0;
            candidate_wheel0<=gap_authorized?-gap_forward+gap_left:16'sd0;
            candidate_wheel1<=gap_authorized?gap_forward+gap_left:16'sd0;
            candidate_wheel2<=gap_authorized?gap_forward+gap_left:16'sd0;
            candidate_wheel3<=gap_authorized?gap_forward-gap_left:16'sd0;
        end else if(!scan_auto_owner && requested_motion==MOT_NONE) begin
            candidate_motion<=MOT_NONE; candidate_speed<=0;
        end else if(heading_authorized && (heading_active || heading_required))begin
            candidate_heading_active<=1;candidate_vector<=1;
            candidate_motion<=heading_active && heading_ready?heading_motion:MOT_NONE;
            candidate_speed<=heading_active && heading_ready?heading_speed:8'd0;
            candidate_heading_turn<=heading_turn;
            candidate_wheel0<=heading_wheel0;candidate_wheel1<=heading_wheel1;
            candidate_wheel2<=heading_wheel2;candidate_wheel3<=heading_wheel3;
        end else begin
            candidate_motion<=scan_auto_owner ? scan_auto_motion :
                follow_vector_mode ? policy_motion_sample : policy_motion;
            candidate_speed<=scan_auto_owner ? scan_auto_speed :
                follow_vector_mode ? (policy_motion_sample==MOT_NONE ? 8'd0 : policy_speed_sample) :
                                     (policy_motion==MOT_NONE ? 8'd0 : policy_speed);
        end
        end
    end
    wire safety_blocked,safety_slow,safety_unknown;
    wire [3:0] guard_motion;
    wire [7:0] guard_speed;
    wire signed [15:0] guard_wheel0,guard_wheel1,guard_wheel2,guard_wheel3;
    // A side may become blocked while the alternative tuple is crossing the
    // request pipeline. Re-mix that NEW policy direction before the registered
    // final guard, rather than create a nanosecond STOP epoch on the OLD side.
    // The guard checks the replacement direction, too; never bypass a stop
    // without an actual nonzero policy decision and live heading authorization.
    wire flow_redirect=ENABLE_FLOW_BYPASS && !ENABLE_RETURN_TO_LINE && avoidance_enabled && requested_motion==MOT_FORWARD &&
        heading_authorized && heading_ready && candidate_heading_active && policy_motion!=MOT_NONE &&
        policy_motion!=candidate_motion;
    wire [3:0] live_guard_request_motion=flow_redirect?policy_motion:candidate_motion;
    wire [7:0] live_guard_request_speed=flow_redirect?
        (candidate_speed==0 || policy_speed<candidate_speed?policy_speed:candidate_speed):candidate_speed;
    wire signed [16:0] vector_yaw_sum=$signed({candidate_wheel0[15],candidate_wheel0})+
        $signed({candidate_wheel1[15],candidate_wheel1});
    // Four-wheel yaw basis also works for left/right translation. A+B alone
    // mistakes a pure strafe for rotation, so navigation uses all four wheels.
    wire signed [17:0] nav_yaw_sum=-$signed({{2{candidate_wheel0[15]}},candidate_wheel0})-
        $signed({{2{candidate_wheel1[15]}},candidate_wheel1})+
        $signed({{2{candidate_wheel2[15]}},candidate_wheel2})-
        $signed({{2{candidate_wheel3[15]}},candidate_wheel3});
    wire live_vector_rotation=candidate_heading_active?(ENABLE_RETURN_TO_LINE?nav_yaw_sum!=0:candidate_heading_turn!=0):
        candidate_vector && !candidate_gap_active && (candidate_nav_vector?nav_yaw_sum!=0:vector_yaw_sum!=0);
    // Sample redirected direction, all wheel/source fields and the range
    // observations together. This prevents an old direction being checked
    // against a newer opposite-side obstacle for one clock. Remote STOP and
    // owner/link fault vetoes remain live at the UART boundary below.
    wire [3:0] guard_request_motion;
    wire [7:0] guard_request_speed;
    wire guard_input_vector,guard_input_rotation;
    wire guard_input_nav;
    wire guard_input_gap;
    wire guard_input_heading;
    wire [15:0] guard_input_turn;
    wire [3:0] guard_input_gap_index;
    wire [15:0] guard_input_wheel0;
    wire [15:0] guard_input_wheel1;
    wire [15:0] guard_input_wheel2;
    wire [15:0] guard_input_wheel3;
    wire guard_input_lidar_healthy;
    wire guard_input_lidar_rotation_valid;
    wire [15:0] guard_input_lidar_rotation_cm;
    wire guard_input_front_valid;
    wire guard_input_rear_valid;
    wire guard_input_left_valid;
    wire guard_input_right_valid;
    wire [15:0] guard_input_front_distance_cm;
    wire [15:0] guard_input_rear_distance_cm;
    wire [15:0] guard_input_left_distance_cm;
    wire [15:0] guard_input_right_distance_cm;
    wire [143:0] guard_input_gap_clearance_mm;
    wire [8:0] guard_input_gap_direction_valid;
    generate if(ENABLE_LIDAR_SAFETY)begin: guard_tuple_pipeline
        reg guard_input_rotation_q;
        assign guard_input_rotation=guard_input_rotation_q;
        reg [3:0] guard_request_motion_q;
        assign guard_request_motion=guard_request_motion_q;
        reg [7:0] guard_request_speed_q;
        assign guard_request_speed=guard_request_speed_q;
        reg guard_input_vector_q;
        assign guard_input_vector=guard_input_vector_q;
        reg guard_input_nav_q;
        assign guard_input_nav=guard_input_nav_q;
        reg guard_input_gap_q;
        assign guard_input_gap=guard_input_gap_q;
        reg guard_input_heading_q;
        assign guard_input_heading=guard_input_heading_q;
        reg [15:0] guard_input_turn_q;
        assign guard_input_turn=guard_input_turn_q;
        reg [3:0] guard_input_gap_index_q;
        assign guard_input_gap_index=guard_input_gap_index_q;
        reg [15:0] guard_input_wheel0_q;
        assign guard_input_wheel0=guard_input_wheel0_q;
        reg [15:0] guard_input_wheel1_q;
        assign guard_input_wheel1=guard_input_wheel1_q;
        reg [15:0] guard_input_wheel2_q;
        assign guard_input_wheel2=guard_input_wheel2_q;
        reg [15:0] guard_input_wheel3_q;
        assign guard_input_wheel3=guard_input_wheel3_q;
        reg guard_input_lidar_healthy_q;
        assign guard_input_lidar_healthy=guard_input_lidar_healthy_q;
        reg guard_input_lidar_rotation_valid_q;
        assign guard_input_lidar_rotation_valid=guard_input_lidar_rotation_valid_q;
        reg [15:0] guard_input_lidar_rotation_cm_q;
        assign guard_input_lidar_rotation_cm=guard_input_lidar_rotation_cm_q;
        reg guard_input_front_valid_q;
        assign guard_input_front_valid=guard_input_front_valid_q;
        reg guard_input_rear_valid_q;
        assign guard_input_rear_valid=guard_input_rear_valid_q;
        reg guard_input_left_valid_q;
        assign guard_input_left_valid=guard_input_left_valid_q;
        reg guard_input_right_valid_q;
        assign guard_input_right_valid=guard_input_right_valid_q;
        reg [15:0] guard_input_front_distance_cm_q;
        assign guard_input_front_distance_cm=guard_input_front_distance_cm_q;
        reg [15:0] guard_input_rear_distance_cm_q;
        assign guard_input_rear_distance_cm=guard_input_rear_distance_cm_q;
        reg [15:0] guard_input_left_distance_cm_q;
        assign guard_input_left_distance_cm=guard_input_left_distance_cm_q;
        reg [15:0] guard_input_right_distance_cm_q;
        assign guard_input_right_distance_cm=guard_input_right_distance_cm_q;
        reg [143:0] guard_input_gap_clearance_mm_q;
        assign guard_input_gap_clearance_mm=guard_input_gap_clearance_mm_q;
        reg [8:0] guard_input_gap_direction_valid_q;
        assign guard_input_gap_direction_valid=guard_input_gap_direction_valid_q;
        always @(posedge clk50M or negedge reset_n)begin
            if(!reset_n)begin
                guard_request_motion_q<=0;
                guard_request_speed_q<=0;
                guard_input_vector_q<=0;guard_input_rotation_q<=0;
                guard_input_nav_q<=0;
                guard_input_gap_q<=0;
                guard_input_heading_q<=0;
                guard_input_turn_q<=0;
                guard_input_gap_index_q<=0;
                guard_input_wheel0_q<=0;
                guard_input_wheel1_q<=0;
                guard_input_wheel2_q<=0;
                guard_input_wheel3_q<=0;
                guard_input_lidar_healthy_q<=0;
                guard_input_lidar_rotation_valid_q<=0;
                guard_input_lidar_rotation_cm_q<=0;
                guard_input_front_valid_q<=0;
                guard_input_rear_valid_q<=0;
                guard_input_left_valid_q<=0;
                guard_input_right_valid_q<=0;
                guard_input_front_distance_cm_q<=0;
                guard_input_rear_distance_cm_q<=0;
                guard_input_left_distance_cm_q<=0;
                guard_input_right_distance_cm_q<=0;
                guard_input_gap_clearance_mm_q<=0;
                guard_input_gap_direction_valid_q<=0;
            end else if(urgent_stop || nav_wait_stop || follow_wait_stop)begin
                guard_request_motion_q<=0;
                guard_request_speed_q<=0;
                guard_input_vector_q<=0;guard_input_rotation_q<=0;
                guard_input_nav_q<=0;
                guard_input_gap_q<=0;
                guard_input_heading_q<=0;
                guard_input_turn_q<=0;
                guard_input_gap_index_q<=0;
                guard_input_wheel0_q<=0;
                guard_input_wheel1_q<=0;
                guard_input_wheel2_q<=0;
                guard_input_wheel3_q<=0;
                guard_input_lidar_healthy_q<=0;
                guard_input_lidar_rotation_valid_q<=0;
                guard_input_lidar_rotation_cm_q<=0;
                guard_input_front_valid_q<=0;
                guard_input_rear_valid_q<=0;
                guard_input_left_valid_q<=0;
                guard_input_right_valid_q<=0;
                guard_input_front_distance_cm_q<=0;
                guard_input_rear_distance_cm_q<=0;
                guard_input_left_distance_cm_q<=0;
                guard_input_right_distance_cm_q<=0;
                guard_input_gap_clearance_mm_q<=0;
                guard_input_gap_direction_valid_q<=0;
            end else begin
                guard_request_motion_q<=live_guard_request_motion;
                guard_request_speed_q<=live_guard_request_speed;
                guard_input_vector_q<=candidate_vector;guard_input_rotation_q<=live_vector_rotation;
                guard_input_nav_q<=candidate_nav_vector;
                guard_input_gap_q<=candidate_gap_active;
                guard_input_heading_q<=candidate_heading_active;
                guard_input_turn_q<=candidate_heading_turn;
                guard_input_gap_index_q<=candidate_gap_index;
                guard_input_wheel0_q<=candidate_wheel0;
                guard_input_wheel1_q<=candidate_wheel1;
                guard_input_wheel2_q<=candidate_wheel2;
                guard_input_wheel3_q<=candidate_wheel3;
                guard_input_lidar_healthy_q<=lidar_healthy;
                guard_input_lidar_rotation_valid_q<=lidar_rotation_valid;
                guard_input_lidar_rotation_cm_q<=lidar_rotation_cm;
                guard_input_front_valid_q<=front_valid;
                guard_input_rear_valid_q<=rear_valid;
                guard_input_left_valid_q<=left_valid;
                guard_input_right_valid_q<=right_valid;
                guard_input_front_distance_cm_q<=front_distance_cm;
                guard_input_rear_distance_cm_q<=rear_distance_cm;
                guard_input_left_distance_cm_q<=left_distance_cm;
                guard_input_right_distance_cm_q<=right_distance_cm;
                guard_input_gap_clearance_mm_q<=gap_clearance_mm;
                guard_input_gap_direction_valid_q<=gap_direction_valid;
            end
        end
    end else begin: no_guard_tuple_pipeline
        assign guard_input_rotation=live_vector_rotation;
        assign guard_request_motion=live_guard_request_motion;
        assign guard_request_speed=live_guard_request_speed;
        assign guard_input_vector=candidate_vector;
        assign guard_input_nav=candidate_nav_vector;
        assign guard_input_gap=candidate_gap_active;
        assign guard_input_heading=candidate_heading_active;
        assign guard_input_turn=candidate_heading_turn;
        assign guard_input_gap_index=candidate_gap_index;
        assign guard_input_wheel0=candidate_wheel0;
        assign guard_input_wheel1=candidate_wheel1;
        assign guard_input_wheel2=candidate_wheel2;
        assign guard_input_wheel3=candidate_wheel3;
        assign guard_input_lidar_healthy=lidar_healthy;
        assign guard_input_lidar_rotation_valid=lidar_rotation_valid;
        assign guard_input_lidar_rotation_cm=lidar_rotation_cm;
        assign guard_input_front_valid=front_valid;
        assign guard_input_rear_valid=rear_valid;
        assign guard_input_left_valid=left_valid;
        assign guard_input_right_valid=right_valid;
        assign guard_input_front_distance_cm=front_distance_cm;
        assign guard_input_rear_distance_cm=rear_distance_cm;
        assign guard_input_left_distance_cm=left_distance_cm;
        assign guard_input_right_distance_cm=right_distance_cm;
        assign guard_input_gap_clearance_mm=gap_clearance_mm;
        assign guard_input_gap_direction_valid=gap_direction_valid;
    end endgenerate
    wire [15:0] live_heading_stop=lidar_stop_at_grade(guard_request_speed);
    wire live_heading_turn_allowed=!lidar_safety_active ||
        (guard_input_lidar_rotation_valid && guard_input_lidar_rotation_cm>live_heading_stop);
    // A ramped heading vector can translate diagonally during handoff. Check
    // BOTH actual translation components, even though the high-level motion
    // code has already changed to the next cardinal leg.
    wire signed [18:0] hw0={{3{guard_input_wheel0[15]}},guard_input_wheel0};
    wire signed [18:0] hw1={{3{guard_input_wheel1[15]}},guard_input_wheel1};
    wire signed [18:0] hw2={{3{guard_input_wheel2[15]}},guard_input_wheel2};
    wire signed [18:0] hw3={{3{guard_input_wheel3[15]}},guard_input_wheel3};
    wire signed [18:0] swept_f=-hw0+hw1+hw2+hw3,swept_l=hw0+hw1+hw2-hw3;
    wire smooth_heading_guard=ENABLE_RETURN_TO_LINE && guard_input_heading && guard_input_vector;
    // Only this automatic-heading path gets the requested front reduction.
    // Keep raw measured ranges (and their plots) unchanged; compare adjusted
    // front clearance only. Left/right/rear and rotation guards stay intact.
    wire [15:0] bounded_front_relief=live_heading_stop>AVOID_FRONT_RELIEF_CM+2?
        AVOID_FRONT_RELIEF_CM:(live_heading_stop>2?live_heading_stop-2:0);
    wire [16:0] adjusted_front_sum={1'b0,guard_input_front_distance_cm}+
        ((ENABLE_RETURN_TO_LINE && guard_input_heading)?bounded_front_relief:0);
    wire [15:0] guard_front_clearance=adjusted_front_sum[16]?16'hffff:adjusted_front_sum[15:0];
    wire swept_unknown=smooth_heading_guard &&
        ((swept_f>0&&!guard_input_front_valid)||(swept_f<0&&!guard_input_rear_valid)||
         (swept_l>0&&!guard_input_left_valid)||(swept_l<0&&!guard_input_right_valid));
    wire swept_blocked=smooth_heading_guard && (swept_unknown ||
        (swept_f>0&&guard_front_clearance<=live_heading_stop)||
        (swept_f<0&&guard_input_rear_distance_cm<=live_heading_stop)||
        (swept_l>0&&guard_input_left_distance_cm<=live_heading_stop)||
        (swept_l<0&&guard_input_right_distance_cm<=live_heading_stop));
    // Guard comparison and wheel re-mixing use separate clock edges. The
    // complete guarded decision is one tuple, so speed/turn/source cannot mix
    // with a newer command. The additional latency is one 50 MHz clock.
    wire [3:0] mix_motion;
    wire [7:0] mix_speed;
    wire mix_vector;
    wire mix_nav;
    wire mix_gap;
    wire mix_heading;
    wire [15:0] mix_turn;
    wire mix_turn_allowed;
    wire [3:0] mix_gap_index;
    wire mix_blocked;
    wire mix_slow;
    wire [15:0] mix_wheel0;
    wire [15:0] mix_wheel1;
    wire [15:0] mix_wheel2;
    wire [15:0] mix_wheel3;
    wire [63:0] protected_gap_wheels,protected_heading_wheels;
    generate if(ENABLE_LIDAR_SAFETY)begin: guard_decision_pipeline
        reg [3:0] mix_motion_q;
        assign mix_motion=mix_motion_q;
        reg [7:0] mix_speed_q;
        assign mix_speed=mix_speed_q;
        reg mix_vector_q;
        assign mix_vector=mix_vector_q;
        reg mix_nav_q;
        assign mix_nav=mix_nav_q;
        reg mix_gap_q;
        assign mix_gap=mix_gap_q;
        reg mix_heading_q;
        assign mix_heading=mix_heading_q;
        reg [15:0] mix_turn_q;
        assign mix_turn=mix_turn_q;
        reg mix_turn_allowed_q;
        assign mix_turn_allowed=mix_turn_allowed_q;
        reg [3:0] mix_gap_index_q;
        assign mix_gap_index=mix_gap_index_q;
        reg mix_blocked_q;
        assign mix_blocked=mix_blocked_q;
        reg mix_slow_q;
        assign mix_slow=mix_slow_q;
        reg [15:0] mix_wheel0_q;
        assign mix_wheel0=mix_wheel0_q;
        reg [15:0] mix_wheel1_q;
        assign mix_wheel1=mix_wheel1_q;
        reg [15:0] mix_wheel2_q;
        assign mix_wheel2=mix_wheel2_q;
        reg [15:0] mix_wheel3_q;
        assign mix_wheel3=mix_wheel3_q;
        always @(posedge clk50M or negedge reset_n)begin
            if(!reset_n)begin
                mix_motion_q<=0;
                mix_speed_q<=0;
                mix_vector_q<=0;
                mix_nav_q<=0;
                mix_gap_q<=0;
                mix_heading_q<=0;
                mix_turn_q<=0;
                mix_turn_allowed_q<=0;
                mix_gap_index_q<=0;
                mix_blocked_q<=0;
                mix_slow_q<=0;
                mix_wheel0_q<=0;
                mix_wheel1_q<=0;
                mix_wheel2_q<=0;
                mix_wheel3_q<=0;
            end else if(urgent_stop || nav_wait_stop || follow_wait_stop)begin
                mix_motion_q<=0;
                mix_speed_q<=0;
                mix_vector_q<=0;
                mix_nav_q<=0;
                mix_gap_q<=0;
                mix_heading_q<=0;
                mix_turn_q<=0;
                mix_turn_allowed_q<=0;
                mix_gap_index_q<=0;
                mix_blocked_q<=0;
                mix_slow_q<=0;
                mix_wheel0_q<=0;
                mix_wheel1_q<=0;
                mix_wheel2_q<=0;
                mix_wheel3_q<=0;
            end else begin
                mix_motion_q<=guard_motion;
                mix_speed_q<=guard_speed;
                mix_vector_q<=guard_input_vector;
                mix_nav_q<=guard_input_nav;
                mix_gap_q<=guard_input_gap;
                mix_heading_q<=guard_input_heading;
                mix_turn_q<=guard_input_turn;
                mix_turn_allowed_q<=live_heading_turn_allowed;
                mix_gap_index_q<=guard_input_gap_index;
                mix_blocked_q<=safety_blocked;
                mix_slow_q<=safety_slow;
                mix_wheel0_q<=guard_input_wheel0;
                mix_wheel1_q<=guard_input_wheel1;
                mix_wheel2_q<=guard_input_wheel2;
                mix_wheel3_q<=guard_input_wheel3;
            end
        end
    end else begin: no_guard_decision_pipeline
        assign mix_motion=guard_motion;
        assign mix_speed=guard_speed;
        assign mix_vector=guard_input_vector;
        assign mix_nav=guard_input_nav;
        assign mix_gap=guard_input_gap;
        assign mix_heading=guard_input_heading;
        assign mix_turn=guard_input_turn;
        assign mix_turn_allowed=live_heading_turn_allowed;
        assign mix_gap_index=guard_input_gap_index;
        assign mix_blocked=safety_blocked;
        assign mix_slow=safety_slow;
        assign mix_wheel0=guard_input_wheel0;
        assign mix_wheel1=guard_input_wheel1;
        assign mix_wheel2=guard_input_wheel2;
        assign mix_wheel3=guard_input_wheel3;
    end endgenerate
    generate if(ENABLE_LIDAR_SAFETY) begin: lidar_guard
        wire raw_blocked,raw_slow,raw_unknown;
        wire [3:0] raw_motion;
        wire [7:0] raw_speed;
        wire [15:0] live_gap_cm=guard_input_gap_clearance_mm[guard_input_gap_index*16+:16]/16'd10;
        wire live_gap_valid=guard_input_gap_direction_valid[guard_input_gap_index];
        // Auto translation checks its actual swept rectangle, not an unrelated
        // front/rotation disk. Manual and follow still use the original guard.
        lidar_motion_guard #(.STOP_CM(LIDAR_STOP_CM),.SLOW_CM(LIDAR_SLOW_CM),.EXTRA_STOP_CM_PER_GRADE(LIDAR_EXTRA_STOP_CM),.SPEED_GRADE_MAX(SPEED_MAX)) guard(
            .motion(guard_input_gap?4'd1:guard_request_motion),.speed(guard_request_speed),
            .rotation_request(guard_input_rotation && (!guard_input_heading || live_heading_turn_allowed)),.healthy(guard_input_lidar_healthy),
            .front_valid(guard_input_gap?live_gap_valid:guard_input_front_valid),.rear_valid(guard_input_rear_valid),.left_valid(guard_input_left_valid),.right_valid(guard_input_right_valid),
            .rotation_valid(guard_input_lidar_rotation_valid),.front_cm(guard_input_gap?live_gap_cm:guard_front_clearance),.rear_cm(guard_input_rear_distance_cm),
            .left_cm(guard_input_left_distance_cm),.right_cm(guard_input_right_distance_cm),.rotation_cm(guard_input_lidar_rotation_cm),
            .blocked(raw_blocked),.slowdown(raw_slow),.unknown(raw_unknown),
            .safe_motion(raw_motion),.safe_speed(raw_speed));
        assign safety_blocked=lidar_safety_active && (raw_blocked||swept_blocked);
        assign safety_slow=lidar_safety_active && raw_slow;
        assign safety_unknown=lidar_safety_active && (raw_unknown||swept_unknown);
        assign guard_motion=lidar_safety_active?((raw_blocked||swept_blocked)?MOT_NONE:guard_request_motion):guard_request_motion;
        assign guard_speed=lidar_safety_active?(swept_blocked?8'd0:raw_speed):guard_request_speed;
        // Keep the entire motion/vector tuple coherent. This one 50MHz clock
        // breaks LiDAR range -> policy -> STOP proof/origin/serializer paths.
        // A register here adds 20ns, not another scan/frame/refresh period.
        reg [3:0] safe_motion_q;
        reg [7:0] safe_speed_q;
        reg safe_vector_q,safe_nav_q;
        reg safe_gap_q;
        reg safe_heading_q;
        reg signed [15:0] safe_wheel0_q,safe_wheel1_q,safe_wheel2_q,safe_wheel3_q;
        wire [16:0] guarded_control;
        wire [63:0] guarded_wheels;
        wire [63:0] ordinary_wheels={
            mix_gap?protected_gap_wheels[63:48]:reduced_wheel(mix_wheel0,mix_slow),
            mix_gap?protected_gap_wheels[47:32]:reduced_wheel(mix_wheel1,mix_slow),
            mix_gap?protected_gap_wheels[31:16]:reduced_wheel(mix_wheel2,mix_slow),
            mix_gap?protected_gap_wheels[15:0]:reduced_wheel(mix_wheel3,mix_slow)};
        motor_guard_mix_pipeline #(.SPEED_MAX(SPEED_MAX),.SCALE(MOTOR_SPEED_SCALE)) u_guard_mix(
            .clk(clk50M),.reset_n(reset_n),
            .control_in({mix_blocked,mix_heading,mix_gap,mix_nav,mix_vector,mix_speed,mix_motion}),
            .smooth(ENABLE_RETURN_TO_LINE && mix_heading),.allow_turn(mix_turn_allowed),
            .raw_wheels({mix_wheel0,mix_wheel1,mix_wheel2,mix_wheel3}),
            .fallback_wheels(mix_heading?protected_heading_wheels:ordinary_wheels),
            .control_out(guarded_control),.wheels_out(guarded_wheels));
        always @(posedge clk50M or negedge reset_n) begin
            if(!reset_n) begin
                safe_motion_q<=0;safe_speed_q<=0;safe_vector_q<=0;safe_nav_q<=0;safe_gap_q<=0;safe_heading_q<=0;
                safe_wheel0_q<=0;safe_wheel1_q<=0;safe_wheel2_q<=0;safe_wheel3_q<=0;
            end else begin
                {safe_heading_q,safe_gap_q,safe_nav_q,safe_vector_q,safe_speed_q,safe_motion_q}<=guarded_control[15:0];
                // Safety cancellation bypasses all arithmetic pipeline stages.
                if(mix_blocked || urgent_stop || nav_wait_stop || follow_wait_stop)begin
                    safe_motion_q<=0;safe_speed_q<=0;safe_vector_q<=0;safe_nav_q<=0;
                    safe_gap_q<=0;safe_heading_q<=0;
                    safe_wheel0_q<=0;safe_wheel1_q<=0;safe_wheel2_q<=0;safe_wheel3_q<=0;
                end else begin
                    safe_wheel0_q<=guarded_wheels[63:48];safe_wheel1_q<=guarded_wheels[47:32];
                    safe_wheel2_q<=guarded_wheels[31:16];safe_wheel3_q<=guarded_wheels[15:0];
                end
            end
        end
        assign effective_motion=safe_motion_q;assign effective_speed=safe_speed_q;
        assign effective_vector=safe_vector_q;
        assign effective_nav_vector=safe_nav_q;
        assign effective_gap_active=safe_gap_q;
        assign effective_heading_active=safe_heading_q;
        assign effective_wheel0=safe_wheel0_q;assign effective_wheel1=safe_wheel1_q;
        assign effective_wheel2=safe_wheel2_q;assign effective_wheel3=safe_wheel3_q;
    end else begin: no_lidar_guard
        assign effective_motion=candidate_motion;
        assign effective_speed=candidate_speed;
        assign effective_vector=guard_input_vector;
        assign effective_nav_vector=guard_input_nav;
        assign effective_gap_active=guard_input_gap;
        assign effective_heading_active=guard_input_heading;
        assign guard_motion=candidate_motion;assign guard_speed=candidate_speed;
        assign effective_wheel0=guard_input_wheel0;assign effective_wheel1=guard_input_wheel1;
        assign effective_wheel2=guard_input_wheel2;assign effective_wheel3=guard_input_wheel3;
        assign safety_blocked=0;assign safety_slow=0;assign safety_unknown=0;
    end endgenerate
    function signed [15:0] reduced_wheel;
        input signed [15:0] value;
        input slow;
        reg signed [15:0] half;
        begin
            half=value/16'sd2;
            reduced_wheel=!slow?value:value==0?16'sd0:
                half==0?(value<0?-16'sd1:16'sd1):half;
        end
    endfunction
    // Rebuild reduced auto vectors from the same rational heading. Independent
    // rounding of odd wheel speeds could otherwise turn (2,1) into (1,0).
    function [63:0] gap_wheels(input [3:0] d,input [7:0] grade);
        reg [15:0] b,u;
        reg signed [15:0] f,l,w0,w1,w2,w3;
        begin
            b=grade*MOTOR_SPEED_SCALE;u=b/16'd3;f=0;l=0;
            case(d)
                0:l=-$signed(b);1:begin f=u;l=-$signed(u)*2;end
                2:begin f=b/16'd2;l=-$signed(b/16'd2);end
                3:begin f=u*2;l=-$signed(u);end
                4:f=b;5:begin f=u*2;l=u;end
                6:begin f=b/16'd2;l=b/16'd2;end
                7:begin f=u;l=u*2;end
                8:l=b;default:begin end
            endcase
            w0=-f+l;w1=f+l;w2=f+l;w3=f-l;gap_wheels={w0,w1,w2,w3};
        end
    endfunction
    assign protected_gap_wheels=gap_wheels(mix_gap_index,mix_speed);
    // Re-mix with the FINAL guard speed, so slow-down never leaves wheel
    // magnitudes above its grade budget. Strafing is not mistaken for rotation.
    localparam integer HOLD_MIX_BITS=(SPEED_MAX*MOTOR_SPEED_SCALE>=1&&SPEED_MAX*MOTOR_SPEED_SCALE<=16383)?
        $clog2(SPEED_MAX*MOTOR_SPEED_SCALE+1)+1:16;
    function [63:0] held_wheels(input [3:0] m,input [7:0] grade,input signed [15:0] turn_value);
        reg signed [HOLD_MIX_BITS:0] f,l,t,drive,w0,w1,w2,w3;
        reg signed [15:0] o0,o1,o2,o3;
        reg [47:0] units;
        reg [HOLD_MIX_BITS-1:0] budget;reg [2:0] mag;reg [15:0] magnitude;
        begin
            units=official_grade_units(grade>SPEED_MAX?SPEED_MAX:grade);
            budget=units[47:32];
            magnitude=turn_value<0?-turn_value:turn_value;mag=magnitude>3?3:magnitude[2:0];
            if(mag>(budget>>1))mag=budget>>1;
            t=turn_value<0?-$signed(mag):$signed(mag);drive=$signed(budget-mag);f=0;l=0;
            case(m)1:f=drive;2:f=-drive;3:l=drive;4:l=-drive;default:begin end endcase
            w0=-f+l-t;w1=f+l-t;w2=f+l+t;w3=f-l-t;
            o0=w0;o1=w1;o2=w2;o3=w3;held_wheels={o0,o1,o2,o3};
        end
    endfunction
    assign protected_heading_wheels=held_wheels(mix_motion,mix_speed,
        mix_turn_allowed?mix_turn:16'sd0);
    function [63:0] smooth_heading_wheels(input signed [15:0] a,b,c,d,input [7:0] grade,input allow_turn);
        reg signed [31:0] f,l,t,budget,remaining,af,al,at;
        reg signed [15:0] a0,b0,c0,d0;
        begin
            f=(-$signed(a)+$signed(b)+$signed(c)+$signed(d))/4;
            l=($signed(a)+$signed(b)+$signed(c)-$signed(d))/4;
            t=allow_turn?(-$signed(a)-$signed(b)+$signed(c)-$signed(d))/4:0;
            budget=(grade>SPEED_MAX?SPEED_MAX:grade)*MOTOR_SPEED_SCALE;
            if(t>2)t=2;if(t< -2)t=-2;
            at=t<0?-t:t;if(at>budget/2)begin t=t<0?-(budget/2):budget/2;at=budget/2;end
            remaining=budget-at;
            if(f>remaining)f=remaining;if(f< -remaining)f=-remaining;
            af=f<0?-f:f;remaining=remaining-af;
            if(l>remaining)l=remaining;if(l< -remaining)l=-remaining;
            a0=-f+l-t;b0=f+l-t;c0=f+l+t;d0=f-l-t;
            smooth_heading_wheels={a0,b0,c0,d0};
        end
    endfunction
    wire [63:0] protected_smooth_wheels=smooth_heading_wheels(mix_wheel0,mix_wheel1,mix_wheel2,mix_wheel3,mix_speed,mix_turn_allowed);
    wire [63:0] selected_heading_wheels=ENABLE_RETURN_TO_LINE?protected_smooth_wheels:protected_heading_wheels;
    assign guard_wheel0=mix_blocked?16'sd0:mix_heading?$signed(selected_heading_wheels[63:48]):mix_gap?$signed(protected_gap_wheels[63:48]):reduced_wheel(mix_wheel0,mix_slow);
    assign guard_wheel1=mix_blocked?16'sd0:mix_heading?$signed(selected_heading_wheels[47:32]):mix_gap?$signed(protected_gap_wheels[47:32]):reduced_wheel(mix_wheel1,mix_slow);
    assign guard_wheel2=mix_blocked?16'sd0:mix_heading?$signed(selected_heading_wheels[31:16]):mix_gap?$signed(protected_gap_wheels[31:16]):reduced_wheel(mix_wheel2,mix_slow);
    assign guard_wheel3=mix_blocked?16'sd0:mix_heading?$signed(selected_heading_wheels[15:0]):mix_gap?$signed(protected_gap_wheels[15:0]):reduced_wheel(mix_wheel3,mix_slow);
    assign avoidance_fault=lidar_safety_active && !lidar_healthy?4'd14:
        safety_blocked?(safety_unknown?4'd14:4'd9):heading_required && !heading_ready?4'd4:policy_fault;
    assign motion_command=urgent_stop ? MOT_NONE : effective_motion;
    assign applied_speed=(motion_command==MOT_NONE) ? 8'd0 : effective_speed;

    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy, tx_done;
    uart_tx_byte #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) tx (
        .clk(clk50M), .reset_n(reset_n), .start(tx_start), .data(tx_data),
        .uart_tx(motor_uart_tx), .busy(tx_busy), .done(tx_done)
    );

    reg [3:0] running_motion, frame_select, previous_effective_motion;
    assign actual_motion=running_motion;
    reg [7:0] running_speed, frame_speed;
    reg [63:0] frame_wheels;
    reg frame_vector_active;
    reg [63:0] official_motion_wheels;
    reg [3:0] official_payload_motion;
    reg [7:0] official_payload_speed;
    reg official_payload_vector;
    reg [15:0] frame_crc;
    reg frame_crc_reset_pending;
    reg [2:0] seq_state;
    reg stop_pending, send_active, waiting_done;
    reg [5:0] frame_length, byte_index;
    reg [31:0] gap_count;
    wire target_stop=(effective_motion==MOT_NONE) || urgent_stop ||
        (MOTOR_OFFICIAL_PROTOCOL && !motor_control_ready) ||
        (effective_vector && !effective_gap_active && !effective_heading_active && !effective_nav_vector &&
            (!vector_authorized || vector_input_stop)) ||
        (effective_nav_vector && (!nav_vector_authorized || nav_vector_stop)) ||
        (effective_heading_active && (!heading_authorized || (heading_required && !heading_ready))) ||
        (effective_gap_active && !gap_authorized);
    wire frame_complete=send_active && waiting_done && tx_done &&
                        (byte_index==frame_length-1'b1);
    wire stop_complete=frame_complete && seq_state==SEQ_STOP;
    assign motion_frame_committed=frame_complete && frame_select!=FRAME_ENABLE;
    assign transmitted_motion=frame_select==FRAME_STOP ? MOT_NONE : frame_select;
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) committed_wheel_targets<=0;
        else if(motion_frame_committed) begin
            // Official STOP serializes zero even when the candidate snapshot
            // still contains a nonzero vector. Observe the actual wire payload.
            if(frame_select==FRAME_STOP) committed_wheel_targets<=0;
            else if(MOTOR_OFFICIAL_PROTOCOL) committed_wheel_targets<=frame_wheels;
            else committed_wheel_targets<=
                {wheel_value(frame_select,2'd0,frame_speed),wheel_value(frame_select,2'd1,frame_speed),
                 wheel_value(frame_select,2'd2,frame_speed),wheel_value(frame_select,2'd3,frame_speed)};
        end
    end
    wire official_new_stop_intent=MOTOR_OFFICIAL_PROTOCOL &&
        (urgent_stop || (target_stop && !official_previous_target_stop) ||
         official_link_lost || official_settle_complete || official_reinit_authorization_event);
    wire official_stop_epoch_complete=stop_complete && official_frame_starts_stop_epoch &&
        !official_stop_epoch_pending && !official_new_stop_intent;
    reg [SILENCE_BITS-1:0] frame_quiet_count;
    reg [31:0] stop_retry_count,stop_feedback_age;
    reg [SCAN_REFRESH_BITS-1:0] scan_refresh_count;
    // Refresh only the continuous scan command. Frame snapshots, minimum
    // silence and urgent STOP priorities remain identical to ordinary motion.
    wire scan_refresh_due=MAP_SCAN_CONTINUOUS && scan_continuous_capture &&
        !target_stop && scan_refresh_count>=SCAN_REFRESH_CLKS;
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) scan_refresh_count<=0;
        else if(!scan_continuous_capture || target_stop ||
                (frame_complete && seq_state==SEQ_MOTION)) scan_refresh_count<=0;
        else if(scan_refresh_count<SCAN_REFRESH_CLKS) scan_refresh_count<=scan_refresh_count+1'b1;
    end
    reg stop_feedback_armed,stop_feedback_confirmed,stop_feedback_seen;
    // Accept an explicit rearm using the proof that existed BEFORE its IR
    // event. The event itself suppresses public confirmed and starts a NEW
    // STOP boundary, which still needs another complete zero-speed frame.
    // In vector mode the public STOP proof waits for the registered motion
    // stage to reach NONE. This is a conservative subset of target_stop, and
    // keeps raw camera/IR vector gates out of the consumers' reply paths.
    // Under this condition target_stop is already true, so the fresh-intent
    // test below is the exact simplified official_new_stop_intent expression.
    // UART STOP priority and the actual stop epoch still use target_stop.
    wire stop_report_target=follow_vector_mode ? effective_motion==MOT_NONE : target_stop;
    wire stop_report_new_intent=follow_vector_mode ?
        (MOTOR_OFFICIAL_PROTOCOL && (urgent_stop || !official_previous_target_stop ||
         official_link_lost || official_settle_complete || official_reinit_authorization_event)) :
        official_new_stop_intent;
    // Official operation uses local STOP-frame completion for handoff. Wheel
    // feedback is telemetry, not a command acknowledgement; residual measured
    // speed must not indefinitely block initialization or remote takeover.
    wire stop_boundary_complete=MOTOR_OFFICIAL_PROTOCOL ? stop_feedback_armed :
        (stop_feedback_confirmed && motor_speeds==0 && stop_feedback_age<STOP_FEEDBACK_CLKS);
    assign official_ready_rearm=motor_control_ready && stop_report_target &&
        stop_boundary_complete && motor_online && !official_stop_epoch_pending && !stop_pending;
    wire frame_launch_allowed=!(MOTOR_OFFICIAL_PROTOCOL || ENABLE_MAP_SCAN || ENABLE_NAVIGATION || ENABLE_FOLLOW) || frame_quiet_count>=MIN_SILENCE_CLKS;
    assign scan_stop_confirmed=(ENABLE_MAP_SCAN || ENABLE_NAVIGATION || ENABLE_FOLLOW) && stop_report_target && !ownership_stop_event && stop_boundary_complete &&
        motor_online &&
        (!MOTOR_OFFICIAL_PROTOCOL || (motor_control_ready && !stop_report_new_intent &&
         !official_stop_epoch_pending && !stop_pending));
    // Register positive stop proof, but retain the live invalidation veto.
    // Follow/thermal/ownership consumers do not all wait for a settling timer;
    // a new intent or link fault must never leave a stale public proof high.
    reg motor_stop_confirmed_q;
    always @(posedge clk50M or negedge reset_n)
        if(!reset_n)motor_stop_confirmed_q<=0;
        else motor_stop_confirmed_q<=scan_stop_confirmed;
    assign motor_stop_confirmed=motor_stop_confirmed_q&&scan_stop_confirmed;
    // Diagnostic only: no timeout or reason code relaxes the real STOP gate.
    assign stop_wait_reason=scan_stop_confirmed ? 4'd0 :
        !motor_online ? 4'd1 :
        MOTOR_OFFICIAL_PROTOCOL ? (!stop_feedback_armed ? 4'd3 : 4'd4) :
        motor_speeds!=0 ? 4'd2 :
        !stop_feedback_armed ? 4'd3 :
        stop_feedback_seen && stop_feedback_age>=STOP_FEEDBACK_CLKS ? 4'd5 : 4'd4;
    wire stop_retry_due=(ENABLE_MAP_SCAN || ENABLE_NAVIGATION || ENABLE_FOLLOW) && target_stop && stop_feedback_armed &&
        !scan_stop_confirmed && stop_retry_count>=STOP_RETRY_CLKS;

    // TX completion is local evidence, not proof of physical standstill.
    // Official-mode handoff uses this local evidence; legacy operation still
    // tracks fresh zero feedback. Periodic zero frames continue after STOP.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            frame_quiet_count<=MIN_SILENCE_CLKS;
            stop_retry_count<=0;stop_feedback_age<=STOP_FEEDBACK_CLKS;
            stop_feedback_armed<=0;stop_feedback_confirmed<=0;stop_feedback_seen<=0;
        end else begin
            if(frame_complete) frame_quiet_count<=0;
            else if(frame_quiet_count<MIN_SILENCE_CLKS) frame_quiet_count<=frame_quiet_count+1'b1;
            if(MOTOR_OFFICIAL_PROTOCOL) begin
                // Only a NEW stop intent invalidates the previous proof. The
                // 50ms zero-speed keepalive is not another stop epoch.
                if(!target_stop || official_new_stop_intent) begin
                    stop_retry_count<=0;stop_feedback_age<=STOP_FEEDBACK_CLKS;
                    stop_feedback_armed<=0;stop_feedback_confirmed<=0;stop_feedback_seen<=0;
                end else if(official_stop_epoch_complete) begin
                    stop_feedback_armed<=1;stop_feedback_confirmed<=0;stop_feedback_seen<=0;
                    stop_retry_count<=0;stop_feedback_age<=STOP_FEEDBACK_CLKS;
                end else begin
                    if(stop_complete) stop_retry_count<=0;
                    else if(stop_retry_count<STOP_RETRY_CLKS) stop_retry_count<=stop_retry_count+1'b1;
                    if(motor_feedback_update && stop_feedback_armed) begin
                        stop_feedback_seen<=1;stop_feedback_age<=0;
                        stop_feedback_confirmed<=motor_online && motor_speeds==0;
                    end else if(stop_feedback_age<STOP_FEEDBACK_CLKS)
                        stop_feedback_age<=stop_feedback_age+1'b1;
                    if(!motor_online || motor_speeds!=0) stop_feedback_confirmed<=0;
                end
            end else if(!(ENABLE_MAP_SCAN || ENABLE_NAVIGATION || ENABLE_FOLLOW) || !target_stop) begin
                stop_retry_count<=0;stop_feedback_age<=STOP_FEEDBACK_CLKS;
                stop_feedback_armed<=0;stop_feedback_confirmed<=0;stop_feedback_seen<=0;
            end else if(stop_complete) begin
                stop_feedback_armed<=1;stop_feedback_confirmed<=0;stop_feedback_seen<=0;
                stop_retry_count<=0;stop_feedback_age<=STOP_FEEDBACK_CLKS;
            end else if(ownership_stop_event) begin
                stop_feedback_armed<=0;stop_feedback_confirmed<=0;stop_feedback_seen<=0;
                stop_retry_count<=0;stop_feedback_age<=STOP_FEEDBACK_CLKS;
            end else begin
                if(stop_retry_count<STOP_RETRY_CLKS) stop_retry_count<=stop_retry_count+1'b1;
                if(motor_feedback_update && stop_feedback_armed) begin
                    stop_feedback_seen<=1;
                    stop_feedback_age<=0;
                    stop_feedback_confirmed<=motor_online && motor_speeds==0;
                end else if(stop_feedback_age<STOP_FEEDBACK_CLKS)
                    stop_feedback_age<=stop_feedback_age+1'b1;
                if(!motor_online || motor_speeds!=0) stop_feedback_confirmed<=0;
            end
        end
    end
    wire motion_not_started=(seq_state==SEQ_MOTION) && send_active &&
                            (byte_index==0) && !waiting_done;
    wire snapshot_obsolete=motion_not_started &&
                          (target_stop || stop_pending ||
                           frame_select!=effective_motion ||
                           frame_speed!=effective_speed ||
                           (MOTOR_OFFICIAL_PROTOCOL &&
                            (frame_vector_active!=effective_vector ||
                             (effective_vector && frame_wheels!=official_motion_wheels))));

    wire [3:0] motion_led={
        (running_motion==MOT_RIGHT_STRAFE || running_motion==MOT_ROTATE_RIGHT || running_motion==MOT_RIGHT_ARC || running_motion==MOT_DIAGONAL_RIGHT),
        (running_motion==MOT_LEFT_STRAFE || running_motion==MOT_ROTATE_LEFT || running_motion==MOT_LEFT_ARC || running_motion==MOT_DIAGONAL_LEFT),
        (running_motion==MOT_BACKWARD), (running_motion==MOT_FORWARD || running_motion==MOT_DIAGONAL_LEFT || running_motion==MOT_DIAGONAL_RIGHT)};
    // A monitor is optional: in latched avoidance HOLD, the four direction
    // lamps show the captured reason. Manual-mode direction lamps are intact.
    assign avoidance_hold=(avoidance_enabled && avoidance_state==6'd7) || safety_blocked ||
        (lidar_safety_active && !lidar_healthy) || (heading_required && !heading_ready);
    assign led=avoidance_hold ? avoidance_fault : motion_led;

    function [15:0] wheel_value;
        input [3:0] m;
        input [1:0] wheel;
        input [7:0] speed;
        integer motor_speed, inner_speed, outer_speed;
        begin
            // Snapshot speed remains a grade. Scale its base motor value before
            // rounding the existing 60%/140% arc differential.
            motor_speed=speed*MOTOR_SPEED_SCALE;
            inner_speed=(motor_speed*3+2)/5;
            outer_speed=(motor_speed*7+2)/5;
            if(motor_speed!=0 && inner_speed<1) inner_speed=1;
            wheel_value=16'h0000;
            case(m)
                MOT_FORWARD:      wheel_value=(wheel==0)?-motor_speed:motor_speed;
                MOT_BACKWARD:     wheel_value=(wheel==0)?motor_speed:-motor_speed;
                MOT_LEFT_STRAFE:  wheel_value=(wheel==3)?-motor_speed:motor_speed;
                MOT_RIGHT_STRAFE: wheel_value=(wheel==3)?motor_speed:-motor_speed;
                MOT_ROTATE_LEFT:  wheel_value=(wheel==2)?-motor_speed:motor_speed;
                MOT_ROTATE_RIGHT: wheel_value=(wheel==2)?motor_speed:-motor_speed;
                MOT_LEFT_ARC: begin
                    case(wheel)
                        0:wheel_value=-inner_speed;
                        1:wheel_value=outer_speed;
                        2:wheel_value=inner_speed;
                        3:wheel_value=outer_speed;
                    endcase
                end
                MOT_RIGHT_ARC: begin
                    case(wheel)
                        0:wheel_value=-outer_speed;
                        1:wheel_value=inner_speed;
                        2:wheel_value=outer_speed;
                        3:wheel_value=inner_speed;
                    endcase
                end
                default:wheel_value=16'h0000;
            endcase
        end
    endfunction

    // Modbus CRC16: reflected 0xA001 polynomial, initial value 0xFFFF.
    // Update once for each immutable payload byte; send low byte first.
    function [15:0] crc16_byte;
        input [15:0] crc;
        input [7:0] value;
        reg [15:0] c;
        integer bit_number;
        begin
            c=crc^{8'h00,value};
            for(bit_number=0;bit_number<8;bit_number=bit_number+1) begin
                if(c[0]) c=(c>>1)^16'hA001;
                else c=c>>1;
            end
            crc16_byte=c;
        end
    endfunction

    function [7:0] frame_byte;
        input [3:0] f;
        input [7:0] speed;
        input [5:0] i;
        input [15:0] crc;
        reg [1:0] wheel;
        reg [15:0] value;
        begin
            frame_byte=0;
            wheel=0;
            value=0;
            if(f==FRAME_ENABLE) begin
                case(i)
                    0:frame_byte=8'h0A; 1:frame_byte=8'h06; 2:frame_byte=8'h00;
                    3:frame_byte=8'h08; 4:frame_byte=8'h00; 5:frame_byte=8'h01;
                    6:frame_byte=8'hC8; 7:frame_byte=8'hB3;
                    default:frame_byte=0;
                endcase
            end else begin
                case(i)
                    0:frame_byte=8'h0A; 1:frame_byte=8'h10; 2:frame_byte=8'h00;
                    3:frame_byte=8'h00; 4:frame_byte=8'h00; 5:frame_byte=8'h04;
                    6:frame_byte=8'h08;
                    7,8,9,10,11,12,13,14: begin
                        case(i)
                            7,8:wheel=0;
                            9,10:wheel=1;
                            11,12:wheel=2;
                            default:wheel=3;
                        endcase
                        value=wheel_value(f,wheel,speed);
                        if(i[0]) frame_byte=value[15:8];
                        else frame_byte=value[7:0];
                    end
                    15:frame_byte=crc[7:0];
                    16:frame_byte=crc[15:8];
                    default:frame_byte=0;
                endcase
            end
        end
    endfunction

    // Every invocation in the small grade table has a literal argument, so
    // division is elaborated into constants instead of a 32-bit datapath.
    function [47:0] official_grade_constants;
        input [7:0] grade;
        integer base,inner,outer;
        begin
            base=grade*MOTOR_SPEED_SCALE;
            inner=(grade*MOTOR_SPEED_SCALE*3+2)/5;
            outer=(grade*MOTOR_SPEED_SCALE*7+2)/5;
            if(grade*MOTOR_SPEED_SCALE!=0 && inner<1) inner=1;
            official_grade_constants={base[15:0],inner[15:0],outer[15:0]};
        end
    endfunction

    function [47:0] official_grade_units;
        input [7:0] grade;
        begin
            case(grade)
                8'd0: official_grade_units=official_grade_constants(8'd0);
                8'd1: official_grade_units=official_grade_constants(8'd1);
                8'd2: official_grade_units=official_grade_constants(8'd2);
                8'd3: official_grade_units=official_grade_constants(8'd3);
                8'd4: official_grade_units=official_grade_constants(8'd4);
                8'd5: official_grade_units=official_grade_constants(8'd5);
                8'd6: official_grade_units=official_grade_constants(8'd6);
                8'd7: official_grade_units=official_grade_constants(8'd7);
                8'd8: official_grade_units=official_grade_constants(8'd8);
                8'd9: official_grade_units=official_grade_constants(8'd9);
                8'd10: official_grade_units=official_grade_constants(8'd10);
                default: official_grade_units=48'd0;
            endcase
        end
    endfunction

    function [63:0] official_scalar_wheels;
        input [3:0] m;
        input [47:0] units;
        reg [15:0] base,inner,outer;
        begin
            {base,inner,outer}=units;
            case(m)
                MOT_FORWARD: official_scalar_wheels={-base,base,base,base};
                MOT_BACKWARD: official_scalar_wheels={base,-base,-base,-base};
                MOT_LEFT_STRAFE: official_scalar_wheels={base,base,base,-base};
                MOT_RIGHT_STRAFE: official_scalar_wheels={-base,-base,-base,base};
                MOT_ROTATE_LEFT: official_scalar_wheels={base,base,-base,base};
                MOT_ROTATE_RIGHT: official_scalar_wheels={-base,-base,base,-base};
                MOT_LEFT_ARC: official_scalar_wheels={-inner,outer,inner,outer};
                MOT_RIGHT_ARC: official_scalar_wheels={-outer,inner,outer,inner};
                default: official_scalar_wheels=64'd0;
            endcase
        end
    endfunction

    // A coherent command tuple breaks grade arithmetic -> 64-bit comparison
    // -> serializer-enable timing. Raw STOP retains priority below; this one
    // clock stage can only defer an ordinary nonzero command update.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            official_motion_wheels<=0;official_payload_motion<=MOT_NONE;
            official_payload_speed<=0;official_payload_vector<=0;
        end else begin
            if(effective_vector)
                official_motion_wheels<=
                    {effective_wheel0,effective_wheel1,effective_wheel2,effective_wheel3};
            else if(SPEED_MAX<=10 && SPEED_INITIAL<=10 && SPEED_MIN<=10 &&
                    FOLLOW_SPEED_MAX<=10 && FOLLOW_ARC_LIMIT<=10)
                official_motion_wheels<=
                    official_scalar_wheels(effective_motion,official_grade_units(effective_speed));
            else
                // Compile-time fallback preserves extended-grade callers.
                official_motion_wheels<=
                    {wheel_value(effective_motion,2'd0,effective_speed),
                     wheel_value(effective_motion,2'd1,effective_speed),
                     wheel_value(effective_motion,2'd2,effective_speed),
                     wheel_value(effective_motion,2'd3,effective_speed)};
            official_payload_motion<=effective_motion;
            official_payload_speed<=effective_speed;
            official_payload_vector<=effective_vector;
        end
    end
    wire official_payload_current=official_payload_motion==effective_motion &&
        official_payload_speed==effective_speed && official_payload_vector==effective_vector;

    function [7:0] official_frame_byte;
        input [3:0] f;
        input [5:0] i;
        input [15:0] crc;
        input [63:0] wheels;
        begin
            if(f==FRAME_ENABLE || f==FRAME_STOP) official_frame_byte=frame_byte(f,8'd0,i,crc);
            else begin
                case(i)
                    7:official_frame_byte=wheels[63:56];
                    8:official_frame_byte=wheels[55:48];
                    9:official_frame_byte=wheels[47:40];
                    10:official_frame_byte=wheels[39:32];
                    11:official_frame_byte=wheels[31:24];
                    12:official_frame_byte=wheels[23:16];
                    13:official_frame_byte=wheels[15:8];
                    14:official_frame_byte=wheels[7:0];
                    default:official_frame_byte=frame_byte(FRAME_STOP,8'd0,i,crc);
                endcase
            end
        end
    endfunction

    generate if(MOTOR_OFFICIAL_PROTOCOL) begin: g_official_protocol
    if(FOLLOW_VECTOR_ENABLE) begin: g_payload_snapshot
    // Capture one coherent candidate while no byte has started. FRAME_STOP
    // encoding ignores these payload fields and always serializes zero; its
    // raw priority therefore need not drive this wide data-register enable.
    // The complete payload freezes once the first UART byte is launched.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin frame_wheels<=0;frame_speed<=0;end
        else if(!send_active || motion_not_started) begin
            frame_wheels<=official_motion_wheels;
            frame_speed<=official_payload_speed;
        end
    end
    end
    // Match the supplied STM32 example: configure closed-loop once, then
    // periodically write ONLY the four speeds. STOP still bypasses that period.
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            running_motion<=MOT_NONE; running_speed<=0;
            previous_effective_motion<=MOT_NONE;
            stop_pending<=1; seq_state<=SEQ_IDLE;
            frame_select<=FRAME_STOP; if(!FOLLOW_VECTOR_ENABLE) frame_speed<=0; frame_crc<=16'hFFFF;
            if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=0;frame_vector_active<=0;
            frame_length<=0; byte_index<=0;
            send_active<=0; waiting_done<=0;
            tx_start<=0; tx_data<=0; gap_count<=0;
            frame_crc_reset_pending<=0;
            official_phase<=OFFICIAL_BOOT_WAIT;
            official_wait_count<=0;official_refresh_count<=0;
            official_boot_stop_sent<=0;official_previous_online<=0;
            official_reinit_pending<=1;official_reinit_authorized<=0;official_reinit_started<=0;
            official_boot_mode_attempted<=0;
            official_recovery_zero_sent<=0;official_previous_target_stop<=1;
            official_stop_epoch_pending<=1;official_frame_starts_stop_epoch<=0;
        end else begin
            tx_start<=0;
            // Initialize CRC one clock after queuing a frame. The fixed header
            // launches before CRC is consumed by the first completed byte.
            frame_crc_reset_pending<=0;
            if(frame_crc_reset_pending) frame_crc<=16'hFFFF;
            previous_effective_motion<=effective_motion;
            official_previous_online<=motor_online;
            official_previous_target_stop<=target_stop;
            if(motion_frame_committed) official_refresh_count<=0;
            else if(official_refresh_count<OFFICIAL_REFRESH_CLKS)
                official_refresh_count<=official_refresh_count+1'b1;
            if(official_phase==OFFICIAL_BOOT_WAIT && official_boot_stop_sent) begin
                if(official_wait_count<OFFICIAL_STARTUP_CLKS)
                    official_wait_count<=official_wait_count+1'b1;
            end else if(official_phase==OFFICIAL_SETTLE) begin
                if(official_wait_count<OFFICIAL_SETTLE_CLKS)
                    official_wait_count<=official_wait_count+1'b1;
            end
            // Startup/reconnect use MODE plus the official settling delay.
            // The post-mode STOP is transmitted before motion, without waiting
            // for a later telemetry packet or exact measured zero speeds.
            if(official_reinit_authorization_event) begin
                official_reinit_authorized<=1;
                // An offline or already-started zero cannot prove that a newly
                // reporting board received STOP before reconfiguration.
                official_recovery_zero_sent<=0;
            end

            // A started frame is immutable, including after loss of feedback.
            // Only a queued frame with no first byte on the wire can be replaced.
            if(send_active && !snapshot_obsolete) begin
                if(!waiting_done && !tx_busy && (byte_index!=0 || frame_launch_allowed)) begin
                    tx_data<=official_frame_byte(frame_select,byte_index,frame_crc,frame_wheels);
                    tx_start<=1;
                    waiting_done<=1;
                end else if(waiting_done && tx_done) begin
                    waiting_done<=0;
                    // tx_data has been registered for the whole UART byte.
                    // Updating CRC here removes wheel/mux arithmetic from its
                    // input path without adding a gap between payload bytes.
                    if(!frame_crc_reset_pending && frame_select!=FRAME_ENABLE && byte_index<15)
                        frame_crc<=crc16_byte(frame_crc,tx_data);
                    if(byte_index>=frame_length-1'b1) begin
                        byte_index<=0;
                        send_active<=0;
                    end else byte_index<=byte_index+1'b1;
                end
            end

            case(seq_state)
                SEQ_IDLE: if(!send_active) begin
                    // Every new STOP intent already latches stop_pending; queued motion
                    // is separately cancelled by snapshot_obsolete before its first byte.
                    if(stop_pending) begin
                        stop_pending<=0;
                        frame_select<=FRAME_STOP;if(!FOLLOW_VECTOR_ENABLE) frame_speed<=0;frame_crc_reset_pending<=1;
                        if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=0;frame_vector_active<=0;
                        frame_length<=17;byte_index<=0;send_active<=1;seq_state<=SEQ_STOP;
                        official_frame_starts_stop_epoch<=official_stop_epoch_pending;
                        official_stop_epoch_pending<=0;
                    end else if(official_phase==OFFICIAL_BOOT_WAIT && official_boot_stop_sent &&
                                official_wait_count>=OFFICIAL_STARTUP_CLKS-1 &&
                                ((!official_boot_mode_attempted && !motor_online) ||
                                 (official_reinit_authorized && official_recovery_zero_sent && motor_online))) begin
                        frame_select<=FRAME_ENABLE;if(!FOLLOW_VECTOR_ENABLE) frame_speed<=0;frame_crc_reset_pending<=1;
                        if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=0;frame_vector_active<=0;
                        frame_length<=8;byte_index<=0;send_active<=1;seq_state<=SEQ_ENABLE;
                        official_frame_starts_stop_epoch<=0;
                        official_reinit_started<=official_reinit_pending;
                        official_boot_mode_attempted<=1;
                        official_phase<=OFFICIAL_SET_MODE;
                        // A board appearing halfway through these eight bytes
                        // has not necessarily received a complete MODE frame.
                        // Latch the unconfirmed bootstrap at launch, not merely
                        // from the online level at the end of the frame.
                        if(!motor_online) begin
                            official_reinit_pending<=1;
                            official_reinit_authorized<=0;official_reinit_started<=0;
                            official_recovery_zero_sent<=0;
                        end
                    end else if(official_phase==OFFICIAL_READY) begin
                        if(official_reinit_pending && official_reinit_authorized &&
                           official_recovery_zero_sent && motor_online) begin
                            official_phase<=OFFICIAL_BOOT_WAIT;official_wait_count<=0;
                        end else if((official_refresh_count>=OFFICIAL_REFRESH_CLKS ||
                                    (ENABLE_FLOW_BYPASS && avoidance_enabled && official_payload_motion!=running_motion)) &&
                                    (target_stop || official_payload_current)) begin
                            frame_select<=target_stop ? FRAME_STOP : official_payload_motion;
                            if(!FOLLOW_VECTOR_ENABLE) frame_speed<=target_stop ? 8'd0 : official_payload_speed;
                            if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=target_stop ? 64'd0 : official_motion_wheels;
                            frame_vector_active<=!target_stop && official_payload_vector;
                            frame_crc_reset_pending<=1;frame_length<=17;byte_index<=0;send_active<=1;
                            seq_state<=target_stop ? SEQ_STOP : SEQ_MOTION;
                            official_frame_starts_stop_epoch<=official_stop_epoch_pending && target_stop;
                            if(target_stop) official_stop_epoch_pending<=0;
                        end
                    end
                end
                SEQ_ENABLE: if(frame_complete) begin
                    official_phase<=OFFICIAL_SETTLE;official_wait_count<=0;
                    seq_state<=SEQ_IDLE;
                    // The motor may be powered after the FPGA. Keep the one
                    // offline bootstrap attempt (some boards only report after
                    // configuration), but its delivery is not evidence that an
                    // absent board entered closed-loop. A first fresh reply must
                    // authorize one new zero/mode/settle/zero handshake.
                    if(!motor_online) begin
                        official_reinit_pending<=1;
                        official_reinit_authorized<=0;official_reinit_started<=0;
                        official_recovery_zero_sent<=0;
                    end
                end
                SEQ_MOTION: begin
                    if(motion_not_started && snapshot_obsolete) begin
                        if(target_stop || stop_pending) begin
                            stop_pending<=0;
                            frame_select<=FRAME_STOP;if(!FOLLOW_VECTOR_ENABLE) frame_speed<=0;frame_crc_reset_pending<=1;
                            if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=0;frame_vector_active<=0;
                            frame_length<=17;byte_index<=0;send_active<=1;seq_state<=SEQ_STOP;
                            official_frame_starts_stop_epoch<=official_stop_epoch_pending;
                            official_stop_epoch_pending<=0;
                        end else begin
                            frame_select<=official_payload_motion;if(!FOLLOW_VECTOR_ENABLE) frame_speed<=official_payload_speed;
                            if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=official_motion_wheels;frame_vector_active<=official_payload_vector;
                            frame_crc_reset_pending<=1;
                        end
                    end else if(frame_complete) begin
                        running_motion<=frame_select;running_speed<=frame_speed;
                        if(target_stop || stop_pending) begin
                            stop_pending<=0;
                            frame_select<=FRAME_STOP;if(!FOLLOW_VECTOR_ENABLE) frame_speed<=0;frame_crc_reset_pending<=1;
                            if(!FOLLOW_VECTOR_ENABLE) frame_wheels<=0;frame_vector_active<=0;
                            frame_length<=17;byte_index<=0;send_active<=1;seq_state<=SEQ_STOP;
                            official_frame_starts_stop_epoch<=official_stop_epoch_pending;
                            official_stop_epoch_pending<=0;
                        end else seq_state<=SEQ_IDLE;
                    end
                end
                SEQ_STOP: if(frame_complete) begin
                    running_motion<=MOT_NONE;running_speed<=0;seq_state<=SEQ_IDLE;
                    official_boot_stop_sent<=1;
                    if(official_stop_epoch_complete && official_reinit_pending &&
                       official_reinit_authorized) official_recovery_zero_sent<=1;
                end
                default:seq_state<=SEQ_IDLE;
            endcase

            if(official_settle_complete) begin
                official_phase<=OFFICIAL_READY;official_wait_count<=0;
                // Invalidate any zero feedback from before changing the mode.
                stop_pending<=1;official_stop_epoch_pending<=1;
                if(!official_reinit_pending || official_reinit_started) begin
                    official_reinit_pending<=0;official_reinit_authorized<=0;
                    official_reinit_started<=0;
                end
            end
            if(official_new_stop_intent) begin
                // Even a zero frame already in flight predates this intent.
                stop_pending<=1;official_stop_epoch_pending<=1;
            end
            if(official_link_lost) begin
                official_reinit_pending<=1;official_reinit_authorized<=0;
                official_reinit_started<=0;official_recovery_zero_sent<=0;
            end
        end
    end
    end else begin: g_legacy_protocol
    always @(posedge clk50M or negedge reset_n) begin
        if(!reset_n) begin
            running_motion<=MOT_NONE; running_speed<=0;
            previous_effective_motion<=MOT_NONE;
            stop_pending<=1; seq_state<=SEQ_IDLE;
            frame_select<=FRAME_STOP; frame_speed<=0; frame_crc<=16'hFFFF;
            frame_length<=0; byte_index<=0;
            send_active<=0; waiting_done<=0;
            tx_start<=0; tx_data<=0; gap_count<=0;
        end else begin
            tx_start<=0;
            previous_effective_motion<=effective_motion;

            // Remember even a brief stop until an actual stop frame is sent.
            // A stop already being transmitted satisfies any new stop request.
            if(seq_state!=SEQ_STOP &&
               (ownership_stop_event || (effective_motion==MOT_NONE && previous_effective_motion!=MOT_NONE) ||
                (urgent_stop && (running_motion!=MOT_NONE ||
                                seq_state==SEQ_ENABLE || seq_state==SEQ_GAP ||
                                seq_state==SEQ_MOTION))))
                stop_pending<=1;

            if(send_active && !snapshot_obsolete) begin
                if(!waiting_done && !tx_busy && (byte_index!=0 || frame_launch_allowed)) begin
                    tx_data<=frame_byte(frame_select,frame_speed,byte_index,frame_crc);
                    tx_start<=1;
                    waiting_done<=1;
                    if(frame_select!=FRAME_ENABLE && byte_index<15)
                        frame_crc<=crc16_byte(frame_crc,
                            frame_byte(frame_select,frame_speed,byte_index,frame_crc));
                end else if(waiting_done && tx_done) begin
                    waiting_done<=0;
                    if(byte_index>=frame_length-1'b1) begin
                        byte_index<=0;
                        send_active<=0;
                    end else byte_index<=byte_index+1'b1;
                end
            end

            case(seq_state)
                SEQ_IDLE: begin
                    gap_count<=0;
                    if(!send_active) begin
                        if(stop_pending || (target_stop && running_motion!=MOT_NONE) || stop_retry_due) begin
                            stop_pending<=0;
                            frame_select<=FRAME_STOP; frame_speed<=0; frame_crc<=16'hFFFF;
                            frame_length<=17; byte_index<=0;
                            send_active<=1; seq_state<=SEQ_STOP;
                        end else if(!target_stop &&
                                    (effective_motion!=running_motion ||
                                     effective_speed!=running_speed || scan_refresh_due)) begin
                            frame_select<=FRAME_ENABLE; frame_speed<=0; frame_crc<=16'hFFFF;
                            frame_length<=8; byte_index<=0;
                            send_active<=1; seq_state<=SEQ_ENABLE;
                        end
                    end
                end
                SEQ_ENABLE: if(frame_complete) begin
                    gap_count<=0;
                    if(target_stop || stop_pending) begin
                        stop_pending<=0;
                        frame_select<=FRAME_STOP; frame_speed<=0; frame_crc<=16'hFFFF;
                        frame_length<=17; byte_index<=0;
                        send_active<=1; seq_state<=SEQ_STOP;
                    end else seq_state<=SEQ_GAP;
                end
                SEQ_GAP: begin
                    if(target_stop || stop_pending) begin
                        stop_pending<=0;
                        frame_select<=FRAME_STOP; frame_speed<=0; frame_crc<=16'hFFFF;
                        frame_length<=17; byte_index<=0;
                        send_active<=1; seq_state<=SEQ_STOP;
                    end else if(gap_count>=GAP_CYCLES-1) begin
                        gap_count<=0;
                        frame_select<=effective_motion; frame_speed<=effective_speed;
                        frame_crc<=16'hFFFF; frame_length<=17; byte_index<=0;
                        send_active<=1; seq_state<=SEQ_MOTION;
                    end else gap_count<=gap_count+1'b1;
                end
                SEQ_MOTION: begin
                    if(motion_not_started && snapshot_obsolete) begin
                        if(target_stop || stop_pending) begin
                            stop_pending<=0;
                            frame_select<=FRAME_STOP; frame_speed<=0;
                            frame_crc<=16'hFFFF; frame_length<=17; byte_index<=0;
                            send_active<=1; seq_state<=SEQ_STOP;
                        end else begin
                            frame_select<=effective_motion; frame_speed<=effective_speed;
                            frame_crc<=16'hFFFF;
                        end
                    end else if(frame_complete) begin
                        // Report what the frame actually contained, never a later request.
                        running_motion<=frame_select;
                        running_speed<=frame_speed;
                        if(target_stop || stop_pending) begin
                            stop_pending<=0;
                            frame_select<=FRAME_STOP; frame_speed<=0;
                            frame_crc<=16'hFFFF; frame_length<=17; byte_index<=0;
                            send_active<=1; seq_state<=SEQ_STOP;
                        end else seq_state<=SEQ_IDLE;
                    end
                end
                SEQ_STOP: if(frame_complete) begin
                    running_motion<=MOT_NONE; running_speed<=0;
                    seq_state<=SEQ_IDLE;
                end
                default:seq_state<=SEQ_IDLE;
            endcase
        end
    end
    end endgenerate
endmodule

// Four fully timed stages replace the donor's serial 32-bit reconstruct /
// turn clamp / forward clamp / lateral clamp / remix combinational chain.
// Signed division by four still truncates toward zero, including negatives.
module motor_guard_mix_pipeline #(parameter integer SPEED_MAX=20,SCALE=1)(
    input wire clk,reset_n,input wire [16:0] control_in,
    input wire smooth,allow_turn,input wire [63:0] raw_wheels,fallback_wheels,
    output wire [16:0] control_out,output wire [63:0] wheels_out
);
    wire signed [17:0] a={{2{raw_wheels[63]}},raw_wheels[63:48]};
    wire signed [17:0] b={{2{raw_wheels[47]}},raw_wheels[47:32]};
    wire signed [17:0] c={{2{raw_wheels[31]}},raw_wheels[31:16]};
    wire signed [17:0] d={{2{raw_wheels[15]}},raw_wheels[15:0]};
    wire signed [17:0] sf=(b-a)+(c+d),sl=(a+b)+(c-d),st=(c-a)-(b+d);
    function signed [17:0] quarter(input signed [17:0] value);
        begin quarter=(value+(value<0?18'sd3:18'sd0))>>>2;end
    endfunction
    reg signed [17:0] f1,l1,t1,budget1,f2,l2,t2,remaining2,f3,l3,t3,remaining3;
    wire signed [17:0] turn_cap=budget1<4 ? budget1>>>1 : 18'sd2;
    wire signed [17:0] limited_turn=t1>turn_cap ? turn_cap : t1< -turn_cap ? -turn_cap : t1;
    wire signed [17:0] limited_forward=f2>remaining2 ? remaining2 : f2< -remaining2 ? -remaining2 : f2;
    wire signed [17:0] limited_lateral=l3>remaining3 ? remaining3 : l3< -remaining3 ? -remaining3 : l3;
    reg [16:0] control_pipe[0:3];reg [63:0] fallback_pipe[0:3];reg [3:0] smooth_pipe;
    reg [63:0] mixed4;integer stage;
    wire signed [15:0] w0=-f3+limited_lateral-t3,w1=f3+limited_lateral-t3,
        w2=f3+limited_lateral+t3,w3=f3-limited_lateral-t3;
    assign control_out=control_pipe[3];
    assign wheels_out=control_pipe[3][16] ? 64'd0 : smooth_pipe[3] ? mixed4 : fallback_pipe[3];
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            f1<=0;l1<=0;t1<=0;budget1<=0;f2<=0;l2<=0;t2<=0;remaining2<=0;
            f3<=0;l3<=0;t3<=0;remaining3<=0;mixed4<=0;smooth_pipe<=0;
            for(stage=0;stage<4;stage=stage+1)begin control_pipe[stage]<=0;fallback_pipe[stage]<=0;end
        end else begin
            f1<=quarter(sf);l1<=quarter(sl);t1<=allow_turn?quarter(st):18'sd0;
            budget1<=(control_in[11:4]>SPEED_MAX?SPEED_MAX:control_in[11:4])*SCALE;
            f2<=f1;l2<=l1;t2<=limited_turn;
            remaining2<=budget1-(limited_turn<0?-limited_turn:limited_turn);
            f3<=limited_forward;l3<=l2;t3<=t2;
            remaining3<=remaining2-(limited_forward<0?-limited_forward:limited_forward);
            mixed4<={w0,w1,w2,w3};
            control_pipe[0]<=control_in;fallback_pipe[0]<=fallback_wheels;
            smooth_pipe<={smooth_pipe[2:0],smooth};
            for(stage=1;stage<4;stage=stage+1)begin
                control_pipe[stage]<=control_pipe[stage-1];fallback_pipe[stage]<=fallback_pipe[stage-1];
            end
        end
    end
endmodule
