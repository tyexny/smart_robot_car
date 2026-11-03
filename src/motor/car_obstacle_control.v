`timescale 1ns/1ps
// Production uses LiDAR body-edge clearances; no ultrasonic input is consulted.
// Dynamic radar mode seeks full forward-corridor clearance; the legacy mode
// retains its calibrated encoder-count extra margin for compatibility.
// A non-selected side or an unavailable rear sensor must not veto a safe
// lateral escape. Rear clearance remains mandatory for actual reverse travel.
module car_obstacle_control #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer LIDAR_POLICY=0,
    // Same simple stop/choose/strafe flow, but radar corridor clearance ends
    // the lateral leg. No encoder-count or fixed extra-distance requirement.
    parameter integer LIDAR_DYNAMIC_FINISH=0, SIDE_START_CM=30,
    parameter integer STOP_CM=12, AVOID_CM=20, SLOW_CM=25, CLEAR_CM=30,
    parameter integer SLOW_SPEED=3, MOVE_SPEED=5, TURN_SPEED=3, FINE_TURN_SPEED=2,
    parameter integer RIGHT_YAW_POSITIVE=0, ENABLE_REVERSE=1,
    // Legacy turn/return parameters remain compatible with named instantiations;
    // open-side bypasses now use lateral movement, with no heading restoration.
    parameter integer ENABLE_AUTO_SCAN=0, ENABLE_RETURN_TO_LINE=0,
    parameter integer ENABLE_YAW_DEVIATION_STOP=0, ENABLE_ENCODER_STOP=0,
    parameter integer SETTLE_MS=200, REVERSE_MS=300,
    parameter integer SCAN_MIN_MS=300, SCAN_MAX_MS=12000,
    parameter integer BYPASS_MS=1000, OFFSET_MS=BYPASS_MS,
    // Lateral-only empirical correction: old1104 measured about35cm in EXTRA.
    // 1104*20/35 ~=631; provisional20cm target requires on-car recheck.
    parameter integer OFFSET_TARGET_COUNTS=631,
    parameter integer PASS_MS=1500, PASS_RETRY_MS=1000,
    parameter integer RETURN_MAX_MS=8000, MAX_RETURN_RETRIES=2,
    parameter integer SCAN_ANGLE=600, ANGLE_TOLERANCE=30, FINE_ANGLE=250,
    parameter integer YAW_MAX_STEP=450, IMU_STALE_MS=500, SONAR_RETRY_MS=500,
    parameter integer SIDE_NEAR_CM=6, SIDE_NUDGE_ANGLE=100,
    parameter integer SIDE_MAX_ANGLE=900, SIDE_SAMPLE_MS=1500,
    parameter integer TURN_NO_PROGRESS_MS=1000, TURN_PROGRESS_TENTHS=10
)(
    input wire clk, reset_n, avoidance_enable,
    input wire [3:0] requested_motion,
    input wire [7:0] requested_speed,
    input wire command_update,
    input wire [15:0] front_distance_cm, rear_distance_cm,
    input wire front_valid, rear_valid, front_update, rear_update,
    input wire [15:0] left_distance_cm, right_distance_cm,
    input wire left_valid, right_valid, left_update, right_update,
    input wire signed [31:0] imu_yaw,
    input wire imu_yaw_valid, imu_yaw_update,
    input wire travel_valid, travel_update, travel_fault,
    input wire [17:0] travel_step,
    input wire [3:0] actual_motion,
    input wire maneuver_stop_confirmed,
    output reg [3:0] effective_motion,
    output reg [7:0] effective_speed,
    output wire [5:0] avoidance_state,
    output reg [3:0] avoidance_fault
);
    localparam [3:0] NONE=0, FORWARD=1, BACKWARD=2, LEFT_STRAFE=3,
        RIGHT_STRAFE=4, ROTATE_LEFT=5, ROTATE_RIGHT=6, LEFT_ARC=7, RIGHT_ARC=8;
    localparam [5:0] IDLE=0, HALT=1, REVERSE=2, POST_REVERSE=3,
        HOLD=7, SIDE_CHOOSE=29, UTURN=30, UTURN_SETTLE=31,
        SIDE_SHIFT=35, SHIFT_STOP=36, SIDE_EXTRA=37;
    localparam integer MS_CLKS=(CLK_FREQ_HZ/1000<1)?1:CLK_FREQ_HZ/1000;
    localparam integer SETTLE_CLKS=(SETTLE_MS<1)?1:MS_CLKS*SETTLE_MS;
    localparam integer REVERSE_CLKS=(REVERSE_MS<1)?1:MS_CLKS*REVERSE_MS;
    localparam integer TURN_CLKS=(SCAN_MAX_MS<1)?1:MS_CLKS*SCAN_MAX_MS;
    localparam integer MOVE_CLKS=(RETURN_MAX_MS<1)?1:MS_CLKS*RETURN_MAX_MS;
    localparam integer IMU_CLKS=(IMU_STALE_MS<1)?1:MS_CLKS*IMU_STALE_MS;
    localparam integer SONAR_RETRY_CLKS=(SONAR_RETRY_MS<1)?1:MS_CLKS*SONAR_RETRY_MS;
    localparam integer SIDE_SAMPLE_CLKS=(SIDE_SAMPLE_MS<1)?1:MS_CLKS*SIDE_SAMPLE_MS;
    localparam integer NO_PROGRESS_CLKS=(TURN_NO_PROGRESS_MS<1)?1:MS_CLKS*TURN_NO_PROGRESS_MS;
    localparam integer SIDE_OPEN_CM=LIDAR_DYNAMIC_FINISH?SIDE_START_CM:CLEAR_CM;
    localparam integer SIDE_VOTES=LIDAR_DYNAMIC_FINISH?1:2;
    function signed [15:0] normalize_yaw;
        input signed [31:0] a;
        begin
            if(a>=1800) normalize_yaw=a-3600;
            else if(a< -1800) normalize_yaw=a+3600;
            else normalize_yaw=a[15:0];
        end
    endfunction
    function signed [15:0] angle_difference;
        input signed [15:0] a,b;
        reg signed [16:0] d;
        begin
            d=$signed({a[15],a})-$signed({b[15],b});
            if(d>1800) angle_difference=d-17'sd3600;
            else if(d< -1800) angle_difference=d+17'sd3600;
            else angle_difference=d[15:0];
        end
    endfunction
    function signed [15:0] add_angle;
        input signed [15:0] a,b;
        reg signed [16:0] s;
        begin
            s=$signed({a[15],a})+$signed({b[15],b});
            if(s>=1800) add_angle=s-17'sd3600;
            else if(s< -1800) add_angle=s+17'sd3600;
            else add_angle=s[15:0];
        end
    endfunction
    function [15:0] abs_angle;
        input signed [15:0] a;
        begin abs_angle=a[15] ? -a : a; end
    endfunction
    reg [5:0] state;
    reg [3:0] previous_request;
    reg [31:0] state_count, stopped_count, motion_count, probe_wait_count;
    reg [31:0] outward_distance, shift_elapsed_count;
    reg [1:0] front_clear_samples;
    reg shift_restart;
    reg choose_right, right_clear, left_clear;
    reg [1:0] right_sample_count, left_sample_count;
    reg front_slow;
    reg signed [15:0] yaw, last_yaw, target_yaw, progress_anchor;
    reg yaw_in_range, yaw_stage_valid, yaw_stage_update, yaw_seen;
    reg [31:0] yaw_age, progress_count;
    reg signed [15:0] sampled_turn_target, sampled_turn_error;
    reg turn_sample_seen;

    // Legacy yaw/UTURN code remains compatible; no automatic entry reaches it.
    // Ordinary forward motion and lateral bypass never command yaw correction.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            yaw<=0; yaw_in_range<=0; yaw_stage_valid<=0; yaw_stage_update<=0;
        end else begin
            yaw<=normalize_yaw(imu_yaw);
            yaw_in_range<=imu_yaw>=-32'sd3600 && imu_yaw<32'sd3600;
            yaw_stage_valid<=imu_yaw_valid; yaw_stage_update<=imu_yaw_update;
        end
    end
    wire yaw_jump=yaw_stage_update && yaw_seen &&
        abs_angle(angle_difference(yaw,last_yaw))>YAW_MAX_STEP;
    wire yaw_good=imu_yaw_valid && yaw_stage_valid && yaw_in_range &&
        yaw_seen && yaw_age<IMU_CLKS && !yaw_jump;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin yaw_seen<=0; yaw_age<=IMU_CLKS; last_yaw<=0; end
        else begin
            if(yaw_age<IMU_CLKS) yaw_age<=yaw_age+1'b1;
            if(yaw_stage_update) begin
                last_yaw<=yaw;
                if(yaw_stage_valid && yaw_in_range && !yaw_jump) begin yaw_seen<=1; yaw_age<=0; end
                else begin yaw_seen<=0; yaw_age<=IMU_CLKS; end
            end
        end
    end

    wire probes_ready=front_valid && (LIDAR_POLICY || rear_valid);
    wire sides_ready=left_valid && right_valid;
    // Upstream flags expire stale/incomplete scans. Stopping uses
    // the latest valid three directions immediately; no extra vote delay.
    wire three_sides_blocked=front_valid && sides_ready &&
        front_distance_cm<=AVOID_CM && left_distance_cm<SIDE_OPEN_CM && right_distance_cm<SIDE_OPEN_CM;
    wire side_phase=state==SIDE_CHOOSE || state==SIDE_SHIFT || state==SIDE_EXTRA ||
        state==SHIFT_STOP || state==UTURN || state==UTURN_SETTLE;
    wire selected_side_valid=choose_right?right_valid:left_valid;
    wire sensors_ready=LIDAR_POLICY ?
        (state==REVERSE ? rear_valid :
         (state==SIDE_SHIFT || state==SIDE_EXTRA) ? front_valid && selected_side_valid :
         state==SIDE_CHOOSE ? front_valid && (left_valid || right_valid) : front_valid) :
        probes_ready && (!side_phase || sides_ready);
    wire left_choice_ready=left_valid && left_sample_count>=SIDE_VOTES &&
        !(left_update && left_clear!=(left_distance_cm>=SIDE_OPEN_CM));
    wire right_choice_ready=right_valid && right_sample_count>=SIDE_VOTES &&
        !(right_update && right_clear!=(right_distance_cm>=SIDE_OPEN_CM));
    wire left_choice_open=left_choice_ready && left_clear;
    wire right_choice_open=right_choice_ready && right_clear;
    // Proximity gates now apply only to requested manual motions in IDLE.
    wire turn_clear=probes_ready && front_distance_cm>STOP_CM;
    wire selected_side_safe=choose_right?(right_valid && right_distance_cm>STOP_CM):
        (left_valid && left_distance_cm>STOP_CM);
    wire encoder_ready=travel_valid && !travel_fault;
    wire distance_update=travel_update && encoder_ready;
    // The caller confirms new zero-speed feedback received after this STOP.
    // A UART frame completing locally does not by itself prove the car stopped.
    wire stopped=actual_motion==NONE && maneuver_stop_confirmed;
    wire shift_phase=state==SIDE_SHIFT || state==SIDE_EXTRA || state==SHIFT_STOP;
    wire shift_moving_phase=state==SIDE_SHIFT || state==SIDE_EXTRA;
    wire extra_count_phase=state==SIDE_EXTRA || state==SHIFT_STOP;
    wire front_new_blocked=front_update && front_valid && front_distance_cm<=AVOID_CM;
    wire front_corridor_clear=front_valid && front_distance_cm>=CLEAR_CM;
    wire radar_finish_event=front_update && front_corridor_clear;
    wire shift_time_expired=shift_elapsed_count>=MOVE_CLKS-1;
    wire return_phase=state==UTURN || state==UTURN_SETTLE;
    wire automatic_phase=state!=IDLE && state!=HOLD;
    wire cancel_maneuver=requested_motion!=previous_request ||
        (command_update && (requested_motion!=FORWARD || state==HOLD));
    wire [7:0] limited_speed=(requested_speed<SLOW_SPEED)?requested_speed:SLOW_SPEED;
    localparam integer LATERAL_SPEED_LIMIT=LIDAR_DYNAMIC_FINISH?SLOW_SPEED:MOVE_SPEED;
    wire [7:0] move_speed=(requested_speed<LATERAL_SPEED_LIMIT)?requested_speed:LATERAL_SPEED_LIMIT;
    wire [7:0] coarse_turn_speed=(requested_speed<TURN_SPEED)?requested_speed:TURN_SPEED;
    wire [7:0] fine_turn_speed=(requested_speed<FINE_TURN_SPEED)?requested_speed:FINE_TURN_SPEED;
    wire signed [15:0] target_error=angle_difference(target_yaw,yaw);
    wire at_target=abs_angle(target_error)<=ANGLE_TOLERANCE;
    wire crossed_target=yaw_stage_update && turn_sample_seen && state==UTURN &&
        sampled_turn_target==target_yaw &&
        sampled_turn_error[15]!=target_error[15] &&
        ({1'b0,abs_angle(sampled_turn_error)}+{1'b0,abs_angle(target_error)})<=YAW_MAX_STEP;
    wire turn_arrived=at_target || crossed_target;
    wire [7:0] turn_speed=abs_angle(target_error)<=FINE_ANGLE ? fine_turn_speed : coarse_turn_speed;
    wire turn_progress=yaw_stage_update && yaw_good &&
        abs_angle(angle_difference(yaw,progress_anchor))>=TURN_PROGRESS_TENTHS;
    wire offset_calibrated=OFFSET_TARGET_COUNTS>0;
    wire offset_done=offset_calibrated && outward_distance>=OFFSET_TARGET_COUNTS;
    wire [31:0] outward_after_update=outward_distance+{14'd0,travel_step};
    assign avoidance_state=state;

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            sampled_turn_target<=0; sampled_turn_error<=0; turn_sample_seen<=0;
            progress_anchor<=0; progress_count<=0;
        end else begin
            if(state!=UTURN) begin
                turn_sample_seen<=0; progress_anchor<=yaw; progress_count<=0;
            end else begin
                if(yaw_stage_update) begin
                    sampled_turn_target<=target_yaw; sampled_turn_error<=target_error;
                    turn_sample_seen<=1;
                end
                if(turn_progress) begin progress_anchor<=yaw; progress_count<=0; end
                else if(actual_motion==ROTATE_RIGHT && progress_count<NO_PROGRESS_CLKS)
                    progress_count<=progress_count+1'b1;
            end
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin front_slow<=0; end
        else if(!avoidance_enable) begin front_slow<=0; end
        else begin
            if(front_valid) begin
                if(front_distance_cm<=SLOW_CM) front_slow<=1;
                else if(front_distance_cm>=CLEAR_CM) front_slow<=0;
            end
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE; previous_request<=NONE;
            state_count<=0; stopped_count<=0; motion_count<=0; probe_wait_count<=0;
            avoidance_fault<=0; choose_right<=1; outward_distance<=0; target_yaw<=0;
            shift_elapsed_count<=0; front_clear_samples<=0; shift_restart<=0;
            right_clear<=0; left_clear<=0; right_sample_count<=0; left_sample_count<=0;
        end else begin
            previous_request<=requested_motion;
            if(!avoidance_enable || requested_motion==NONE || cancel_maneuver) begin
                state<=IDLE; state_count<=0; stopped_count<=0; motion_count<=0;
                probe_wait_count<=0; avoidance_fault<=0; choose_right<=1; outward_distance<=0;
                shift_elapsed_count<=0; front_clear_samples<=0; shift_restart<=0;
                right_sample_count<=0; left_sample_count<=0; right_clear<=0; left_clear<=0;
            end else begin
                if(sensors_ready || state==IDLE || state==HOLD) probe_wait_count<=0;
                else if(probe_wait_count<SONAR_RETRY_CLKS) probe_wait_count<=probe_wait_count+1'b1;
                if(state_count!=32'hffffffff) state_count<=state_count+1'b1;
                if(stopped) begin
                    if(stopped_count!=32'hffffffff) stopped_count<=stopped_count+1'b1;
                end else stopped_count<=0;
                if(state==REVERSE && actual_motion==BACKWARD && motion_count!=32'hffffffff)
                    motion_count<=motion_count+1'b1;
                // One cumulative budget across all seek/extra retries. STOP has
                // its own confirmation timeout and cannot reset this drive budget.
                if(shift_moving_phase && shift_elapsed_count<MOVE_CLKS)
                    shift_elapsed_count<=shift_elapsed_count+1'b1;
                if(state!=SIDE_SHIFT) front_clear_samples<=0;
                // Count only after a fresh post-clear baseline, including STOP/coast.
                if(distance_update && extra_count_phase) outward_distance<=outward_after_update;
                if(state==SHIFT_STOP && front_new_blocked) shift_restart<=1;

                if(state!=HOLD && requested_motion==FORWARD && three_sides_blocked &&
                        (!LIDAR_DYNAMIC_FINISH || state==IDLE || state==SIDE_CHOOSE)) begin
                    // Normal blocked STOP, latched until a fresh motion/STOP command.
                    // Preserve previously latched faults by excluding HOLD above.
                    state<=HOLD; avoidance_fault<=0; state_count<=0;
                    stopped_count<=0; motion_count<=0; probe_wait_count<=0;
                    front_clear_samples<=0; outward_distance<=0; shift_restart<=0;
                end else if(automatic_phase && return_phase && !yaw_good) begin
                    state<=HOLD; state_count<=0; avoidance_fault<=4'd4;
                end else if(!LIDAR_DYNAMIC_FINISH && automatic_phase && shift_phase && !encoder_ready) begin
                    state<=HOLD; state_count<=0; avoidance_fault<=4'd8;
                end else if(LIDAR_POLICY && automatic_phase &&
                        ((state==REVERSE && rear_valid && rear_distance_cm<=STOP_CM) ||
                         (shift_moving_phase && !selected_side_safe))) begin
                    // A newly appeared rear/side obstacle always wins over a
                    // timed reverse or the empirical extra-distance target.
                    state<=HOLD;state_count<=0;avoidance_fault<=4'd9;
                end else if(automatic_phase && !sensors_ready) begin
                    if(state==SIDE_CHOOSE) begin right_sample_count<=0; left_sample_count<=0; end
                    if(state==SIDE_SHIFT || state==SIDE_EXTRA) begin
                        // Unknown data cannot preserve a front-clear decision.
                        front_clear_samples<=0; outward_distance<=0;
                        if(state==SIDE_EXTRA) state<=SIDE_SHIFT;
                    end
                    if(state==SHIFT_STOP) shift_restart<=1;
                    if(probe_wait_count>=SONAR_RETRY_CLKS-1) begin
                        state<=HOLD; state_count<=0;
                        avoidance_fault<=!front_valid ? 4'd1 : !rear_valid ? 4'd2 : 4'd5;
                    end
                end else case(state)
                    IDLE: begin
                        state_count<=0; stopped_count<=0; motion_count<=0;
                        if(requested_motion==FORWARD && front_valid && front_distance_cm<=AVOID_CM) begin
                            state<=HALT; choose_right<=1; outward_distance<=0;
                            right_sample_count<=0; left_sample_count<=0;
                        end
                    end
                    HALT: begin
                        if(stopped && stopped_count>=SETTLE_CLKS-1) begin
                            state_count<=0; stopped_count<=0; motion_count<=0;
                            if(ENABLE_REVERSE!=0 && rear_valid && rear_distance_cm>=CLEAR_CM) state<=REVERSE;
                            else state<=POST_REVERSE;
                        end else if(state_count>=TURN_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd3; end
                    end
                    REVERSE: begin
                        if(motion_count>=REVERSE_CLKS-1) begin
                            state<=POST_REVERSE; state_count<=0; stopped_count<=0; motion_count<=0;
                        end else if(state_count>=MOVE_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd12; end
                    end
                    POST_REVERSE: begin
                        if(stopped && stopped_count>=SETTLE_CLKS-1) begin
                            state<=SIDE_CHOOSE; state_count<=0;
                            right_sample_count<=0; left_sample_count<=0;
                        end else if(state_count>=TURN_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd3; end
                    end
                    SIDE_CHOOSE: begin
                        // Fresh vote(s) on a usable side, only while
                        // physically stopped. An unknown non-selected side is
                        // not a reason to reject an independently checked escape.
                        if(!stopped || stopped_count<SETTLE_CLKS-1) begin
                            right_sample_count<=0; left_sample_count<=0;
                            if(state_count>=TURN_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd3; end
                        end else begin
                            if(!left_valid) left_sample_count<=0;
                            else if(left_update) begin
                                left_clear<=left_distance_cm>=SIDE_OPEN_CM;
                                if(left_sample_count==0 || left_clear!=(left_distance_cm>=SIDE_OPEN_CM))
                                    left_sample_count<=1;
                                else left_sample_count<=2;
                            end
                            if(!right_valid) right_sample_count<=0;
                            else if(right_update) begin
                                right_clear<=right_distance_cm>=SIDE_OPEN_CM;
                                if(right_sample_count==0 || right_clear!=(right_distance_cm>=SIDE_OPEN_CM))
                                    right_sample_count<=1;
                                else right_sample_count<=2;
                            end
                            if(LIDAR_POLICY ? (left_choice_open || right_choice_open ||
                                                (left_choice_ready && right_choice_ready)) :
                                               (left_choice_ready && right_choice_ready)) begin
                                if(right_choice_open || left_choice_open) begin
                                    if(!LIDAR_DYNAMIC_FINISH && !offset_calibrated) begin state<=HOLD; avoidance_fault<=4'd12; end
                                    else if(!LIDAR_DYNAMIC_FINISH && !encoder_ready) begin state<=HOLD; avoidance_fault<=4'd8; end
                                    else if(LIDAR_DYNAMIC_FINISH || distance_update) begin
                                        // Discard the just-received stopped interval,
                                        // then count only the lateral leg and its coast.
                                        choose_right<=right_choice_open && (!LIDAR_DYNAMIC_FINISH ||
                                            !left_choice_open || right_distance_cm>=left_distance_cm);
                                        outward_distance<=0;
                                        shift_elapsed_count<=0; front_clear_samples<=0; shift_restart<=0;
                                        state<=SIDE_SHIFT; state_count<=0; stopped_count<=0;
                                    end else if(state_count>=SIDE_SAMPLE_CLKS-1) begin
                                        state<=HOLD; avoidance_fault<=4'd8;
                                    end
                                end else if(front_distance_cm>AVOID_CM) begin
                                    // The front has cleared while physically stopped.
                                    state<=IDLE; state_count<=0; stopped_count<=0;
                                end else begin
                                    state<=HOLD; avoidance_fault<=0;
                                    state_count<=0; stopped_count<=0;
                                end
                            end else if(state_count>=SIDE_SAMPLE_CLKS-1) begin
                                state<=HOLD; avoidance_fault<=4'd5;
                            end
                        end
                    end
                    SIDE_SHIFT: begin
                        if((!LIDAR_DYNAMIC_FINISH && !offset_calibrated) || shift_time_expired) begin
                            state<=HOLD; avoidance_fault<=4'd12;
                        end else if(LIDAR_DYNAMIC_FINISH && radar_finish_event) begin
                            // The complete padded forward footprint is clear.
                            // STOP here: no arbitrary extra 20cm/631-count leg.
                            state<=SHIFT_STOP;state_count<=0;stopped_count<=0;shift_restart<=0;
                        end else begin
                            if(front_update) begin
                                if(front_distance_cm>AVOID_CM) begin
                                    if(front_clear_samples<2) front_clear_samples<=front_clear_samples+1'b1;
                                end else front_clear_samples<=0;
                            end
                            if(!LIDAR_DYNAMIC_FINISH && front_clear_samples==2 && !front_new_blocked &&
                                front_distance_cm>AVOID_CM && distance_update) begin
                                // Discard this whole interval; it may contain travel
                                // before the latest clear confirmation. Count the next.
                                outward_distance<=0; shift_restart<=0;
                                state<=SIDE_EXTRA;
                            end
                        end
                    end
                    SIDE_EXTRA: begin
                        if(!offset_calibrated || shift_time_expired) begin
                            state<=HOLD; avoidance_fault<=4'd12;
                        end else if(front_new_blocked) begin
                            // New blockage wins over reaching the margin this clock.
                            state<=SIDE_SHIFT; front_clear_samples<=0; outward_distance<=0;
                        end else if(offset_done || (distance_update && outward_after_update>=OFFSET_TARGET_COUNTS)) begin
                            state<=SHIFT_STOP; state_count<=0; stopped_count<=0; shift_restart<=0;
                        end
                    end
                    SHIFT_STOP: begin
                        if(stopped && stopped_count>=SETTLE_CLKS-1 && (LIDAR_DYNAMIC_FINISH || distance_update)) begin
                            if(shift_restart || (LIDAR_DYNAMIC_FINISH?!front_corridor_clear:
                                    front_distance_cm<=AVOID_CM || front_new_blocked)) begin
                                // Complete the real STOP before returning to same-side seek.
                                state<=SIDE_SHIFT; front_clear_samples<=0; outward_distance<=0;
                                shift_restart<=0; state_count<=0; stopped_count<=0;
                            end else begin state<=IDLE; state_count<=0; stopped_count<=0; end
                        end else if(state_count>=TURN_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd12; end
                    end
                    UTURN: begin
                        if(turn_arrived && yaw_stage_update) begin
                            state<=UTURN_SETTLE; state_count<=0; stopped_count<=0;
                        end else if(state_count>=TURN_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd3; end
                        else if(actual_motion==ROTATE_RIGHT && progress_count>=NO_PROGRESS_CLKS-1 && !turn_progress) begin
                            state<=HOLD; avoidance_fault<=4'd13;
                        end
                    end
                    UTURN_SETTLE: begin
                        if(stopped && stopped_count>=SETTLE_CLKS-1 && yaw_stage_update) begin
                            state<=IDLE; state_count<=0; stopped_count<=0;
                        end else if(state_count>=TURN_CLKS-1) begin state<=HOLD; avoidance_fault<=4'd3; end
                    end
                    HOLD: begin state_count<=0; stopped_count<=0; motion_count<=0; end
                    default: begin state<=HOLD; avoidance_fault<=4'd15; end
                endcase
            end
        end
    end

    always @(*) begin
        effective_motion=NONE; effective_speed=0;
        if(!avoidance_enable) begin
            effective_motion=requested_motion; effective_speed=requested_motion==NONE ? 8'd0 : requested_speed;
        end else if(requested_motion==FORWARD && three_sides_blocked &&
                (!LIDAR_DYNAMIC_FINISH || state==IDLE || state==SIDE_CHOOSE)) begin
            // Remove drive immediately, including an in-progress pre-reverse
            // or lateral leg, without waiting for the sequential HOLD entry.
            effective_motion=NONE; effective_speed=0;
        end else if(requested_motion!=NONE && requested_speed!=0 && !(cancel_maneuver && state!=IDLE)) begin
            if(state==IDLE) begin
                case(requested_motion)
                    // No automatic heading correction: an unobstructed forward
                    // request remains forward, even with stale or unavailable IMU data.
                    FORWARD: if(front_valid && front_distance_cm>AVOID_CM) begin
                        effective_motion=FORWARD;
                        effective_speed=(front_slow || front_distance_cm<=SLOW_CM) ? limited_speed : requested_speed;
                    end
                    LEFT_ARC, RIGHT_ARC: if(front_valid && front_distance_cm>STOP_CM) begin
                        effective_motion=requested_motion;
                        effective_speed=(front_slow || front_distance_cm<=SLOW_CM) ? limited_speed : requested_speed;
                    end
                    BACKWARD: if(rear_valid && (!LIDAR_POLICY || rear_distance_cm>STOP_CM)) begin
                        effective_motion=BACKWARD;
                        effective_speed=requested_speed;
                    end
                    LEFT_STRAFE, RIGHT_STRAFE, ROTATE_LEFT, ROTATE_RIGHT: if(LIDAR_POLICY ?
                        (requested_motion==LEFT_STRAFE?(left_valid && left_distance_cm>STOP_CM):
                         requested_motion==RIGHT_STRAFE?(right_valid && right_distance_cm>STOP_CM):1'b1):turn_clear) begin
                        effective_motion=requested_motion;
                        effective_speed=!LIDAR_POLICY && (front_slow || front_distance_cm<=SLOW_CM) ?
                            limited_speed : requested_speed;
                    end
                    default: effective_motion=NONE;
                endcase
            end else if(sensors_ready) begin
                case(state)
                    REVERSE: begin
                        if(!LIDAR_POLICY || (rear_valid && rear_distance_cm>STOP_CM)) begin
                            effective_motion=BACKWARD; effective_speed=limited_speed;
                        end
                    end
                    SIDE_SHIFT, SIDE_EXTRA: if((LIDAR_DYNAMIC_FINISH || (encoder_ready && offset_calibrated)) && !shift_time_expired &&
                        (!LIDAR_POLICY || selected_side_safe) &&
                         !(LIDAR_DYNAMIC_FINISH && state==SIDE_SHIFT && radar_finish_event) &&
                        (state==SIDE_SHIFT || !offset_done || front_new_blocked)) begin
                        effective_motion=choose_right ? RIGHT_STRAFE : LEFT_STRAFE;
                        effective_speed=move_speed;
                    end
                    UTURN: if(yaw_good && !turn_arrived &&
                        !(actual_motion==ROTATE_RIGHT && progress_count>=NO_PROGRESS_CLKS-1 && !turn_progress)) begin
                        effective_motion=ROTATE_RIGHT; effective_speed=turn_speed;
                    end
                    default: effective_motion=NONE;
                endcase
            end
        end
        if(effective_motion==NONE) effective_speed=0;
    end
endmodule
