`timescale 1ns/1ps
// Image-only following uses lateral or optional turn correction; ownership is external.
// New detections update a locked geometric target, never rearm LOST/BLOCKED.
module vision_follow_control #(
    parameter integer CLK_FREQ_HZ=50_000_000, MAX_BOXES=8,
    parameter integer TARGET_TIMEOUT_MS=400, FOLLOW_SPEED_MAX=2,
    parameter integer REF_HEIGHT_INITIAL=160, HEIGHT_STEP=8,
    parameter integer CENTER_ENTER_PX=80, CENTER_EXIT_PX=40,
    parameter integer IMAGE_WIDTH=1280, IMAGE_HEIGHT=720,
    parameter integer MIN_TARGET_HEIGHT=16, REF_HEIGHT_MAX=640,
    parameter integer HEIGHT_DEADZONE_MIN=4, HEIGHT_DEADZONE_SHIFT=4,
    parameter integer MAX_CENTER_DELTA_PX=160, MAX_SIZE_DELTA_PX=160,
    parameter integer MATCH_AMBIGUITY_PX=24,
    parameter integer TRACK_ID_ENABLE=0, TARGET_RETAIN_MS=600,
    parameter integer TRACK_CONFIDENCE_MIN=350,
    parameter [15:0] APPROACH_STOP_CM=16'd30,
    parameter integer FOLLOW_TURN_ENABLE=0,
    parameter integer FOLLOW_TURN_SPEED_MAX=FOLLOW_SPEED_MAX,
    parameter integer FOLLOW_CURVE_ENABLE=0, FOLLOW_ARC_SPEED_MAX=FOLLOW_SPEED_MAX,
    parameter integer ALIGN_ENTER_PX=320, ALIGN_EXIT_PX=256,
    parameter integer FOLLOW_SERVO_ENABLE=0, FOLLOW_REVERSE_ENABLE=0,
    parameter integer FOLLOW_PROXIMITY_ENABLE=1, FOLLOW_WIDTH_FALLBACK_ENABLE=0,
    parameter integer TARGET_EDGE_MARGIN_PX=8,
    parameter integer TURN_PULSE_MS=80, TURN_NEAR_PULSE_MS=40,
    parameter integer TURN_TIMER_FROM_TX=0, TURN_TX_WAIT_MS=200,
    parameter integer FOLLOW_VECTOR_ENABLE=0, VECTOR_SPEED_SCALE=2,
    parameter integer VECTOR_MIN_LINEAR=4, VECTOR_MIN_TURN=2, VECTOR_MIN_PIVOT=4,
    parameter integer VECTOR_ACCEL_STEP=2, VECTOR_OBSERVATION_MS=200,
    parameter integer REGISTER_IR_DECODE=0
)(
    input wire clk, reset_n, mode_available,
    input wire ir_valid,
    input wire [7:0] ir_command,
    input wire boxes_valid,
    input wire [31:0] frame_id,
    input wire [3:0] count,
    input wire [MAX_BOXES*64-1:0] coords,
    input wire [MAX_BOXES*7-1:0] classes,
    input wire [31:0] tracker_session,
    input wire [MAX_BOXES*32-1:0] track_ids,
    input wire [MAX_BOXES*10-1:0] confidences,
    input wire [MAX_BOXES-1:0] observed,
    input wire motor_ready,
    input wire [15:0] front_cm,rear_cm,left_cm,right_cm,
    input wire front_valid,rear_valid,left_valid,right_valid,
    input wire motor_online,
    input wire motor_motion_committed,
    input wire [3:0] motor_transmitted_motion,
    input wire [7:0] requested_speed,
    output wire owner,
    output wire [3:0] motion,
    output wire [7:0] speed,
    output wire signed [15:0] follow_linear,follow_turn,
    output wire follow_request_nonzero,
    output reg [2:0] status,
    output reg [3:0] selected_slot,
    output reg [31:0] selected_frame_id,
    output reg [15:0] reference_height,
    output wire [31:0] target_track_id,target_tracker_session,
    output wire lock_valid,
    output wire [3:0] reason,
    output reg arrived,output reg [31:0] arrival_session,arrival_id
);
    localparam [2:0] OFF=0, SELECT=1, FOLLOW=2, LOST=3, BLOCKED=4, WAIT_TARGET=5;
    localparam [3:0] NONE=0, FORWARD=1, BACKWARD=2, LEFT_STRAFE=3, RIGHT_STRAFE=4;
    localparam [3:0] LEFT_ARC=7, RIGHT_ARC=8;
    localparam integer MS_CLKS=CLK_FREQ_HZ/1000<1 ? 1 : CLK_FREQ_HZ/1000;
    localparam integer TIMEOUT_CLKS=MS_CLKS*TARGET_TIMEOUT_MS<1 ? 1 : MS_CLKS*TARGET_TIMEOUT_MS;
    localparam integer SPEED_LIMIT=FOLLOW_SPEED_MAX<1 ? 1 :
                                    FOLLOW_SPEED_MAX>255 ? 255 : FOLLOW_SPEED_MAX;
    localparam integer TURN_SPEED_LIMIT=FOLLOW_TURN_SPEED_MAX<1 ? 1 :
        FOLLOW_TURN_SPEED_MAX>SPEED_LIMIT ? SPEED_LIMIT : FOLLOW_TURN_SPEED_MAX;
    localparam integer ARC_SPEED_LIMIT=FOLLOW_ARC_SPEED_MAX<1 ? 1 :
        FOLLOW_ARC_SPEED_MAX>SPEED_LIMIT ? SPEED_LIMIT : FOLLOW_ARC_SPEED_MAX;
    localparam integer RAW_DELTA_MAX=MAX_CENTER_DELTA_PX>MAX_SIZE_DELTA_PX ?
                                    MAX_CENTER_DELTA_PX : MAX_SIZE_DELTA_PX;
    localparam integer DELTA_MAX=RAW_DELTA_MAX>65535 ? 65535 : RAW_DELTA_MAX;
    localparam integer DELTA_BITS=DELTA_MAX<1 ? 1 : $clog2(DELTA_MAX+1);
    localparam integer SCORE_BITS=DELTA_BITS+2;
    localparam [SCORE_BITS-1:0] SCORE_NONE={SCORE_BITS{1'b1}};
    localparam [1:0] SCAN_GEOMETRY=0,SCAN_SCORE=1,SCAN_RANK=2,SCAN_FINALIZE=3;

    reg [31:0] frame_age, last_input_frame;
    reg input_frame_seen;
    wire signed [31:0] input_frame_delta=frame_id-last_input_frame;
    wire new_frame=boxes_valid && (!input_frame_seen || input_frame_delta>0);
    wire active=status==SELECT || status==FOLLOW || (TRACK_ID_ENABLE && status==WAIT_TARGET);
    wire timeout_due=frame_age>=TIMEOUT_CLKS-1;
    wire triple_blocked=FOLLOW_PROXIMITY_ENABLE && front_valid && left_valid && right_valid &&
                        front_cm<=20 && left_cm<30 && right_cm<30;
    // Optional board integration stage: keep IR payload and its valid pulse
    // together and predecode cancellation before the wide cached-state CE.
    reg [7:0] ir_command_r;
    reg ir_valid_r,exit_key_r;
    wire [7:0] control_ir_command=REGISTER_IR_DECODE?ir_command_r:ir_command;
    wire control_ir_valid=REGISTER_IR_DECODE?ir_valid_r:ir_valid;
    wire manual_key=ir_command==8'h40 || ir_command==8'h19 ||
        ir_command==8'h07 || ir_command==8'h09 || ir_command==8'h16 ||
        ir_command==8'h0D || ir_command==8'h44 || ir_command==8'h43;
    wire raw_exit_key=ir_valid && (ir_command==8'h15 || ir_command==8'h4A || manual_key);
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin ir_command_r<=0;ir_valid_r<=0;exit_key_r<=0;end
        else begin ir_command_r<=ir_command;ir_valid_r<=ir_valid;exit_key_r<=raw_exit_key;end
    end
    wire exit_key=REGISTER_IR_DECODE?exit_key_r:raw_exit_key;
    wire select_key=control_ir_valid && (control_ir_command==8'h46 || (control_ir_command==8'h42 && status!=OFF));
    wire menu_key=control_ir_valid && (control_ir_command==8'h08 || control_ir_command==8'h5A) &&
                  (status==SELECT || status==FOLLOW);

    // Three clocks per candidate separate coordinate selection, distance
    // arithmetic and ranking. Final association uses the registered ranking,
    // never the final candidate's entire combinational score/ambiguity chain.
    reg scan_busy;
    reg [1:0] scan_phase;
    reg [3:0] scan_index, staged_count, scan_first;
    reg [2:0] staged_status;
    reg [31:0] staged_frame;
    reg [MAX_BOXES*64-1:0] staged_coords,cached_coords;
    reg [MAX_BOXES*7-1:0] staged_classes,cached_classes;
    reg [MAX_BOXES-1:0] scan_mask,cached_mask;
    reg [31:0] cached_frame;
    reg [SCORE_BITS-1:0] best_score, second_score;
    reg [3:0] best_slot;
    reg [15:0] best_cx,best_cy,best_w,best_h;
    reg [15:0] id_match_cx,id_match_cy,id_match_w,id_match_h;
    reg [15:0] target_cx,target_cy,target_w,target_h;
    reg [6:0] target_class;
    reg [1:0] stable_samples,lateral_bias;
    reg selection_locked,selection_failed;
    reg [3:0] desired_motion;
    reg servo_tx_fault;
    reg vector_tx_fault;
    wire vector_motion_active;
    localparam [1:0] VECTOR_IDLE=0,VECTOR_ACTIVE=1,VECTOR_STOP=2,VECTOR_OBSERVE=3;
    reg [1:0] vector_state;
    wire vector_observation_stale;
    wire [7:0] vector_display_grade;
    reg vector_motor_ready;
    // The vector request feeds the motor STOP policy. Sample its reply before
    // using it to advance either vector or identity-recovery state, so that
    // request generation and STOP acknowledgement do not form one long path.
    wire identity_motor_ready=FOLLOW_VECTOR_ENABLE ? vector_motor_ready : motor_ready;

    wire [63:0] scan_box=staged_coords[scan_index*64 +: 64];
    wire [15:0] scan_x1=scan_box[15:0], scan_y1=scan_box[31:16];
    wire [15:0] scan_x2=scan_box[47:32],scan_y2=scan_box[63:48];
    wire [6:0] scan_class=staged_classes[scan_index*7 +: 7];
    wire [16:0] scan_x_sum={1'b0,scan_x1}+{1'b0,scan_x2};
    wire [16:0] scan_y_sum={1'b0,scan_y1}+{1'b0,scan_y2};
    wire [15:0] scan_cx=scan_x_sum[16:1],scan_cy=scan_y_sum[16:1];
    wire [15:0] scan_w=scan_x2-scan_x1,scan_h=scan_y2-scan_y1;
    wire rect_valid=scan_index<staged_count && scan_x2>scan_x1 &&
        scan_y2>scan_y1 && scan_x2<=IMAGE_WIDTH && scan_y2<=IMAGE_HEIGHT;
    reg geometry_valid,candidate_allowed;
    reg [15:0] geometry_cx,geometry_cy,geometry_w,geometry_h;
    reg [6:0] geometry_class;
    reg [DELTA_BITS:0] score_xy,score_wh;
    function [15:0] abs_difference;
        input [15:0] a,b;
        begin abs_difference=a>=b ? a-b : b-a; end
    endfunction
    wire [15:0] dx=abs_difference(geometry_cx,target_cx);
    wire [15:0] dy=abs_difference(geometry_cy,target_cy);
    wire [15:0] dw=abs_difference(geometry_w,target_w);
    wire [15:0] dh=abs_difference(geometry_h,target_h);
    // Full-width deltas still decide validity. Only accepted deltas (<= the
    // configured bounds) enter this exact narrow score; no score saturation.
    wire [SCORE_BITS-1:0] candidate_score={1'b0,score_xy}+{1'b0,score_wh};
    wire candidate_valid=geometry_valid && geometry_class==target_class &&
        geometry_h>=MIN_TARGET_HEIGHT && dx<=MAX_CENTER_DELTA_PX &&
        dy<=MAX_CENTER_DELTA_PX && dw<=MAX_SIZE_DELTA_PX && dh<=MAX_SIZE_DELTA_PX &&
        {geometry_w,1'b0}>={1'b0,target_w} && {1'b0,geometry_w}<={target_w,1'b0} &&
        {geometry_h,1'b0}>={1'b0,target_h} && {1'b0,geometry_h}<={target_h,1'b0};
    reg [MAX_BOXES-1:0] mask_next;
    reg [3:0] first_next,best_slot_next;
    reg [SCORE_BITS-1:0] best_next,second_next;
    reg [15:0] best_cx_next,best_cy_next,best_w_next,best_h_next;
    always @(*) begin
        mask_next=scan_mask; first_next=scan_first;
        best_next=best_score; second_next=second_score; best_slot_next=best_slot;
        best_cx_next=best_cx; best_cy_next=best_cy; best_w_next=best_w; best_h_next=best_h;
        if(geometry_valid) begin
            mask_next[scan_index]=1;
            if(scan_first==15) first_next=scan_index;
        end
        if(candidate_allowed) begin
            if(candidate_score<best_score) begin
                second_next=best_score; best_next=candidate_score; best_slot_next=scan_index;
                best_cx_next=geometry_cx;best_cy_next=geometry_cy;
                best_w_next=geometry_w;best_h_next=geometry_h;
            end else if(candidate_score<second_score) second_next=candidate_score;
        end
    end
    wire [SCORE_BITS-1:0] score_gap=second_score-best_score;
    wire match_unambiguous=best_slot!=15 &&
        (second_score==SCORE_NONE || score_gap>MATCH_AMBIGUITY_PX);
    function [3:0] cycle_slot;
        input [MAX_BOXES-1:0] valid_mask;
        input [3:0] current;
        input next_direction;
        integer i;
        reg [3:0] first,last,following,preceding;
        begin
            // Separate priority encoders avoid a direction-controlled modulo
            // adder before the wide cached-box mux and coordinate arithmetic.
            first=15;last=15;following=15;preceding=15;
            for(i=0;i<MAX_BOXES;i=i+1) begin
                if(valid_mask[i]) begin
                    if(first==15) first=i;
                    last=i;
                    if(i>current && following==15) following=i;
                    if(i<current) preceding=i;
                end
            end
            if(next_direction)
                cycle_slot=current==15 || following==15 ? first : following;
            else cycle_slot=current==15 || preceding==15 ? last : preceding;
        end
    endfunction
    wire [3:0] menu_slot=cycle_slot(cached_mask,selected_slot,control_ir_command==8'h5A);
    wire [63:0] menu_box=cached_coords[menu_slot*64 +: 64];
    wire [16:0] menu_x_sum={1'b0,menu_box[15:0]}+{1'b0,menu_box[47:32]};
    wire [16:0] menu_y_sum={1'b0,menu_box[31:16]}+{1'b0,menu_box[63:48]};
    wire [63:0] first_box=staged_coords[scan_first*64 +: 64];
    wire [16:0] first_x_sum={1'b0,first_box[15:0]}+{1'b0,first_box[47:32]};
    wire [16:0] first_y_sum={1'b0,first_box[31:16]}+{1'b0,first_box[63:48]};
    wire [63:0] selected_box=cached_coords[selected_slot*64 +: 64];
    wire [15:0] selected_x1=selected_box[15:0],selected_y1=selected_box[31:16];
    wire [15:0] selected_x2=selected_box[47:32],selected_y2=selected_box[63:48];
    wire [16:0] selected_x_sum={1'b0,selected_x1}+{1'b0,selected_x2};
    wire [16:0] selected_y_sum={1'b0,selected_y1}+{1'b0,selected_y2};
    // Subtract each fixed slot before the variable selection. Confirmation
    // then selects a precomputed size/threshold, avoiding a mux -> subtract
    // -> compare chain on selected_slot's wide target register enables.
    wire [MAX_BOXES*16-1:0] cached_heights;
    wire [MAX_BOXES-1:0] cached_min_height;
    genvar height_slot;
    generate for(height_slot=0;height_slot<MAX_BOXES;height_slot=height_slot+1)begin: g_cached_height
        wire [15:0] h=cached_coords[height_slot*64+48 +: 16]-cached_coords[height_slot*64+16 +: 16];
        assign cached_heights[height_slot*16 +: 16]=h;
        assign cached_min_height[height_slot]=h>=MIN_TARGET_HEIGHT;
    end endgenerate
    wire [15:0] selected_h=cached_heights[selected_slot*16 +: 16];
    wire selected_present=selected_slot<MAX_BOXES && cached_mask[selected_slot] && !timeout_due;
    reg [15:0] reference_width;
    reg reference_use_width;
    reg approach_mode;
    wire approach_near=approach_mode && front_valid && front_cm<=APPROACH_STOP_CM;
    wire [15:0] distance_reference=reference_use_width ? reference_width : reference_height;
    wire [15:0] distance_measurement=reference_use_width ? best_w : best_h;
    wire [15:0] height_deadzone=(distance_reference>>HEIGHT_DEADZONE_SHIFT)>HEIGHT_DEADZONE_MIN ?
        (distance_reference>>HEIGHT_DEADZONE_SHIFT) : HEIGHT_DEADZONE_MIN;
    wire [15:0] height_exit_deadzone=height_deadzone>1 ? height_deadzone>>1 : 16'd1;
    // Rotation must stop on the latest measured center, without waiting for
    // the position smoothing used by legacy lateral control. Height remains
    // lightly filtered; prediction is reserved for identity association.
    wire [15:0] control_cx=((FOLLOW_SERVO_ENABLE || FOLLOW_CURVE_ENABLE) && TRACK_ID_ENABLE) ? id_match_cx : best_cx;
    reg hard_alignment;
    wire [15:0] center_error=abs_difference(control_cx,IMAGE_WIDTH/2);
    wire matched_hard_alignment=center_error>=ALIGN_ENTER_PX ||
        (hard_alignment && center_error>ALIGN_EXIT_PX);
    reg [1:0] matched_bias;
    reg [1:0] distance_bias;
    reg [3:0] matched_motion,matched_distance;
    always @(*) begin
        matched_bias=0;
        if(control_cx+CENTER_ENTER_PX<IMAGE_WIDTH/2) matched_bias=1;
        else if(control_cx>IMAGE_WIDTH/2+CENTER_ENTER_PX) matched_bias=2;
        else if(lateral_bias==1 && control_cx+CENTER_EXIT_PX<IMAGE_WIDTH/2) matched_bias=1;
        else if(lateral_bias==2 && control_cx>IMAGE_WIDTH/2+CENTER_EXIT_PX) matched_bias=2;
        matched_distance=NONE;
        if(distance_measurement+height_deadzone<distance_reference) matched_distance=FORWARD;
        else if(FOLLOW_REVERSE_ENABLE && distance_measurement>distance_reference+height_deadzone) matched_distance=BACKWARD;
        else if(FOLLOW_REVERSE_ENABLE && distance_bias==1 &&
                distance_measurement+height_exit_deadzone<distance_reference) matched_distance=FORWARD;
        else if(FOLLOW_REVERSE_ENABLE && distance_bias==2 &&
                distance_measurement>distance_reference+height_exit_deadzone) matched_distance=BACKWARD;
        if(FOLLOW_CURVE_ENABLE && !matched_hard_alignment && matched_distance==FORWARD)
            matched_motion=matched_bias==1 ? LEFT_ARC : matched_bias==2 ? RIGHT_ARC : FORWARD;
        else if(FOLLOW_CURVE_ENABLE && !matched_hard_alignment && matched_distance==BACKWARD)
            matched_motion=BACKWARD;
        else if(matched_bias==1) matched_motion=LEFT_STRAFE;
        else if(matched_bias==2) matched_motion=RIGHT_STRAFE;
        else matched_motion=matched_distance;
    end

    reg [3:0] drive_motion,pending_motion;
    reg handoff_wait,handoff_requires_new_stop,handoff_saw_not_ready;
    reg [1:0] handoff_ticks;

    // IDs belong to one Pi tracker session. Neither a slot number nor a
    // predicted rectangle is evidence that a different object is this target.
    localparam integer RETAIN_CLKS=MS_CLKS*TARGET_RETAIN_MS<1 ? 1 : MS_CLKS*TARGET_RETAIN_MS;
    reg [31:0] locked_id,locked_session,last_input_session,staged_session,cached_session;
    reg [MAX_BOXES*32-1:0] staged_ids,cached_ids;
    reg [MAX_BOXES*10-1:0] staged_confidences,cached_confidences;
    reg [MAX_BOXES-1:0] staged_observed,cached_observed;
    reg [31:0] geometry_id,target_missing_age;
    reg [9:0] geometry_confidence;
    reg geometry_observed,id_candidate_allowed;
    reg [3:0] id_candidate_reason,track_reason,id_scan_reason;
    reg [1:0] id_matches,id_recovery_count;
    reg signed [16:0] velocity_x,velocity_y;
    reg recovery_stop_pending,recovery_saw_not_ready;
    reg vector_recovery_ready_armed;
    wire identity_ready_sample_valid=!FOLLOW_VECTOR_ENABLE || vector_recovery_ready_armed;
    assign target_track_id=TRACK_ID_ENABLE ? locked_id : 32'd0;
    assign target_tracker_session=TRACK_ID_ENABLE ? locked_session : 32'd0;
    assign lock_valid=TRACK_ID_ENABLE && locked_id!=0 && locked_session!=0 &&
                      (status==FOLLOW || status==WAIT_TARGET);
    wire identity_new_frame=boxes_valid && (!input_frame_seen ||
        tracker_session!=last_input_session || input_frame_delta>0);
    wire retain_timeout=target_missing_age>=RETAIN_CLKS-1;

    function [15:0] predict_center;
        input [15:0] center;
        input signed [16:0] velocity;
        input [15:0] limit;
        reg signed [17:0] prediction;
        begin
            prediction=$signed({1'b0,center})+velocity;
            predict_center=prediction<0 ? 16'd0 : prediction>limit ? limit : prediction[15:0];
        end
    endfunction
    function signed [16:0] update_velocity;
        input signed [16:0] previous;
        input [15:0] current,old;
        reg signed [17:0] delta,estimate;
        begin
            delta=$signed({1'b0,current})-$signed({1'b0,old});
            estimate=($signed(previous)+delta)>>>1;
            update_velocity=estimate>96 ? 17'sd96 : estimate< -96 ? -17'sd96 : estimate[16:0];
        end
    endfunction
    wire [15:0] predicted_cx=predict_center(target_cx,velocity_x,IMAGE_WIDTH);
    wire [15:0] predicted_cy=predict_center(target_cy,velocity_y,IMAGE_HEIGHT);
    // A bounded gate grows with object size and a short real-time detection
    // gap. Constant shifts/comparators avoid division and matrix arithmetic.
    wire [15:0] gap_margin=target_missing_age>=MS_CLKS*400 ? 16'd128 :
        target_missing_age>=MS_CLKS*300 ? 16'd96 :
        target_missing_age>=MS_CLKS*200 ? 16'd64 :
        target_missing_age>=MS_CLKS*100 ? 16'd32 : 16'd0;
    wire [16:0] gate_x_raw=17'd96+{1'b0,target_w[15:1]}+gap_margin;
    wire [16:0] gate_y_raw=17'd80+{1'b0,target_h[15:1]}+gap_margin;
    wire [15:0] gate_x=gate_x_raw>384 ? 16'd384 : gate_x_raw[15:0];
    wire [15:0] gate_y=gate_y_raw>288 ? 16'd288 : gate_y_raw[15:0];
    // Exact 18-bit triples (3*65535 fits) use adders, not inferred MULTALU
    // input registers on the menu->target->association path.
    wire [17:0] geometry_w_triple={1'b0,geometry_w,1'b0}+{2'b0,geometry_w};
    wire [17:0] geometry_h_triple={1'b0,geometry_h,1'b0}+{2'b0,geometry_h};
    wire [17:0] target_w_triple={1'b0,target_w,1'b0}+{2'b0,target_w};
    wire [17:0] target_h_triple={1'b0,target_h,1'b0}+{2'b0,target_h};
    wire identity_geometry_ok=geometry_valid && geometry_h>=MIN_TARGET_HEIGHT &&
        abs_difference(geometry_cx,predicted_cx)<=gate_x &&
        abs_difference(geometry_cy,predicted_cy)<=gate_y &&
        geometry_w_triple>={2'b0,target_w} &&
        {2'b0,geometry_w}<=target_w_triple &&
        geometry_h_triple>={2'b0,target_h} &&
        {2'b0,geometry_h}<=target_h_triple;
    wire identity_same=geometry_id!=0 && geometry_id==locked_id;
    wire identity_selectable=geometry_valid && geometry_h>=MIN_TARGET_HEIGHT &&
        geometry_class<84 && geometry_id!=0 && staged_session!=0 &&
        geometry_observed && geometry_confidence>=TRACK_CONFIDENCE_MIN;
    wire [31:0] menu_id=cached_ids[menu_slot*32 +: 32];
    wire [31:0] selected_id=cached_ids[selected_slot*32 +: 32];
    integer identity_menu_index;
    reg selected_id_duplicate;
    always @(*) begin
        selected_id_duplicate=0;
        for(identity_menu_index=0;identity_menu_index<MAX_BOXES;identity_menu_index=identity_menu_index+1)
            if(identity_menu_index!=selected_slot && cached_mask[identity_menu_index] && selected_id!=0 &&
               cached_ids[identity_menu_index*32 +: 32]==selected_id)
                selected_id_duplicate=1;
    end

    generate if(!TRACK_ID_ENABLE) begin: legacy_geometry_tracking
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            status<=OFF;selected_slot<=15;selected_frame_id<=0;reference_height<=REF_HEIGHT_INITIAL;
            frame_age<=0;last_input_frame<=0;input_frame_seen<=0;scan_busy<=0;
            scan_phase<=SCAN_GEOMETRY;geometry_valid<=0;candidate_allowed<=0;
            geometry_cx<=0;geometry_cy<=0;geometry_w<=0;geometry_h<=0;geometry_class<=0;
            score_xy<=0;score_wh<=0;
            scan_index<=0;staged_count<=0;staged_status<=OFF;staged_frame<=0;
            staged_coords<=0;staged_classes<=0;cached_coords<=0;cached_classes<=0;
            scan_mask<=0;cached_mask<=0;cached_frame<=0;scan_first<=15;
            best_score<=SCORE_NONE;second_score<=SCORE_NONE;best_slot<=15;
            best_cx<=0;best_cy<=0;best_w<=0;best_h<=0;
            target_cx<=0;target_cy<=0;target_w<=0;target_h<=0;target_class<=0;
            stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;selection_locked<=0;selection_failed<=0;
        end else begin
            if(active && frame_age<TIMEOUT_CLKS) frame_age<=frame_age+1'b1;
            if(!mode_available || exit_key) begin
                status<=OFF;selected_slot<=15;selected_frame_id<=0;frame_age<=0;
                input_frame_seen<=0;scan_busy<=0;cached_mask<=0;selection_locked<=0;selection_failed<=0;
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
            end else if(select_key) begin
                status<=SELECT;selected_slot<=15;selected_frame_id<=0;frame_age<=0;
                input_frame_seen<=0;scan_busy<=0;cached_mask<=0;selection_locked<=0;selection_failed<=0;
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
            end else if(status==FOLLOW && triple_blocked) begin
                status<=BLOCKED;scan_busy<=0;stable_samples<=0;desired_motion<=NONE;lateral_bias<=0;
            end else if(menu_key) begin
                status<=SELECT;selected_slot<=menu_slot;
                selected_frame_id<=cached_frame;scan_busy<=0;selection_locked<=1;selection_failed<=menu_slot==15;
                if(menu_slot<MAX_BOXES) begin
                    target_cx<=menu_x_sum[16:1];target_cy<=menu_y_sum[16:1];
                    target_w<=menu_box[47:32]-menu_box[15:0];target_h<=menu_box[63:48]-menu_box[31:16];
                    target_class<=cached_classes[menu_slot*7 +: 7];
                end
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
            end else if(control_ir_valid && control_ir_command==8'h1C && status==SELECT) begin
                if(selected_present && cached_min_height[selected_slot]) begin
                    target_cx<=selected_x_sum[16:1];target_cy<=selected_y_sum[16:1];
                    target_w<=selected_x2-selected_x1;target_h<=selected_h;
                    target_class<=cached_classes[selected_slot*7 +: 7];
                    reference_height<=selected_h>REF_HEIGHT_MAX ? REF_HEIGHT_MAX : selected_h;
                    status<=FOLLOW;scan_busy<=0;stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
                end
            end else if(control_ir_valid && (control_ir_command==8'h18 || control_ir_command==8'h52)) begin
                if(control_ir_command==8'h18)
                    reference_height<=reference_height>MIN_TARGET_HEIGHT+HEIGHT_STEP ?
                        reference_height-HEIGHT_STEP : MIN_TARGET_HEIGHT;
                else reference_height<=reference_height+HEIGHT_STEP<REF_HEIGHT_MAX ?
                        reference_height+HEIGHT_STEP : REF_HEIGHT_MAX;
                // Recompute only after two detections newer than this distance key.
                desired_motion<=NONE;stable_samples<=0;scan_busy<=0;
            end else begin
                if(active) begin
                    if(new_frame) begin
                        last_input_frame<=frame_id;input_frame_seen<=1;frame_age<=0;
                        staged_coords<=coords;staged_classes<=classes;
                        staged_count<=count>MAX_BOXES ? MAX_BOXES : count;
                        staged_frame<=frame_id;staged_status<=status;scan_index<=0;scan_busy<=1;
                        scan_phase<=SCAN_GEOMETRY;candidate_allowed<=0;
                        scan_mask<=0;scan_first<=15;best_score<=SCORE_NONE;
                        second_score<=SCORE_NONE;best_slot<=15;
                        if(status==FOLLOW && count==0) begin
                            status<=LOST;selected_slot<=15;selected_frame_id<=frame_id;
                            scan_busy<=0;stable_samples<=0;desired_motion<=NONE;lateral_bias<=0;
                        end
                    end else if(timeout_due) begin
                        scan_busy<=0;cached_mask<=0;selected_slot<=15;
                        stable_samples<=0;desired_motion<=NONE;lateral_bias<=0;
                        if(status==FOLLOW) status<=LOST;
                    end else if(scan_busy) begin
                        case(scan_phase)
                        SCAN_GEOMETRY: begin
                            geometry_valid<=rect_valid;geometry_class<=scan_class;
                            geometry_cx<=scan_cx;geometry_cy<=scan_cy;
                            geometry_w<=scan_w;geometry_h<=scan_h;
                            scan_phase<=SCAN_SCORE;
                        end
                        SCAN_SCORE: begin
                            candidate_allowed<=candidate_valid;
                            score_xy<={1'b0,dx[DELTA_BITS-1:0]}+{1'b0,dy[DELTA_BITS-1:0]};
                            score_wh<={1'b0,dw[DELTA_BITS-1:0]}+{1'b0,dh[DELTA_BITS-1:0]};
                            scan_phase<=SCAN_RANK;
                        end
                        SCAN_RANK: begin
                        scan_mask<=mask_next;scan_first<=first_next;
                        best_score<=best_next;second_score<=second_next;best_slot<=best_slot_next;
                        best_cx<=best_cx_next;best_cy<=best_cy_next;best_w<=best_w_next;best_h<=best_h_next;
                        if(scan_index==MAX_BOXES-1) scan_phase<=SCAN_FINALIZE;
                        else begin scan_index<=scan_index+1'b1;scan_phase<=SCAN_GEOMETRY;end
                        end
                        SCAN_FINALIZE: begin
                            scan_busy<=0;cached_coords<=staged_coords;cached_classes<=staged_classes;
                            cached_mask<=scan_mask;cached_frame<=staged_frame;
                            selected_frame_id<=staged_frame;
                            if(staged_status==SELECT && status==SELECT) begin
                                if(!selection_locked) begin
                                    selected_slot<=scan_first;selection_locked<=scan_first!=15;selection_failed<=0;
                                    if(scan_first<MAX_BOXES) begin
                                        target_cx<=first_x_sum[16:1];target_cy<=first_y_sum[16:1];
                                        target_w<=first_box[47:32]-first_box[15:0];target_h<=first_box[63:48]-first_box[31:16];
                                        target_class<=staged_classes[scan_first*7 +: 7];
                                    end
                                end else if(!selection_failed && match_unambiguous) begin
                                    selected_slot<=best_slot;target_cx<=best_cx;target_cy<=best_cy;
                                    target_w<=best_w;target_h<=best_h;
                                end else begin
                                    selected_slot<=15;selection_failed<=1;
                                end
                            end else if(staged_status==FOLLOW && status==FOLLOW) begin
                                if(match_unambiguous) begin
                                    selected_slot<=best_slot;
                                    target_cx<=best_cx;target_cy<=best_cy;
                                    target_w<=best_w;target_h<=best_h;
                                    if(stable_samples<2) stable_samples<=stable_samples+1'b1;
                                    lateral_bias<=matched_bias;desired_motion<=matched_motion;
                                end else begin
                                    status<=LOST;selected_slot<=15;stable_samples<=0;
                                    lateral_bias<=0;desired_motion<=NONE;
                                end
                            end
                        end
                        endcase
                    end
                end
            end
        end
    end

    end else begin: persistent_identity_tracking
    // Capture the private coherent observation tuple on its own event.
    // Session/timeout/key vetoes below still abort scanning immediately; they
    // never need to drive the CE of these 1000+ data bits. A duplicate frame
    // cannot overwrite an in-flight tuple (identity_new_frame excludes it).
    // A vetoed new tuple is unused because every higher-priority veto cancels
    // the scan; a normally accepted tuple keeps the exact original clock.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            staged_count<=0;staged_status<=OFF;staged_frame<=0;
            staged_coords<=0;staged_classes<=0;staged_ids<=0;
            staged_confidences<=0;staged_observed<=0;staged_session<=0;
        end else if(identity_new_frame)begin
            staged_coords<=coords;staged_classes<=classes;staged_ids<=track_ids;
            staged_confidences<=confidences;staged_observed<=observed;staged_session<=tracker_session;
            staged_count<=count>MAX_BOXES?MAX_BOXES:count;
            staged_frame<=frame_id;staged_status<=status;
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            status<=OFF;selected_slot<=15;selected_frame_id<=0;reference_height<=REF_HEIGHT_INITIAL;
            frame_age<=0;last_input_frame<=0;input_frame_seen<=0;scan_busy<=0;
            scan_phase<=SCAN_GEOMETRY;geometry_valid<=0;candidate_allowed<=0;
            geometry_cx<=0;geometry_cy<=0;geometry_w<=0;geometry_h<=0;geometry_class<=0;
            score_xy<=0;score_wh<=0;scan_index<=0;
            cached_coords<=0;cached_classes<=0;
            scan_mask<=0;cached_mask<=0;cached_frame<=0;scan_first<=15;
            best_score<=SCORE_NONE;second_score<=SCORE_NONE;best_slot<=15;
            best_cx<=0;best_cy<=0;best_w<=0;best_h<=0;
            target_cx<=0;target_cy<=0;target_w<=0;target_h<=0;target_class<=0;
            stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;selection_locked<=0;selection_failed<=0;
            locked_id<=0;locked_session<=0;last_input_session<=0;cached_session<=0;
            cached_ids<=0;cached_confidences<=0;
            cached_observed<=0;geometry_id<=0;geometry_confidence<=0;geometry_observed<=0;
            target_missing_age<=0;track_reason<=0;id_scan_reason<=0;id_candidate_reason<=0;
            id_candidate_allowed<=0;id_matches<=0;id_recovery_count<=0;
            id_match_cx<=0;id_match_cy<=0;id_match_w<=0;id_match_h<=0;
            velocity_x<=0;velocity_y<=0;recovery_stop_pending<=0;recovery_saw_not_ready<=0;
            vector_recovery_ready_armed<=0;
            approach_mode<=0;
        end else begin
            if(active && frame_age<TIMEOUT_CLKS) frame_age<=frame_age+1'b1;
            if(status==WAIT_TARGET && target_missing_age<RETAIN_CLKS)
                target_missing_age<=target_missing_age+1'b1;
            if(!FOLLOW_VECTOR_ENABLE || !recovery_stop_pending) vector_recovery_ready_armed<=0;
            else vector_recovery_ready_armed<=1;
            // Keep this STOP epoch across WAIT; the common handoff intentionally
            // clears drive_motion while stopped and cannot preserve it itself.
            if(recovery_stop_pending) begin
                if(identity_ready_sample_valid && !identity_motor_ready) recovery_saw_not_ready<=1;
                if(identity_ready_sample_valid && recovery_saw_not_ready && identity_motor_ready) begin
                    recovery_stop_pending<=0;recovery_saw_not_ready<=0;
                end
            end
            if(!mode_available || exit_key) begin
                approach_mode<=0;
                status<=OFF;selected_slot<=15;selected_frame_id<=0;frame_age<=0;
                input_frame_seen<=0;scan_busy<=0;cached_mask<=0;selection_locked<=0;selection_failed<=0;
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
                locked_id<=0;locked_session<=0;track_reason<=0;target_missing_age<=0;id_recovery_count<=0;
                velocity_x<=0;velocity_y<=0;recovery_stop_pending<=0;recovery_saw_not_ready<=0;
            end else if(select_key) begin
                approach_mode<=0;
                status<=SELECT;selected_slot<=15;selected_frame_id<=0;frame_age<=0;
                input_frame_seen<=0;scan_busy<=0;cached_mask<=0;selection_locked<=0;selection_failed<=0;
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
                locked_id<=0;locked_session<=0;track_reason<=0;target_missing_age<=0;id_recovery_count<=0;
                velocity_x<=0;velocity_y<=0;recovery_stop_pending<=0;recovery_saw_not_ready<=0;
            end else if((TURN_TIMER_FROM_TX || FOLLOW_VECTOR_ENABLE) && (status==FOLLOW || status==WAIT_TARGET) &&
                        (!motor_online || servo_tx_fault || vector_tx_fault)) begin
                // A recovered motor link cannot resume an old target lock.
                // Only a new selection and confirmation can authorize motion.
                status<=LOST;track_reason<=4'hF;scan_busy<=0;cached_mask<=0;selected_slot<=15;
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
                locked_id<=0;locked_session<=0;id_recovery_count<=0;
                velocity_x<=0;velocity_y<=0;recovery_stop_pending<=0;recovery_saw_not_ready<=0;
            end else if((status==FOLLOW || status==WAIT_TARGET) && triple_blocked) begin
                status<=BLOCKED;scan_busy<=0;stable_samples<=0;desired_motion<=NONE;lateral_bias<=0;
                selected_slot<=15;id_recovery_count<=0;recovery_stop_pending<=0;track_reason<=0;
            end else if(menu_key) begin
                status<=SELECT;selected_slot<=menu_slot;selected_frame_id<=cached_frame;
                scan_busy<=0;selection_locked<=1;selection_failed<=menu_slot==15;
                locked_id<=menu_slot<MAX_BOXES ? menu_id : 32'd0;locked_session<=cached_session;
                track_reason<=menu_slot<MAX_BOXES ? 4'd0 : 4'd8;
                if(menu_slot<MAX_BOXES) begin
                    target_cx<=menu_x_sum[16:1];target_cy<=menu_y_sum[16:1];
                    target_w<=menu_box[47:32]-menu_box[15:0];target_h<=menu_box[63:48]-menu_box[31:16];
                    target_class<=cached_classes[menu_slot*7 +: 7];
                end
                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;target_missing_age<=0;id_recovery_count<=0;
                velocity_x<=0;velocity_y<=0;recovery_stop_pending<=0;recovery_saw_not_ready<=0;
            end else if(control_ir_valid && (control_ir_command==8'h1C || control_ir_command==8'hA1) && status==SELECT) begin
                if(selected_present && cached_min_height[selected_slot] && selected_id!=0 &&
                   cached_session!=0 && !selected_id_duplicate && cached_observed[selected_slot] &&
                   cached_confidences[selected_slot*10 +: 10]>=TRACK_CONFIDENCE_MIN) begin
                    target_cx<=selected_x_sum[16:1];target_cy<=selected_y_sum[16:1];
                    target_w<=selected_x2-selected_x1;target_h<=selected_h;
                    target_class<=cached_classes[selected_slot*7 +: 7];
                    locked_id<=selected_id;locked_session<=cached_session;track_reason<=0;
                    approach_mode<=control_ir_command==8'hA1;
                    reference_height<=control_ir_command==8'hA1 ? REF_HEIGHT_MAX :
                        (selected_h>REF_HEIGHT_MAX ? REF_HEIGHT_MAX : selected_h);
                    status<=FOLLOW;scan_busy<=0;stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;
                    target_missing_age<=0;id_recovery_count<=0;velocity_x<=0;velocity_y<=0;
                end else track_reason<=selected_id_duplicate ? 4'd4 : 4'd8;
            end else if(control_ir_valid && (control_ir_command==8'h18 || control_ir_command==8'h52)) begin
                if(control_ir_command==8'h18)
                    reference_height<=reference_height>MIN_TARGET_HEIGHT+HEIGHT_STEP ?
                        reference_height-HEIGHT_STEP : MIN_TARGET_HEIGHT;
                else reference_height<=reference_height+HEIGHT_STEP<REF_HEIGHT_MAX ?
                        reference_height+HEIGHT_STEP : REF_HEIGHT_MAX;
                desired_motion<=NONE;stable_samples<=0;scan_busy<=0;id_recovery_count<=0;
            end else if(approach_near && status==FOLLOW) begin
                status<=OFF;selected_slot<=15;desired_motion<=NONE;stable_samples<=0;
                approach_mode<=0;locked_id<=0;locked_session<=0;
            end else if(active) begin
                // Communication freshness and target retention are independent.
                // Other objects may refresh frame_age, never target_missing_age.
                if(identity_new_frame && (status==FOLLOW || status==WAIT_TARGET) &&
                   tracker_session!=locked_session) begin
                    status<=LOST;track_reason<=6;scan_busy<=0;selected_slot<=15;
                    desired_motion<=NONE;stable_samples<=0;lateral_bias<=0;id_recovery_count<=0;
                end else if(timeout_due) begin
                    scan_busy<=0;cached_mask<=0;selected_slot<=15;stable_samples<=0;
                    desired_motion<=NONE;lateral_bias<=0;id_recovery_count<=0;
                    track_reason<=5;if(status!=SELECT) status<=LOST;
                end else if(status==WAIT_TARGET && retain_timeout) begin
                    status<=LOST;scan_busy<=0;selected_slot<=15;stable_samples<=0;
                    desired_motion<=NONE;lateral_bias<=0;id_recovery_count<=0;
                end else if(identity_new_frame) begin
                    last_input_frame<=frame_id;last_input_session<=tracker_session;input_frame_seen<=1;frame_age<=0;
                    scan_index<=0;scan_busy<=1;
                    scan_phase<=SCAN_GEOMETRY;scan_mask<=0;scan_first<=15;best_slot<=15;
                    id_matches<=0;id_candidate_allowed<=0;id_scan_reason<=count==0 ? 4'd1 : 4'd2;
                    if((status==FOLLOW || status==WAIT_TARGET) && count==0) begin
                        if(status==FOLLOW) begin
                            target_missing_age<=0;
                            recovery_stop_pending<=(recovery_stop_pending &&
                                                   !(identity_ready_sample_valid && recovery_saw_not_ready && identity_motor_ready)) ||
                                                   drive_motion!=NONE || vector_motion_active || handoff_wait || !identity_motor_ready;
                            // A new WAIT epoch must not inherit an already completed STOP.
                            recovery_saw_not_ready<=FOLLOW_VECTOR_ENABLE ? 1'b0 : !motor_ready;
                            vector_recovery_ready_armed<=0;
                        end
                        status<=WAIT_TARGET;track_reason<=1;selected_slot<=15;selected_frame_id<=frame_id;
                        scan_busy<=0;stable_samples<=0;desired_motion<=NONE;lateral_bias<=0;id_recovery_count<=0;
                    end
                end else if(scan_busy) begin
                    case(scan_phase)
                    SCAN_GEOMETRY: begin
                        geometry_valid<=rect_valid;geometry_class<=scan_class;
                        geometry_cx<=scan_cx;geometry_cy<=scan_cy;geometry_w<=scan_w;geometry_h<=scan_h;
                        geometry_id<=staged_ids[scan_index*32 +: 32];
                        geometry_confidence<=staged_confidences[scan_index*10 +: 10];
                        geometry_observed<=staged_observed[scan_index];scan_phase<=SCAN_SCORE;
                    end
                    SCAN_SCORE: begin
                        id_candidate_allowed<=identity_same && staged_session==locked_session &&
                            geometry_class==target_class && identity_selectable && identity_geometry_ok;
                        id_candidate_reason<=!geometry_observed ? 4'd8 :
                            geometry_confidence<TRACK_CONFIDENCE_MIN ? 4'd7 :
                            geometry_class!=target_class ? 4'd2 : !identity_geometry_ok ? 4'd3 : 4'd0;
                        candidate_allowed<=identity_selectable;scan_phase<=SCAN_RANK;
                    end
                    SCAN_RANK: begin
                        if(candidate_allowed) begin
                            scan_mask[scan_index]<=1;
                            if(scan_first==15) scan_first<=scan_index;
                        end
                        if(scan_index<staged_count && identity_same) begin
                            if(id_matches<3) id_matches<=id_matches+1'b1;
                            if(id_candidate_allowed) begin
                                best_slot<=scan_index;
                                id_match_cx<=geometry_cx;id_match_cy<=geometry_cy;
                                id_match_w<=geometry_w;id_match_h<=geometry_h;
                                // This two-sample filter affects control, not ID or
                                // evidence provenance. Only measured boxes drive.
                                best_cx<=({1'b0,geometry_cx}+{1'b0,target_cx})>>1;
                                best_cy<=({1'b0,geometry_cy}+{1'b0,target_cy})>>1;
                                best_w<=({1'b0,geometry_w}+{1'b0,target_w})>>1;
                                best_h<=({1'b0,geometry_h}+{1'b0,target_h})>>1;
                            end else id_scan_reason<=id_candidate_reason;
                        end
                        if(scan_index==MAX_BOXES-1) scan_phase<=SCAN_FINALIZE;
                        else begin scan_index<=scan_index+1'b1;scan_phase<=SCAN_GEOMETRY;end
                    end
                    SCAN_FINALIZE: begin
                        scan_busy<=0;cached_coords<=staged_coords;cached_classes<=staged_classes;
                        cached_ids<=staged_ids;cached_confidences<=staged_confidences;
                        cached_observed<=staged_observed;cached_session<=staged_session;
                        cached_mask<=scan_mask;cached_frame<=staged_frame;selected_frame_id<=staged_frame;
                        if(staged_status==SELECT && status==SELECT) begin
                            if(!selection_locked || staged_session!=locked_session) begin
                                selected_slot<=scan_first;selection_locked<=scan_first!=15;selection_failed<=0;
                                locked_session<=staged_session;locked_id<=scan_first<MAX_BOXES ? staged_ids[scan_first*32 +: 32] : 32'd0;
                                track_reason<=scan_first<MAX_BOXES ? 4'd0 : 4'd8;
                                if(scan_first<MAX_BOXES) begin
                                    target_cx<=first_x_sum[16:1];target_cy<=first_y_sum[16:1];
                                    target_w<=first_box[47:32]-first_box[15:0];target_h<=first_box[63:48]-first_box[31:16];
                                    target_class<=staged_classes[scan_first*7 +: 7];
                                end
                            end else if(id_matches==1 && best_slot!=15) begin
                                selected_slot<=best_slot;selection_failed<=0;track_reason<=0;
                                target_cx<=id_match_cx;target_cy<=id_match_cy;target_w<=id_match_w;target_h<=id_match_h;
                            end else begin
                                selected_slot<=15;selection_failed<=1;track_reason<=id_matches>1 ? 4'd4 : id_scan_reason;
                            end
                        end else if((staged_status==FOLLOW || staged_status==WAIT_TARGET) &&
                                    (status==FOLLOW || status==WAIT_TARGET)) begin
                            if(id_matches==1 && best_slot!=15) begin
                                selected_slot<=best_slot;target_cx<=id_match_cx;target_cy<=id_match_cy;
                                target_w<=id_match_w;target_h<=id_match_h;
                                velocity_x<=update_velocity(velocity_x,id_match_cx,target_cx);
                                velocity_y<=update_velocity(velocity_y,id_match_cy,target_cy);
                                if(status==WAIT_TARGET) begin
                                    if(id_recovery_count>=1) begin
                                        status<=FOLLOW;track_reason<=0;target_missing_age<=0;
                                        stable_samples<=2;id_recovery_count<=0;
                                        lateral_bias<=matched_bias;desired_motion<=matched_motion;
                                    end else begin
                                        id_recovery_count<=1;stable_samples<=0;desired_motion<=NONE;lateral_bias<=0;
                                    end
                                end else begin
                                    if(stable_samples<2) stable_samples<=stable_samples+1'b1;
                                    track_reason<=0;lateral_bias<=matched_bias;desired_motion<=matched_motion;
                                end
                            end else begin
                                if(status==FOLLOW) begin
                                    target_missing_age<=0;
                                    recovery_stop_pending<=(recovery_stop_pending &&
                                                           !(identity_ready_sample_valid && recovery_saw_not_ready && identity_motor_ready)) ||
                                                           drive_motion!=NONE || vector_motion_active || handoff_wait || !identity_motor_ready;
                                    recovery_saw_not_ready<=FOLLOW_VECTOR_ENABLE ? 1'b0 : !motor_ready;
                                    vector_recovery_ready_armed<=0;
                                end
                                status<=WAIT_TARGET;selected_slot<=15;track_reason<=id_matches>1 ? 4'd4 : id_scan_reason;
                                stable_samples<=0;lateral_bias<=0;desired_motion<=NONE;id_recovery_count<=0;
                            end
                        end
                    end
                    endcase
                end
            end
        end
    end
    end endgenerate

    // A low camera often crops a person at the top or bottom, leaving their
    // height constant as distance changes. Choose width once at confirmation;
    // never switch the reference axis during a lock. Capture the actual width,
    // without clamping a small measured box to a larger artificial reference.
    wire valid_distance_confirmation=control_ir_valid && (control_ir_command==8'h1C || control_ir_command==8'hA1) && status==SELECT &&
        selected_present && cached_min_height[selected_slot] &&
        (!TRACK_ID_ENABLE || (selected_id!=0 && cached_session!=0 && !selected_id_duplicate &&
         cached_observed[selected_slot] && cached_confidences[selected_slot*10 +: 10]>=TRACK_CONFIDENCE_MIN));
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin reference_width<=MIN_TARGET_HEIGHT;reference_use_width<=0;end
        else if(!mode_available || status==OFF || exit_key || select_key || menu_key) reference_use_width<=0;
        else if(valid_distance_confirmation) begin
            reference_width<=control_ir_command==8'hA1 ? IMAGE_WIDTH : selected_x2-selected_x1;
            reference_use_width<=FOLLOW_WIDTH_FALLBACK_ENABLE &&
                cached_classes[selected_slot*7 +: 7]==0 && selected_x2-selected_x1>=MIN_TARGET_HEIGHT &&
                (selected_y1<=TARGET_EDGE_MARGIN_PX || selected_y2>=IMAGE_HEIGHT-TARGET_EDGE_MARGIN_PX);
        end else if(reference_use_width && control_ir_valid && (control_ir_command==8'h18 || control_ir_command==8'h52)) begin
            if(control_ir_command==8'h18) reference_width<=reference_width>MIN_TARGET_HEIGHT+HEIGHT_STEP ?
                reference_width-HEIGHT_STEP : MIN_TARGET_HEIGHT;
            else reference_width<=reference_width+HEIGHT_STEP<IMAGE_WIDTH ? reference_width+HEIGHT_STEP : IMAGE_WIDTH;
        end
    end

    // Only a completed, unique, reliable association is a new control sample.
    // Arrival of boxes, predicted rectangles and repeated frames cannot rearm
    // a turn. An observation in the STOP-ack clock is deliberately not reused.
    wire control_observation=scan_busy && scan_phase==SCAN_FINALIZE &&
        (TRACK_ID_ENABLE ?
         ((staged_status==FOLLOW || staged_status==WAIT_TARGET) &&
          (status==FOLLOW || status==WAIT_TARGET) && id_matches==1 && best_slot!=15) :
         (staged_status==FOLLOW && status==FOLLOW && match_unambiguous));
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) distance_bias<=0;
        else if(!FOLLOW_REVERSE_ENABLE || status!=FOLLOW || exit_key || select_key || menu_key ||
                (control_ir_valid && (control_ir_command==8'h18 || control_ir_command==8'h52))) distance_bias<=0;
        else if(control_observation) distance_bias<=matched_distance==FORWARD ? 2'd1 :
            matched_distance==BACKWARD ? 2'd2 : 2'd0;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) hard_alignment<=0;
        else if(!FOLLOW_CURVE_ENABLE || status!=FOLLOW || !mode_available || exit_key || select_key || menu_key)
            hard_alignment<=0;
        else if(control_observation) hard_alignment<=matched_hard_alignment;
    end

    // Directional proximity guards are independent of the turn budget. In
    // particular, a reverse request needs rear evidence, not front evidence.
    reg [3:0] policy_desired_next;
    wire [3:0] safe_desired_next;
    wire [3:0] safe_desired;
    always @(*) begin
        policy_desired_next=NONE;
        if(status==FOLLOW && stable_samples>=2 && mode_available && motor_online &&
           (!approach_mode || (front_valid && front_cm>APPROACH_STOP_CM)) &&
           !triple_blocked && requested_speed!=0 &&
           (!TRACK_ID_ENABLE || !recovery_stop_pending)) begin
            case(desired_motion)
                FORWARD: if(!FOLLOW_PROXIMITY_ENABLE || (front_valid && front_cm>20)) policy_desired_next=FORWARD;
                BACKWARD: if(FOLLOW_REVERSE_ENABLE && (!FOLLOW_PROXIMITY_ENABLE || (rear_valid && rear_cm>20))) policy_desired_next=BACKWARD;
                LEFT_STRAFE: if(!FOLLOW_PROXIMITY_ENABLE || (front_valid && left_valid && left_cm>12))
                    policy_desired_next=FOLLOW_TURN_ENABLE ? 4'd5 : LEFT_STRAFE;
                RIGHT_STRAFE: if(!FOLLOW_PROXIMITY_ENABLE || (front_valid && right_valid && right_cm>12))
                    policy_desired_next=FOLLOW_TURN_ENABLE ? 4'd6 : RIGHT_STRAFE;
                LEFT_ARC: if(FOLLOW_CURVE_ENABLE && (!FOLLOW_PROXIMITY_ENABLE ||
                    (front_valid && front_cm>20 && left_valid && left_cm>12))) policy_desired_next=LEFT_ARC;
                RIGHT_ARC: if(FOLLOW_CURVE_ENABLE && (!FOLLOW_PROXIMITY_ENABLE ||
                    (front_valid && front_cm>20 && right_valid && right_cm>12))) policy_desired_next=RIGHT_ARC;
                default: policy_desired_next=NONE;
            endcase
        end
    end
    localparam [1:0] SERVO_READY=0,SERVO_TURN=1,SERVO_STOP=2,SERVO_OBSERVE=3;
    localparam integer TURN_CLKS=MS_CLKS*TURN_PULSE_MS<1 ? 1 : MS_CLKS*TURN_PULSE_MS;
    localparam integer NEAR_TURN_CLKS=MS_CLKS*TURN_NEAR_PULSE_MS<1 ? 1 :
        MS_CLKS*TURN_NEAR_PULSE_MS>TURN_CLKS ? TURN_CLKS : MS_CLKS*TURN_NEAR_PULSE_MS;
    localparam integer TURN_COUNT_BITS=TURN_CLKS<2 ? 1 : $clog2(TURN_CLKS+1);
    localparam integer TURN_TX_WAIT_CLKS=MS_CLKS*TURN_TX_WAIT_MS<1 ? 1 : MS_CLKS*TURN_TX_WAIT_MS;
    localparam integer TURN_TX_WAIT_BITS=TURN_TX_WAIT_CLKS<2 ? 1 : $clog2(TURN_TX_WAIT_CLKS+1);
    reg [1:0] servo_state;
    reg [3:0] servo_direction;
    reg [TURN_COUNT_BITS-1:0] servo_age;
    reg [TURN_TX_WAIT_BITS-1:0] servo_tx_wait_age;
    reg servo_near;
    reg [31:0] servo_stop_frame;
    wire signed [31:0] servo_frame_delta=staged_frame-servo_stop_frame;
    wire [TURN_COUNT_BITS-1:0] servo_budget=servo_near ? NEAR_TURN_CLKS : TURN_CLKS;
    reg servo_started,servo_requires_stop,servo_saw_not_ready;
    wire policy_turn=policy_desired_next==4'd5 || policy_desired_next==4'd6;
    wire servo_turn_commit=motor_motion_committed && motor_transmitted_motion==servo_direction;
    wire turn_due=(!TURN_TIMER_FROM_TX || servo_started) &&
        (servo_near ? servo_age>=NEAR_TURN_CLKS-1 : servo_age>=TURN_CLKS-1);
    wire turn_tx_timeout=TURN_TIMER_FROM_TX && !servo_started && !servo_turn_commit &&
        servo_tx_wait_age>=TURN_TX_WAIT_CLKS-1;
    wire servo_enabled=FOLLOW_SERVO_ENABLE && FOLLOW_TURN_ENABLE && !FOLLOW_VECTOR_ENABLE;
    assign safe_desired_next=TURN_TIMER_FROM_TX && servo_tx_fault ? NONE :
        !servo_enabled ? policy_desired_next :
        servo_state==SERVO_READY ? (policy_turn ? NONE : policy_desired_next) :
        servo_state==SERVO_TURN && !turn_due && !turn_tx_timeout && policy_desired_next==servo_direction ?
            policy_desired_next : NONE;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            servo_state<=SERVO_READY;servo_direction<=NONE;servo_age<=0;servo_near<=0;servo_stop_frame<=0;
            servo_started<=0;servo_requires_stop<=0;servo_saw_not_ready<=0;
            servo_tx_wait_age<=0;servo_tx_fault<=0;
        end else if(!servo_enabled || status!=FOLLOW || !mode_available || exit_key || select_key || menu_key) begin
            servo_state<=SERVO_READY;servo_direction<=NONE;servo_age<=0;servo_near<=0;servo_stop_frame<=0;
            servo_started<=0;servo_requires_stop<=0;servo_saw_not_ready<=0;
            servo_tx_wait_age<=0;servo_tx_fault<=0;
        end else case(servo_state)
            SERVO_READY: if(policy_turn && motor_ready && !handoff_wait) begin
                servo_state<=SERVO_TURN;servo_direction<=policy_desired_next;servo_age<=0;servo_started<=0;
                servo_tx_wait_age<=0;
                servo_near<=abs_difference(target_cx,IMAGE_WIDTH/2)<CENTER_ENTER_PX*2;
            end
            SERVO_TURN: begin
                // The 50 ms wheel refresh may be longer than a near turn.
                // Start its budget after the complete turn frame was sent;
                // this is local TX completion, not a driver acknowledgement.
                if(TURN_TIMER_FROM_TX) begin
                    if(!servo_started) begin
                        if(servo_turn_commit) begin
                            servo_started<=1;servo_age<=0;servo_tx_wait_age<=0;
                        end else if(!turn_tx_timeout) servo_tx_wait_age<=servo_tx_wait_age+1'b1;
                    end else if(motion==servo_direction && !turn_due) servo_age<=servo_age+1'b1;
                    if(turn_tx_timeout) servo_tx_fault<=1;
                end else if(motion==servo_direction) begin
                    servo_started<=1;
                    if(!turn_due) servo_age<=servo_age+1'b1;
                end
                if(turn_due || turn_tx_timeout || policy_desired_next!=servo_direction) begin
                    servo_state<=SERVO_STOP;
                    servo_requires_stop<=servo_started || drive_motion==servo_direction || !motor_ready;
                    servo_saw_not_ready<=!motor_ready;
                end
            end
            SERVO_STOP: begin
                if(!motor_ready) servo_saw_not_ready<=1;
                if(motor_ready && (!servo_requires_stop || servo_saw_not_ready)) begin
                    servo_state<=SERVO_OBSERVE;servo_age<=0;
                    servo_stop_frame<=(TRACK_ID_ENABLE ? identity_new_frame : new_frame) ? frame_id : last_input_frame;
                end
            end
            SERVO_OBSERVE: if(control_observation && stable_samples>=2 && servo_frame_delta>0) begin
                servo_state<=SERVO_READY;servo_direction<=NONE;servo_started<=0;
                servo_requires_stop<=0;servo_saw_not_ready<=0;
            end
        endcase
    end

    // FOLLOW alone says the identity is locked, not that driving is allowed.
    // These additional reasons make a centered-but-stopped car diagnosable.
    wire [3:0] follow_drive_reason=(!motor_online || (TURN_TIMER_FROM_TX && servo_tx_fault) || vector_tx_fault) ? 4'hF :
        (stable_samples<2 || servo_state==SERVO_OBSERVE ||
         (FOLLOW_VECTOR_ENABLE && (vector_observation_stale || vector_state==VECTOR_OBSERVE))) ? 4'hE :
        (recovery_stop_pending || handoff_wait || servo_state==SERVO_STOP ||
         (FOLLOW_VECTOR_ENABLE && vector_state==VECTOR_STOP) ||
         (!FOLLOW_VECTOR_ENABLE && drive_motion==NONE && !motor_ready)) ? 4'hD :
        FOLLOW_PROXIMITY_ENABLE && desired_motion==FORWARD && (!front_valid || front_cm<=20) ? 4'hA :
        FOLLOW_PROXIMITY_ENABLE && desired_motion==BACKWARD && (!rear_valid || rear_cm<=20) ? 4'hB :
        FOLLOW_PROXIMITY_ENABLE && desired_motion==LEFT_STRAFE && (!front_valid || !left_valid || left_cm<=12) ? 4'hC :
        FOLLOW_PROXIMITY_ENABLE && desired_motion==RIGHT_STRAFE && (!front_valid || !right_valid || right_cm<=12) ? 4'hC :
        ((FOLLOW_VECTOR_ENABLE ? !vector_motion_active : desired_motion==NONE) || requested_speed==0) ? 4'h9 : 4'd0;
    assign reason=!TRACK_ID_ENABLE ? 4'd0 :
        (FOLLOW_SERVO_ENABLE || FOLLOW_VECTOR_ENABLE) && status==FOLLOW && track_reason==0 ? follow_drive_reason : track_reason;
    generate if(TRACK_ID_ENABLE) begin: registered_tracking_safety
        // One SYS50 tick (20 ns) breaks the sensor->policy->drive comparison
        // ->motor selection path. WAIT/LOST/mode changes still gate motion
        // directly below; this register can never extend their drive command.
        reg [3:0] permitted_motion;
        always @(posedge clk or negedge reset_n)
            if(!reset_n) permitted_motion<=NONE;
            else permitted_motion<=safe_desired_next;
        assign safe_desired=permitted_motion;
    end else begin: legacy_combinational_safety
        assign safe_desired=safe_desired_next;
    end endgenerate
    function forward_family;
        input [3:0] m;
        begin forward_family=m==FORWARD || m==LEFT_ARC || m==RIGHT_ARC;end
    endfunction
    wire continuous_forward_change=FOLLOW_CURVE_ENABLE && forward_family(drive_motion) &&
                                   forward_family(safe_desired);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            drive_motion<=NONE;pending_motion<=NONE;handoff_wait<=0;
            handoff_requires_new_stop<=0;handoff_saw_not_ready<=0;handoff_ticks<=0;
        end else if(FOLLOW_VECTOR_ENABLE || status!=FOLLOW || !mode_available || exit_key || select_key || menu_key) begin
            drive_motion<=NONE;pending_motion<=NONE;handoff_wait<=0;
            handoff_requires_new_stop<=0;handoff_saw_not_ready<=0;handoff_ticks<=0;
        end else if(!handoff_wait) begin
            if(continuous_forward_change) begin
                drive_motion<=safe_desired;pending_motion<=safe_desired;
            end else if(safe_desired!=drive_motion) begin
                pending_motion<=safe_desired;drive_motion<=NONE;handoff_wait<=1;handoff_ticks<=0;
                handoff_requires_new_stop<=drive_motion!=NONE;
                handoff_saw_not_ready<=!motor_ready;
            end
        end else begin
            drive_motion<=NONE;pending_motion<=safe_desired;
            if(handoff_ticks<3) handoff_ticks<=handoff_ticks+1'b1;
            if(!motor_ready) handoff_saw_not_ready<=1;
            if(handoff_ticks>=1 && motor_ready &&
               (!handoff_requires_new_stop || handoff_saw_not_ready)) begin
                drive_motion<=safe_desired;handoff_wait<=0;handoff_ticks<=0;
            end
        end
    end
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin arrived<=0;arrival_session<=0;arrival_id<=0;end
        else if(!mode_available || select_key || menu_key ||
            (control_ir_valid && (control_ir_command==8'hA1 || control_ir_command==8'hA0 || (exit_key && control_ir_command!=8'h15))))begin
            arrived<=0;arrival_session<=0;arrival_id<=0;
        end else if(!exit_key && !select_key && !menu_key && !triple_blocked && approach_near && status==FOLLOW && motor_online && !timeout_due && locked_id!=0 && locked_session!=0)begin
            arrived<=1;arrival_session<=locked_session;arrival_id<=locked_id;
        end
    end
    assign owner=status!=OFF;
    assign motion=FOLLOW_VECTOR_ENABLE ? vector_motion_code(follow_linear,follow_turn) :
                  (mode_available && motor_online && status==FOLLOW &&
                   (TRACK_ID_ENABLE || !triple_blocked) &&
                   (drive_motion==safe_desired || continuous_forward_change)) ?
                    (continuous_forward_change ? safe_desired : drive_motion) : NONE;
    wire [7:0] motion_speed_limit=(motion==4'd5 || motion==4'd6) ? TURN_SPEED_LIMIT :
        (motion==LEFT_ARC || motion==RIGHT_ARC) ? ARC_SPEED_LIMIT : SPEED_LIMIT;
    assign speed=FOLLOW_VECTOR_ENABLE ? vector_display_grade : motion==NONE ? 8'd0 :
                 requested_speed>motion_speed_limit ? motion_speed_limit : requested_speed;

    // Independent longitudinal and yaw requests, expressed in motor command
    // units. These are not calibrated m/s or rad/s. Only new reliable measured
    // boxes change the request; prediction remains an association tool.
    localparam integer VECTOR_OBSERVATION_CLKS=MS_CLKS*VECTOR_OBSERVATION_MS<1 ? 1 :
        MS_CLKS*VECTOR_OBSERVATION_MS;
    localparam integer VECTOR_OBSERVATION_BITS=VECTOR_OBSERVATION_CLKS<2 ? 1 :
        $clog2(VECTOR_OBSERVATION_CLKS+1);
    reg signed [15:0] vector_linear_reg,vector_turn_reg;
    reg [VECTOR_OBSERVATION_BITS-1:0] vector_observation_age;
    reg [TURN_TX_WAIT_BITS-1:0] vector_tx_age;
    reg [1:0] vector_last_linear_sign,vector_last_turn_sign,vector_turn_candidate;
    reg vector_turn_candidate_seen,vector_saw_not_ready,vector_stop_ready_armed;
    reg [31:0] vector_stop_frame;
    reg signed [15:0] vector_observed_linear,vector_observed_turn;
    reg [31:0] vector_observed_frame;
    reg vector_control_observation;
    wire signed [31:0] vector_frame_delta=vector_observed_frame-vector_stop_frame;
    assign vector_observation_stale=vector_observation_age>=VECTOR_OBSERVATION_CLKS-1 &&
        !control_observation;
    wire vector_policy_ok=mode_available && motor_online && status==FOLLOW &&
        stable_samples>=2 &&
        !triple_blocked && requested_speed!=0 && !recovery_stop_pending && !vector_tx_fault &&
        !exit_key && !select_key && !menu_key;
    wire [7:0] vector_linear_grade=requested_speed>SPEED_LIMIT ? SPEED_LIMIT : requested_speed;
    wire [7:0] vector_turn_grade=requested_speed>TURN_SPEED_LIMIT ? TURN_SPEED_LIMIT : requested_speed;
    reg [15:0] vector_linear_cap,vector_turn_cap,vector_turn_move_limit;
    function [15:0] move_turn_bound_at_grade;
        input [7:0] grade;
        reg [15:0] cap;
        begin
            cap=grade*VECTOR_SPEED_SCALE;
            move_turn_bound_at_grade=((cap*7+2)/5)>>1;
        end
    endfunction
    function [15:0] move_turn_bound_lookup;
        input [7:0] grade;
        begin
            // Each expression is elaboration-time constant for board grades.
            // Preserve the original cap truncation and both integer floors.
            if(SPEED_LIMIT<=10) begin
                case(grade)
                    0:move_turn_bound_lookup=move_turn_bound_at_grade(0);
                    1:move_turn_bound_lookup=move_turn_bound_at_grade(1);
                    2:move_turn_bound_lookup=move_turn_bound_at_grade(2);
                    3:move_turn_bound_lookup=move_turn_bound_at_grade(3);
                    4:move_turn_bound_lookup=move_turn_bound_at_grade(4);
                    5:move_turn_bound_lookup=move_turn_bound_at_grade(5);
                    6:move_turn_bound_lookup=move_turn_bound_at_grade(6);
                    7:move_turn_bound_lookup=move_turn_bound_at_grade(7);
                    8:move_turn_bound_lookup=move_turn_bound_at_grade(8);
                    9:move_turn_bound_lookup=move_turn_bound_at_grade(9);
                    10:move_turn_bound_lookup=move_turn_bound_at_grade(10);
                    default:move_turn_bound_lookup=move_turn_bound_at_grade(SPEED_LIMIT);
                endcase
            end else move_turn_bound_lookup=move_turn_bound_at_grade(grade);
        end
    endfunction
    // Keep the IR grade comparison out of the request/STOP-reply path. A lower
    // grade reaches these caps one clock later; the motor independently applies
    // the current IR limits. The raw zero-grade policy above still stops at once.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            vector_linear_cap<=0;vector_turn_cap<=0;vector_turn_move_limit<=0;vector_motor_ready<=0;
        end else begin
            vector_linear_cap<=vector_linear_grade*VECTOR_SPEED_SCALE;
            vector_turn_move_limit<=move_turn_bound_lookup(vector_linear_grade);
            vector_turn_cap<=vector_turn_grade*VECTOR_SPEED_SCALE;
            vector_motor_ready<=motor_ready;
        end
    end
    // Evaluate both fixed reference axes before selecting their signed result.
    // A late width/height mux removes reference_use_width from the beginning
    // of the subtract -> deadzone -> magnitude -> sign path. This is purely
    // combinational factoring: observations and control latency are unchanged.
    function signed [15:0] vector_linear_at_axis;
        input [15:0] reference_size,measured_size,cap;
        input alignment;
        input [1:0] previous_bias;
        reg [15:0] deadzone,exit_deadzone,size_error,magnitude;
        reg [3:0] direction;
        begin
            deadzone=(reference_size>>HEIGHT_DEADZONE_SHIFT)>HEIGHT_DEADZONE_MIN ?
                (reference_size>>HEIGHT_DEADZONE_SHIFT) : HEIGHT_DEADZONE_MIN;
            exit_deadzone=deadzone>1 ? deadzone>>1 : 16'd1;
            direction=NONE;
            if(measured_size+deadzone<reference_size)direction=FORWARD;
            else if(FOLLOW_REVERSE_ENABLE && measured_size>reference_size+deadzone)direction=BACKWARD;
            else if(FOLLOW_REVERSE_ENABLE && previous_bias==1 &&
                measured_size+exit_deadzone<reference_size)direction=FORWARD;
            else if(FOLLOW_REVERSE_ENABLE && previous_bias==2 &&
                measured_size>reference_size+exit_deadzone)direction=BACKWARD;
            size_error=abs_difference(reference_size,measured_size);
            magnitude=VECTOR_MIN_LINEAR;
            if({2'b0,size_error}>={deadzone,2'b0})magnitude=cap;
            else if({1'b0,size_error}>={deadzone,1'b0})begin
                magnitude=(cap*3+2)>>2;
                if(magnitude<VECTOR_MIN_LINEAR)magnitude=VECTOR_MIN_LINEAR;
            end
            if(magnitude>cap)magnitude=cap;
            vector_linear_at_axis=0;
            if(!alignment)begin
                if(direction==FORWARD)vector_linear_at_axis=$signed(magnitude);
                else if(FOLLOW_REVERSE_ENABLE && direction==BACKWARD)
                    vector_linear_at_axis=-$signed(magnitude);
            end
        end
    endfunction
    wire signed [15:0] vector_height_linear=vector_linear_at_axis(
        reference_height,best_h,vector_linear_cap,matched_hard_alignment,distance_bias);
    wire signed [15:0] vector_width_linear=vector_linear_at_axis(
        reference_width,best_w,vector_linear_cap,matched_hard_alignment,distance_bias);
    reg signed [15:0] vector_sample_linear,vector_sample_turn;
    reg [15:0] vector_turn_magnitude;
    // With a positive minimum, every nonzero cap produces a nonzero linear
    // magnitude. Decide pivot sizing before size quantization and sign encoding.
    // Unusual minima keep the generic word test; proximity remains later.
    wire vector_linear_sample_nonzero=!matched_hard_alignment && vector_linear_cap!=0 &&
        (matched_distance==FORWARD || (FOLLOW_REVERSE_ENABLE && matched_distance==BACKWARD));
    always @(*) begin
        vector_sample_linear=reference_use_width ? vector_width_linear : vector_height_linear;
        vector_turn_magnitude=({1'b0,center_error}+17'd32)>>6;
        if(((VECTOR_MIN_LINEAR>0 && VECTOR_MIN_LINEAR<=65535) ?
            !vector_linear_sample_nonzero : vector_sample_linear==0) &&
            vector_turn_magnitude<VECTOR_MIN_PIVOT)
            vector_turn_magnitude=VECTOR_MIN_PIVOT;
        else if(vector_turn_magnitude<VECTOR_MIN_TURN) vector_turn_magnitude=VECTOR_MIN_TURN;
        if(vector_turn_magnitude>vector_turn_cap) vector_turn_magnitude=vector_turn_cap;
        vector_sample_turn=0;
        if(matched_bias==1) vector_sample_turn=-$signed(vector_turn_magnitude);
        else if(matched_bias==2) vector_sample_turn=$signed(vector_turn_magnitude);
        if(approach_mode && (!front_valid || front_cm<=APPROACH_STOP_CM)) begin
            vector_sample_linear=0;vector_sample_turn=0;
        end
        if(FOLLOW_PROXIMITY_ENABLE) begin
            if(vector_sample_linear>0 && (!front_valid || front_cm<=20)) vector_sample_linear=0;
            if(vector_sample_linear<0 && (!rear_valid || rear_cm<=20)) vector_sample_linear=0;
            if(vector_sample_turn<0 && (!left_valid || left_cm<=12)) vector_sample_turn=0;
            if(vector_sample_turn>0 && (!right_valid || right_cm<=12)) vector_sample_turn=0;
        end
    end
    // Register one complete measured request and its frame identity before
    // sign confirmation and acceleration. This splits size/center arithmetic
    // from the vector state machine without mixing observations. WAIT samples
    // cannot become motion when recovery changes status to FOLLOW on this edge.
    // Raw mode/exit/zero-grade/online policy still gates the outputs directly.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            vector_observed_linear<=0;vector_observed_turn<=0;
            vector_observed_frame<=0;vector_control_observation<=0;
        end else begin
            vector_control_observation<=FOLLOW_VECTOR_ENABLE && control_observation &&
                status==FOLLOW && staged_status==FOLLOW && mode_available && motor_online &&
                requested_speed!=0 && !exit_key && !select_key && !menu_key;
            if(FOLLOW_VECTOR_ENABLE && control_observation) begin
                vector_observed_linear<=vector_sample_linear;
                vector_observed_turn<=vector_sample_turn;
                vector_observed_frame<=staged_frame;
            end
        end
    end
    function [1:0] vector_sign;
        input signed [15:0] value;
        begin vector_sign=value>0 ? 2'd1 : value<0 ? 2'd2 : 2'd0;end
    endfunction
    function [15:0] vector_magnitude;
        input signed [15:0] value;
        begin vector_magnitude=value<0 ? -value : value;end
    endfunction
    function signed [15:0] vector_accelerate;
        input signed [15:0] previous,goal;
        reg [15:0] old_magnitude,new_magnitude,limited;
        begin
            old_magnitude=vector_magnitude(previous);new_magnitude=vector_magnitude(goal);
            limited=new_magnitude;
            // Deceleration and zero have priority over the acceleration ramp.
            if(new_magnitude>old_magnitude+VECTOR_ACCEL_STEP)
                limited=old_magnitude+VECTOR_ACCEL_STEP;
            vector_accelerate=goal<0 ? -$signed(limited) : $signed(limited);
        end
    endfunction
    function [3:0] vector_motion_code;
        input signed [15:0] longitudinal,yaw;
        begin
            if(longitudinal>0) vector_motion_code=yaw<0 ? LEFT_ARC : yaw>0 ? RIGHT_ARC : FORWARD;
            else if(longitudinal<0) vector_motion_code=BACKWARD;
            else vector_motion_code=yaw<0 ? 4'd5 : yaw>0 ? 4'd6 : NONE;
        end
    endfunction
    wire [1:0] vector_sample_linear_sign=vector_sign(vector_observed_linear);
    wire [1:0] vector_sample_turn_sign=vector_sign(vector_observed_turn);
    wire vector_turn_opposite=vector_last_turn_sign!=0 && vector_sample_turn_sign!=0 &&
        vector_sample_turn_sign!=vector_last_turn_sign;
    wire vector_turn_confirmed=!vector_turn_opposite ||
        (vector_turn_candidate_seen && vector_turn_candidate==vector_sample_turn_sign);
    wire signed [15:0] vector_confirmed_turn=vector_turn_confirmed ? vector_observed_turn : 16'sd0;
    wire vector_linear_reverse=vector_last_linear_sign!=0 && vector_sample_linear_sign!=0 &&
        vector_sample_linear_sign!=vector_last_linear_sign;
    wire vector_pivot_reverse=vector_linear_reg==0 && vector_observed_linear==0 &&
        vector_turn_opposite && vector_turn_confirmed;
    wire signed [15:0] vector_ramped_linear=vector_accelerate(vector_linear_reg,vector_observed_linear);
    wire signed [15:0] vector_ramped_turn=vector_accelerate(vector_turn_reg,vector_confirmed_turn);
    wire vector_sample_zero=vector_observed_linear==0 && vector_confirmed_turn==0;
    wire vector_request_nonzero=vector_linear_reg!=0 || vector_turn_reg!=0;
    // Break public IR/navigation decode -> vector policy -> motor STOP/frame
    // selection. The local state gates still cancel a WAIT/LOST, mode change,
    // stale observation or recovery STOP on its original publication cycle.
    reg vector_policy_sample;
    always @(posedge clk or negedge reset_n)
        if(!reset_n) vector_policy_sample<=0;
        else vector_policy_sample<=vector_policy_ok;
    wire vector_output_allowed=FOLLOW_VECTOR_ENABLE && vector_policy_sample &&
        mode_available && motor_online && status==FOLLOW && stable_samples>=2 && requested_speed!=0 &&
        (!approach_mode || (front_valid && front_cm>APPROACH_STOP_CM)) &&
        !triple_blocked && !recovery_stop_pending && !vector_tx_fault &&
        !vector_observation_stale && vector_state==VECTOR_ACTIVE;
    wire signed [15:0] vector_limited_linear=vector_linear_reg>$signed(vector_linear_cap) ?
        $signed(vector_linear_cap) : vector_linear_reg< -$signed(vector_linear_cap) ?
        -$signed(vector_linear_cap) : vector_linear_reg;
    wire signed [15:0] vector_limited_turn=vector_turn_reg>$signed(vector_turn_cap) ?
        $signed(vector_turn_cap) : vector_turn_reg< -$signed(vector_turn_cap) ?
        -$signed(vector_turn_cap) : vector_turn_reg;
    // While moving, the differential is bounded by both the linear magnitude and
    // half of the motor wheel limit (1.4x linear cap), so the inner wheels can
    // never reverse. A stationary request keeps the full turn limit so in-place
    // alignment still reaches the whole range.
    wire [15:0] vector_linear_mag=vector_magnitude(vector_limited_linear);
    wire [15:0] vector_turn_mag=vector_magnitude(vector_limited_turn);
    wire [15:0] vector_turn_move_bound=vector_linear_mag<vector_turn_move_limit ?
        vector_linear_mag : vector_turn_move_limit;
    wire [15:0] vector_turn_bounded=vector_linear_mag==0 ? vector_turn_mag :
        (vector_turn_mag>vector_turn_move_bound ? vector_turn_move_bound : vector_turn_mag);
    wire signed [15:0] vector_turn_output=vector_limited_turn[15] ?
        -$signed(vector_turn_bounded) : $signed(vector_turn_bounded);
    assign follow_linear=vector_output_allowed ? vector_limited_linear : 16'sd0;
    assign follow_turn=vector_output_allowed ? vector_turn_output : 16'sd0;
    // The motor STOP policy needs request activity, not another comparison of
    // the signed, saturated words. For a positive cap the clamp is nonzero iff
    // its input is; a zero cap is zero and a signed-negative cap is nonzero.
    // Retain this identity even for parameter values outside the board limits.
    assign follow_request_nonzero=vector_output_allowed &&
        (vector_linear_cap[15] || ((|vector_linear_cap) && (|vector_linear_reg)) ||
         vector_turn_cap[15] || ((|vector_turn_cap) && (|vector_turn_reg)));
    assign vector_motion_active=follow_request_nonzero;
    wire [15:0] vector_display_magnitude=vector_magnitude(follow_linear)>vector_magnitude(follow_turn) ?
        vector_magnitude(follow_linear) : vector_magnitude(follow_turn);
    assign vector_display_grade=vector_motion_active ?
        (vector_display_magnitude+VECTOR_SPEED_SCALE-1)/VECTOR_SPEED_SCALE : 8'd0;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) vector_observation_age<=VECTOR_OBSERVATION_CLKS;
        else if(!FOLLOW_VECTOR_ENABLE || status!=FOLLOW || !mode_available || exit_key || select_key || menu_key)
            vector_observation_age<=VECTOR_OBSERVATION_CLKS;
        else if(control_observation) vector_observation_age<=0;
        else if(vector_observation_age<VECTOR_OBSERVATION_CLKS)
            vector_observation_age<=vector_observation_age+1'b1;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin vector_tx_age<=0;vector_tx_fault<=0;end
        else if(!FOLLOW_VECTOR_ENABLE || status!=FOLLOW || !mode_available || exit_key || select_key || menu_key) begin
            vector_tx_age<=0;vector_tx_fault<=0;
        end else if(!vector_motion_active) vector_tx_age<=0;
        else if(motor_motion_committed && motor_transmitted_motion!=NONE) vector_tx_age<=0;
        else if(vector_tx_age>=TURN_TX_WAIT_CLKS-1) vector_tx_fault<=1;
        else vector_tx_age<=vector_tx_age+1'b1;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            vector_state<=VECTOR_IDLE;vector_linear_reg<=0;vector_turn_reg<=0;
            vector_last_linear_sign<=0;vector_last_turn_sign<=0;vector_turn_candidate<=0;
            vector_turn_candidate_seen<=0;vector_saw_not_ready<=0;vector_stop_ready_armed<=0;vector_stop_frame<=0;
        end else if(!FOLLOW_VECTOR_ENABLE || status!=FOLLOW || !mode_available || exit_key || select_key || menu_key) begin
            vector_state<=VECTOR_IDLE;vector_linear_reg<=0;vector_turn_reg<=0;
            vector_last_linear_sign<=0;vector_last_turn_sign<=0;vector_turn_candidate<=0;
            vector_turn_candidate_seen<=0;vector_saw_not_ready<=0;vector_stop_ready_armed<=0;vector_stop_frame<=0;
        end else begin
            if(vector_control_observation) begin
                if(vector_turn_opposite) begin
                    vector_turn_candidate<=vector_sample_turn_sign;vector_turn_candidate_seen<=1;
                end else begin vector_turn_candidate<=0;vector_turn_candidate_seen<=0;end
            end
            case(vector_state)
                VECTOR_IDLE: begin
                    vector_linear_reg<=0;vector_turn_reg<=0;
                    if(vector_policy_ok && vector_motor_ready && vector_control_observation && !vector_sample_zero) begin
                        vector_state<=VECTOR_ACTIVE;
                        vector_linear_reg<=vector_ramped_linear;vector_turn_reg<=vector_ramped_turn;
                        vector_last_linear_sign<=vector_sign(vector_ramped_linear);
                        vector_last_turn_sign<=vector_sign(vector_ramped_turn);
                    end
                end
                VECTOR_ACTIVE: begin
                    if(!vector_policy_ok || vector_observation_stale ||
                       (vector_control_observation && (vector_sample_zero || vector_linear_reverse || vector_pivot_reverse))) begin
                        vector_state<=VECTOR_STOP;vector_linear_reg<=0;vector_turn_reg<=0;
                        // Ignore the reply sampled before this STOP request.
                        // Only post-entry samples may establish its low/high proof.
                        vector_saw_not_ready<=0;vector_stop_ready_armed<=0;
                    end else if(vector_control_observation) begin
                        vector_linear_reg<=vector_ramped_linear;vector_turn_reg<=vector_ramped_turn;
                        if(vector_ramped_linear!=0) vector_last_linear_sign<=vector_sign(vector_ramped_linear);
                        if(vector_ramped_turn!=0) vector_last_turn_sign<=vector_sign(vector_ramped_turn);
                    end
                end
                VECTOR_STOP: begin
                    vector_linear_reg<=0;vector_turn_reg<=0;
                    vector_stop_ready_armed<=1;
                    if(vector_stop_ready_armed && !vector_motor_ready) vector_saw_not_ready<=1;
                    if(vector_stop_ready_armed && vector_saw_not_ready && vector_motor_ready) begin
                        vector_state<=VECTOR_OBSERVE;vector_last_linear_sign<=0;vector_last_turn_sign<=0;
                        vector_turn_candidate<=0;vector_turn_candidate_seen<=0;
                        vector_stop_frame<=identity_new_frame ? frame_id : last_input_frame;
                    end
                end
                VECTOR_OBSERVE: begin
                    vector_linear_reg<=0;vector_turn_reg<=0;
                    if(vector_policy_ok && vector_motor_ready && vector_control_observation && vector_frame_delta>0 &&
                       !vector_sample_zero) begin
                        vector_state<=VECTOR_ACTIVE;
                        vector_linear_reg<=vector_ramped_linear;vector_turn_reg<=vector_ramped_turn;
                        vector_last_linear_sign<=vector_sign(vector_ramped_linear);
                        vector_last_turn_sign<=vector_sign(vector_ramped_turn);
                    end
                end
            endcase
        end
    end
endmodule
