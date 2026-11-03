// IR goal editing, planning, turn/stop/forward path following.
// Live mapping is optional; planning always drains and freezes map writes.
module navigation_control #(
    parameter integer CLK_FREQ_HZ=50000000,
    parameter integer CELL_SHIFT=4, MULTI_POSE_MAPPING=0,
    parameter integer CELL_MM=(1<<CELL_SHIFT), GRID_SIZE=128, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y),
    parameter integer GOAL_STEP_CELLS=6, NAV_SPEED=2, NAV_TURN_SPEED=NAV_SPEED,
    parameter integer TURN_TOLERANCE_TENTHS=30, MOVE_YAW_TOLERANCE_TENTHS=60,
    parameter integer SETTLE_MS=200, STOP_TIMEOUT_MS=2000,
    parameter integer TURN_TIMEOUT_MS=12000, MOVE_TIMEOUT_MS=30000,
    parameter integer PLAN_TIMEOUT_MS=30000, TOF_STALE_MS=500,
    parameter integer FRONT_STOP_MM=100, GOAL_TOLERANCE_MM=32,
    parameter integer STOP_EARLY_MM=12, CROSS_TRACK_MM=32,
    parameter integer ENABLE_CROSS_TRACK_STOP=1,
    parameter integer ENABLE_LIVE_MAPPING=0,
    parameter integer AUTO_REPLAN_OFF_PATH=0,OFF_PATH_RETRY_MS=1000,OFF_PATH_MAX_RETRIES=3,
    parameter integer USE_RAW_ODOMETRY_PROGRESS=0,
    parameter integer ENABLE_CONTINUOUS_STEER=0, NAV_ARC_SPEED=3,
    parameter integer STEER_UPDATE_MS=50,
    parameter integer STEER_ENTER_TENTHS=30, STEER_EXIT_TENTHS=10,
    parameter integer CROSS_TRACK_DEADBAND_MM=16, CROSS_TRACK_BIAS_LIMIT_TENTHS=50,
    parameter integer ENABLE_FRONT_STOP=1, ENABLE_HOLONOMIC=0,
    parameter integer ENABLE_FORWARD_TRIM=0,
    parameter integer STOP_ON_MOVE_YAW_DEVIATION=0,
    parameter integer ENABLE_HOLONOMIC_TRACKING=0,NAV_YAW_TRIM_BITS=2,NAV_YAW_TRIM_MAX=1,
    parameter integer TRACK_UPDATE_MS=20,TRACK_YAW_ENTER_TENTHS=5,TRACK_YAW_EXIT_TENTHS=2,
    parameter integer TRACK_YAW_STRONG_TENTHS=30,TRACK_CROSS_DEADBAND_MM=5,
    parameter integer TRACK_CROSS_BIAS_LIMIT_TENTHS=30,TRACK_CROSS_GAIN=2,
    parameter integer TRACK_SLOW_YAW_TENTHS=30,TRACK_SLOW_CROSS_MM=15,NAV_RECOVERY_SPEED=2,
    parameter integer NAV_START_SPEED=NAV_SPEED,NAV_START_BOOST_MS=0
)(
    input wire clk,reset_n,enable,
    input wire ir_valid,
    input wire [7:0] ir_command,
    input wire [2:0] scan_state,
    input wire scan_resume_event,
    input wire [15:0] scan_samples,
    input wire map_busy,map_clear_done,
    input wire localization_replan,
    input wire pose_valid,pose_fault,pose_moved,
    input wire signed [15:0] pose_x_mm,pose_y_mm,
    input wire signed [15:0] raw_pose_x_mm,raw_pose_y_mm,
    input wire [GRID_BITS-1:0] pose_cell_x,pose_cell_y,
    input wire signed [31:0] heading_tenths,
    input wire heading_valid,yaw_update,motor_online,stop_confirmed,
    input wire [63:0] motor_speeds,
    input wire [15:0] distance_mm,
    input wire range_valid,range_update,tof_online,
    input wire planner_busy,planner_done,planner_success,
    input wire [3:0] planner_fail,
    input wire signed [12:0] planner_body_heading,
    input wire waypoint_busy,waypoint_valid,waypoint_last,
    input wire [GRID_BITS-1:0] waypoint_x,waypoint_y,
    output reg planner_start,planner_abort,path_rewind,waypoint_next,
    output reg scan_restart,
    output wire map_sample_allow,
    output wire nav_live_mapping_active,
    output wire nav_owner,
    output wire manual_takeover,
    output reg [3:0] nav_motion,
    output wire [7:0] nav_speed,
    output reg signed [NAV_YAW_TRIM_BITS-1:0] nav_yaw_trim,
    output reg [3:0] nav_status,nav_fault,
    output reg [GRID_BITS-1:0] goal_x_cell,goal_y_cell,
    output reg goal_valid,path_valid
);
    localparam [3:0] IDLE=0,SELECT=1,PREPLAN=2,PLAN=3,REWIND=4,FETCH=5,
        TURN=6,TURN_STOP=7,MOVE=8,MOVE_STOP=9,ARRIVED=10,STOPPED=11,FAULT=12,NOPATH=13,MANUAL_TAKEOVER=14;
    localparam integer SETTLE_CLKS=CLK_FREQ_HZ/1000*SETTLE_MS;
    localparam integer STOP_CLKS=CLK_FREQ_HZ/1000*STOP_TIMEOUT_MS;
    localparam integer TURN_CLKS=CLK_FREQ_HZ/1000*TURN_TIMEOUT_MS;
    localparam integer MOVE_CLKS=CLK_FREQ_HZ/1000*MOVE_TIMEOUT_MS;
    localparam integer PLAN_CLKS=CLK_FREQ_HZ/1000*PLAN_TIMEOUT_MS;
    localparam integer TOF_CLKS=CLK_FREQ_HZ/1000*TOF_STALE_MS;
    localparam integer START_BOOST_CLKS=CLK_FREQ_HZ/1000*NAV_START_BOOST_MS;
    localparam integer OFF_PATH_RETRY_CLKS=(CLK_FREQ_HZ/1000*OFF_PATH_RETRY_MS)>0 ? CLK_FREQ_HZ/1000*OFF_PATH_RETRY_MS : 1;
    localparam integer RETRY_BITS=OFF_PATH_MAX_RETRIES>0?$clog2(OFF_PATH_MAX_RETRIES+1):1;
    localparam integer STEER_CLKS=(CLK_FREQ_HZ/1000*STEER_UPDATE_MS)>0 ?
                                  CLK_FREQ_HZ/1000*STEER_UPDATE_MS : 1;
    localparam integer STEER_COUNT_BITS=STEER_CLKS>1 ? $clog2(STEER_CLKS) : 1;
    reg [3:0] state;
    reg [31:0] timer,settle,tof_age;
    reg map_ready,map_frozen,first_point,target_last,new_yaw_after_stop;
    reg [GRID_BITS-1:0] previous_x,previous_y,target_x,target_y;
    reg signed [12:0] target_heading;
    reg signed [12:0] current_heading;
    reg signed [12:0] held_heading;
    reg signed [NAV_YAW_TRIM_BITS-1:0] yaw_trim;
    reg [3:0] translation_motion;
    reg [1:0] corrections;
    reg correcting,request_sent,previous_scanning;
    reg [RETRY_BITS-1:0] off_path_retries;
    reg signed [15:0] retry_origin_x,retry_origin_y;
    reg signed [15:0] recovery_origin_x,recovery_origin_y;
    wire signed [15:0] progress_x_mm=USE_RAW_ODOMETRY_PROGRESS?raw_pose_x_mm:pose_x_mm;
    wire signed [15:0] progress_y_mm=USE_RAW_ODOMETRY_PROGRESS?raw_pose_y_mm:pose_y_mm;
    wire signed [16:0] retry_progress_x=$signed({progress_x_mm[15],progress_x_mm})-$signed({retry_origin_x[15],retry_origin_x});
    wire signed [16:0] retry_progress_y=$signed({progress_y_mm[15],progress_y_mm})-$signed({retry_origin_y[15],retry_origin_y});
    wire signed [16:0] recovery_progress_x=$signed({progress_x_mm[15],progress_x_mm})-$signed({recovery_origin_x[15],recovery_origin_x});
    wire signed [16:0] recovery_progress_y=$signed({progress_y_mm[15],progress_y_mm})-$signed({recovery_origin_y[15],recovery_origin_y});
    wire retry_made_progress=retry_progress_x>=2*CELL_MM || retry_progress_x<=-2*CELL_MM ||
        retry_progress_y>=2*CELL_MM || retry_progress_y<=-2*CELL_MM;
    wire recovery_made_progress=recovery_progress_x>=CELL_MM || recovery_progress_x<=-CELL_MM ||
        recovery_progress_y>=CELL_MM || recovery_progress_y<=-CELL_MM;
    wire auto_retry_pending=AUTO_REPLAN_OFF_PATH && state==FAULT && nav_fault==7 && goal_valid &&
        off_path_retries<OFF_PATH_MAX_RETRIES;
    wire scanning=scan_state>=1 && scan_state<=4;
    wire edit_key=ir_command==8'h5A || ir_command==8'h08 || ir_command==8'h18 || ir_command==8'h52;
    wire manual_key=ir_command==8'h40 || ir_command==8'h19 || ir_command==8'h07 || ir_command==8'h09 ||
        ir_command==8'h44 || ir_command==8'h43 || ir_command==8'h16 || ir_command==8'h0D;
    wire moving=state==TURN || state==MOVE;
    wire active=state>=PREPLAN && state<=MOVE_STOP;
    wire tof_good=tof_online && range_valid && tof_age<TOF_CLKS;
    wire healthy=pose_valid && !pose_fault && heading_valid && motor_online && tof_good;
    wire front_stop_active=ENABLE_FRONT_STOP && distance_mm<=FRONT_STOP_MM;
    // Break the IMU decoder/relative-heading/wrap/steering combinational path.
    // Track valid relative headings every clock so a scan-origin rebase is also
    // observed. Freshness, faults and emergency STOP remain live gates below.
    wire signed [12:0] raw_error=(ENABLE_HOLONOMIC?held_heading:target_heading)-current_heading;
    wire signed [12:0] turn_error=raw_error>1800 ? raw_error-3600 : raw_error < -1800 ? raw_error+3600 : raw_error;
    wire aligned=turn_error>=-TURN_TOLERANCE_TENTHS && turn_error<=TURN_TOLERANCE_TENTHS;
    wire yaw_deviation=turn_error>MOVE_YAW_TOLERANCE_TENTHS || turn_error < -MOVE_YAW_TOLERANCE_TENTHS;
    localparam integer WAYPOINT_OFFSET_X_MM=(ENABLE_HOLONOMIC||GRID_X!=GRID_Y)?CELL_MM/2-(GRID_X%2)*(CELL_MM/2):0;
    localparam integer WAYPOINT_OFFSET_Y_MM=(ENABLE_HOLONOMIC||GRID_X!=GRID_Y)?CELL_MM/2-(GRID_Y%2)*(CELL_MM/2):0;
    // Snapshot metric coordinates on the SAME edge as their waypoint cells.
    // Conversion then stays outside the live remaining/STOP/settle path.
    reg signed [15:0] target_mm_x,target_mm_y;
    wire signed [15:0] remaining=target_heading==0 ? target_mm_x-pose_x_mm :
        target_heading==1800 ? pose_x_mm-target_mm_x :
        target_heading==900 ? target_mm_y-pose_y_mm : pose_y_mm-target_mm_y;
    wire signed [15:0] cross_error=(target_heading==0 || target_heading==1800) ?
        pose_y_mm-target_mm_y : pose_x_mm-target_mm_x;
    wire cross_bad=ENABLE_CROSS_TRACK_STOP &&
        (cross_error>CROSS_TRACK_MM || cross_error < -CROSS_TRACK_MM);
    reg signed [15:0] goal_mm_x,goal_mm_y;
    // Goal editing leaves SELECT; confirmation enters PREPLAN on a later
    // edge. The metric registers are therefore coherent before at_goal can
    // be consumed, without putting cell conversion on the live state path.
    always @(posedge clk)begin
        goal_mm_x<=($signed({1'b0,goal_x_cell})-(GRID_X/2))*CELL_MM+WAYPOINT_OFFSET_X_MM;
        goal_mm_y<=($signed({1'b0,goal_y_cell})-(GRID_Y/2))*CELL_MM+WAYPOINT_OFFSET_Y_MM;
    end
    wire signed [16:0] goal_dx=goal_mm_x-pose_x_mm,goal_dy=goal_mm_y-pose_y_mm;
    wire at_goal=goal_dx>=-GOAL_TOLERANCE_MM && goal_dx<=GOAL_TOLERANCE_MM &&
                 goal_dy>=-GOAL_TOLERANCE_MM && goal_dy<=GOAL_TOLERANCE_MM;
    // Keep target_heading cardinal: remaining/cross_error describe the route,
    // while only this separate, bounded steering heading bends a MOVE segment.
    // A newly accepted Euler pulse is consumed at most once. Every decision
    // snapshots heading, cross-track and segment direction before the narrow
    // arithmetic stages, so a later UART/IMU update cannot mix this sample.
    reg [STEER_COUNT_BITS-1:0] steer_count;
    reg pending_yaw,previous_steer_move;
    reg [3:0] steer_phase;
    reg [1:0] steer_direction; // 0 straight; 1 left arc; 2 right arc.
    reg signed [12:0] latest_steer_yaw,steer_sample_yaw,steer_sample_target;
    reg signed [15:0] steer_sample_cross,steer_left_cross,steer_cross_excess;
    reg signed [12:0] steer_cross_bias,steer_unwrapped_heading,steering_heading;
    reg signed [12:0] steer_raw_error,steering_error;
    wire steer_cancel_key=ir_valid && (manual_key || ir_command==8'h15 ||
        ir_command==8'h42 || ir_command==8'h4A || ir_command==8'h1C);
    wire steer_cancel=ENABLE_HOLONOMIC || !ENABLE_CONTINUOUS_STEER || !enable || scanning || state!=MOVE ||
        !healthy || yaw_deviation || cross_bad || remaining<=STOP_EARLY_MM ||
        front_stop_active || steer_cancel_key;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            steer_count<=0;pending_yaw<=0;previous_steer_move<=0;steer_phase<=0;steer_direction<=0;
            latest_steer_yaw<=0;steer_sample_yaw<=0;steer_sample_target<=0;
            steer_sample_cross<=0;steer_left_cross<=0;steer_cross_excess<=0;
            steer_cross_bias<=0;steer_unwrapped_heading<=0;steering_heading<=0;
            steer_raw_error<=0;steering_error<=0;
        end else begin
            previous_steer_move<=state==MOVE;
            if(steer_cancel || !previous_steer_move) begin
                steer_count<=0;pending_yaw<=0;steer_phase<=0;steer_direction<=0;
                latest_steer_yaw<=0;steer_sample_yaw<=0;steer_sample_target<=0;
                steer_sample_cross<=0;steer_left_cross<=0;steer_cross_excess<=0;
                steer_cross_bias<=0;steer_unwrapped_heading<=0;steering_heading<=0;
                steer_raw_error<=0;steering_error<=0;
            end else begin
                if(steer_count<STEER_CLKS-1) steer_count<=steer_count+1'b1;
                if(yaw_update && heading_valid) begin
                    latest_steer_yaw<=heading_tenths[12:0];pending_yaw<=1;
                end
                case(steer_phase)
                    0:if(steer_count>=STEER_CLKS-1 && (pending_yaw || yaw_update)) begin
                        steer_count<=0;pending_yaw<=0;
                        steer_sample_yaw<=yaw_update ? heading_tenths[12:0] : latest_steer_yaw;
                        steer_sample_target<=target_heading;steer_sample_cross<=cross_error;
                        steer_phase<=1;
                    end
                    1:begin
                        // Positive left-of-route error needs a rightward bias.
                        case(steer_sample_target)
                            13'sd0,-13'sd900:steer_left_cross<=steer_sample_cross;
                            default:steer_left_cross<=-steer_sample_cross;
                        endcase
                        steer_phase<=2;
                    end
                    2:begin
                        if(steer_left_cross>CROSS_TRACK_DEADBAND_MM)
                            steer_cross_excess<=steer_left_cross-CROSS_TRACK_DEADBAND_MM;
                        else if(steer_left_cross < -CROSS_TRACK_DEADBAND_MM)
                            steer_cross_excess<=steer_left_cross+CROSS_TRACK_DEADBAND_MM;
                        else steer_cross_excess<=0;
                        steer_phase<=3;
                    end
                    3:begin
                        // One tenth-degree per mm OUTSIDE the deadband, capped.
                        if(steer_cross_excess>CROSS_TRACK_BIAS_LIMIT_TENTHS)
                            steer_cross_bias<=-CROSS_TRACK_BIAS_LIMIT_TENTHS;
                        else if(steer_cross_excess < -CROSS_TRACK_BIAS_LIMIT_TENTHS)
                            steer_cross_bias<=CROSS_TRACK_BIAS_LIMIT_TENTHS;
                        else steer_cross_bias<=-steer_cross_excess[12:0];
                        steer_phase<=4;
                    end
                    4:begin
                        steer_unwrapped_heading<=steer_sample_target+steer_cross_bias;steer_phase<=5;
                    end
                    5:begin
                        steering_heading<=steer_unwrapped_heading>1800 ? steer_unwrapped_heading-3600 :
                            steer_unwrapped_heading < -1800 ? steer_unwrapped_heading+3600 : steer_unwrapped_heading;
                        steer_phase<=6;
                    end
                    6:begin steer_raw_error<=steering_heading-steer_sample_yaw;steer_phase<=7;end
                    7:begin
                        steering_error<=steer_raw_error>1800 ? steer_raw_error-3600 :
                            steer_raw_error < -1800 ? steer_raw_error+3600 : steer_raw_error;
                        steer_phase<=8;
                    end
                    8:begin
                        case(steer_direction)
                            1:if(steering_error<=-STEER_ENTER_TENTHS) steer_direction<=2;
                              else if(steering_error<=STEER_EXIT_TENTHS) steer_direction<=0;
                            2:if(steering_error>=STEER_ENTER_TENTHS) steer_direction<=1;
                              else if(steering_error>=-STEER_EXIT_TENTHS) steer_direction<=0;
                            default:if(steering_error>=STEER_ENTER_TENTHS) steer_direction<=1;
                                    else if(steering_error<=-STEER_ENTER_TENTHS) steer_direction<=2;
                        endcase
                        steer_phase<=0;
                    end
                    default:begin steer_phase<=0;steer_direction<=0;pending_yaw<=0;end
                endcase
            end
        end
    end
    // Navigation-only tracking. A bounded heading bias turns the velocity
    // back toward the same published segment; it does not change waypoints.
    // Odometry projects the actual IMU heading, so this small curved return
    // remains observable without inventing a lateral wheel displacement.
    wire signed [NAV_YAW_TRIM_BITS-1:0] tracked_yaw_trim;
    localparam integer TRACK_CLKS=CLK_FREQ_HZ/1000*TRACK_UPDATE_MS>0 ? CLK_FREQ_HZ/1000*TRACK_UPDATE_MS : 1;
    localparam integer TRACK_CB=TRACK_CLKS>1?$clog2(TRACK_CLKS):1;
    wire track_cancel=!enable||scanning||state!=MOVE||!healthy||yaw_deviation||cross_bad||
        remaining<=STOP_EARLY_MM||front_stop_active||steer_cancel_key;
    generate if(ENABLE_HOLONOMIC&&ENABLE_HOLONOMIC_TRACKING)begin:g_holonomic_tracking
        reg [TRACK_CB-1:0] count;
        reg [3:0] phase;
        reg signed [12:0] sample_held,sample_yaw,sample_route,bias,unwrapped_reference,reference_yaw,raw_track_error,track_error;
        reg signed [15:0] sample_cross,left_cross,excess;
        reg signed [NAV_YAW_TRIM_BITS-1:0] trim;
        assign tracked_yaw_trim=trim;
        always @(posedge clk or negedge reset_n)begin
            if(!reset_n)begin
                count<=TRACK_CLKS-1;phase<=0;trim<=0;
                sample_held<=0;sample_yaw<=0;sample_route<=0;sample_cross<=0;
                left_cross<=0;excess<=0;bias<=0;unwrapped_reference<=0;reference_yaw<=0;raw_track_error<=0;track_error<=0;
            end else if(track_cancel)begin count<=TRACK_CLKS-1;phase<=0;trim<=0;end
            else begin
                if(count<TRACK_CLKS-1)count<=count+1'b1;
                case(phase)
                    0:if(count>=TRACK_CLKS-1)begin
                        count<=0;sample_held<=held_heading;sample_yaw<=current_heading;
                        sample_route<=target_heading;sample_cross<=cross_error;phase<=1;
                    end
                    1:begin
                        // Error to the LEFT of travel has the same steering
                        // sign for forward, reverse and either strafe direction.
                        case(sample_route)
                            13'sd0,-13'sd900:left_cross<=sample_cross;
                            default:left_cross<=-sample_cross;
                        endcase
                        phase<=2;
                    end
                    2:begin
                        if(left_cross>TRACK_CROSS_DEADBAND_MM)excess<=left_cross-TRACK_CROSS_DEADBAND_MM;
                        else if(left_cross< -TRACK_CROSS_DEADBAND_MM)excess<=left_cross+TRACK_CROSS_DEADBAND_MM;
                        else excess<=0;
                        phase<=3;
                    end
                    3:begin
                        if(excess>TRACK_CROSS_BIAS_LIMIT_TENTHS/TRACK_CROSS_GAIN)bias<=-TRACK_CROSS_BIAS_LIMIT_TENTHS;
                        else if(excess< -TRACK_CROSS_BIAS_LIMIT_TENTHS/TRACK_CROSS_GAIN)bias<=TRACK_CROSS_BIAS_LIMIT_TENTHS;
                        else bias<=-excess*TRACK_CROSS_GAIN;
                        phase<=4;
                    end
                    4:begin unwrapped_reference<=sample_held+bias;phase<=5;end
                    5:begin
                        reference_yaw<=unwrapped_reference>1800?unwrapped_reference-3600:
                            unwrapped_reference< -1800?unwrapped_reference+3600:unwrapped_reference;
                        phase<=6;
                    end
                    6:begin raw_track_error<=reference_yaw-sample_yaw;phase<=7;end
                    7:begin
                        track_error<=raw_track_error>1800?raw_track_error-3600:
                            raw_track_error< -1800?raw_track_error+3600:raw_track_error;
                        phase<=8;
                    end
                    8:begin
                        if(NAV_YAW_TRIM_MAX>=2&&track_error>=TRACK_YAW_STRONG_TENTHS)trim<=-2;
                        else if(NAV_YAW_TRIM_MAX>=2&&track_error<=-TRACK_YAW_STRONG_TENTHS)trim<=2;
                        else if(track_error>=TRACK_YAW_ENTER_TENTHS)trim<=-1;
                        else if(track_error<=-TRACK_YAW_ENTER_TENTHS)trim<=1;
                        else if(track_error>=-TRACK_YAW_EXIT_TENTHS&&track_error<=TRACK_YAW_EXIT_TENTHS)trim<=0;
                        else if(trim>1)trim<=1;
                        else if(trim< -1)trim<=-1;
                        phase<=0;
                    end
                    default:begin phase<=0;trim<=0;end
                endcase
            end
        end
    end else begin:g_legacy_holonomic_trim
        assign tracked_yaw_trim=0;
    end endgenerate
    wire tracking_slow=ENABLE_HOLONOMIC&&ENABLE_HOLONOMIC_TRACKING&&state==MOVE&&
        (turn_error>=TRACK_SLOW_YAW_TENTHS||turn_error<=-TRACK_SLOW_YAW_TENTHS||
         cross_error>=TRACK_SLOW_CROSS_MM||cross_error<=-TRACK_SLOW_CROSS_MM);
    // Retain the AUTO session and its published route while the manual motor
    // controller owns motion. No automatic command can run in this state.
    assign nav_owner=enable && !scanning && state!=IDLE && state!=MANUAL_TAKEOVER;
    // Route retention is a control signal, independent of display formatting
    // (SELECT's display status also depends on the live STOP handshake).
    assign manual_takeover=state==MANUAL_TAKEOVER;
    assign nav_speed=nav_motion==0 ? 0 :
                     (nav_motion==5 || nav_motion==6) ? NAV_TURN_SPEED :
                     (nav_motion==7 || nav_motion==8) ? NAV_ARC_SPEED :
                     state==MOVE && START_BOOST_CLKS>0 && timer<START_BOOST_CLKS ? NAV_START_SPEED :
                     tracking_slow && NAV_RECOVERY_SPEED<NAV_SPEED ? NAV_RECOVERY_SPEED : NAV_SPEED;
    // This gate permits projection; manual capture separately requires 42.
    // Legacy single-pose mode freezes the map after navigation or translation.
    // Grant capture permission during execution. The top-level42 switch still
    // decides whether acquisition actually runs. PREPLAN/PLAN freeze the map.
    assign nav_live_mapping_active=enable && ENABLE_LIVE_MAPPING && active &&
        state!=PREPLAN && state!=PLAN && !pose_fault;
    assign map_sample_allow=MULTI_POSE_MAPPING ? ((!active || nav_live_mapping_active) && !pose_fault) :
        (!map_frozen && !active && state!=ARRIVED && state!=FAULT);
    always @* begin
        nav_motion=0;
        nav_yaw_trim=0;
        // The route direction and chassis heading are independent. Positive
        // world Y is body left at heading zero (same convention as odometry).
        if(held_heading==0) begin
            case(target_heading) 0:translation_motion=1;1800:translation_motion=2;
                900:translation_motion=3;default:translation_motion=4;endcase
        end else if(held_heading==1800) begin
            case(target_heading) 0:translation_motion=2;1800:translation_motion=1;
                900:translation_motion=4;default:translation_motion=3;endcase
        end else if(held_heading==900) begin
            case(target_heading) 0:translation_motion=4;1800:translation_motion=3;
                900:translation_motion=1;default:translation_motion=2;endcase
        end else begin
            case(target_heading) 0:translation_motion=3;1800:translation_motion=4;
                900:translation_motion=2;default:translation_motion=1;endcase
        end
        if(enable && !scanning && healthy) begin
            if(state==TURN && !aligned) nav_motion=turn_error>0 ? 4'd5 : 4'd6;
            else if(state==MOVE && !yaw_deviation && !cross_bad && remaining>STOP_EARLY_MM &&
                    !front_stop_active) begin
                if(ENABLE_HOLONOMIC)begin nav_motion=translation_motion;nav_yaw_trim=yaw_trim;end
                else if(ENABLE_CONTINUOUS_STEER && steer_direction==1) nav_motion=7;
                else if(ENABLE_CONTINUOUS_STEER && steer_direction==2) nav_motion=8;
                else begin
                    nav_motion=1;
                    if(ENABLE_FORWARD_TRIM)nav_yaw_trim=yaw_trim;
                end
            end
        end
        case(state)
            IDLE:nav_status=0;
            SELECT:nav_status=stop_confirmed ? 1 : 9;
            PREPLAN:nav_status=9;
            PLAN,REWIND,FETCH:nav_status=2;
            TURN:nav_status=4;
            MOVE:nav_status=5;
            TURN_STOP,MOVE_STOP:nav_status=9;
            ARRIVED:nav_status=6;
            STOPPED:nav_status=7;
            FAULT:nav_status=8;
            NOPATH:nav_status=3;
            MANUAL_TAKEOVER:nav_status=14;
            default:nav_status=8;
        endcase
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;timer<=0;settle<=0;tof_age<=TOF_CLKS;map_ready<=0;map_frozen<=0;
            first_point<=1;target_last<=0;new_yaw_after_stop<=0;previous_x<=GRID_X/2;previous_y<=GRID_Y/2;
            target_x<=GRID_X/2;target_y<=GRID_Y/2;target_heading<=0;current_heading<=0;corrections<=0;correcting<=0;request_sent<=0;previous_scanning<=0;
            target_mm_x<=WAYPOINT_OFFSET_X_MM;target_mm_y<=WAYPOINT_OFFSET_Y_MM;
            held_heading<=0;yaw_trim<=0;off_path_retries<=0;retry_origin_x<=0;retry_origin_y<=0;
            recovery_origin_x<=0;recovery_origin_y<=0;
            planner_start<=0;planner_abort<=0;path_rewind<=0;waypoint_next<=0;scan_restart<=0;
            nav_fault<=0;goal_x_cell<=GRID_X/2;goal_y_cell<=GRID_Y/2;goal_valid<=0;path_valid<=0;
        end else begin
            if(heading_valid) current_heading<=heading_tenths[12:0];
            // Bounded two-level forward correction. Existing instances with
            // max1 keep their original behaviour; max2 adds a stronger level
            // at3deg and releases that level at2deg. Mild trim still enters
            // at1deg and releases at0.3deg. Translation remains forward.
            if(!(ENABLE_HOLONOMIC||ENABLE_FORWARD_TRIM)||state!=MOVE||!healthy||yaw_deviation||cross_bad)yaw_trim<=0;
            else if(ENABLE_HOLONOMIC_TRACKING)yaw_trim<=tracked_yaw_trim;
            else if(ENABLE_FORWARD_TRIM && NAV_YAW_TRIM_MAX>=2 &&
                    (turn_error>=30 || (yaw_trim==-2 && turn_error>20)))yaw_trim<=-2;
            else if(ENABLE_FORWARD_TRIM && NAV_YAW_TRIM_MAX>=2 &&
                    (turn_error<=-30 || (yaw_trim==2 && turn_error< -20)))yaw_trim<=2;
            else if(turn_error>=10)yaw_trim<=-1;
            else if(turn_error<=-10)yaw_trim<=1;
            else if(turn_error>=-3&&turn_error<=3)yaw_trim<=0;
            else if(yaw_trim>1)yaw_trim<=1;
            else if(yaw_trim< -1)yaw_trim<=-1;
            previous_scanning<=scanning;
            planner_start<=0;planner_abort<=0;path_rewind<=0;waypoint_next<=0;scan_restart<=0;
            if(range_update) tof_age<=0;else if(tof_age<TOF_CLKS) tof_age<=tof_age+1'b1;
            if(map_clear_done) begin map_ready<=1;map_frozen<=0;end
            if(pose_moved && !MULTI_POSE_MAPPING) map_frozen<=1;
            if(AUTO_REPLAN_OFF_PATH && state==FAULT && nav_fault==7)begin
                if(timer<OFF_PATH_RETRY_CLKS)timer<=timer+1'b1;
            end else timer<=timer+1'b1;
            if(AUTO_REPLAN_OFF_PATH && state==MOVE && healthy && retry_made_progress)off_path_retries<=0;
            // A successful alignment followed by real encoder travel is not
            // a consecutive alignment failure. SLAM coordinate jumps cannot
            // clear either recovery budget in the production configuration.
            if(USE_RAW_ODOMETRY_PROGRESS && state==MOVE && healthy && aligned && recovery_made_progress)corrections<=0;
            // The motor interface completes STOP locally. Residual measured
            // wheel speed must not restart this settling timer indefinitely.
            if(stop_confirmed) begin
                if(settle<SETTLE_CLKS) settle<=settle+1'b1;
                if(yaw_update && heading_valid) new_yaw_after_stop<=1;
            end else begin settle<=0;new_yaw_after_stop<=0;end
            if(!enable) begin
                off_path_retries<=0;
                state<=IDLE;timer<=0;settle<=0;map_ready<=0;map_frozen<=0;
                goal_valid<=0;path_valid<=0;nav_fault<=0;planner_abort<=planner_busy;
                goal_x_cell<=GRID_X/2;goal_y_cell<=GRID_Y/2;
            end else if(MULTI_POSE_MAPPING && ir_valid && ir_command==8'h4A) begin
                off_path_retries<=0;
                state<=IDLE;timer<=0;settle<=0;planner_abort<=1;path_valid<=0;
                scan_restart<=1;map_ready<=0;map_frozen<=0;goal_valid<=0;nav_fault<=0;
                goal_x_cell<=GRID_X/2;goal_y_cell<=GRID_Y/2;
            end else if(ir_valid && ir_command==8'h15) begin
                off_path_retries<=0;
                // An automatic scan owns its PAUSED/STOP confirmation display.
                // Do not cover that status with an unrelated NAV STOP owner.
                state<=state==MANUAL_TAKEOVER ? MANUAL_TAKEOVER : (scanning || scan_state==3'd7) ? IDLE : STOPPED;
                timer<=0;settle<=0;nav_fault<=0;
                planner_abort<=1;
                if(state!=MANUAL_TAKEOVER)path_valid<=0;
            end else if(ir_valid && manual_key) begin
                off_path_retries<=0;
                state<=(active || path_valid || state==MANUAL_TAKEOVER) ? MANUAL_TAKEOVER : IDLE;
                timer<=0;planner_abort<=1;nav_fault<=0;
            end else if(ir_valid && ir_command==8'h42 && !scanning && !previous_scanning &&
                        !(ENABLE_LIVE_MAPPING && (active || auto_retry_pending))) begin
                state<=state==MANUAL_TAKEOVER ? MANUAL_TAKEOVER : IDLE;
                timer<=0;settle<=0;planner_abort<=1;
                if(state!=MANUAL_TAKEOVER)path_valid<=0;
                nav_fault<=0;
                if(!MULTI_POSE_MAPPING && !scan_resume_event) begin
                    scan_restart<=1;map_ready<=0;map_frozen<=0;goal_valid<=0;
                    goal_x_cell<=GRID_X/2;goal_y_cell<=GRID_Y/2;
                end
                // Multi-pose 42 preserves map RAM, origin, sample count and goal.
                // The legacy single-pose branch above may establish a new map.
            end else if(scanning && state!=MANUAL_TAKEOVER) begin
                state<=IDLE;timer<=0;path_valid<=0;
            end else if(ir_valid && edit_key && !active) begin
                state<=SELECT;timer<=0;path_valid<=0;goal_valid<=1;nav_fault<=0;
                // Keypad directions follow the fixed map: +X is screen up,
                // +Y is screen left, regardless of the current car heading.
                case(ir_command)
                    8'h18:goal_x_cell<=goal_x_cell>GRID_X-1-GOAL_STEP_CELLS ? GRID_X-1 : goal_x_cell+GOAL_STEP_CELLS;
                    8'h52:goal_x_cell<=goal_x_cell<GOAL_STEP_CELLS ? 0 : goal_x_cell-GOAL_STEP_CELLS;
                    8'h08:goal_y_cell<=goal_y_cell>GRID_Y-1-GOAL_STEP_CELLS ? GRID_Y-1 : goal_y_cell+GOAL_STEP_CELLS;
                    8'h5A:goal_y_cell<=goal_y_cell<GOAL_STEP_CELLS ? 0 : goal_y_cell-GOAL_STEP_CELLS;
                endcase
            end else if(ir_valid && ir_command==8'h1C && !active) begin
                off_path_retries<=0;retry_origin_x<=progress_x_mm;retry_origin_y<=progress_y_mm;
                state<=PREPLAN;timer<=0;settle<=0;new_yaw_after_stop<=0;
                goal_valid<=1;path_valid<=0;nav_fault<=0;map_frozen<=1;
            end else if(active && !healthy) begin
                state<=FAULT;timer<=0;planner_abort<=1;path_valid<=0;
                nav_fault<=pose_fault ? 3 : !heading_valid ? 1 : !motor_online ? 2 : !tof_good ? 4 : 8;
            end else if(ENABLE_LIVE_MAPPING && active && localization_replan) begin
                // A substantial accepted map<-odom correction changes the
                // route's start pose. STOP, drain SLAM, then plan from the new
                // coherent map pose to the same goal; never reuse old segments.
                state<=PREPLAN;timer<=0;settle<=0;new_yaw_after_stop<=0;
                planner_abort<=1;path_valid<=0;nav_fault<=0;
                correcting<=0;corrections<=0;yaw_trim<=0;
            end else begin
                case(state)
                    PREPLAN:begin
                        if(stop_confirmed && settle>=SETTLE_CLKS && new_yaw_after_stop && !map_busy) begin
                            if(!map_ready || scan_samples==0) begin state<=NOPATH;nav_fault<=5;end
                            else if(at_goal) state<=ARRIVED;
                            else begin planner_start<=1;state<=PLAN;end
                            timer<=0;
                        end else if(timer>=STOP_CLKS) begin state<=FAULT;nav_fault<=6;timer<=0;end
                    end
                    PLAN:begin
                        if(planner_done) begin
                            if(planner_success) begin path_valid<=1;state<=REWIND;
                                if(ENABLE_HOLONOMIC)held_heading<=planner_body_heading;end
                            else begin
                                state<=NOPATH;
                                // Preserve the planner's actual rejection so
                                // FREE center cells do not mask a blocked disk.
                                case(planner_fail)
                                    1:nav_fault<=10; // Start clearance blocked
                                    2:nav_fault<=11; // Goal clearance blocked
                                    3:nav_fault<=12; // No connected safe route
                                    4:nav_fault<=13; // Map read timeout
                                    5:nav_fault<=14; // Plan aborted
                                    6:nav_fault<=15; // Invalid parent chain
                                    7:nav_fault<=12; // Route re-check found OCCUPIED
                                    default:nav_fault<=9;
                                endcase
                            end
                            timer<=0;
                        end else if(timer>=PLAN_CLKS) begin state<=FAULT;nav_fault<=6;planner_abort<=1;timer<=0;end
                    end
                    REWIND:begin path_rewind<=1;first_point<=1;request_sent<=0;state<=FETCH;timer<=0;end
                    FETCH:begin
                        if(timer>=1 && !waypoint_busy && !request_sent) begin waypoint_next<=1;request_sent<=1;end
                        if(waypoint_valid) begin
                            request_sent<=0;
                            if(first_point) begin
                                previous_x<=waypoint_x;previous_y<=waypoint_y;first_point<=0;
                                if(waypoint_last) begin state<=ARRIVED;timer<=0;end
                                else begin timer<=0;end
                            end else begin
                                target_x<=waypoint_x;target_y<=waypoint_y;target_last<=waypoint_last;
                                target_mm_x<=($signed({1'b0,waypoint_x})-(GRID_X/2))*CELL_MM+WAYPOINT_OFFSET_X_MM;
                                target_mm_y<=($signed({1'b0,waypoint_y})-(GRID_Y/2))*CELL_MM+WAYPOINT_OFFSET_Y_MM;
                                if(waypoint_x>previous_x) target_heading<=0;
                                else if(waypoint_x<previous_x) target_heading<=1800;
                                else if(waypoint_y>previous_y) target_heading<=900;
                                else target_heading<=-900;
                                previous_x<=waypoint_x;previous_y<=waypoint_y;
                                corrections<=0;correcting<=0;state<=TURN;timer<=0;
                            end
                        end else if(timer>=STOP_CLKS) begin state<=FAULT;nav_fault<=9;timer<=0;end
                    end
                    TURN:begin
                        if(aligned) begin state<=TURN_STOP;timer<=0;settle<=0;new_yaw_after_stop<=0;end
                        else if(timer>=TURN_CLKS) begin state<=FAULT;nav_fault<=6;timer<=0;end
                    end
                    TURN_STOP:begin
                        if(stop_confirmed && settle>=SETTLE_CLKS && new_yaw_after_stop) begin
                            if(aligned) begin
                                state<=MOVE;timer<=0;
                                recovery_origin_x<=progress_x_mm;recovery_origin_y<=progress_y_mm;
                            end
                            else if(corrections<3) begin state<=TURN;corrections<=corrections+1'b1;timer<=0;end
                            else begin state<=FAULT;nav_fault<=7;timer<=0;end
                        end else if(timer>=STOP_CLKS) begin state<=FAULT;nav_fault<=6;timer<=0;end
                    end
                    MOVE:begin
                        if(front_stop_active) begin state<=FAULT;nav_fault<=4;timer<=0;end
                        else if(cross_bad) begin state<=FAULT;nav_fault<=7;timer<=0;end
                        else if(remaining<=STOP_EARLY_MM) begin
                            state<=MOVE_STOP;correcting<=0;timer<=0;settle<=0;new_yaw_after_stop<=0;
                        end else if(yaw_deviation) begin
                            if(ENABLE_HOLONOMIC||STOP_ON_MOVE_YAW_DEVIATION)begin state<=FAULT;nav_fault<=7;timer<=0;end
                            else begin state<=MOVE_STOP;correcting<=1;timer<=0;settle<=0;new_yaw_after_stop<=0;end
                        end else if(timer>=MOVE_CLKS) begin state<=FAULT;nav_fault<=6;timer<=0;end
                    end
                    MOVE_STOP:begin
                        if(stop_confirmed && settle>=SETTLE_CLKS && new_yaw_after_stop) begin
                            if(correcting) begin
                                if(corrections<3) begin state<=TURN;corrections<=corrections+1'b1;end
                                else begin state<=FAULT;nav_fault<=7;end
                            end else if(remaining < -GOAL_TOLERANCE_MM || cross_bad) begin state<=FAULT;nav_fault<=7;end
                            else if(target_last) begin
                                if(at_goal) state<=ARRIVED;else begin state<=FAULT;nav_fault<=7;end
                            end else begin state<=FETCH;request_sent<=0;end
                            timer<=0;
                        end else if(timer>=STOP_CLKS) begin state<=FAULT;nav_fault<=6;timer<=0;end
                    end
                    FAULT:begin
                        // Only N07 may restart automatically. STOP/manual/
                        // clear/mode changes above have priority. Link/pose/
                        // sensor faults and NO PATH never auto-start motion.
                        if(AUTO_REPLAN_OFF_PATH && nav_fault==7 && !healthy)begin
                            // A link/sensor loss during the retry wait replaces
                            // N07 with that fault. Reconnection alone must not
                            // revive a navigation attempt after such a fault.
                            nav_fault<=pose_fault ? 3 : !heading_valid ? 1 : !motor_online ? 2 : !tof_good ? 4 : 8;
                            timer<=0;
                        end else if(auto_retry_pending && healthy && stop_confirmed && timer>=OFF_PATH_RETRY_CLKS)begin
                            state<=PREPLAN;nav_fault<=0;path_valid<=0;planner_abort<=1;
                            timer<=0;settle<=0;new_yaw_after_stop<=0;correcting<=0;corrections<=0;
                            off_path_retries<=off_path_retries+1'b1;
                            retry_origin_x<=progress_x_mm;retry_origin_y<=progress_y_mm;
                        end
                    end
                    default:begin timer<=0;end
                endcase
            end
        end
    end
endmodule

// Optional coherent-pose guard for live mapping. Production compares the
// vehicle's map pose immediately before/after an accepted correction, not the
// translation of the map origin. Small corrections never accumulate into a
// stop request. The legacy transform-based interface remains the default for
// existing standalone callers/tests.
module navigation_localization_guard #(
    parameter integer POSITION_MM=40,YAW_TENTHS=20,
    parameter integer USE_POSE_DELTA=0,CLK_FREQ_HZ=50000000,COOLDOWN_MS=3000,
    parameter integer EMERGENCY_POSITION_MM=160,EMERGENCY_YAW_TENTHS=100
)(
    input wire clk,reset_n,enable,clear,active,transform_update,
    input wire signed [15:0] transform_x_mm,transform_y_mm,
    input wire signed [12:0] transform_yaw_tenths,
    input wire pose_valid,pose_transform_pending,
    input wire signed [15:0] pose_x_mm,pose_y_mm,
    input wire signed [31:0] pose_heading_tenths,
    output reg replan
);
    generate if(USE_POSE_DELTA)begin:g_coherent_pose
        localparam integer COOLDOWN_CLKS=(CLK_FREQ_HZ/1000*COOLDOWN_MS)>0?CLK_FREQ_HZ/1000*COOLDOWN_MS:1;
        reg [31:0] cooldown;
        reg [1:0] phase;
        reg pending_seen;
        reg signed [15:0] before_x,before_y,after_x,after_y,last_transform_x,last_transform_y;
        reg signed [12:0] before_yaw,after_yaw,last_transform_yaw;
        wire transform_changed=transform_x_mm!=last_transform_x || transform_y_mm!=last_transform_y ||
            transform_yaw_tenths!=last_transform_yaw;
        wire signed [16:0] dx=$signed({after_x[15],after_x})-$signed({before_x[15],before_x});
        wire signed [16:0] dy=$signed({after_y[15],after_y})-$signed({before_y[15],before_y});
        wire signed [13:0] da=$signed({after_yaw[12],after_yaw})-$signed({before_yaw[12],before_yaw});
        wire signed [13:0] wrapped_da=da>1800?da-3600:da< -1800?da+3600:da;
        wire substantial=dx>=POSITION_MM || dx<=-POSITION_MM || dy>=POSITION_MM || dy<=-POSITION_MM ||
            wrapped_da>=YAW_TENTHS || wrapped_da<=-YAW_TENTHS;
        wire emergency=dx>=EMERGENCY_POSITION_MM || dx<=-EMERGENCY_POSITION_MM ||
            dy>=EMERGENCY_POSITION_MM || dy<=-EMERGENCY_POSITION_MM ||
            wrapped_da>=EMERGENCY_YAW_TENTHS || wrapped_da<=-EMERGENCY_YAW_TENTHS;
        always @(posedge clk or negedge reset_n)begin
            if(!reset_n)begin
                replan<=0;phase<=0;pending_seen<=0;cooldown<=0;
                before_x<=0;before_y<=0;after_x<=0;after_y<=0;before_yaw<=0;after_yaw<=0;
                last_transform_x<=0;last_transform_y<=0;last_transform_yaw<=0;
            end else begin
                replan<=0;
                if(cooldown!=0)cooldown<=cooldown-1'b1;
                if(!enable || clear)begin
                    phase<=0;pending_seen<=0;cooldown<=0;
                    last_transform_x<=0;last_transform_y<=0;last_transform_yaw<=0;
                end else begin
                    if(transform_update)begin
                        last_transform_x<=transform_x_mm;last_transform_y<=transform_y_mm;
                        last_transform_yaw<=transform_yaw_tenths;
                    end
                    if(!active || !pose_valid)begin phase<=0;pending_seen<=0;end
                    else if(transform_update && transform_changed)begin
                        before_x<=pose_x_mm;before_y<=pose_y_mm;before_yaw<=pose_heading_tenths[12:0];
                        pending_seen<=pose_transform_pending;phase<=1;
                    end else case(phase)
                        1:begin
                            if(pose_transform_pending)pending_seen<=1;
                            else if(pending_seen)begin
                                after_x<=pose_x_mm;after_y<=pose_y_mm;after_yaw<=pose_heading_tenths[12:0];phase<=2;
                            end
                        end
                        2:begin
                            if(substantial && (cooldown==0 || emergency))begin replan<=1;cooldown<=COOLDOWN_CLKS;end
                            phase<=0;
                        end
                        default:begin end
                    endcase
                end
            end
        end
    end else begin:g_legacy_transform
    reg signed [15:0] reference_x,reference_y;
    reg signed [12:0] reference_yaw;
    wire signed [16:0] dx=$signed({transform_x_mm[15],transform_x_mm})-$signed({reference_x[15],reference_x});
    wire signed [16:0] dy=$signed({transform_y_mm[15],transform_y_mm})-$signed({reference_y[15],reference_y});
    wire signed [13:0] da=$signed({transform_yaw_tenths[12],transform_yaw_tenths})-$signed({reference_yaw[12],reference_yaw});
    wire signed [13:0] wrapped_da=da>1800?da-3600:da< -1800?da+3600:da;
    wire substantial=dx>=POSITION_MM || dx<=-POSITION_MM ||
        dy>=POSITION_MM || dy<=-POSITION_MM || wrapped_da>=YAW_TENTHS || wrapped_da<=-YAW_TENTHS;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin reference_x<=0;reference_y<=0;reference_yaw<=0;replan<=0;end
        else begin
            replan<=0;
            if(!enable || clear || !active)begin
                reference_x<=transform_x_mm;reference_y<=transform_y_mm;reference_yaw<=transform_yaw_tenths;
            end else if(transform_update && substantial)begin
                replan<=1;
                reference_x<=transform_x_mm;reference_y<=transform_y_mm;reference_yaw<=transform_yaw_tenths;
            end
        end
    end
    end endgenerate
endmodule
