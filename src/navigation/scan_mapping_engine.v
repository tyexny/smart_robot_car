// Bounded scan-to-map mapping frontend, entirely on the existing system clock.
// Capture one complete revolution at full input density; only the matcher
// selects the return nearest each 3-degree bin centre. Freeze
// the preceding map, match, then replay only an accepted frame. Matching and
// replay discard incoming laps; the next capture starts at a new lap boundary.
// No partial/backlogged lap is presented as a new observation.
//
// This is local SLAM, not Cartographer/global pose-graph loop closure. The
// correction is a single map<-odometry SE(2) transform. Old scans are not saved
// or globally reoptimized. Arrival-time per-point pose snapshots provide only
// a bounded slow-motion approximation, not acquisition-time LiDAR deskew.
module scan_mapping_engine #(
    parameter integer CLK_FREQ_HZ=50000000,CELL_SHIFT=6,MAX_POINTS=128,MAX_RAYS=2048,MIN_POINTS=24,
    parameter integer FRAME_TIMEOUT_MS=800,MAX_FRAME_TRANSLATION_MM=192,MAX_FRAME_YAW_TENTHS=80,
    parameter integer ALLOW_LOW_SUPPORT_UPDATE=1,
    // The coarse matcher only sees one return per 3-degree bin. After the search
    // the SAME endpoints are re-scored at full density against the zero transform
    // and against the selected peak: ENABLE_VERIFY runs that sweep and reports it
    // (diagnostic_hits/support become percentages of original in-map endpoints).
    // Nonzero correction needs BOTH strict coarse acceptance and full-density
    // corroboration. Low-support write-through always uses zero correction.
    parameter integer ENABLE_VERIFY=1,
    parameter integer USE_VERIFY_GATE=1,
    parameter integer VERIFY_ZERO_SUPPORT_PERCENT=85,
    parameter integer VERIFY_ZERO_HIT_PERCENT=30,
    parameter integer VERIFY_BEST_HIT_PERCENT=60,
    parameter integer VERIFY_IMPROVEMENT_PERCENT=10,
    parameter integer VERIFY_MIN_COVERAGE_PERCENT=25
)(
    input wire clk,reset_n,enable,clear,rescan,allow,healthy,stationary,
    input wire ray_update,ray_frame_start,
    input wire [6:0] ray_bin,
    input wire [3:0] ray_bin_error,
    input wire [31:0] ray_timestamp,
    input wire signed [31:0] ray_start_x_mm,ray_start_y_mm,ray_end_x_mm,ray_end_y_mm,
    input wire signed [31:0] current_heading_tenths,
    input wire map_busy,map_clear_done,
    output reg map_ray_update,
    output reg signed [31:0] map_start_x_mm,map_start_y_mm,map_end_x_mm,map_end_y_mm,
    output reg [7:0] map_epoch,
    output wire map_req,output wire [13:0] map_addr,
    input wire map_req_ready,map_rsp_valid,input wire [1:0] map_rsp_data,
    output wire busy,
    output reg [3:0] status,
    output reg [15:0] accepted_samples,accepted_frames,rejected_frames,
    output reg [15:0] match_score,
    output reg [3:0] reject_reason,
    output reg [7:0] diagnostic_points,diagnostic_hits,diagnostic_support,
    output reg transform_update,
    output reg signed [15:0] transform_x_mm,transform_y_mm,
    output reg signed [12:0] transform_yaw_tenths
);
    localparam [4:0] WAIT_LAP=0,COLLECT=1,CHECK=2,START_MATCH=3,WAIT_MATCH=4,
        START_COMPOSE=5,WAIT_COMPOSE=6,PUBLISH=7,EPOCH_WAIT=8,
        READ_RAY=9,START_RAY=10,WAIT_RAY=11,SEND_RAY=12,DRAIN_RAY=13,FINISH=14,
        VERIFY_ZERO=15,VERIFY_READ=16,VERIFY_SEND=17,VERIFY_WAIT=18,VERIFY_DECIDE=19,
        VERIFY_PERCENT=20;
    localparam integer TIMEOUT_CLKS=CLK_FREQ_HZ/1000*FRAME_TIMEOUT_MS;
    localparam integer RAY_INDEX_BITS=MAX_RAYS>1 ? $clog2(MAX_RAYS) : 1;
    reg [4:0] state;
    reg seeded,frame_bad,frame_stationary,last_frame_rejected;
    reg [3:0] frame_bad_reason;
    reg [7:0] frame_match_hits,frame_match_support;
    // Nonzero pose changes need two adjacent complete observations with
    // consistent GLOBAL map<-odom proposals. First proposal never writes map.
    reg proposal_valid;
    reg signed [15:0] proposal_tx,proposal_ty;
    reg signed [12:0] proposal_yaw;
    reg [7:0] point_count;
    reg [119:0] observed_bins;
    reg [7:0] observed_points;
    reg [RAY_INDEX_BITS-1:0] replay_index;
    reg [RAY_INDEX_BITS:0] ray_count;
    reg [127:0] used_bins;
    // A compact bin->dense matcher slot lookup lets a later, better-centred
    // return overwrite that bin's representative. used_bins is the only
    // validity state; stale table contents never survive a new/cancelled lap.
    reg [6:0] bin_point_index[0:119];
    reg [3:0] bin_best_error[0:119];
    reg [3:0] quadrants;
    reg [31:0] frame_age,first_timestamp;
    reg signed [15:0] pivot_x,pivot_y;
    reg signed [12:0] first_heading;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [63:0] rays[0:MAX_RAYS-1];
    reg [63:0] replay_ray;
    reg matcher_start,matcher_load;
    reg [6:0] matcher_load_index;
    reg signed [15:0] matcher_load_x,matcher_load_y;
    wire matcher_busy,matcher_done,matched;
    wire signed [15:0] match_dx,match_dy;
    wire signed [12:0] match_dyaw;
    wire [15:0] best_score,baseline_score;
    wire [7:0] hit_count,support_count;
    wire [7:0] baseline_hit_count,baseline_support_count;
    wire [3:0] matcher_reject_reason;
    wire identity_ambiguous;
    // Full-density verification. The matcher rescans the very endpoints that
    // were just replayed, once at the zero transform and once at the selected
    // peak, so the reported support covers the whole revolution instead of the
    // one-per-3-degree subsample the coarse search can hold.
    reg verify_start,verify_zero,verify_valid;
    reg [11:0] verify_total,verify_index;
    wire verify_ready,verify_done;
    wire [15:0] verify_score;
    wire [11:0] verify_hits,verify_support,verify_eligible;
    wire signed [3:0] selected_x,selected_y,selected_yaw;
    reg [11:0] vz_hits,vz_support,vb_hits,vb_support;
    reg [15:0] vz_score,vb_score;
    reg [11:0] vz_eligible;
    reg verify_rejected;
    reg [3:0] verify_reject_code;
    // The page fields are three decimal digits fed by an 8-bit value, while a
    // real revolution holds well over 255 endpoints, so the reported support
    // and hit counts are percentages. A descending multiply-and-compare replaces
    // a divider; it runs while the accepted frame is being replayed.
    localparam [1:0] PCT_IDLE=0,PCT_INIT=1,PCT_SCAN=2,PCT_NEXT=3;
    reg [1:0] pct_state;
    reg [18:0] pct_num,pct_acc;
    reg [11:0] pct_den;
    reg [7:0] pct_p,pct_val,pct_hit,pct_sup;
    reg pct_second;
    wire match_nonzero=match_dx!=0 || match_dy!=0 || match_dyaw!=0;
    wire verify_coverage_ok=vz_eligible>=MIN_POINTS &&
        vz_eligible*100>=verify_total*VERIFY_MIN_COVERAGE_PERCENT;
    // Never rescue a rejected coarse peak: full verification can veto a
    // correction, but cannot remove its uniqueness/strict-centre gates.
    wire verify_best_ok=verify_coverage_ok && !identity_ambiguous &&
        vb_hits*100>=vz_eligible*VERIFY_BEST_HIT_PERCENT &&
        vb_score>=vz_score+12 &&
        vb_score*100>=vz_score*(100+VERIFY_IMPROVEMENT_PERCENT) &&
        matched && match_nonzero;
    // The current transform is accepted on full-density evidence even when the
    // subsample refused it: most endpoints land on, or beside, known obstacles.
    wire verify_zero_ok=verify_coverage_ok && !identity_ambiguous &&
        vz_support*100>=vz_eligible*VERIFY_ZERO_SUPPORT_PERCENT &&
        vz_hits*100>=vz_eligible*VERIFY_ZERO_HIT_PERCENT;
    wire verify_take_correction=matched && match_nonzero &&
        (!USE_VERIFY_GATE || verify_best_ok);
    wire verify_take_zero=(!USE_VERIFY_GATE && matched && !match_nonzero) ||
        (USE_VERIFY_GATE && verify_zero_ok);
    wire verify_low_fallback=ALLOW_LOW_SUPPORT_UPDATE &&
        (matcher_reject_reason==7 || (matched && !verify_take_correction && !verify_take_zero));
    reg signed [15:0] chosen_dx,chosen_dy,pending_tx,pending_ty;
    reg signed [12:0] chosen_dyaw,pending_yaw;
    reg compose_start,replay_start;
    wire compose_busy,compose_done,start_busy,start_done,end_busy,end_done;
    wire signed [31:0] composed_x,composed_y,replayed_sx,replayed_sy,replayed_ex,replayed_ey;
    wire abort_frame=!enable || clear || rescan || !allow || !healthy;
    wire signed [31:0] frame_dx=ray_start_x_mm-$signed(pivot_x);
    wire signed [31:0] frame_dy=ray_start_y_mm-$signed(pivot_y);
    wire signed [31:0] yaw_difference=current_heading_tenths-$signed(first_heading);
    wire signed [31:0] yaw_difference_wrapped=yaw_difference>1800 ? yaw_difference-3600 :
        yaw_difference< -1800 ? yaw_difference+3600 : yaw_difference;
    wire excessive_motion=frame_dx>MAX_FRAME_TRANSLATION_MM || frame_dx< -MAX_FRAME_TRANSLATION_MM ||
        frame_dy>MAX_FRAME_TRANSLATION_MM || frame_dy< -MAX_FRAME_TRANSLATION_MM ||
        yaw_difference_wrapped>MAX_FRAME_YAW_TENTHS || yaw_difference_wrapped< -MAX_FRAME_YAW_TENTHS;
    wire ray_fits=ray_start_x_mm>=-32768 && ray_start_x_mm<=32767 &&
        ray_start_y_mm>=-32768 && ray_start_y_mm<=32767 &&
        ray_end_x_mm>=-32768 && ray_end_x_mm<=32767 && ray_end_y_mm>=-32768 && ray_end_y_mm<=32767;
    wire ray_in_map=ray_end_x_mm>=-(64<<CELL_SHIFT) && ray_end_x_mm<(64<<CELL_SHIFT) &&
        ray_end_y_mm>=-(64<<CELL_SHIFT) && ray_end_y_mm<(64<<CELL_SHIFT);
    wire [2:0] quadrant_count={2'd0,quadrants[0]}+{2'd0,quadrants[1]}+
        {2'd0,quadrants[2]}+{2'd0,quadrants[3]};
    wire [16:0] completed_sample_count={1'b0,accepted_samples}+ray_count;
    wire can_append=point_count<MAX_POINTS && ray_bin<120 && !used_bins[ray_bin] && ray_fits && ray_in_map;
    wire can_replace=ray_bin<120 && used_bins[ray_bin] && ray_fits && ray_in_map &&
        ray_bin_error<bin_best_error[ray_bin];
    wire signed [13:0] new_yaw=$signed({transform_yaw_tenths[12],transform_yaw_tenths})+
        $signed({chosen_dyaw[12],chosen_dyaw});
    wire signed [13:0] new_yaw_wrapped=new_yaw>1800 ? new_yaw-3600 : new_yaw< -1800 ? new_yaw+3600 : new_yaw;
    wire signed [16:0] proposal_dx=$signed(composed_x[16:0])-$signed({proposal_tx[15],proposal_tx});
    wire signed [16:0] proposal_dy=$signed(composed_y[16:0])-$signed({proposal_ty[15],proposal_ty});
    wire signed [13:0] proposal_dyaw=new_yaw_wrapped-$signed({proposal_yaw[12],proposal_yaw});
    wire signed [13:0] proposal_dyaw_wrapped=proposal_dyaw>1800 ? proposal_dyaw-3600 :
        proposal_dyaw< -1800 ? proposal_dyaw+3600 : proposal_dyaw;
    wire nonzero_correction=chosen_dx!=0 || chosen_dy!=0 || chosen_dyaw!=0;
    wire consistent_proposal=proposal_valid && proposal_dx>=-64 && proposal_dx<=64 &&
        proposal_dy>=-64 && proposal_dy<=64 && proposal_dyaw_wrapped>=-10 && proposal_dyaw_wrapped<=10;
    assign busy=state>=CHECK || matcher_busy;
    scan_matcher_2d #(.MAX_POINTS(MAX_POINTS),.MIN_POINTS(MIN_POINTS),.CELL_SHIFT(CELL_SHIFT)) matcher(
        .clk(clk),.reset_n(reset_n),.cancel(abort_frame),
        .load_valid(matcher_load),.load_index(matcher_load_index),.load_x_mm(matcher_load_x),.load_y_mm(matcher_load_y),
        .start(matcher_start),.point_count(point_count),.pivot_x_mm(pivot_x),.pivot_y_mm(pivot_y),
        .allow_zero_near(frame_stationary && stationary),
        .busy(matcher_busy),.done(matcher_done),.matched(matched),
        .delta_x_mm(match_dx),.delta_y_mm(match_dy),.delta_yaw_tenths(match_dyaw),
        .best_score(best_score),.baseline_score(baseline_score),.matched_points(hit_count),
        .supported_points(support_count),.reject_reason(matcher_reject_reason),
        .baseline_matched_points(baseline_hit_count),.baseline_supported_points(baseline_support_count),
        .identity_ambiguous(identity_ambiguous),
        .map_req_ready(map_req_ready),.map_rsp_valid(map_rsp_valid),.map_rsp_data(map_rsp_data),.map_req(map_req),.map_addr(map_addr),
        .verify_start(verify_start),.verify_zero(verify_zero),.verify_total(verify_total),
        .verify_valid(verify_valid),.verify_x_mm(replay_ray[31:16]),.verify_y_mm(replay_ray[15:0]),
        .verify_ready(verify_ready),.verify_done(verify_done),.verify_score(verify_score),
        .verify_hits(verify_hits),.verify_support(verify_support),
        .verify_eligible(verify_eligible),
        .selected_x(selected_x),.selected_y(selected_y),.selected_yaw(selected_yaw));
    slam_rigid_transform compose(.clk(clk),.reset_n(reset_n),.start(compose_start),.cancel(abort_frame),
        .point_x(transform_x_mm),.point_y(transform_y_mm),.pivot_x(pivot_x),.pivot_y(pivot_y),
        .translation_x(chosen_dx),.translation_y(chosen_dy),.angle_tenths(chosen_dyaw),
        .busy(compose_busy),.done(compose_done),.result_x(composed_x),.result_y(composed_y));
    slam_rigid_transform replay_origin(.clk(clk),.reset_n(reset_n),.start(replay_start),.cancel(abort_frame),
        .point_x(replay_ray[63:48]),.point_y(replay_ray[47:32]),.pivot_x(pivot_x),.pivot_y(pivot_y),
        .translation_x(chosen_dx),.translation_y(chosen_dy),.angle_tenths(chosen_dyaw),
        .busy(start_busy),.done(start_done),.result_x(replayed_sx),.result_y(replayed_sy));
    slam_rigid_transform replay_endpoint(.clk(clk),.reset_n(reset_n),.start(replay_start),.cancel(abort_frame),
        .point_x(replay_ray[31:16]),.point_y(replay_ray[15:0]),.pivot_x(pivot_x),.pivot_y(pivot_y),
        .translation_x(chosen_dx),.translation_y(chosen_dy),.angle_tenths(chosen_dyaw),
        .busy(end_busy),.done(end_done),.result_x(replayed_ex),.result_y(replayed_ey));
    // A single synchronous write port, with no memory reset, is required for
    // block-RAM inference. Merge the first-point and continuation addresses
    // before the port; the FSM still owns all counters and frame validity.
    wire first_ray_write=state==WAIT_LAP && !matcher_busy && ray_update &&
        ray_frame_start && ray_fits && ray_bin<120;
    wire following_ray_write=state==COLLECT && ray_update && !ray_frame_start &&
        ray_fits && ray_count<MAX_RAYS;
    wire ray_write_enable=reset_n && !abort_frame && (first_ray_write || following_ray_write);
    wire [RAY_INDEX_BITS-1:0] ray_write_address=first_ray_write ?
        {RAY_INDEX_BITS{1'b0}} : ray_count[RAY_INDEX_BITS-1:0];
    always @(posedge clk) begin
        if(ray_write_enable)
            rays[ray_write_address]<=
                {ray_start_x_mm[15:0],ray_start_y_mm[15:0],ray_end_x_mm[15:0],ray_end_y_mm[15:0]};
    end
    // One synchronous read port; no replay uses a frame still being written.
    // The verification sweep walks the same buffer, so it borrows the port
    // rather than adding a second one (which would cost the block RAM).
    wire [RAY_INDEX_BITS-1:0] ray_read_address=
        state>=VERIFY_ZERO ? verify_index[RAY_INDEX_BITS-1:0] : replay_index;
    always @(posedge clk) replay_ray<=rays[ray_read_address];
    always @* begin
        if(!enable) status=0;
        else if(!healthy || !allow) status=4;
        else if(state>=CHECK) status=13; // MATCH / COMMIT
        else if(!seeded) status=15; // SEED: keep the car stationary
        else if(last_frame_rejected && state==WAIT_LAP) status=14; // last completed frame skipped
        else status=5;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=WAIT_LAP;seeded<=0;frame_bad<=0;frame_stationary<=0;point_count<=0;ray_count<=0;replay_index<=0;
            observed_bins<=0;observed_points<=0;
            used_bins<=0;quadrants<=0;frame_age<=0;first_timestamp<=0;pivot_x<=0;pivot_y<=0;first_heading<=0;
            matcher_start<=0;matcher_load<=0;matcher_load_index<=0;matcher_load_x<=0;matcher_load_y<=0;
            chosen_dx<=0;chosen_dy<=0;chosen_dyaw<=0;pending_tx<=0;pending_ty<=0;pending_yaw<=0;
            compose_start<=0;replay_start<=0;map_ray_update<=0;map_epoch<=0;
            map_start_x_mm<=0;map_start_y_mm<=0;map_end_x_mm<=0;map_end_y_mm<=0;
            transform_update<=0;transform_x_mm<=0;transform_y_mm<=0;transform_yaw_tenths<=0;
            accepted_samples<=0;accepted_frames<=0;rejected_frames<=0;match_score<=0;
            reject_reason<=0;diagnostic_points<=0;diagnostic_hits<=0;diagnostic_support<=0;
            last_frame_rejected<=0;frame_bad_reason<=0;frame_match_hits<=0;frame_match_support<=0;
            proposal_valid<=0;proposal_tx<=0;proposal_ty<=0;proposal_yaw<=0;
            verify_start<=0;verify_zero<=0;verify_valid<=0;verify_total<=0;verify_index<=0;
            vz_hits<=0;vz_support<=0;vz_score<=0;vb_hits<=0;vb_support<=0;vb_score<=0;
            pct_state<=PCT_IDLE;pct_num<=0;pct_acc<=0;pct_p<=0;pct_val<=0;
            pct_hit<=0;pct_sup<=0;pct_second<=0;
            pct_den<=0;vz_eligible<=0;verify_rejected<=0;verify_reject_code<=0;
        end else begin
            matcher_start<=0;matcher_load<=0;compose_start<=0;replay_start<=0;map_ray_update<=0;transform_update<=0;
            if(!enable || clear) begin
                state<=WAIT_LAP;seeded<=0;point_count<=0;ray_count<=0;used_bins<=0;quadrants<=0;frame_bad<=0;
                frame_age<=0;map_epoch<=0;transform_x_mm<=0;transform_y_mm<=0;transform_yaw_tenths<=0;
                accepted_samples<=0;accepted_frames<=0;rejected_frames<=0;match_score<=0;
                reject_reason<=0;diagnostic_points<=0;diagnostic_hits<=0;diagnostic_support<=0;
                last_frame_rejected<=0;frame_bad_reason<=0;frame_match_hits<=0;frame_match_support<=0;
                proposal_valid<=0;proposal_tx<=0;proposal_ty<=0;proposal_yaw<=0;
                verify_start<=0;verify_valid<=0;verify_index<=0;
                observed_bins<=0;observed_points<=0;
                pct_state<=PCT_IDLE;pct_hit<=0;pct_sup<=0;pct_den<=0;
                verify_rejected<=0;vz_eligible<=0;
            end else if(rescan || !allow || !healthy) begin
                state<=WAIT_LAP;point_count<=0;ray_count<=0;used_bins<=0;quadrants<=0;frame_age<=0;
                frame_bad<=0;frame_bad_reason<=0;proposal_valid<=0;
                verify_start<=0;verify_valid<=0;verify_index<=0;
                observed_bins<=0;observed_points<=0;pct_state<=PCT_IDLE;verify_rejected<=0;
                // Operator pause/rescan and navigation ownership are deliberate
                // cancellations. Only an active observation lost to sensor
                // health is a rejected frame; holding unhealthy cannot recount.
                if(!rescan && allow && !healthy && state!=WAIT_LAP) begin
                    rejected_frames<=rejected_frames+1'b1;last_frame_rejected<=1;reject_reason<=10;
                    diagnostic_points<=point_count;diagnostic_hits<=0;diagnostic_support<=0;
                end
            end else case(state)
                WAIT_LAP:if(!matcher_busy && ray_update && ray_frame_start) begin
                  if(!ray_fits || ray_bin>=120) begin
                    rejected_frames<=rejected_frames+1'b1;last_frame_rejected<=1;reject_reason<=12;
                    diagnostic_points<=0;diagnostic_hits<=0;diagnostic_support<=0;proposal_valid<=0;
                  end else begin
                    pivot_x<=ray_start_x_mm[15:0];pivot_y<=ray_start_y_mm[15:0];first_heading<=current_heading_tenths[12:0];
                    first_timestamp<=ray_timestamp;frame_age<=0;frame_bad<=0;frame_bad_reason<=0;frame_stationary<=stationary;
                    used_bins<=ray_in_map ? 128'd1<<ray_bin : 128'd0;
                    observed_bins<=120'd1<<ray_bin;observed_points<=1;
                    quadrants<=ray_bin<30 ? 4'b0001 : ray_bin<60 ? 4'b0010 : ray_bin<90 ? 4'b0100 : 4'b1000;
                    bin_point_index[ray_bin]<=0;bin_best_error[ray_bin]<=ray_bin_error;
                    point_count<=ray_in_map ? 1 : 0;ray_count<=1;matcher_load<=ray_in_map;matcher_load_index<=0;
                    matcher_load_x<=ray_end_x_mm[15:0];matcher_load_y<=ray_end_y_mm[15:0];state<=COLLECT;
                  end
                end
                COLLECT:begin
                  if(frame_age>=TIMEOUT_CLKS) begin
                    state<=WAIT_LAP;rejected_frames<=rejected_frames+1'b1;
                    last_frame_rejected<=1;reject_reason<=frame_bad ? frame_bad_reason : 1;
                    diagnostic_points<=point_count;diagnostic_hits<=0;diagnostic_support<=0;proposal_valid<=0;
                  end else begin
                    frame_age<=frame_age+1'b1;
                    if(!stationary) frame_stationary<=0;
                    if(ray_update) begin
                        if(ray_frame_start) begin
                            if(!frame_bad && ((ray_timestamp-first_timestamp)>TIMEOUT_CLKS || excessive_motion)) begin
                                frame_bad<=1;frame_bad_reason<=(ray_timestamp-first_timestamp)>TIMEOUT_CLKS ? 1 : 5;
                            end
                            state<=CHECK;
                        end else begin
                            if(!frame_bad && (excessive_motion || !ray_fits || ray_bin>=120 || ray_count>=MAX_RAYS)) begin
                                frame_bad<=1;
                                frame_bad_reason<=(!ray_fits || ray_bin>=120) ? 12 : excessive_motion ? 5 : 4;
                            end
                            if(ray_fits && ray_count<MAX_RAYS) begin
                                ray_count<=ray_count+1'b1;
                            end
                            if(ray_fits && ray_bin<120) begin
                                if(!observed_bins[ray_bin]) begin
                                    observed_bins[ray_bin]<=1;observed_points<=observed_points+1'b1;
                                end
                                if(ray_bin<30) quadrants[0]<=1;else if(ray_bin<60) quadrants[1]<=1;
                                else if(ray_bin<90) quadrants[2]<=1;else quadrants[3]<=1;
                            end
                            if(can_append) begin
                                used_bins[ray_bin]<=1;
                                bin_point_index[ray_bin]<=point_count[6:0];bin_best_error[ray_bin]<=ray_bin_error;
                                if(ray_bin<30) quadrants[0]<=1;else if(ray_bin<60) quadrants[1]<=1;
                                else if(ray_bin<90) quadrants[2]<=1;else quadrants[3]<=1;
                                matcher_load<=1;matcher_load_index<=point_count[6:0];
                                matcher_load_x<=ray_end_x_mm[15:0];matcher_load_y<=ray_end_y_mm[15:0];
                                point_count<=point_count+1'b1;
                            end else if(can_replace) begin
                                // Ties keep the earlier real return. Neither
                                // replacement nor matching subsampling removes
                                // a ray from the full-density replay buffer.
                                bin_best_error[ray_bin]<=ray_bin_error;
                                matcher_load<=1;matcher_load_index<=bin_point_index[ray_bin];
                                matcher_load_x<=ray_end_x_mm[15:0];matcher_load_y<=ray_end_y_mm[15:0];
                            end
                        end
                    end
                  end
                end
                CHECK:begin
                    if(frame_bad || observed_points<MIN_POINTS || quadrant_count<3 || (!seeded && !frame_stationary)) begin
                        rejected_frames<=rejected_frames+1'b1;state<=WAIT_LAP;
                        last_frame_rejected<=1;
                        reject_reason<=frame_bad ? frame_bad_reason : observed_points<MIN_POINTS ? 2 : quadrant_count<3 ? 3 : 6;
                        diagnostic_points<=point_count;diagnostic_hits<=0;diagnostic_support<=0;proposal_valid<=0;
                    end else if(!seeded) begin
                        chosen_dx<=0;chosen_dy<=0;chosen_dyaw<=0;match_score<=0;frame_match_hits<=0;frame_match_support<=0;state<=START_COMPOSE;
                    end
                    else if(point_count<MIN_POINTS) begin
                        // Valid far-only scans still update FREE rays. There
                        // are too few in-map landmarks to estimate correction.
                        proposal_valid<=0;frame_match_hits<=0;frame_match_support<=0;match_score<=0;
                        if(ALLOW_LOW_SUPPORT_UPDATE) begin
                            chosen_dx<=0;chosen_dy<=0;chosen_dyaw<=0;state<=START_COMPOSE;
                        end else begin
                            rejected_frames<=rejected_frames+1'b1;state<=WAIT_LAP;
                            last_frame_rejected<=1;reject_reason<=7;
                            diagnostic_points<=point_count;diagnostic_hits<=0;diagnostic_support<=0;
                        end
                    end
                    else state<=START_MATCH;
                end
                START_MATCH:if(!matcher_busy && !map_busy) begin matcher_start<=1;state<=WAIT_MATCH;end
                WAIT_MATCH:if(matcher_done) begin
                    match_score<=best_score;frame_match_hits<=hit_count;frame_match_support<=support_count;
                    // The subsample decision is held back: both hypotheses are
                    // re-scored over every endpoint of this revolution first.
                    if(ENABLE_VERIFY) state<=VERIFY_ZERO;
                    else if(matched) begin chosen_dx<=match_dx;chosen_dy<=match_dy;chosen_dyaw<=match_dyaw;state<=START_COMPOSE;end
                    else if(ALLOW_LOW_SUPPORT_UPDATE && matcher_reject_reason==7) begin
                        // User-selected low-support policy: retain the current
                        // transform and draw this geometrically valid frame.
                        // Report ZERO-candidate evidence, never the displaced
                        // best candidate that was refused by the matcher.
                        chosen_dx<=0;chosen_dy<=0;chosen_dyaw<=0;proposal_valid<=0;
                        frame_match_hits<=baseline_hit_count;frame_match_support<=baseline_support_count;
                        match_score<=baseline_score;state<=START_COMPOSE;
                    end
                    else begin
                        rejected_frames<=rejected_frames+1'b1;state<=WAIT_LAP;last_frame_rejected<=1;
                        reject_reason<=matcher_reject_reason;diagnostic_points<=point_count;diagnostic_hits<=hit_count;
                        diagnostic_support<=support_count;proposal_valid<=0;
                    end
                end
                // Full-density sweep 1: every replayed endpoint against the
                // current transform (zero correction).
                VERIFY_ZERO:begin
                    verify_start<=1;verify_zero<=1;verify_total<=ray_count;verify_index<=0;state<=VERIFY_READ;
                end
                VERIFY_READ:begin
                    verify_start<=0;
                    if(verify_ready) state<=VERIFY_SEND;
                end
                VERIFY_SEND:begin
                    verify_valid<=1;
                    if(verify_valid && verify_ready) begin
                        verify_valid<=0;
                        if(verify_index+1'b1>=verify_total) state<=VERIFY_WAIT;
                        else begin verify_index<=verify_index+1'b1;state<=VERIFY_READ;end
                    end
                end
                VERIFY_WAIT:if(verify_done) begin
                    if(verify_zero) begin
                        vz_hits<=verify_hits;vz_support<=verify_support;vz_score<=verify_score;vz_eligible<=verify_eligible;
                        verify_zero<=0;verify_start<=1;verify_index<=0;state<=VERIFY_READ;
                    end else begin
                        vb_hits<=verify_hits;vb_support<=verify_support;vb_score<=verify_score;
                        state<=VERIFY_DECIDE;
                    end
                end
                VERIFY_DECIDE:begin
                    verify_rejected<=!(verify_take_correction || verify_take_zero || verify_low_fallback);
                    verify_reject_code<=identity_ambiguous ? 8 : matched ? 7 : matcher_reject_reason;
                    if(verify_take_correction) begin
                        chosen_dx<=match_dx;chosen_dy<=match_dy;chosen_dyaw<=match_dyaw;
                        match_score<=vb_score;
                    end else begin
                        chosen_dx<=0;chosen_dy<=0;chosen_dyaw<=0;proposal_valid<=0;
                        match_score<=vz_score;
                    end
                    // Complete both percentages BEFORE publishing a coherent
                    // frame snapshot. Clear/pause/health cancellation revokes it.
                    pct_num<=(verify_take_correction ? vb_hits : vz_hits)*100;
                    pct_den<=vz_eligible;pct_acc<=vz_eligible*100;
                    pct_p<=100;pct_second<=0;
                    if(vz_eligible==0) begin
                        frame_match_hits<=0;frame_match_support<=0;pct_state<=PCT_IDLE;
                    end else pct_state<=PCT_SCAN;
                    state<=VERIFY_PERCENT;
                end
                VERIFY_PERCENT:if(pct_state==PCT_IDLE) begin
                    if(verify_rejected) begin
                        rejected_frames<=rejected_frames+1'b1;state<=WAIT_LAP;last_frame_rejected<=1;
                        reject_reason<=verify_reject_code;diagnostic_points<=point_count;
                        diagnostic_hits<=frame_match_hits;diagnostic_support<=frame_match_support;proposal_valid<=0;
                    end else state<=START_COMPOSE;
                end
                START_COMPOSE:begin compose_start<=1;state<=WAIT_COMPOSE;end
                WAIT_COMPOSE:if(compose_done) begin
                    if(composed_x< -32768 || composed_x>32767 || composed_y< -32768 || composed_y>32767) begin
                        rejected_frames<=rejected_frames+1'b1;state<=WAIT_LAP;
                        last_frame_rejected<=1;reject_reason<=11;
                        diagnostic_points<=point_count;
                        diagnostic_hits<=frame_match_hits;diagnostic_support<=frame_match_support;
                        proposal_valid<=0;
                    end else if(seeded && nonzero_correction && !consistent_proposal) begin
                        // CONFIRM is an uncommitted complete observation, not
                        // a sensor fault. An inconsistent proposal replaces the
                        // pending one; a subsequent frame must agree with it.
                        proposal_valid<=1;proposal_tx<=composed_x[15:0];proposal_ty<=composed_y[15:0];
                        proposal_yaw<=new_yaw_wrapped[12:0];
                        rejected_frames<=rejected_frames+1'b1;state<=WAIT_LAP;last_frame_rejected<=1;reject_reason<=13;
                        diagnostic_points<=point_count;
                        diagnostic_hits<=frame_match_hits;diagnostic_support<=frame_match_support;
                    end else begin
                        proposal_valid<=0;
                        pending_tx<=composed_x[15:0];pending_ty<=composed_y[15:0];pending_yaw<=new_yaw_wrapped[12:0];state<=PUBLISH;
                    end
                end
                PUBLISH:if(!map_busy) begin
                    // Publish before replay. If a mode/health change interrupts
                    // replay, the already written subset still uses this same
                    // accepted frame transform. Previously written cells remain.
                    transform_x_mm<=pending_tx;transform_y_mm<=pending_ty;transform_yaw_tenths<=pending_yaw;
                    transform_update<=1;map_epoch<=map_epoch+1'b1;replay_index<=0;state<=EPOCH_WAIT;
                end
                EPOCH_WAIT:state<=READ_RAY; // allow epoch-wrap scrub to assert busy
                READ_RAY:if(!map_busy) state<=START_RAY;
                START_RAY:begin replay_start<=1;state<=WAIT_RAY;end
                WAIT_RAY:if(start_done && end_done) begin
                    map_start_x_mm<=replayed_sx;map_start_y_mm<=replayed_sy;
                    map_end_x_mm<=replayed_ex;map_end_y_mm<=replayed_ey;state<=SEND_RAY;
                end
                SEND_RAY:if(!map_busy) begin map_ray_update<=1;state<=DRAIN_RAY;end
                DRAIN_RAY:if(!map_ray_update && !map_busy) begin
                    if(replay_index==ray_count-1'b1) state<=FINISH;
                    else begin replay_index<=replay_index+1'b1;state<=READ_RAY;end
                end
                FINISH:begin
                    seeded<=1;accepted_frames<=accepted_frames+1'b1;
                    // Only a COMPLETE frame can make navigation map-ready.
                    // Interrupted first-frame replay leaves this counter zero.
                    accepted_samples<=completed_sample_count[16] ? 16'hffff : completed_sample_count[15:0];
                    last_frame_rejected<=0;reject_reason<=0;
                    diagnostic_points<=point_count;
                    diagnostic_hits<=frame_match_hits;diagnostic_support<=frame_match_support;
                    proposal_valid<=0;
                    state<=WAIT_LAP;
                end
                default:state<=WAIT_LAP;
            endcase
            // Percentages of the verified counts for the page. Started at the
            // decision point so both sweeps are complete; the three-digit page
            // fields cannot carry the raw counts of a full revolution.
            if(!abort_frame && state==VERIFY_PERCENT) case(pct_state)
                PCT_SCAN:
                    if(pct_num>=pct_acc) begin pct_val<=pct_p;pct_state<=PCT_NEXT;end
                    else begin pct_acc<=pct_acc-pct_den;pct_p<=pct_p-1'b1;end
                PCT_NEXT:
                    if(!pct_second) begin
                        pct_hit<=pct_val;pct_second<=1;
                        pct_num<=(verify_take_correction ? vb_support : vz_support)*100;pct_acc<=pct_den*100;
                        pct_p<=100;pct_state<=PCT_SCAN;
                    end else begin
                        pct_sup<=pct_val;frame_match_hits<=pct_hit;frame_match_support<=pct_val;
                        pct_state<=PCT_IDLE;
                    end
                default:pct_state<=PCT_IDLE;
            endcase
        end
    end
endmodule
