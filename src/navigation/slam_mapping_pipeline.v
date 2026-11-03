// FPGA continuous SLAM frontend and bounded historical backend.
// Input rays are in RAW odometry coordinates, never in a changing map frame.
// Two live scan banks decouple acquisition from matching/replay. Historical
// keyframes retain uniformly decimated measured rays, not invented endpoints.
// Loop candidates are matched against an isolated historical keyframe map,
// independently confirmed with an interleaved point subset, then constrained
// in an anchored SE(2) pose graph. All historical keyframes are re-rendered.
module slam_mapping_pipeline #(
    parameter integer CLK_FREQ_HZ=50000000,CELL_MM=20,GRID_SIZE=210, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y),
    parameter integer MAX_RAYS=2048,MAX_KEYFRAMES=64,HISTORY_RAYS=512,MIN_POINTS=24,
    parameter integer KEYFRAME_DISTANCE_MM=350,KEYFRAME_YAW_TENTHS=300,
    parameter integer LOOP_MIN_GAP=6,LOOP_DISTANCE_MM=800,LOOP_SEARCH_RADIUS=10,
    parameter integer FRAME_TIMEOUT_MS=800,INPUT_RAY_PIPELINE=0,
    parameter integer DEFER_LOOPS_DURING_NAV=0,
    parameter integer KB=$clog2(MAX_KEYFRAMES),RB=$clog2(MAX_RAYS),HB=$clog2(HISTORY_RAYS)
)(
    input wire clk,reset_n,enable,clear,rescan,allow,healthy,stationary,
    input wire navigation_active,
    input wire ray_update,ray_frame_start,input wire [6:0] ray_bin,
    input wire [3:0] ray_bin_error,input wire [31:0] ray_timestamp,
    input wire signed [31:0] ray_start_x_mm,ray_start_y_mm,ray_end_x_mm,ray_end_y_mm,
    input wire signed [31:0] current_heading_tenths,
    input wire map_busy,map_clear_done,
    output reg rebuild_clear,output reg map_ray_update,output wire loop_reference,
    output reg signed [31:0] map_start_x_mm,map_start_y_mm,map_end_x_mm,map_end_y_mm,
    output reg [7:0] map_epoch,
    output wire map_req,output wire [2*GRID_BITS-1:0] map_addr,
    input wire map_req_ready,map_rsp_valid,input wire [1:0] map_rsp_data,
    output wire busy,output wire [3:0] status,
    output reg [15:0] accepted_samples,accepted_frames,rejected_frames,match_score,
    output reg [3:0] reject_reason,output reg [7:0] diagnostic_points,diagnostic_hits,diagnostic_support,
    output reg transform_update,output reg signed [15:0] transform_x_mm,transform_y_mm,
    output reg signed [12:0] transform_yaw_tenths,
    output reg [15:0] loop_closures,optimized_loops,dropped_frames,
    output wire history_full,output wire [KB:0] keyframe_count
);
    // Retain the whole ray transaction together: health, heading, timestamp
    // and coordinates always refer to the same sample. Live cancel controls
    // flush pending data before the scan-bank write enable can accept it.
    wire capture_healthy;
    wire capture_stationary;
    wire capture_ray_update;
    wire capture_ray_frame_start;
    wire [6:0] capture_ray_bin;
    wire [3:0] capture_ray_bin_error;
    wire [31:0] capture_ray_timestamp;
    wire signed [31:0] capture_ray_start_x_mm;
    wire signed [31:0] capture_ray_start_y_mm;
    wire signed [31:0] capture_ray_end_x_mm;
    wire signed [31:0] capture_ray_end_y_mm;
    wire signed [31:0] capture_current_heading_tenths;
    wire capture_ray_fits;
    // Evaluate the range predicate on the input side of the existing ray
    // register. The flag travels with exactly the same transaction, so scan
    // admission keeps its original cycle while removing eight wide compares
    // from the high-fanout bank/seen-count enables.
    function ray_coordinates_fit;
        input signed [31:0] sx,sy,ex,ey;
        begin ray_coordinates_fit=sx>=-16000&&sx<=16000&&sy>=-16000&&sy<=16000&&
            ex>=-32768&&ex<=32767&&ey>=-32768&&ey<=32767;end
    endfunction
    generate if(INPUT_RAY_PIPELINE)begin:g_input_ray_pipeline
        reg ray_fits_r;
        assign capture_ray_fits=ray_fits_r;
        always @(posedge clk)ray_fits_r<=ray_coordinates_fit(ray_start_x_mm,ray_start_y_mm,ray_end_x_mm,ray_end_y_mm);
        reg healthy_r;
        assign capture_healthy=healthy_r;
        reg stationary_r;
        assign capture_stationary=stationary_r;
        reg ray_update_r;
        assign capture_ray_update=ray_update_r;
        reg ray_frame_start_r;
        assign capture_ray_frame_start=ray_frame_start_r;
        reg [6:0] ray_bin_r;
        assign capture_ray_bin=ray_bin_r;
        reg [3:0] ray_bin_error_r;
        assign capture_ray_bin_error=ray_bin_error_r;
        reg [31:0] ray_timestamp_r;
        assign capture_ray_timestamp=ray_timestamp_r;
        reg signed [31:0] ray_start_x_mm_r;
        assign capture_ray_start_x_mm=ray_start_x_mm_r;
        reg signed [31:0] ray_start_y_mm_r;
        assign capture_ray_start_y_mm=ray_start_y_mm_r;
        reg signed [31:0] ray_end_x_mm_r;
        assign capture_ray_end_x_mm=ray_end_x_mm_r;
        reg signed [31:0] ray_end_y_mm_r;
        assign capture_ray_end_y_mm=ray_end_y_mm_r;
        reg signed [31:0] current_heading_tenths_r;
        assign capture_current_heading_tenths=current_heading_tenths_r;
        always @(posedge clk or negedge reset_n)
            if(!reset_n)begin healthy_r<=0;stationary_r<=0;ray_update_r<=0;ray_frame_start_r<=0;end
            else if(!enable||clear||rescan||!allow)begin healthy_r<=0;stationary_r<=0;ray_update_r<=0;ray_frame_start_r<=0;end
            else begin healthy_r<=healthy;stationary_r<=stationary;ray_update_r<=ray_update;ray_frame_start_r<=ray_frame_start;end
        always @(posedge clk)begin ray_bin_r<=ray_bin;ray_bin_error_r<=ray_bin_error;ray_timestamp_r<=ray_timestamp;ray_start_x_mm_r<=ray_start_x_mm;ray_start_y_mm_r<=ray_start_y_mm;ray_end_x_mm_r<=ray_end_x_mm;ray_end_y_mm_r<=ray_end_y_mm;current_heading_tenths_r<=current_heading_tenths;end
    end else begin:g_direct_ray
        assign capture_ray_fits=ray_coordinates_fit(ray_start_x_mm,ray_start_y_mm,ray_end_x_mm,ray_end_y_mm);
        assign capture_healthy=healthy;
        assign capture_stationary=stationary;
        assign capture_ray_update=ray_update;
        assign capture_ray_frame_start=ray_frame_start;
        assign capture_ray_bin=ray_bin;
        assign capture_ray_bin_error=ray_bin_error;
        assign capture_ray_timestamp=ray_timestamp;
        assign capture_ray_start_x_mm=ray_start_x_mm;
        assign capture_ray_start_y_mm=ray_start_y_mm;
        assign capture_ray_end_x_mm=ray_end_x_mm;
        assign capture_ray_end_y_mm=ray_end_y_mm;
        assign capture_current_heading_tenths=current_heading_tenths;
    end endgenerate
    localparam IDLE=0,PIVOT=1,PIVOT_WAIT=2,LOAD_READ=3,LOAD_START=4,LOAD_WAIT=5,MATCH=6,MATCH_WAIT=7,
        COMPOSE=8,COMPOSE_WAIT=9,PUBLISH=10,REPLAY_READ=11,REPLAY_START=12,REPLAY_START_WAIT=13,
        REPLAY_END=14,REPLAY_END_WAIT=15,SEND=16,DRAIN=17,COMPLETE=18,
        ADD_NODE=19,EDGE_PREP=20,EDGE_WAIT=21,EDGE_SEND=22,LOOP_FIND=23,LOOP_CLEAR=24,
        LOOP_CLEAR_WAIT=25,HISTORY_HEADER=26,HISTORY_READ=27,HISTORY_START=28,HISTORY_START_WAIT=29,
        HISTORY_END=30,HISTORY_END_WAIT=31,HISTORY_SEND=32,HISTORY_DRAIN=33,
        LOOP_LOAD=34,LOOP_MATCH=35,LOOP_MATCH_WAIT=36,LOOP_CONFIRM=37,
        LOOP_EDGE=38,LOOP_EDGE_WAIT=39,LOOP_EDGE_SEND=40,OPTIMIZE=41,OPTIMIZE_WAIT=42,
        REBUILD_CLEAR=43,REBUILD_WAIT=44,REBUILD_FINISH=45,NEW_TRANSFORM=46,NEW_TRANSFORM_WAIT=47,
        VERIFY_INIT=48,VERIFY_READ=49,VERIFY_START=50,VERIFY_WAIT=51,VERIFY_SCORE=52,
        VERIFY_SCORE_WAIT=53,VERIFY_NEXT=54,VERIFY_DECIDE=55,
        LOOP_DISTANCE=56,LOOP_SELECT=57,LOOP_ABORT=58;
    localparam integer MAP_HALF_X_MM=GRID_X*CELL_MM/2, MAP_HALF_Y_MM=GRID_Y*CELL_MM/2;
    reg [5:0] state;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [63:0] scan_ram[0:2*MAX_RAYS-1];
    // Lossless mm start offsets from the keyframe origin, full 16-bit endpoints.
    // Reject a whole frame if an origin offset cannot fit; never wrap a ray.
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [53:0] history_ram[0:MAX_KEYFRAMES*HISTORY_RAYS-1];
    reg [1:0] ready;
    reg collecting,write_bank,processing_bank,bad_scan;
    reg [RB:0] write_count,counts[0:1],scan_count;
    reg [31:0] scan_time,bank_time[0:1];
    reg signed [15:0] bank_x[0:1],bank_y[0:1];
    reg signed [12:0] bank_yaw[0:1];
    reg signed [15:0] ox,oy,px,py,pending_x,pending_y,matched_dx,matched_dy;
    reg signed [12:0] oa,pa,pending_yaw,matched_da;
    reg [RB-1:0] index;
    reg [63:0] scan_data;
    reg [53:0] history_data;
    reg [7:0] load_count;
    reg [RB:0] sample_stride,next_sample,archive_stride,next_archive;
    reg [HB:0] archived_count;
    reg load_second;
    reg matcher_load,matcher_start;
    reg [6:0] matcher_index;
    reg signed [15:0] matcher_x,matcher_y;
    wire matcher_busy,matcher_done,matched,ambiguous;
    wire signed [15:0] mdx,mdy;
    wire signed [12:0] mda;
    wire [15:0] mscore,mbase;
    wire [7:0] mhits,msupport;
    wire [3:0] mreason;
    wire matcher_req;
    wire [2*GRID_BITS-1:0] matcher_addr;
    reg verify_pass,verify_start;
    reg [19:0] verify_score,zero_score;
    reg [RB:0] verify_hits,verify_support,verify_count,zero_hits,zero_support,zero_count;
    wire verification=state>=VERIFY_INIT&&state<=VERIFY_DECIDE;
    wire verifier_busy,verifier_done,verifier_hit,verifier_support,verifier_req;
    wire [8:0] verifier_score;
    wire [2*GRID_BITS-1:0] verifier_addr;
    reg read_pending,read_owner;
    wire zero_good=zero_count>=MIN_POINTS&&zero_hits*100>=zero_count*30&&zero_support*100>=zero_count*85;
    wire verified_correction=zero_count>=MIN_POINTS&&verify_hits*100>=zero_count*40&&
        verify_support*100>=zero_count*75&&verify_score>=zero_score+32&&verify_score*100>=zero_score*103;
    wire correction_nonzero=matched_dx!=0||matched_dy!=0||matched_da!=0;
    reg [1:0] pct_state;
    reg [7:0] pct_candidate,pct_hit;
    reg [RB:0] pct_den;
    // Separate match-quality selection from percentage scaling. The decision
    // depends on wide score comparisons; multiplying its selected count by
    // 100 in the same cycle created the routed SYS50 critical path.
    reg [RB:0] pct_hit_count,pct_sup_count;
    reg [31:0] pct_hit_num,pct_sup_num;
    wire [31:0] pct_product=pct_den*pct_candidate;
    reg tr_start;
    reg signed [15:0] tr_x,tr_y,tr_px,tr_py,tr_dx,tr_dy;
    reg signed [12:0] tr_a;
    wire tr_busy,tr_done;
    wire signed [31:0] tr_result_x,tr_result_y;
    // Every flag is written in the baseline verification pass before the
    // correction pass reads it. No reset sweep is needed: cancellation starts
    // a fresh baseline pass. Synchronous RAM avoids a2048-bit FF/mux tree.
    (* ram_style="block",syn_ramstyle="block_ram" *) reg eligible[0:MAX_RAYS-1];
    reg eligible_read,current_eligible;
    reg [RB-1:0] eligible_read_address;
    wire eligible_value=tr_result_x>=-MAP_HALF_X_MM&&tr_result_x<MAP_HALF_X_MM&&
        tr_result_y>=-MAP_HALF_Y_MM&&tr_result_y<MAP_HALF_Y_MM;
    // Independent registered read address infers the supported simple-dual-
    // port RAM. It is settled many transform clocks before correction scoring;
    // the baseline pass uses the explicit current-value bypass below.
    always @(posedge clk)begin
        eligible_read_address<=index;
        eligible_read<=eligible[eligible_read_address];
        if(reset_n&&enable&&!clear&&state==VERIFY_WAIT&&tr_done&&!verify_pass)
            eligible[index]<=eligible_value;
    end

    reg [KB-1:0] current_node,candidate,historical_node,query_node;
    reg [KB-1:0] best_candidate;
    reg candidate_found;
    reg [31:0] candidate_distance;
    reg signed [16:0] candidate_x,candidate_y;
    reg [31:0] candidate_square_x,candidate_square_y;
    reg candidate_near;
    reg [119:0] seen_bins;
    reg [7:0] seen_count;
    reg [3:0] seen_quadrants;
    wire [2:0] quadrant_count={2'd0,seen_quadrants[0]}+{2'd0,seen_quadrants[1]}+
        {2'd0,seen_quadrants[2]}+{2'd0,seen_quadrants[3]};
    reg [HB-1:0] historical_index,archive_index;
    reg [HB:0] history_counts[0:MAX_KEYFRAMES-1];
    reg signed [15:0] history_ox[0:MAX_KEYFRAMES-1],history_oy[0:MAX_KEYFRAMES-1];
    reg signed [12:0] history_oa[0:MAX_KEYFRAMES-1];
    reg signed [15:0] last_kx,last_ky,last_map_x,last_map_y;
    reg signed [12:0] last_ka,last_map_a;
    reg signed [15:0] hx,hy,hox,hoy;
    reg signed [12:0] ha,hoa;
    reg graph_add_node,graph_add_edge,graph_optimize;
    reg [KB-1:0] graph_i,graph_j;
    reg signed [15:0] graph_dx,graph_dy;
    reg signed [12:0] graph_da;
    reg graph_loop,loop_map,second_confirmation,rebuilding;
    reg signed [15:0] loop_dx,loop_dy;
    reg signed [12:0] loop_da;
    wire graph_busy,graph_done,graph_accepted,graph_capacity;
    wire signed [15:0] graph_x,graph_y;
    wire signed [12:0] graph_yaw;
    wire [47:0] graph_error_before,graph_error_after;
    wire capture_allowed=enable&&allow&&capture_healthy&&!rescan;
    wire ray_fits=capture_ray_fits;
    wire process_active=state!=IDLE;
    wire other_free=!ready[!write_bank]&&(!process_active||processing_bank!=!write_bank);
    wire taking_scan=state==IDLE&&(ready[0]||ready[1]);
    wire write_first=capture_ray_update&&capture_ray_frame_start&&other_free&&ray_fits;
    wire signed [31:0] frame_motion_x=capture_ray_start_x_mm-$signed(bank_x[write_bank]);
    wire signed [31:0] frame_motion_y=capture_ray_start_y_mm-$signed(bank_y[write_bank]);
    wire motion_fits=frame_motion_x>=-1024&&frame_motion_x<=1023&&frame_motion_y>=-1024&&frame_motion_y<=1023;
    wire write_next=capture_ray_update&&!capture_ray_frame_start&&collecting&&write_count<MAX_RAYS&&ray_fits&&motion_fits;
    wire scan_write=capture_allowed&&(write_first||write_next);
    wire [RB:0] scan_write_addr=write_first?{!write_bank,{RB{1'b0}}}:{write_bank,write_count[RB-1:0]};
    wire [KB+HB-1:0] history_read_addr={historical_node,historical_index};
    wire [KB+HB-1:0] history_write_addr={current_node,archive_index};
    wire archive_write=state==DRAIN&&!map_busy&&!map_ray_update&&
        keyframe_count<MAX_KEYFRAMES&&index==next_archive&&archived_count<HISTORY_RAYS;
    wire signed [15:0] archive_start_dx=$signed(scan_data[63:48])-$signed(ox);
    wire signed [15:0] archive_start_dy=$signed(scan_data[47:32])-$signed(oy);
    wire signed [31:0] key_dx=$signed(ox)-$signed(last_kx),key_dy=$signed(oy)-$signed(last_ky);
    wire signed [13:0] key_da=wrap_angle($signed(oa)-$signed(last_ka));
    wire new_keyframe=keyframe_count==0||key_dx>KEYFRAME_DISTANCE_MM||key_dx< -KEYFRAME_DISTANCE_MM||
        key_dy>KEYFRAME_DISTANCE_MM||key_dy< -KEYFRAME_DISTANCE_MM||key_da>KEYFRAME_YAW_TENTHS||key_da< -KEYFRAME_YAW_TENTHS;
    wire signed [31:0] candidate_dx=$signed(last_map_x)-$signed(graph_x),candidate_dy=$signed(last_map_y)-$signed(graph_y);
    assign busy=process_active||matcher_busy||verifier_busy||read_pending||graph_busy;
    assign map_req=verification?verifier_req:matcher_req;
    assign map_addr=verification?verifier_addr:matcher_addr;
    assign status=!enable?0:!capture_healthy?4:busy?13:reject_reason!=0?14:5;
    assign history_full=keyframe_count>=MAX_KEYFRAMES||graph_capacity;
    assign loop_reference=loop_map&&!rebuilding;
    // Route a delayed response to the requester that actually issued it.
    // The pipeline state can change on clear/cancel before that response.
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin read_pending<=0;read_owner<=0;end
        else begin
            if(map_rsp_valid)read_pending<=0;
            if(map_req&&map_req_ready)begin read_pending<=1;read_owner<=verification;end
        end
    end
    function signed [12:0] wrap_angle;
        input signed [31:0] a;
        begin wrap_angle=a>1800?a-3600:a< -1800?a+3600:a;end
    endfunction
    // Register a complete immutable archive transaction before the large
    // banked history RAM. Metadata/epoch/map-busy decode no longer reaches
    // every RAM clock-enable through a long combinational path. A pending
    // pre-clear write is harmless: clear invalidates all history descriptors.
    reg archive_pending;
    reg [KB+HB-1:0] archive_addr_r;
    reg [53:0] archive_data_r;
    always @(posedge clk or negedge reset_n)
        if(!reset_n) archive_pending<=0;
        else archive_pending<=archive_write;
    always @(posedge clk) begin
        archive_addr_r<=history_write_addr;
        archive_data_r<={archive_start_dx[10:0],archive_start_dy[10:0],scan_data[31:0]};
    end
    always @(posedge clk)begin
        if(scan_write)scan_ram[scan_write_addr]<={capture_ray_start_x_mm[15:0],capture_ray_start_y_mm[15:0],capture_ray_end_x_mm[15:0],capture_ray_end_y_mm[15:0]};
        scan_data<=scan_ram[{processing_bank,index}];
        if(archive_pending)history_ram[archive_addr_r]<=archive_data_r;
        history_data<=history_ram[history_read_addr];
    end
    slam_rigid_transform transform(.clk(clk),.reset_n(reset_n),.start(tr_start),.cancel(clear||!enable),
        .point_x(tr_x),.point_y(tr_y),.pivot_x(tr_px),.pivot_y(tr_py),.translation_x(tr_dx),.translation_y(tr_dy),
        .angle_tenths(tr_a),.busy(tr_busy),.done(tr_done),.result_x(tr_result_x),.result_y(tr_result_y));
    scan_matcher_multires #(.CELL_MM(CELL_MM),.GRID_SIZE(GRID_SIZE),.GRID_X(GRID_X),.GRID_Y(GRID_Y),.GRID_BITS(GRID_BITS),.WIDE_RADIUS(LOOP_SEARCH_RADIUS)) matcher(
        .clk(clk),.reset_n(reset_n),.cancel(clear||!enable),.load_valid(matcher_load),.load_index(matcher_index),
        .load_x_mm(matcher_x),.load_y_mm(matcher_y),.start(matcher_start),.search_wide(loop_map),.point_count(load_count),
        .pivot_x_mm(px),.pivot_y_mm(py),.busy(matcher_busy),.done(matcher_done),.matched(matched),.ambiguous(ambiguous),
        .delta_x_mm(mdx),.delta_y_mm(mdy),.delta_yaw_tenths(mda),.best_score(mscore),.baseline_score(mbase),
        .hits(mhits),.support(msupport),.reject_reason(mreason),.map_req(matcher_req),.map_addr(matcher_addr),
        .map_req_ready(map_req_ready&&!verification),.map_rsp_valid(map_rsp_valid&&read_pending&&!read_owner),.map_rsp_data(map_rsp_data));
    slam_endpoint_score #(.CELL_MM(CELL_MM),.GRID_SIZE(GRID_SIZE),.GRID_X(GRID_X),.GRID_Y(GRID_Y),.GRID_BITS(GRID_BITS)) verifier(
        .clk(clk),.reset_n(reset_n),.start(verify_start),.cancel(clear||!enable),.x_mm(tr_result_x),.y_mm(tr_result_y),
        .busy(verifier_busy),.done(verifier_done),.score(verifier_score),.hit(verifier_hit),.supported(verifier_support),
        .map_req(verifier_req),.map_addr(verifier_addr),.map_req_ready(map_req_ready&&verification),
        .map_rsp_valid(map_rsp_valid&&read_pending&&read_owner),.map_rsp_data(map_rsp_data));
    slam_pose_graph #(.MAX_NODES(MAX_KEYFRAMES),.MAX_EDGES(2*MAX_KEYFRAMES),.NODE_BITS(KB)) graph(
        .clk(clk),.reset_n(reset_n),.clear(clear||!enable),.add_node(graph_add_node),
        .node_x(last_map_x),.node_y(last_map_y),.node_yaw(last_map_a),
        .add_edge(graph_add_edge),.edge_i(graph_i),.edge_j(graph_j),.edge_dx(graph_dx),.edge_dy(graph_dy),
        .edge_dyaw(graph_da),.edge_loop(graph_loop),.start_optimize(graph_optimize),
        .busy(graph_busy),.done(graph_done),.accepted(graph_accepted),.capacity_fault(graph_capacity),
        .node_count(keyframe_count),.query_node(query_node),.query_x(graph_x),.query_y(graph_y),.query_yaw(graph_yaw),
        .initial_error(graph_error_before),.final_error(graph_error_after));
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;ready<=0;collecting<=0;write_bank<=0;processing_bank<=0;bad_scan<=0;write_count<=0;scan_time<=0;
            scan_count<=0;ox<=0;oy<=0;oa<=0;px<=0;py<=0;pa<=0;pending_x<=0;pending_y<=0;pending_yaw<=0;
            matched_dx<=0;matched_dy<=0;matched_da<=0;index<=0;load_count<=0;sample_stride<=1;next_sample<=0;archive_stride<=1;next_archive<=0;archived_count<=0;
            load_second<=0;matcher_load<=0;matcher_start<=0;matcher_index<=0;matcher_x<=0;matcher_y<=0;
            tr_start<=0;tr_x<=0;tr_y<=0;tr_px<=0;tr_py<=0;tr_dx<=0;tr_dy<=0;tr_a<=0;
            current_node<=0;candidate<=0;historical_node<=0;query_node<=0;historical_index<=0;archive_index<=0;
            best_candidate<=0;candidate_found<=0;candidate_distance<=32'hffffffff;seen_bins<=0;seen_count<=0;seen_quadrants<=0;
            candidate_x<=0;candidate_y<=0;candidate_square_x<=0;candidate_square_y<=0;candidate_near<=0;
            last_kx<=0;last_ky<=0;last_ka<=0;last_map_x<=0;last_map_y<=0;last_map_a<=0;
            hx<=0;hy<=0;ha<=0;hox<=0;hoy<=0;hoa<=0;
            graph_add_node<=0;graph_add_edge<=0;graph_optimize<=0;graph_i<=0;graph_j<=0;graph_dx<=0;graph_dy<=0;graph_da<=0;graph_loop<=0;
            loop_map<=0;second_confirmation<=0;rebuilding<=0;loop_dx<=0;loop_dy<=0;loop_da<=0;
            rebuild_clear<=0;map_ray_update<=0;map_start_x_mm<=0;map_start_y_mm<=0;map_end_x_mm<=0;map_end_y_mm<=0;map_epoch<=0;
            accepted_samples<=0;accepted_frames<=0;rejected_frames<=0;match_score<=0;reject_reason<=0;
            diagnostic_points<=0;diagnostic_hits<=0;diagnostic_support<=0;transform_update<=0;transform_x_mm<=0;transform_y_mm<=0;transform_yaw_tenths<=0;
            loop_closures<=0;optimized_loops<=0;dropped_frames<=0;
            verify_pass<=0;verify_start<=0;current_eligible<=0;verify_score<=0;zero_score<=0;verify_hits<=0;verify_support<=0;verify_count<=0;zero_hits<=0;zero_support<=0;zero_count<=0;
            pct_state<=0;pct_candidate<=0;pct_hit<=0;pct_den<=0;pct_hit_num<=0;pct_sup_num<=0;
            pct_hit_count<=0;pct_sup_count<=0;
        end else begin
            matcher_load<=0;matcher_start<=0;tr_start<=0;verify_start<=0;transform_update<=0;map_ray_update<=0;rebuild_clear<=0;
            graph_add_node<=0;graph_add_edge<=0;graph_optimize<=0;
            if(!enable||clear)begin
                state<=IDLE;ready<=0;collecting<=0;write_count<=0;bad_scan<=0;
                transform_x_mm<=0;transform_y_mm<=0;transform_yaw_tenths<=0;map_epoch<=0;
                accepted_samples<=0;accepted_frames<=0;rejected_frames<=0;reject_reason<=0;
                loop_closures<=0;optimized_loops<=0;dropped_frames<=0;loop_map<=0;rebuilding<=0;
                pct_state<=0;diagnostic_hits<=0;diagnostic_support<=0;
            end else begin
                // Reception never waits for matching, optimization or rendering.
                if(!capture_allowed)begin collecting<=0;write_count<=0;ready<=0;end
                else if(capture_ray_update)begin
                    if(capture_ray_frame_start)begin
                        if(collecting)begin
                            if(!bad_scan&&write_count>=MIN_POINTS&&seen_count>=MIN_POINTS&&quadrant_count>=3&&
                               capture_ray_timestamp-scan_time<CLK_FREQ_HZ/1000*FRAME_TIMEOUT_MS)begin
                                counts[write_bank]<=write_count;ready[write_bank]<=1;
                            end else dropped_frames<=dropped_frames+1'b1;
                        end
                        if(other_free&&ray_fits)begin
                            write_bank<=!write_bank;collecting<=1;bad_scan<=0;write_count<=1;scan_time<=capture_ray_timestamp;
                            bank_x[!write_bank]<=capture_ray_start_x_mm[15:0];bank_y[!write_bank]<=capture_ray_start_y_mm[15:0];
                            bank_yaw[!write_bank]<=capture_current_heading_tenths[12:0];bank_time[!write_bank]<=capture_ray_timestamp;
                            seen_bins<=120'd1<<capture_ray_bin;seen_count<=1;
                            seen_quadrants<=capture_ray_bin<30?1:capture_ray_bin<60?2:capture_ray_bin<90?4:8;
                        end else begin collecting<=0;write_count<=0;end
                    end else if(collecting)begin
                        if(write_count<MAX_RAYS&&ray_fits&&motion_fits)write_count<=write_count+1'b1;
                        else bad_scan<=1;
                        if(capture_ray_bin<120)begin
                            if(!seen_bins[capture_ray_bin])begin seen_bins[capture_ray_bin]<=1;seen_count<=seen_count+1'b1;end
                            if(capture_ray_bin<30)seen_quadrants[0]<=1;else if(capture_ray_bin<60)seen_quadrants[1]<=1;
                            else if(capture_ray_bin<90)seen_quadrants[2]<=1;else seen_quadrants[3]<=1;
                        end
                    end
                end
                case(state)
                    IDLE:if(capture_allowed&&(ready[0]||ready[1])&&!matcher_busy&&!verifier_busy&&!map_busy)begin
                        processing_bank<=ready[0]?0:1;ready[ready[0]?0:1]<=0;
                        scan_count<=counts[ready[0]?0:1];ox<=bank_x[ready[0]?0:1];oy<=bank_y[ready[0]?0:1];oa<=bank_yaw[ready[0]?0:1];
                        archive_stride<=(counts[ready[0]?0:1]+HISTORY_RAYS-1)/HISTORY_RAYS;
                        sample_stride<=(counts[ready[0]?0:1]+119)/120;index<=0;next_sample<=0;load_count<=0;load_second<=0;loop_map<=0;
                        state<=PIVOT;
                    end
                    PIVOT:begin
                        tr_x<=ox;tr_y<=oy;tr_px<=0;tr_py<=0;tr_dx<=transform_x_mm;tr_dy<=transform_y_mm;tr_a<=transform_yaw_tenths;
                        tr_start<=1;state<=PIVOT_WAIT;
                    end
                    PIVOT_WAIT:if(tr_done)begin
                        px<=tr_result_x[15:0];py<=tr_result_y[15:0];pa<=wrap_angle($signed(oa)+$signed(transform_yaw_tenths));state<=LOAD_READ;
                    end
                    LOAD_READ:state<=LOAD_START;
                    LOAD_START:begin
                        tr_x<=scan_data[31:16];tr_y<=scan_data[15:0];tr_px<=0;tr_py<=0;
                        tr_dx<=transform_x_mm;tr_dy<=transform_y_mm;tr_a<=transform_yaw_tenths;tr_start<=1;state<=LOAD_WAIT;
                    end
                    LOAD_WAIT:if(tr_done)begin
                        if(tr_result_x>=-MAP_HALF_X_MM&&tr_result_x<MAP_HALF_X_MM&&
                           tr_result_y>=-MAP_HALF_Y_MM&&tr_result_y<MAP_HALF_Y_MM&&load_count<120)begin
                            matcher_load<=1;matcher_index<=load_count[6:0];matcher_x<=tr_result_x[15:0];matcher_y<=tr_result_y[15:0];load_count<=load_count+1'b1;
                        end
                        if(index+sample_stride>=scan_count)state<=loop_map?LOOP_MATCH:MATCH;
                        else begin index<=index+sample_stride;state<=LOAD_READ;end
                    end
                    MATCH:if(keyframe_count==0)begin
                        if(capture_stationary)begin matched_dx<=0;matched_dy<=0;matched_da<=0;state<=COMPOSE;end
                        else begin rejected_frames<=rejected_frames+1'b1;reject_reason<=6;state<=IDLE;end
                    end else if(!matcher_busy)begin matcher_start<=1;state<=MATCH_WAIT;end
                    MATCH_WAIT:if(matcher_done)begin
                        diagnostic_points<=load_count;match_score<=mscore;
                        if(matched&&!ambiguous)begin matched_dx<=mdx;matched_dy<=mdy;matched_da<=mda;state<=COMPOSE;end
                        else begin rejected_frames<=rejected_frames+1'b1;reject_reason<=mreason;state<=IDLE;end
                    end
                    COMPOSE:begin
                        tr_x<=transform_x_mm;tr_y<=transform_y_mm;tr_px<=px;tr_py<=py;tr_dx<=matched_dx;tr_dy<=matched_dy;tr_a<=matched_da;
                        tr_start<=1;pending_yaw<=wrap_angle($signed(transform_yaw_tenths)+$signed(matched_da));state<=COMPOSE_WAIT;
                    end
                    COMPOSE_WAIT:if(tr_done)begin
                        pending_x<=tr_result_x[15:0];pending_y<=tr_result_y[15:0];verify_pass<=0;
                        state<=keyframe_count==0?PUBLISH:VERIFY_INIT;
                    end
                    VERIFY_INIT:begin
                        index<=0;verify_score<=0;verify_hits<=0;verify_support<=0;verify_count<=0;state<=VERIFY_READ;
                    end
                    VERIFY_READ:state<=VERIFY_START;
                    VERIFY_START:begin
                        tr_x<=scan_data[31:16];tr_y<=scan_data[15:0];tr_px<=0;tr_py<=0;
                        tr_dx<=verify_pass?pending_x:transform_x_mm;tr_dy<=verify_pass?pending_y:transform_y_mm;
                        tr_a<=verify_pass?pending_yaw:transform_yaw_tenths;tr_start<=1;state<=VERIFY_WAIT;
                    end
                    VERIFY_WAIT:if(tr_done)begin
                        if(!verify_pass)begin
                            current_eligible<=eligible_value;
                        end else current_eligible<=eligible_read;
                        state<=VERIFY_SCORE;
                    end
                    VERIFY_SCORE:if(current_eligible)begin verify_start<=1;state<=VERIFY_SCORE_WAIT;end
                        else state<=VERIFY_NEXT;
                    VERIFY_SCORE_WAIT:if(verifier_done)begin
                        verify_score<=verify_score+verifier_score;verify_hits<=verify_hits+verifier_hit;
                        verify_support<=verify_support+verifier_support;verify_count<=verify_count+1'b1;state<=VERIFY_NEXT;
                    end
                    VERIFY_NEXT:if(index+1>=scan_count)begin
                        if(!verify_pass)begin
                            zero_score<=verify_score;zero_hits<=verify_hits;zero_support<=verify_support;zero_count<=verify_count;
                            verify_pass<=1;state<=VERIFY_INIT;
                        end else state<=VERIFY_DECIDE;
                    end else begin index<=index+1'b1;state<=VERIFY_READ;end
                    VERIFY_DECIDE:begin
                        diagnostic_points<=load_count;
                        if(zero_count!=0)begin
                            pct_den<=zero_count;pct_candidate<=100;pct_state<=3;
                            pct_hit_count<=correction_nonzero&&verified_correction?verify_hits:zero_hits;
                            pct_sup_count<=correction_nonzero&&verified_correction?verify_support:zero_support;
                        end else begin diagnostic_hits<=0;diagnostic_support<=0;end
                        if(correction_nonzero&&verified_correction)begin
                            state<=PUBLISH;
                        end else if(zero_good)begin
                            pending_x<=transform_x_mm;pending_y<=transform_y_mm;pending_yaw<=transform_yaw_tenths;
                            state<=PUBLISH;
                        end else begin reject_reason<=7;rejected_frames<=rejected_frames+1'b1;state<=IDLE;end
                    end
                    PUBLISH:if(!map_busy)begin
                        transform_x_mm<=pending_x;transform_y_mm<=pending_y;transform_yaw_tenths<=pending_yaw;transform_update<=1;
                        map_epoch<=map_epoch+1'b1;index<=0;archive_index<=0;archived_count<=0;next_archive<=0;
                        current_node<=keyframe_count[KB-1:0];state<=REPLAY_READ;
                    end
                    REPLAY_READ:if(!map_busy)state<=REPLAY_START;
                    REPLAY_START:begin
                        tr_x<=scan_data[63:48];tr_y<=scan_data[47:32];tr_px<=0;tr_py<=0;tr_dx<=pending_x;tr_dy<=pending_y;tr_a<=pending_yaw;
                        tr_start<=1;state<=REPLAY_START_WAIT;
                    end
                    REPLAY_START_WAIT:if(tr_done)begin map_start_x_mm<=tr_result_x;map_start_y_mm<=tr_result_y;state<=REPLAY_END;end
                    REPLAY_END:begin tr_x<=scan_data[31:16];tr_y<=scan_data[15:0];tr_start<=1;state<=REPLAY_END_WAIT;end
                    REPLAY_END_WAIT:if(tr_done)begin map_end_x_mm<=tr_result_x;map_end_y_mm<=tr_result_y;state<=SEND;end
                    SEND:if(!map_busy)begin map_ray_update<=1;state<=DRAIN;end
                    DRAIN:if(!map_busy&&!map_ray_update)begin
                        if(archive_write)begin archive_index<=archive_index+1'b1;archived_count<=archived_count+1'b1;next_archive<=next_archive+archive_stride;end
                        if(index+1>=scan_count)state<=COMPLETE;
                        else begin index<=index+1'b1;state<=REPLAY_READ;end
                    end
                    COMPLETE:begin
                        accepted_frames<=accepted_frames+1'b1;accepted_samples<=accepted_samples>65535-scan_count?65535:accepted_samples+scan_count;
                        reject_reason<=0;
                        if(new_keyframe&&!history_full)begin
                            history_counts[current_node]<=archived_count;
                            history_ox[current_node]<=ox;history_oy[current_node]<=oy;history_oa[current_node]<=oa;
                            last_kx<=ox;last_ky<=oy;last_ka<=oa;state<=ADD_NODE;
                        end else state<=IDLE;
                    end
                    ADD_NODE:begin
                        tr_x<=ox;tr_y<=oy;tr_px<=0;tr_py<=0;tr_dx<=pending_x;tr_dy<=pending_y;tr_a<=pending_yaw;
                        tr_start<=1;state<=EDGE_PREP;
                    end
                    EDGE_PREP:if(tr_done)begin
                        last_map_x<=tr_result_x[15:0];last_map_y<=tr_result_y[15:0];last_map_a<=wrap_angle($signed(oa)+$signed(pending_yaw));
                        px<=tr_result_x[15:0];py<=tr_result_y[15:0];pa<=wrap_angle($signed(oa)+$signed(pending_yaw));
                        graph_add_node<=1;query_node<=current_node-1'b1;state<=EDGE_WAIT;
                    end
                    EDGE_WAIT:if(current_node==0)state<=IDLE;
                        else begin
                            tr_x<=last_map_x;tr_y<=last_map_y;tr_px<=graph_x;tr_py<=graph_y;
                            tr_dx<=-graph_x;tr_dy<=-graph_y;tr_a<=-graph_yaw;
                            graph_da<=wrap_angle($signed(last_map_a)-$signed(graph_yaw));tr_start<=1;state<=EDGE_SEND;
                        end
                    EDGE_SEND:if(tr_done)begin
                        graph_dx<=tr_result_x[15:0];graph_dy<=tr_result_y[15:0];graph_i<=current_node-1'b1;graph_j<=current_node;
                        graph_loop<=0;graph_add_edge<=1;candidate<=0;query_node<=0;candidate_found<=0;candidate_distance<=32'hffffffff;state<=LOOP_FIND;
                    end
                    LOOP_FIND:begin
                        // Continue live scan matching and map writes, but do
                        // not erase/rebuild the historical map while driving.
                        // A backend already running before navigation is
                        // drained by PREPLAN before the planner can start.
                        if(DEFER_LOOPS_DURING_NAV && navigation_active) state<=IDLE;
                        else if(candidate+LOOP_MIN_GAP>current_node)begin
                            if(candidate_found)begin
                                candidate<=best_candidate;historical_node<=best_candidate;query_node<=best_candidate;
                                loop_map<=1;second_confirmation<=0;rebuilding<=0;state<=LOOP_CLEAR;
                            end else state<=IDLE;
                        end else begin
                            candidate_x<=candidate_dx;candidate_y<=candidate_dy;
                            candidate_near<=candidate_dx<=LOOP_DISTANCE_MM&&candidate_dx>=-LOOP_DISTANCE_MM&&
                                candidate_dy<=LOOP_DISTANCE_MM&&candidate_dy>=-LOOP_DISTANCE_MM;
                            state<=LOOP_DISTANCE;
                        end
                    end
                    LOOP_DISTANCE:begin
                        candidate_square_x<=candidate_x*candidate_x;candidate_square_y<=candidate_y*candidate_y;
                        state<=LOOP_SELECT;
                    end
                    LOOP_SELECT:begin
                        if(candidate_near&&candidate_square_x+candidate_square_y<candidate_distance)begin
                            best_candidate<=candidate;candidate_found<=1;candidate_distance<=candidate_square_x+candidate_square_y;
                        end
                        candidate<=candidate+1'b1;query_node<=candidate+1'b1;state<=LOOP_FIND;
                    end
                    LOOP_CLEAR:if(!map_busy)begin rebuild_clear<=1;state<=LOOP_CLEAR_WAIT;end
                    LOOP_CLEAR_WAIT:if(map_clear_done&&!rebuild_clear)state<=HISTORY_HEADER;
                    HISTORY_HEADER:begin
                        hx<=graph_x;hy<=graph_y;ha<=graph_yaw;hox<=history_ox[historical_node];hoy<=history_oy[historical_node];hoa<=history_oa[historical_node];
                        historical_index<=0;map_epoch<=map_epoch+1'b1;state<=HISTORY_READ;
                    end
                    HISTORY_READ:if(!map_busy)state<=HISTORY_START;
                    HISTORY_START:begin
                        tr_x<=hox+$signed(history_data[53:43]);tr_y<=hoy+$signed(history_data[42:32]);tr_px<=hox;tr_py<=hoy;
                        tr_dx<=hx-hox;tr_dy<=hy-hoy;tr_a<=wrap_angle($signed(ha)-$signed(hoa));tr_start<=1;state<=HISTORY_START_WAIT;
                    end
                    HISTORY_START_WAIT:if(tr_done)begin map_start_x_mm<=tr_result_x;map_start_y_mm<=tr_result_y;state<=HISTORY_END;end
                    HISTORY_END:begin tr_x<=history_data[31:16];tr_y<=history_data[15:0];tr_start<=1;state<=HISTORY_END_WAIT;end
                    HISTORY_END_WAIT:if(tr_done)begin map_end_x_mm<=tr_result_x;map_end_y_mm<=tr_result_y;state<=HISTORY_SEND;end
                    HISTORY_SEND:if(!map_busy)begin map_ray_update<=1;state<=HISTORY_DRAIN;end
                    HISTORY_DRAIN:if(!map_busy&&!map_ray_update)begin
                        if(historical_index+1>=history_counts[historical_node])begin
                            if(!rebuilding)state<=LOOP_LOAD;
                            else if(historical_node+1>=keyframe_count)state<=REBUILD_FINISH;
                            else begin historical_node<=historical_node+1'b1;query_node<=historical_node+1'b1;state<=HISTORY_HEADER;end
                        end else begin historical_index<=historical_index+1'b1;state<=HISTORY_READ;end
                    end
                    LOOP_LOAD:begin
                        if(sample_stride<2)sample_stride<=2;
                        index<=second_confirmation?(sample_stride<2?1:(sample_stride>>1)):0;
                        load_count<=0;loop_map<=1;state<=LOAD_READ;
                    end
                    LOOP_MATCH:if(!matcher_busy)begin matcher_start<=1;state<=LOOP_MATCH_WAIT;end
                    LOOP_MATCH_WAIT:if(matcher_done)begin
                        // Strict independent subsets: every point is measured,
                        // and neither pass can modify the reference keyframe.
                        if(matched&&!ambiguous&&load_count>=MIN_POINTS&&mhits*100>=load_count*60&&msupport*100>=load_count*90)begin
                            if(!second_confirmation)begin loop_dx<=mdx;loop_dy<=mdy;loop_da<=mda;second_confirmation<=1;state<=LOOP_LOAD;end
                            else if($signed(mdx)-$signed(loop_dx)<=20&&$signed(mdx)-$signed(loop_dx)>=-20&&
                                    $signed(mdy)-$signed(loop_dy)<=20&&$signed(mdy)-$signed(loop_dy)>=-20&&
                                    $signed(mda)-$signed(loop_da)<=4&&$signed(mda)-$signed(loop_da)>=-4)state<=LOOP_EDGE;
                            else state<=LOOP_ABORT;
                        end else state<=LOOP_ABORT;
                    end
                    LOOP_ABORT:begin loop_map<=0;state<=IDLE;end
                    LOOP_EDGE:begin
                        tr_x<=last_map_x;tr_y<=last_map_y;tr_px<=px;tr_py<=py;tr_dx<=loop_dx;tr_dy<=loop_dy;tr_a<=loop_da;
                        tr_start<=1;state<=LOOP_EDGE_WAIT;
                    end
                    LOOP_EDGE_WAIT:if(tr_done)begin
                        tr_x<=tr_result_x[15:0];tr_y<=tr_result_y[15:0];tr_px<=graph_x;tr_py<=graph_y;
                        tr_dx<=-graph_x;tr_dy<=-graph_y;tr_a<=-graph_yaw;tr_start<=1;
                        graph_da<=wrap_angle($signed(last_map_a)+$signed(loop_da)-$signed(graph_yaw));state<=LOOP_EDGE_SEND;
                    end
                    LOOP_EDGE_SEND:if(tr_done)begin
                        graph_dx<=tr_result_x[15:0];graph_dy<=tr_result_y[15:0];graph_i<=candidate;graph_j<=current_node;graph_loop<=1;
                        graph_add_edge<=1;loop_closures<=loop_closures+1'b1;state<=OPTIMIZE;
                    end
                    OPTIMIZE:begin graph_optimize<=1;state<=OPTIMIZE_WAIT;end
                    OPTIMIZE_WAIT:if(graph_done)begin
                        if(graph_accepted)begin optimized_loops<=optimized_loops+1'b1;state<=REBUILD_CLEAR;end
                        else state<=LOOP_ABORT;
                    end
                    REBUILD_CLEAR:if(!map_busy&&!matcher_busy)begin
                        rebuild_clear<=1;rebuilding<=1;historical_node<=0;query_node<=0;loop_map<=0;state<=REBUILD_WAIT;
                    end
                    REBUILD_WAIT:if(map_clear_done&&!rebuild_clear)state<=HISTORY_HEADER;
                    REBUILD_FINISH:begin query_node<=current_node;state<=NEW_TRANSFORM;end
                    NEW_TRANSFORM:begin
                        tr_x<=ox;tr_y<=oy;tr_px<=0;tr_py<=0;tr_dx<=0;tr_dy<=0;tr_a<=wrap_angle($signed(graph_yaw)-$signed(oa));
                        pending_yaw<=wrap_angle($signed(graph_yaw)-$signed(oa));tr_start<=1;state<=NEW_TRANSFORM_WAIT;
                    end
                    NEW_TRANSFORM_WAIT:if(tr_done)begin
                        transform_x_mm<=$signed(graph_x)-tr_result_x;transform_y_mm<=$signed(graph_y)-tr_result_y;
                        transform_yaw_tenths<=pending_yaw;transform_update<=1;rebuilding<=0;state<=IDLE;
                    end
                    default:state<=IDLE;
                endcase
                case(pct_state)
                    3:begin pct_hit_num<=pct_hit_count*100;pct_sup_num<=pct_sup_count*100;pct_state<=1;end
                    1:if(pct_hit_num>=pct_product)begin pct_hit<=pct_candidate;pct_candidate<=100;pct_state<=2;end
                        else pct_candidate<=pct_candidate-1'b1;
                    2:if(pct_sup_num>=pct_product)begin diagnostic_hits<=pct_hit;diagnostic_support<=pct_candidate;pct_state<=0;end
                        else pct_candidate<=pct_candidate-1'b1;
                    default:begin end
                endcase
            end
        end
    end
endmodule
