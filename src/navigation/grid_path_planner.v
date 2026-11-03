// Four-neighbour, uniform-cost BFS on an unchanged 128x128 scan map.
// Search runs backwards from goal, so each visited cell's parent leads to
// goal. Two alternating frontier bitmaps replace a large 16K-entry queue.
// RAM cost: 16384x4 unknown/visited/parent bits + 16384x2 frontier bits.
// The caller must freeze map writes for the entire busy interval.
//
// By default only FREE=1 is traversable. Every cell of the circular footprint with
// dx*dx+dy*dy<=CLEARANCE_CELLS^2 must be FREE and inside the map. UNKNOWN and OCCUPIED
// are blocked. CLEARANCE_CELLS=0 is a point model; the board must configure
// actual vehicle clearance (board: 14 cells =224mm, covering 175mm vehicle
// radius +16mm margin +up to22.6mm conservative grid-position uncertainty).
// ALLOW_UNKNOWN allows UNKNOWN=0 without changing the stored map. Each
// accepted cell records whether its circular footprint used unknown cells;
// route_uses_unknown reflects only the final replayed route, not the search.
// IGNORE_ENDPOINT_BLOCK skips the start/goal occupancy rejection and treats
// the two endpoint cells as traversable during footprint checks. Other
// occupied intermediate footprint cells and all map bounds remain enforced. An endpoint
// whose footprint extends outside the map reports no path in this mode.
// This planner never edits the map.
//
// map_req/address remain stable until map_req_ready. Exactly one response is
// outstanding; map_rsp_valid/data arrive no earlier than the following edge.
// Either handshake wait exceeding MAP_TIMEOUT_CLKS fails with code 4.
// start_plan is a pulse accepted when !busy (also cancels waypoint reading).
// abort has highest priority and immediately cancels requests and both busy
// outputs. It invalidates the previous result/path; a late response is ignored.
//
// path_clear pulses on a new plan, abort or failure. On a successful search,
// path_cell_valid/address replay every start-to-goal cell for a separate HDMI
// path mask. done/success are published after its final cell is written.
// After success: first waypoint_next emits start, subsequent requests emit
// straight-run endpoints (turns and goal). Requests after waypoint_last are
// ignored until path_rewind; rewind restores the first-start behaviour.
// waypoint_valid and done are one-clock pulses; success/fail_code are latched.
// fail: 1=start footprint blocked, 2=goal footprint blocked, 3=no path,
//       4=map timeout, 5=abort, 6=invalid internal parent chain.
module grid_path_planner #(
    parameter integer GRID_SIZE=128, GRID_BITS=$clog2(GRID_SIZE),
    parameter integer CLEARANCE_CELLS = 0,
    parameter integer MAP_TIMEOUT_CLKS = 500000,
    parameter integer IGNORE_ENDPOINT_BLOCK = 0,
    parameter integer ALLOW_UNKNOWN = 0
)(
    input wire clk, reset_n, start_plan, abort,
    input wire [GRID_BITS-1:0] start_x, start_y, goal_x, goal_y,
    output wire map_req,
    output wire [2*GRID_BITS-1:0] map_addr,
    input wire map_req_ready, map_rsp_valid,
    input wire [1:0] map_rsp_data,
    output wire busy,
    output reg done, success,
    output reg [3:0] fail_code,
    input wire path_rewind, waypoint_next,
    output wire waypoint_busy,
    output reg waypoint_valid,
    output reg [GRID_BITS-1:0] waypoint_x, waypoint_y,
    output reg waypoint_last,
    output reg path_clear, path_cell_valid,
    output reg [2*GRID_BITS-1:0] path_cell_addr,
    output reg route_uses_unknown
);
    localparam [4:0] IDLE=0, INITIALIZE=1, CHECK_GOAL=2, CHECK_START=3,
        FOOT_REQ=4, FOOT_WAIT=5, SEED_GOAL=6, FRONT_READ=7, FRONT_CHECK=8,
        NEIGHBOUR_SETUP=9, PARENT_READ=10, PARENT_CHECK=11,
        ADD_FRONT_READ=12, ADD_FRONT_WRITE=13, NEIGHBOUR_NEXT=14,
        SCAN_ADVANCE=15, TRACE_READ=16, TRACE_CHECK=17,
        WAY_READ=18, WAY_CHECK=19;
    localparam [1:0] CHECKING_GOAL=0, CHECKING_START=1, CHECKING_NEIGHBOUR=2;
    localparam integer WAIT_BITS=(MAP_TIMEOUT_CLKS<=1) ? 1 : $clog2(MAP_TIMEOUT_CLKS);
    reg [4:0] state;
    reg previous_abort;
    reg [GRID_BITS-1:0] saved_start_x, saved_start_y, saved_goal_x, saved_goal_y;
    reg [2*GRID_BITS-1:0] clear_addr, scan_addr;
    reg current_plane;
    reg [2*GRID_BITS:0] next_frontier_count;
    reg [GRID_BITS-1:0] current_x, current_y, candidate_x, candidate_y;
    // Directions: 0=+X, 1=-X, 2=+Y, 3=-Y. Opposite direction = dir ^ 1.
    reg [1:0] neighbour_dir, footprint_context;
    reg [GRID_BITS-1:0] check_x, check_y;
    reg signed [GRID_BITS+1:0] footprint_dx, footprint_dy;
    reg [WAIT_BITS-1:0] map_wait;
    reg footprint_uses_unknown, goal_uses_unknown;
    reg [GRID_BITS-1:0] trace_x, trace_y;
    reg [2*GRID_BITS:0] trace_steps;
    reg path_available, path_started, path_exhausted;
    reg [GRID_BITS-1:0] cursor_x, cursor_y;
    reg segment_direction_valid;
    reg [1:0] segment_direction;
    reg [2*GRID_BITS:0] segment_steps;

    assign waypoint_busy=(state==WAY_READ || state==WAY_CHECK);
    assign busy=(state!=IDLE) && !waypoint_busy;
    assign map_req=reset_n && !abort && (state==FOOT_REQ);
    wire signed [GRID_BITS+1:0] footprint_x=$signed({2'b00,check_x})+footprint_dx;
    wire signed [GRID_BITS+1:0] footprint_y=$signed({2'b00,check_y})+footprint_dy;
    assign map_addr={footprint_y[GRID_BITS-1:0],footprint_x[GRID_BITS-1:0]};
    // Row extents are a constant table for the configured disk. Loop
    // products below involve elaboration constants only, avoiding variable
    // multipliers/DSPs in the footprint address path.
    function [GRID_BITS-1:0] disk_row_extent;
        input signed [GRID_BITS+1:0] row;
        integer row_i,dx_i;
        begin
            disk_row_extent=0;
            for(row_i=-CLEARANCE_CELLS;row_i<=CLEARANCE_CELLS;row_i=row_i+1)
                if(row==row_i)
                    for(dx_i=0;dx_i<=CLEARANCE_CELLS;dx_i=dx_i+1)
                        if(dx_i*dx_i+row_i*row_i<=CLEARANCE_CELLS*CLEARANCE_CELLS)
                            disk_row_extent=dx_i;
        end
    endfunction
    wire [GRID_BITS-1:0] row_extent=disk_row_extent(footprint_dy);
    wire [GRID_BITS-1:0] next_row_extent=disk_row_extent(footprint_dy+9'sd1);
    wire footprint_last=(footprint_dx==9'sd0 && footprint_dy==CLEARANCE_CELLS);
    wire endpoint_cell=IGNORE_ENDPOINT_BLOCK &&
        ((map_addr=={saved_start_y,saved_start_x}) ||
         (map_addr=={saved_goal_y,saved_goal_x}));
    wire unknown_sample=ALLOW_UNKNOWN && (map_rsp_data==2'd0);
    wire footprint_cell_clear=(map_rsp_data==2'd1) || unknown_sample || endpoint_cell ||
        (IGNORE_ENDPOINT_BLOCK && footprint_context!=CHECKING_NEIGHBOUR);
    wire [GRID_BITS:0] neighbour_x=neighbour_dir==0 ? {1'b0,current_x}+8'd1 :
                             neighbour_dir==1 ? {1'b0,current_x}-8'd1 :
                             {1'b0,current_x};
    wire [GRID_BITS:0] neighbour_y=neighbour_dir==2 ? {1'b0,current_y}+8'd1 :
                             neighbour_dir==3 ? {1'b0,current_y}-8'd1 :
                             {1'b0,current_y};
    function footprint_inside;
        input [GRID_BITS-1:0] x,y;
        begin
            footprint_inside=(CLEARANCE_CELLS>=0 && CLEARANCE_CELLS<=63 &&
                x>=CLEARANCE_CELLS && x<=GRID_SIZE-1-CLEARANCE_CELLS &&
                y>=CLEARANCE_CELLS && y<=GRID_SIZE-1-CLEARANCE_CELLS);
        end
    endfunction
    function [GRID_BITS-1:0] step_x;
        input [GRID_BITS-1:0] x;
        input [1:0] direction;
        begin step_x=direction==0 ? x+7'd1 : direction==1 ? x-7'd1 : x; end
    endfunction
    function [GRID_BITS-1:0] step_y;
        input [GRID_BITS-1:0] y;
        input [1:0] direction;
        begin step_y=direction==2 ? y+7'd1 : direction==3 ? y-7'd1 : y; end
    endfunction

    // No asynchronous RAM resets or third read ports. Both small memories
    // clear sequentially at each plan; read outputs hold during writes so
    // Gowin can infer its supported NO_CHANGE block-RAM mode.
    (* ram_style="block", syn_ramstyle="block_ram" *)
    reg [3:0] parents [0:(1<<(2*GRID_BITS))-1];
    (* ram_style="block", syn_ramstyle="block_ram" *)
    reg [1:0] frontiers [0:(1<<(2*GRID_BITS))-1];
    reg [3:0] parent_data;
    reg [1:0] frontier_data;
    wire new_plan=start_plan && !busy;
    wire memory_enable=reset_n && !abort && !new_plan;
    wire accepted_neighbour=(state==FOOT_WAIT && map_rsp_valid &&
        footprint_cell_clear && footprint_last && footprint_context==CHECKING_NEIGHBOUR);
    wire [2*GRID_BITS-1:0] parent_addr=state==INITIALIZE ? clear_addr :
        state==SEED_GOAL ? {saved_goal_y,saved_goal_x} :
        (state==TRACE_READ || state==TRACE_CHECK) ? {trace_y,trace_x} :
        (state==WAY_READ || state==WAY_CHECK) ? {cursor_y,cursor_x} :
        {candidate_y,candidate_x};
    wire parent_write=memory_enable &&
        (state==INITIALIZE || state==SEED_GOAL || accepted_neighbour);
    wire [3:0] parent_write_data=state==INITIALIZE ? 4'd0 :
        state==SEED_GOAL ? {goal_uses_unknown,3'b100} :
            {(footprint_uses_unknown || unknown_sample),1'b1,(neighbour_dir ^ 2'b01)};
    wire [2*GRID_BITS-1:0] frontier_addr=state==INITIALIZE ? clear_addr :
        state==SEED_GOAL ? {saved_goal_y,saved_goal_x} :
        (state==ADD_FRONT_READ || state==ADD_FRONT_WRITE) ? {candidate_y,candidate_x} : scan_addr;
    wire consume_front=(state==FRONT_CHECK && frontier_data[current_plane]);
    wire frontier_write=memory_enable &&
        (state==INITIALIZE || state==SEED_GOAL || consume_front || state==ADD_FRONT_WRITE);
    wire [1:0] frontier_write_data=state==INITIALIZE ? 2'b00 :
        state==SEED_GOAL ? 2'b01 :
        state==ADD_FRONT_WRITE ?
            (current_plane ? {frontier_data[1],1'b1} : {1'b1,frontier_data[0]}) :
            (current_plane ? {1'b0,frontier_data[0]} : {frontier_data[1],1'b0});
    always @(posedge clk) begin
        if(parent_write) parents[parent_addr]<=parent_write_data;
        else parent_data<=parents[parent_addr];
        if(frontier_write) frontiers[frontier_addr]<=frontier_write_data;
        else frontier_data<=frontiers[frontier_addr];
    end

    task finish_failure;
        input [3:0] reason;
        begin
            state<=IDLE; done<=1; success<=0; fail_code<=reason;
            path_available<=0; path_started<=0; path_exhausted<=0;
            route_uses_unknown<=0;
            path_clear<=1;
        end
    endtask
    task start_footprint;
        input [GRID_BITS-1:0] x,y;
        input [1:0] check_context;
        begin
            check_x<=x; check_y<=y; footprint_context<=check_context;
            footprint_dx<=0; footprint_dy<=-CLEARANCE_CELLS;
            footprint_uses_unknown<=0;
            map_wait<=0; state<=FOOT_REQ;
        end
    endtask
    task begin_trace;
        begin trace_x<=saved_start_x; trace_y<=saved_start_y;
            trace_steps<=0; state<=TRACE_READ; end
    endtask

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE; previous_abort<=0; done<=0; success<=0; fail_code<=0;
            saved_start_x<=0; saved_start_y<=0; saved_goal_x<=0; saved_goal_y<=0;
            clear_addr<=0; scan_addr<=0; current_plane<=0; next_frontier_count<=0;
            current_x<=0; current_y<=0; candidate_x<=0; candidate_y<=0;
            neighbour_dir<=0; footprint_context<=0; check_x<=0; check_y<=0;
            footprint_dx<=0; footprint_dy<=0; map_wait<=0;
            footprint_uses_unknown<=0;goal_uses_unknown<=0;route_uses_unknown<=0;
            trace_x<=0; trace_y<=0; trace_steps<=0;
            path_available<=0; path_started<=0; path_exhausted<=0;
            cursor_x<=0; cursor_y<=0; segment_direction_valid<=0;
            segment_direction<=0; segment_steps<=0;
            waypoint_valid<=0; waypoint_x<=0; waypoint_y<=0; waypoint_last<=0;
            path_clear<=0; path_cell_valid<=0; path_cell_addr<=0;
        end else begin
            done<=0; waypoint_valid<=0; path_clear<=0; path_cell_valid<=0;
            previous_abort<=abort;
            if(abort) begin
                // A held cancellation keeps everything idle without turning
                // done/clear into level signals or restarting mask clearing.
                if(!previous_abort) finish_failure(4'd5);
            end
            else if(new_plan) begin
                saved_start_x<=start_x; saved_start_y<=start_y;
                saved_goal_x<=goal_x; saved_goal_y<=goal_y;
                state<=INITIALIZE; clear_addr<=0;
                success<=0; fail_code<=0; path_available<=0;
                route_uses_unknown<=0;goal_uses_unknown<=0;
                path_started<=0; path_exhausted<=0; waypoint_last<=0;
                path_clear<=1;
            end else case(state)
                IDLE: if(path_available) begin
                    if(path_rewind) begin
                        cursor_x<=saved_start_x; cursor_y<=saved_start_y;
                        path_started<=0; path_exhausted<=0; waypoint_last<=0;
                    end else if(waypoint_next && !path_exhausted) begin
                        if(!path_started) begin
                            waypoint_x<=saved_start_x; waypoint_y<=saved_start_y;
                            waypoint_valid<=1; path_started<=1;
                            waypoint_last<=(saved_start_x==saved_goal_x && saved_start_y==saved_goal_y);
                            path_exhausted<=(saved_start_x==saved_goal_x && saved_start_y==saved_goal_y);
                        end else begin
                            segment_direction_valid<=0; segment_steps<=0;
                            state<=WAY_READ;
                        end
                    end
                end
                INITIALIZE: if(clear_addr==((1<<(2*GRID_BITS))-1)) state<=CHECK_GOAL;
                            else clear_addr<=clear_addr+14'd1;
                CHECK_GOAL: if(!footprint_inside(saved_goal_x,saved_goal_y))
                                finish_failure(IGNORE_ENDPOINT_BLOCK ? 4'd3 : 4'd2);
                            else start_footprint(saved_goal_x,saved_goal_y,CHECKING_GOAL);
                CHECK_START: if(!footprint_inside(saved_start_x,saved_start_y))
                                 finish_failure(IGNORE_ENDPOINT_BLOCK ? 4'd3 : 4'd1);
                             else start_footprint(saved_start_x,saved_start_y,CHECKING_START);
                FOOT_REQ: begin
                    if(map_req_ready) begin map_wait<=0; state<=FOOT_WAIT; end
                    else if(map_wait>=MAP_TIMEOUT_CLKS-1) finish_failure(4'd4);
                    else map_wait<=map_wait+1'b1;
                end
                FOOT_WAIT: begin
                    if(map_rsp_valid) begin
                        map_wait<=0;
                        if(unknown_sample) footprint_uses_unknown<=1;
                        if(footprint_cell_clear) begin
                                if(!footprint_last) begin
                                    if(footprint_dx<$signed({2'b00,row_extent})) footprint_dx<=footprint_dx+9'sd1;
                                    else begin
                                        footprint_dx<=-$signed({2'b00,next_row_extent});
                                        footprint_dy<=footprint_dy+9'sd1;
                                    end
                                    state<=FOOT_REQ;
                                end else if(footprint_context==CHECKING_GOAL) begin
                                    goal_uses_unknown<=footprint_uses_unknown || unknown_sample;
                                    state<=CHECK_START;
                                end
                                else if(footprint_context==CHECKING_START) state<=SEED_GOAL;
                                else if(candidate_x==saved_start_x && candidate_y==saved_start_y) begin_trace();
                                else begin next_frontier_count<=next_frontier_count+15'd1; state<=ADD_FRONT_READ; end
                        end else begin
                                if(footprint_context==CHECKING_GOAL) finish_failure(4'd2);
                                else if(footprint_context==CHECKING_START) finish_failure(4'd1);
                                else state<=NEIGHBOUR_NEXT;
                        end
                    end else if(map_wait>=MAP_TIMEOUT_CLKS-1) finish_failure(4'd4);
                    else map_wait<=map_wait+1'b1;
                end
                SEED_GOAL: begin
                    current_plane<=0; scan_addr<=0; next_frontier_count<=0;
                    if(saved_start_x==saved_goal_x && saved_start_y==saved_goal_y) begin_trace();
                    else state<=FRONT_READ;
                end
                FRONT_READ: state<=FRONT_CHECK;
                FRONT_CHECK: begin
                    if(frontier_data[current_plane]) begin
                        current_x<=scan_addr[GRID_BITS-1:0]; current_y<=scan_addr[2*GRID_BITS-1:GRID_BITS];
                        neighbour_dir<=0; state<=NEIGHBOUR_SETUP;
                    end else state<=SCAN_ADVANCE;
                end
                NEIGHBOUR_SETUP: begin
                    if(neighbour_x[GRID_BITS] || neighbour_y[GRID_BITS] ||
                       !footprint_inside(neighbour_x[GRID_BITS-1:0],neighbour_y[GRID_BITS-1:0])) state<=NEIGHBOUR_NEXT;
                    else begin
                        candidate_x<=neighbour_x[GRID_BITS-1:0]; candidate_y<=neighbour_y[GRID_BITS-1:0];
                        state<=PARENT_READ;
                    end
                end
                PARENT_READ: state<=PARENT_CHECK;
                PARENT_CHECK: if(parent_data[2]) state<=NEIGHBOUR_NEXT;
                              else start_footprint(candidate_x,candidate_y,CHECKING_NEIGHBOUR);
                ADD_FRONT_READ: state<=ADD_FRONT_WRITE;
                ADD_FRONT_WRITE: state<=NEIGHBOUR_NEXT;
                NEIGHBOUR_NEXT: if(neighbour_dir==3) state<=SCAN_ADVANCE;
                                else begin neighbour_dir<=neighbour_dir+2'd1; state<=NEIGHBOUR_SETUP; end
                SCAN_ADVANCE: begin
                    if(scan_addr==((1<<(2*GRID_BITS))-1)) begin
                        if(next_frontier_count==0) finish_failure(4'd3);
                        else begin
                            current_plane<=~current_plane; next_frontier_count<=0;
                            scan_addr<=0; state<=FRONT_READ;
                        end
                    end else begin scan_addr<=scan_addr+14'd1; state<=FRONT_READ; end
                end
                TRACE_READ: begin
                    path_cell_valid<=1; path_cell_addr<={trace_y,trace_x};
                    state<=TRACE_CHECK;
                end
                TRACE_CHECK: begin
                    route_uses_unknown<=route_uses_unknown || (ALLOW_UNKNOWN && parent_data[3]);
                    if(trace_x==saved_goal_x && trace_y==saved_goal_y) begin
                        state<=IDLE; done<=1; success<=1; fail_code<=0;
                        path_available<=1; path_started<=0; path_exhausted<=0;
                        cursor_x<=saved_start_x; cursor_y<=saved_start_y;
                    end else if(!parent_data[2] || trace_steps>=((1<<(2*GRID_BITS))-1)) finish_failure(4'd6);
                    else begin
                        trace_x<=step_x(trace_x,parent_data[1:0]);
                        trace_y<=step_y(trace_y,parent_data[1:0]);
                        trace_steps<=trace_steps+15'd1; state<=TRACE_READ;
                    end
                end
                WAY_READ: state<=WAY_CHECK;
                WAY_CHECK: begin
                    if(cursor_x==saved_goal_x && cursor_y==saved_goal_y) begin
                        waypoint_valid<=1; waypoint_x<=cursor_x; waypoint_y<=cursor_y;
                        waypoint_last<=1; path_exhausted<=1; state<=IDLE;
                    end else if(!parent_data[2] || segment_steps>=((1<<(2*GRID_BITS))-1)) finish_failure(4'd6);
                    else if(segment_direction_valid && parent_data[1:0]!=segment_direction) begin
                        waypoint_valid<=1; waypoint_x<=cursor_x; waypoint_y<=cursor_y;
                        waypoint_last<=0; state<=IDLE;
                    end else begin
                        segment_direction<=parent_data[1:0]; segment_direction_valid<=1;
                        cursor_x<=step_x(cursor_x,parent_data[1:0]);
                        cursor_y<=step_y(cursor_y,parent_data[1:0]);
                        segment_steps<=segment_steps+15'd1; state<=WAY_READ;
                    end
                end
                default: finish_failure(4'd6);
            endcase
        end
    end
endmodule
