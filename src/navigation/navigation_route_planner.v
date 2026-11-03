`timescale 1ns/1ps
// Shortest BFS remains the complete fallback. Finite H-V/V-H/H-V-H/V-H-V
// plus four-leg H-V-H-V/V-H-V-H candidates may add max(ceil(shortest/5),2) cells to reduce total turns.
// Initial heading changes count as turns (inclusive five-degree tolerance).
// No extra grid memory. Only the selected route is replayed/published.
module navigation_route_planner #(
parameter integer GRID_SIZE=128, GRID_BITS=$clog2(GRID_SIZE),
    parameter integer CLEARANCE_CELLS=0,MAP_TIMEOUT_CLKS=500000,
parameter integer IGNORE_ENDPOINT_BLOCK=0,ALLOW_UNKNOWN=0,
parameter integer ENABLE_ROUTE_OPTIMIZATION=1,INITIAL_TURN_TOLERANCE_TENTHS=50,
// 1 = re-read every cell of the replayed route before it is published.  A cell
// that reads back OCCUPIED aborts the plan (fail 7) instead of drawing a route
// the map says is blocked.  Endpoints keep the IGNORE_ENDPOINT_BLOCK exemption.
parameter integer VERIFY_PATH_BEFORE_PUBLISH=1
)(
input wire clk,reset_n,start_plan,abort,
input wire [GRID_BITS-1:0] start_x,start_y,goal_x,goal_y,
input wire signed [31:0] start_heading_tenths,
output wire map_req,output wire [2*GRID_BITS-1:0] map_addr,
input wire map_req_ready,map_rsp_valid,input wire [1:0] map_rsp_data,
output wire busy,output reg done,success,output reg [3:0] fail_code,
input wire path_rewind,waypoint_next,output wire waypoint_busy,waypoint_valid,
output wire [GRID_BITS-1:0] waypoint_x,waypoint_y,output wire waypoint_last,
output reg path_clear,path_cell_valid,output reg [2*GRID_BITS-1:0] path_cell_addr,
output reg route_uses_unknown,output wire optimized,
// High while a published cell is re-read from the map.  The map port is busy
// during publication in that mode, so observers must not flag it as a search.
output wire verify_active,
output reg [2*GRID_BITS:0] route_steps,route_turns
);
localparam [5:0] IDLE=0,CORE_START=1,CORE_WAIT=2,BASE_SCORE=3,DIVIDE=4,LIMIT_COMMIT=5,
ENUM_BUILD=6,ENUM_SCORE=7,ENUM_SELECT=8,CAND_START=9,FOOT_START=10,FOOT_REQ=11,FOOT_WAIT=12,
CAND_ADVANCE=13,ENUM_NEXT=14,CHOOSE_FINAL=15,BASE_REWIND=16,BASE_FIRST_REQ=17,BASE_FIRST_WAIT=18,
EMIT_START=19,BASE_NEXT_REQ=20,BASE_NEXT_WAIT=21,OPT_NEXT_LEG=22,SEG_EMIT=23,SEG_FINISH=24,
FINAL_REWIND=25,FINAL_PUBLISH=26,OPT_WAY_FIND=27,SEG_CHK_REQ=28,SEG_CHK_WAIT=29;
localparam integer WAIT_BITS=MAP_TIMEOUT_CLKS<=1?1:$clog2(MAP_TIMEOUT_CLKS);
reg [5:0] state;
// High while a published cell is re-read from the map.  The map port is busy
// during publication in that mode, so observers must not flag it as a search.
assign verify_active=VERIFY_PATH_BEFORE_PUBLISH&&(state==SEG_CHK_REQ||state==SEG_CHK_WAIT);
reg previous_abort,core_abort_pulse,result_available,best_opt;
reg [GRID_BITS-1:0] saved_start_x,saved_start_y,saved_goal_x,saved_goal_y;
reg signed [12:0] saved_heading;reg saved_heading_valid;
reg base_seen,base_dir_valid;
reg [GRID_BITS-1:0] base_prev_x,base_prev_y;
reg [1:0] base_prev_dir,base_first_dir;
reg [2*GRID_BITS:0] base_steps,base_internal_turns,best_steps,best_turns;
reg best_unknown;
reg [2*GRID_BITS:0] div_value,div_quotient;reg [2:0] div_remainder;reg [4:0] div_bits;reg [15:0] length_limit;
reg [2:0] candidate_kind;reg [GRID_BITS-1:0] hinge,hinge_second;
reg [GRID_BITS-1:0] candidate_end_x[0:3],candidate_end_y[0:3],best_end_x[0:3],best_end_y[0:3];
reg [2:0] score_index;reg [1:0] score_prev_dir;reg [GRID_BITS-1:0] score_prev_x,score_prev_y;reg score_dir_valid;
reg [2*GRID_BITS:0] candidate_steps,candidate_turns;reg candidate_unknown;
reg [2:0] candidate_leg;reg [GRID_BITS-1:0] candidate_cursor_x,candidate_cursor_y;
reg signed [GRID_BITS+1:0] footprint_dx,footprint_dy;reg [WAIT_BITS-1:0] map_wait;
reg [15:0] waypoint_wait;
reg replay_opt,segment_last;reg [2:0] replay_leg;
reg [GRID_BITS-1:0] replay_x,replay_y,segment_end_x,segment_end_y;
reg [GRID_BITS-1:0] check_x,check_y;
reg opt_way_started,opt_way_exhausted;reg [2:0] opt_way_index;
reg [GRID_BITS-1:0] opt_way_cursor_x,opt_way_cursor_y,own_way_x,own_way_y;reg own_way_valid,own_way_last;
integer i;
wire core_busy,core_done,core_success,core_map_req,core_wp_busy,core_wp_valid,core_wp_last;
wire [3:0] core_fail;wire [2*GRID_BITS-1:0] core_map_addr,core_path_addr;
wire core_path_valid,core_route_unknown;wire [GRID_BITS-1:0] core_wp_x,core_wp_y;
wire core_phase=state==CORE_START||state==CORE_WAIT;
assign waypoint_busy=result_available&&(best_opt?state==OPT_WAY_FIND:(state==IDLE&&core_wp_busy));
assign busy=state!=IDLE&&!waypoint_busy;
wire accept_plan=start_plan&&!busy&&!abort;
wire core_abort=abort||accept_plan||core_abort_pulse;
wire core_start=state==CORE_START&&reset_n&&!abort;
wire core_rewind=state==BASE_REWIND||state==FINAL_REWIND||
(state==IDLE&&result_available&&!best_opt&&path_rewind&&!accept_plan);
wire core_next=((state==BASE_FIRST_REQ||state==BASE_NEXT_REQ)&&!core_wp_busy)||
(state==IDLE&&result_available&&!best_opt&&waypoint_next&&!path_rewind&&!accept_plan);
assign waypoint_valid=result_available&&!abort&&!accept_plan&&
(best_opt?own_way_valid:(state==IDLE&&core_wp_valid));
assign waypoint_x=best_opt?own_way_x:core_wp_x;
assign waypoint_y=best_opt?own_way_y:core_wp_y;
assign waypoint_last=best_opt?own_way_last:core_wp_last;
assign optimized=result_available&&best_opt;
grid_path_planner #(.GRID_SIZE(GRID_SIZE),.GRID_BITS(GRID_BITS),.CLEARANCE_CELLS(CLEARANCE_CELLS),.MAP_TIMEOUT_CLKS(MAP_TIMEOUT_CLKS),
.IGNORE_ENDPOINT_BLOCK(IGNORE_ENDPOINT_BLOCK),.ALLOW_UNKNOWN(ALLOW_UNKNOWN)) u_shortest(
.clk(clk),.reset_n(reset_n),.start_plan(core_start),.abort(core_abort),
.start_x(saved_start_x),.start_y(saved_start_y),.goal_x(saved_goal_x),.goal_y(saved_goal_y),
.map_req(core_map_req),.map_addr(core_map_addr),.map_req_ready(map_req_ready&&core_phase),
.map_rsp_valid(map_rsp_valid&&core_phase),.map_rsp_data(map_rsp_data),
.busy(core_busy),.done(core_done),.success(core_success),.fail_code(core_fail),
.path_rewind(core_rewind),.waypoint_next(core_next),.waypoint_busy(core_wp_busy),
.waypoint_valid(core_wp_valid),.waypoint_x(core_wp_x),.waypoint_y(core_wp_y),
.waypoint_last(core_wp_last),.path_clear(),.path_cell_valid(core_path_valid),
.path_cell_addr(core_path_addr),.route_uses_unknown(core_route_unknown));

function [1:0] direction;
input [GRID_BITS-1:0] ax,ay,bx,by;
begin direction=bx>ax?2'd0:bx<ax?2'd1:by>ay?2'd2:2'd3;end endfunction
function [7:0] axis_distance;
input [GRID_BITS-1:0] a,b;
begin axis_distance=a>=b?{1'b0,a}-{1'b0,b}:{1'b0,b}-{1'b0,a};end endfunction
function needs_initial_turn;
input [1:0] d;input signed [12:0] h;input valid_heading;
begin
if(!valid_heading)needs_initial_turn=1;
else case(d)
0:needs_initial_turn=!(h>=-INITIAL_TURN_TOLERANCE_TENTHS&&h<=INITIAL_TURN_TOLERANCE_TENTHS);
1:needs_initial_turn=!(h>=1800-INITIAL_TURN_TOLERANCE_TENTHS||h<=-1800+INITIAL_TURN_TOLERANCE_TENTHS);
2:needs_initial_turn=!(h>=900-INITIAL_TURN_TOLERANCE_TENTHS&&h<=900+INITIAL_TURN_TOLERANCE_TENTHS);
default:needs_initial_turn=!(h>=-900-INITIAL_TURN_TOLERANCE_TENTHS&&h<=-900+INITIAL_TURN_TOLERANCE_TENTHS);
endcase end endfunction
function footprint_inside;
input [GRID_BITS-1:0] x,y;
begin footprint_inside=CLEARANCE_CELLS>=0&&CLEARANCE_CELLS<=63&&x>=CLEARANCE_CELLS&&
x<=GRID_SIZE-1-CLEARANCE_CELLS&&y>=CLEARANCE_CELLS&&y<=GRID_SIZE-1-CLEARANCE_CELLS;end endfunction
function [GRID_BITS-1:0] row_extent;
input signed [GRID_BITS+1:0] row;integer ry,rx;
begin row_extent=0;
for(ry=-CLEARANCE_CELLS;ry<=CLEARANCE_CELLS;ry=ry+1)if(row==ry)
for(rx=0;rx<=CLEARANCE_CELLS;rx=rx+1)if(rx*rx+ry*ry<=CLEARANCE_CELLS*CLEARANCE_CELLS)row_extent=rx;
end endfunction
wire signed [GRID_BITS+1:0] footprint_x=$signed({2'b00,candidate_cursor_x})+footprint_dx;
wire signed [GRID_BITS+1:0] footprint_y=$signed({2'b00,candidate_cursor_y})+footprint_dy;
wire [2*GRID_BITS-1:0] candidate_map_addr={footprint_y[GRID_BITS-1:0],footprint_x[GRID_BITS-1:0]};
assign map_req=reset_n&&!abort&&!accept_plan&&
(core_phase?core_map_req:(state==FOOT_REQ||(VERIFY_PATH_BEFORE_PUBLISH&&state==SEG_CHK_REQ)));
assign map_addr=core_phase?core_map_addr:
(VERIFY_PATH_BEFORE_PUBLISH&&state==SEG_CHK_REQ)?{check_y,check_x}:candidate_map_addr;
// The published route may start or end inside an obstacle by policy only.
wire check_endpoint=(check_x==saved_start_x&&check_y==saved_start_y)||
(check_x==saved_goal_x&&check_y==saved_goal_y);
wire sample_unknown=ALLOW_UNKNOWN&&map_rsp_data==2'd0;
wire endpoint_sample=IGNORE_ENDPOINT_BLOCK&&(candidate_map_addr=={saved_start_y,saved_start_x}||
candidate_map_addr=={saved_goal_y,saved_goal_x});
// Match the unchanged BFS's goal seed: its complete disk is admitted;
// all other candidate disks still block occupied non-endpoint cells.
wire goal_disk_exception=IGNORE_ENDPOINT_BLOCK&&candidate_cursor_x==saved_goal_x&&candidate_cursor_y==saved_goal_y;
wire sample_clear=map_rsp_data==2'd1||sample_unknown||endpoint_sample||goal_disk_exception;
wire footprint_last=footprint_dx==0&&footprint_dy==CLEARANCE_CELLS;
wire [GRID_BITS-1:0] extent=row_extent(footprint_dy),next_extent=row_extent(footprint_dy+9'sd1);
wire [3:0] div_trial={div_remainder,div_value[2*GRID_BITS]};
wire [2*GRID_BITS:0] rounded_extra=div_quotient+(div_remainder!=0);
wire [1:0] score_direction=direction(score_prev_x,score_prev_y,candidate_end_x[score_index],candidate_end_y[score_index]);
wire [8:0] score_distance={1'b0,axis_distance(score_prev_x,candidate_end_x[score_index])}+
{1'b0,axis_distance(score_prev_y,candidate_end_y[score_index])};
wire [1:0] base_direction=direction(base_prev_x,base_prev_y,core_path_addr[GRID_BITS-1:0],core_path_addr[2*GRID_BITS-1:GRID_BITS]);
wire [8:0] base_distance={1'b0,axis_distance(base_prev_x,core_path_addr[GRID_BITS-1:0])}+
{1'b0,axis_distance(base_prev_y,core_path_addr[2*GRID_BITS-1:GRID_BITS])};
wire [GRID_BITS-1:0] candidate_step_x=candidate_cursor_x<candidate_end_x[candidate_leg]?candidate_cursor_x+7'd1:
candidate_cursor_x>candidate_end_x[candidate_leg]?candidate_cursor_x-7'd1:candidate_cursor_x;
wire [GRID_BITS-1:0] candidate_step_y=candidate_cursor_y<candidate_end_y[candidate_leg]?candidate_cursor_y+7'd1:
candidate_cursor_y>candidate_end_y[candidate_leg]?candidate_cursor_y-7'd1:candidate_cursor_y;
wire [GRID_BITS-1:0] replay_step_x=replay_x<segment_end_x?replay_x+7'd1:replay_x>segment_end_x?replay_x-7'd1:replay_x;
wire [GRID_BITS-1:0] replay_step_y=replay_y<segment_end_y?replay_y+7'd1:replay_y>segment_end_y?replay_y-7'd1:replay_y;
// Zero-length trailing legs are omitted consistently in publication/waypoints.
function last_best_leg;
input [2:0] index;
integer j;
begin last_best_leg=1;
for(j=0;j<4;j=j+1)if(j>index &&
(best_end_x[j]!=best_end_x[index] || best_end_y[j]!=best_end_y[index]))last_best_leg=0;
end endfunction

task finish_failure;input [3:0] code;
begin state<=IDLE;done<=1;success<=0;fail_code<=code;result_available<=0;best_opt<=0;
route_uses_unknown<=0;route_steps<=0;route_turns<=0;path_clear<=1;own_way_valid<=0;
opt_way_exhausted<=1;core_abort_pulse<=1;end endtask
task start_segment;input [GRID_BITS-1:0] x,y;input last;
begin if(x!=replay_x&&y!=replay_y)finish_failure(4'd6);
else begin segment_end_x<=x;segment_end_y<=y;segment_last<=last;state<=SEG_EMIT;end end endtask

always @(posedge clk or negedge reset_n)begin
if(!reset_n)begin
state<=IDLE;previous_abort<=0;core_abort_pulse<=0;result_available<=0;best_opt<=0;
done<=0;success<=0;fail_code<=0;path_clear<=0;path_cell_valid<=0;path_cell_addr<=0;
route_uses_unknown<=0;route_steps<=0;route_turns<=0;
saved_start_x<=0;saved_start_y<=0;saved_goal_x<=0;saved_goal_y<=0;saved_heading<=0;saved_heading_valid<=0;
base_seen<=0;base_dir_valid<=0;base_prev_x<=0;base_prev_y<=0;base_prev_dir<=0;base_first_dir<=0;
base_steps<=0;base_internal_turns<=0;best_steps<=0;best_turns<=0;best_unknown<=0;
div_value<=0;div_quotient<=0;div_remainder<=0;div_bits<=0;length_limit<=0;
candidate_kind<=0;hinge<=0;hinge_second<=0;score_index<=0;score_prev_x<=0;score_prev_y<=0;score_dir_valid<=0;
score_prev_dir<=0;candidate_steps<=0;candidate_turns<=0;candidate_unknown<=0;
candidate_leg<=0;candidate_cursor_x<=0;candidate_cursor_y<=0;footprint_dx<=0;footprint_dy<=0;map_wait<=0;waypoint_wait<=0;
replay_opt<=0;segment_last<=0;replay_leg<=0;replay_x<=0;replay_y<=0;segment_end_x<=0;segment_end_y<=0;
check_x<=0;check_y<=0;
opt_way_started<=0;opt_way_exhausted<=1;opt_way_index<=0;opt_way_cursor_x<=0;opt_way_cursor_y<=0;
own_way_valid<=0;own_way_last<=0;own_way_x<=0;own_way_y<=0;
for(i=0;i<4;i=i+1)begin candidate_end_x[i]<=0;candidate_end_y[i]<=0;best_end_x[i]<=0;best_end_y[i]<=0;end
end else begin
done<=0;path_clear<=0;path_cell_valid<=0;own_way_valid<=0;core_abort_pulse<=0;previous_abort<=abort;
if(abort)begin if(!previous_abort)finish_failure(4'd5);end
else if(accept_plan)begin
saved_start_x<=start_x;saved_start_y<=start_y;saved_goal_x<=goal_x;saved_goal_y<=goal_y;
saved_heading<=start_heading_tenths[12:0];saved_heading_valid<=start_heading_tenths>=-1800&&start_heading_tenths<=1800;
state<=CORE_START;success<=0;fail_code<=0;result_available<=0;best_opt<=0;
route_uses_unknown<=0;route_steps<=0;route_turns<=0;path_clear<=1;
base_seen<=0;base_dir_valid<=0;base_steps<=0;base_internal_turns<=0;
opt_way_started<=0;opt_way_exhausted<=0;own_way_last<=0;map_wait<=0;waypoint_wait<=0;
end else case(state)
IDLE:if(result_available&&best_opt)begin
if(path_rewind)begin opt_way_started<=0;opt_way_exhausted<=0;opt_way_index<=0;
opt_way_cursor_x<=saved_start_x;opt_way_cursor_y<=saved_start_y;own_way_last<=0;end
else if(waypoint_next&&!opt_way_exhausted)begin
if(!opt_way_started)begin own_way_x<=saved_start_x;own_way_y<=saved_start_y;own_way_valid<=1;opt_way_started<=1;
own_way_last<=saved_start_x==saved_goal_x&&saved_start_y==saved_goal_y;
opt_way_exhausted<=saved_start_x==saved_goal_x&&saved_start_y==saved_goal_y;end
else state<=OPT_WAY_FIND;end end
CORE_START:state<=CORE_WAIT;
CORE_WAIT:begin
if(core_path_valid)begin
if(!base_seen)begin if(core_path_addr!={saved_start_y,saved_start_x})finish_failure(4'd6);
else begin base_seen<=1;base_prev_x<=saved_start_x;base_prev_y<=saved_start_y;end end
else if(base_distance!=1||base_steps>=((1<<(2*GRID_BITS))-1))finish_failure(4'd6);
else begin base_steps<=base_steps+15'd1;
if(!base_dir_valid)begin base_first_dir<=base_direction;base_dir_valid<=1;end
else if(base_direction!=base_prev_dir)base_internal_turns<=base_internal_turns+15'd1;
base_prev_dir<=base_direction;base_prev_x<=core_path_addr[GRID_BITS-1:0];base_prev_y<=core_path_addr[2*GRID_BITS-1:GRID_BITS];end
end else if(core_done)begin
if(!core_success)finish_failure(core_fail);
else if(!base_seen||base_prev_x!=saved_goal_x||base_prev_y!=saved_goal_y)finish_failure(4'd6);
else begin best_steps<=base_steps;best_unknown<=core_route_unknown;best_opt<=0;state<=BASE_SCORE;end end end
BASE_SCORE:begin
best_turns<=base_internal_turns+(base_dir_valid&&needs_initial_turn(base_first_dir,saved_heading,saved_heading_valid));
if(!ENABLE_ROUTE_OPTIMIZATION||!base_dir_valid||
(base_internal_turns==0&&!needs_initial_turn(base_first_dir,saved_heading,saved_heading_valid)))state<=CHOOSE_FINAL;
else begin div_value<=base_steps;div_quotient<=0;div_remainder<=0;div_bits<=2*GRID_BITS+1;state<=DIVIDE;end end
DIVIDE:begin div_value<={div_value[2*GRID_BITS-1:0],1'b0};div_quotient<={div_quotient[2*GRID_BITS-1:0],div_trial>=4'd5};
div_remainder<=div_trial>=4'd5 ? div_trial-4'd5:div_trial[2:0];div_bits<=div_bits-1'b1;if(div_bits==1)state<=LIMIT_COMMIT;end
LIMIT_COMMIT:begin length_limit<={1'b0,base_steps}+(rounded_extra<2?16'd2:{1'b0,rounded_extra});
candidate_kind<=0;hinge<=CLEARANCE_CELLS;state<=ENUM_BUILD;end
ENUM_BUILD:begin
candidate_end_x[3]<=saved_goal_x;candidate_end_y[3]<=saved_goal_y;
case(candidate_kind)
0:begin candidate_end_x[0]<=saved_goal_x;candidate_end_y[0]<=saved_start_y;
candidate_end_x[1]<=saved_goal_x;candidate_end_y[1]<=saved_goal_y;
candidate_end_x[2]<=saved_goal_x;candidate_end_y[2]<=saved_goal_y;end
1:begin candidate_end_x[0]<=saved_start_x;candidate_end_y[0]<=saved_goal_y;
candidate_end_x[1]<=saved_goal_x;candidate_end_y[1]<=saved_goal_y;
candidate_end_x[2]<=saved_goal_x;candidate_end_y[2]<=saved_goal_y;end
2:begin candidate_end_x[0]<=hinge;candidate_end_y[0]<=saved_start_y;
candidate_end_x[1]<=hinge;candidate_end_y[1]<=saved_goal_y;
candidate_end_x[2]<=saved_goal_x;candidate_end_y[2]<=saved_goal_y;end
3:begin candidate_end_x[0]<=saved_start_x;candidate_end_y[0]<=hinge;
candidate_end_x[1]<=saved_goal_x;candidate_end_y[1]<=hinge;
candidate_end_x[2]<=saved_goal_x;candidate_end_y[2]<=saved_goal_y;end
4:begin candidate_end_x[0]<=hinge;candidate_end_y[0]<=saved_start_y;
candidate_end_x[1]<=hinge;candidate_end_y[1]<=hinge_second;
candidate_end_x[2]<=saved_goal_x;candidate_end_y[2]<=hinge_second;end
default:begin candidate_end_x[0]<=saved_start_x;candidate_end_y[0]<=hinge;
candidate_end_x[1]<=hinge_second;candidate_end_y[1]<=hinge;
candidate_end_x[2]<=hinge_second;candidate_end_y[2]<=saved_goal_y;end endcase
score_index<=0;score_prev_x<=saved_start_x;score_prev_y<=saved_start_y;score_dir_valid<=0;
candidate_steps<=0;candidate_turns<=0;candidate_unknown<=0;state<=ENUM_SCORE;end
ENUM_SCORE:begin
if(score_index==4)state<=ENUM_SELECT;
else begin
if(score_distance!=0)begin candidate_steps<=candidate_steps+score_distance;
candidate_turns<=candidate_turns+(!score_dir_valid?
needs_initial_turn(score_direction,saved_heading,saved_heading_valid):score_direction!=score_prev_dir);
score_prev_dir<=score_direction;score_dir_valid<=1;end
score_prev_x<=candidate_end_x[score_index];score_prev_y<=candidate_end_y[score_index];score_index<=score_index+1'b1;end end
ENUM_SELECT:if({1'b0,candidate_steps}<=length_limit&&
(candidate_turns<best_turns||(candidate_turns==best_turns&&candidate_steps<best_steps)))state<=CAND_START;
else state<=ENUM_NEXT;
CAND_START:begin candidate_cursor_x<=saved_start_x;candidate_cursor_y<=saved_start_y;
candidate_leg<=0;candidate_unknown<=0;state<=FOOT_START;end
FOOT_START:if(!footprint_inside(candidate_cursor_x,candidate_cursor_y))state<=ENUM_NEXT;
else begin footprint_dx<=0;footprint_dy<=-CLEARANCE_CELLS;map_wait<=0;state<=FOOT_REQ;end
FOOT_REQ:begin if(map_req_ready)begin map_wait<=0;state<=FOOT_WAIT;end
else if(map_wait>=MAP_TIMEOUT_CLKS-1)finish_failure(4'd4);else map_wait<=map_wait+1'b1;end
FOOT_WAIT:begin
if(map_rsp_valid)begin map_wait<=0;if(sample_unknown)candidate_unknown<=1;
if(!sample_clear)state<=ENUM_NEXT;
else if(footprint_last)state<=CAND_ADVANCE;
else begin if(footprint_dx<$signed({2'b00,extent}))footprint_dx<=footprint_dx+9'sd1;
else begin footprint_dx<=-$signed({2'b00,next_extent});footprint_dy<=footprint_dy+9'sd1;end state<=FOOT_REQ;end end
else if(map_wait>=MAP_TIMEOUT_CLKS-1)finish_failure(4'd4);else map_wait<=map_wait+1'b1;end
CAND_ADVANCE:begin
if(candidate_leg==4)begin best_steps<=candidate_steps;best_turns<=candidate_turns;best_unknown<=candidate_unknown;best_opt<=1;
for(i=0;i<4;i=i+1)begin best_end_x[i]<=candidate_end_x[i];best_end_y[i]<=candidate_end_y[i];end state<=ENUM_NEXT;end
else if(candidate_cursor_x==candidate_end_x[candidate_leg]&&candidate_cursor_y==candidate_end_y[candidate_leg])
candidate_leg<=candidate_leg+1'b1;
else begin candidate_cursor_x<=candidate_step_x;candidate_cursor_y<=candidate_step_y;state<=FOOT_START;end end
ENUM_NEXT:begin
if(best_turns==0)state<=CHOOSE_FINAL;
else case(candidate_kind)
0:begin candidate_kind<=1;state<=ENUM_BUILD;end
1:begin candidate_kind<=2;hinge<=CLEARANCE_CELLS;state<=ENUM_BUILD;end
2:if(hinge>=GRID_SIZE-1-CLEARANCE_CELLS)begin candidate_kind<=3;hinge<=CLEARANCE_CELLS;state<=ENUM_BUILD;end
else begin hinge<=hinge+1'b1;state<=ENUM_BUILD;end
3:if(hinge>=GRID_SIZE-1-CLEARANCE_CELLS)begin
if(best_turns<=2)state<=CHOOSE_FINAL;
else begin candidate_kind<=4;hinge<=CLEARANCE_CELLS;hinge_second<=CLEARANCE_CELLS;state<=ENUM_BUILD;end end
else begin hinge<=hinge+1'b1;state<=ENUM_BUILD;end
default:if(hinge_second<GRID_SIZE-1-CLEARANCE_CELLS)begin hinge_second<=hinge_second+1'b1;state<=ENUM_BUILD;end
else if(hinge<GRID_SIZE-1-CLEARANCE_CELLS)begin hinge<=hinge+1'b1;hinge_second<=CLEARANCE_CELLS;state<=ENUM_BUILD;end
else if(candidate_kind==4)begin candidate_kind<=5;hinge<=CLEARANCE_CELLS;hinge_second<=CLEARANCE_CELLS;state<=ENUM_BUILD;end
else state<=CHOOSE_FINAL;endcase end
CHOOSE_FINAL:begin replay_opt<=best_opt;replay_x<=saved_start_x;replay_y<=saved_start_y;replay_leg<=0;
if(best_opt)state<=EMIT_START;else state<=BASE_REWIND;end
BASE_REWIND:begin waypoint_wait<=0;state<=BASE_FIRST_REQ;end
BASE_FIRST_REQ:if(!core_wp_busy)begin waypoint_wait<=0;state<=BASE_FIRST_WAIT;end
BASE_FIRST_WAIT:begin
if(core_wp_valid)begin if(core_wp_x!=saved_start_x||core_wp_y!=saved_start_y)finish_failure(4'd6);
else begin segment_last<=core_wp_last;state<=EMIT_START;end end
else if(core_done&&!core_success)finish_failure(core_fail);
else if(waypoint_wait>=16'd32770)finish_failure(4'd6);else waypoint_wait<=waypoint_wait+16'd1;end
EMIT_START:begin path_cell_valid<=1;path_cell_addr<={saved_start_y,saved_start_x};
if(replay_opt)state<=OPT_NEXT_LEG;else if(segment_last)state<=FINAL_REWIND;else state<=BASE_NEXT_REQ;end
BASE_NEXT_REQ:if(!core_wp_busy)begin waypoint_wait<=0;state<=BASE_NEXT_WAIT;end
BASE_NEXT_WAIT:begin if(core_wp_valid)start_segment(core_wp_x,core_wp_y,core_wp_last);
else if(core_done&&!core_success)finish_failure(core_fail);
else if(waypoint_wait>=16'd32770)finish_failure(4'd6);else waypoint_wait<=waypoint_wait+16'd1;end
OPT_NEXT_LEG:begin
if(replay_leg==4)begin if(replay_x!=saved_goal_x||replay_y!=saved_goal_y)finish_failure(4'd6);else state<=FINAL_PUBLISH;end
else if(replay_x==best_end_x[replay_leg]&&replay_y==best_end_y[replay_leg])replay_leg<=replay_leg+1'b1;
else start_segment(best_end_x[replay_leg],best_end_y[replay_leg],replay_leg==3);end
SEG_EMIT:begin
if(replay_x==segment_end_x&&replay_y==segment_end_y)state<=SEG_FINISH;
else if(!VERIFY_PATH_BEFORE_PUBLISH)begin
path_cell_valid<=1;path_cell_addr<={replay_step_y,replay_step_x};replay_x<=replay_step_x;replay_y<=replay_step_y;
if(replay_step_x==segment_end_x&&replay_step_y==segment_end_y)state<=SEG_FINISH;end
else begin check_x<=replay_step_x;check_y<=replay_step_y;map_wait<=0;state<=SEG_CHK_REQ;end end
SEG_CHK_REQ:if(map_req_ready)begin map_wait<=0;state<=SEG_CHK_WAIT;end
else if(map_wait>=MAP_TIMEOUT_CLKS-1)finish_failure(4'd4);else map_wait<=map_wait+1'b1;
SEG_CHK_WAIT:if(map_rsp_valid)begin map_wait<=0;
if(map_rsp_data==2'd2&&!check_endpoint)finish_failure(4'd7);
else begin path_cell_valid<=1;path_cell_addr<={check_y,check_x};replay_x<=check_x;replay_y<=check_y;
if(check_x==segment_end_x&&check_y==segment_end_y)state<=SEG_FINISH;else state<=SEG_EMIT;end end
else if(map_wait>=MAP_TIMEOUT_CLKS-1)finish_failure(4'd4);else map_wait<=map_wait+1'b1;
SEG_FINISH:begin
if(replay_opt)begin replay_leg<=replay_leg+1'b1;state<=OPT_NEXT_LEG;end
else if(segment_last)begin if(replay_x!=saved_goal_x||replay_y!=saved_goal_y)finish_failure(4'd6);else state<=FINAL_REWIND;end
else state<=BASE_NEXT_REQ;end
FINAL_REWIND:state<=FINAL_PUBLISH;
FINAL_PUBLISH:begin success<=1;fail_code<=0;done<=1;result_available<=1;
route_uses_unknown<=best_unknown;route_steps<=best_steps;route_turns<=best_turns;
opt_way_started<=0;opt_way_exhausted<=0;opt_way_index<=0;
opt_way_cursor_x<=saved_start_x;opt_way_cursor_y<=saved_start_y;state<=IDLE;end
OPT_WAY_FIND:begin
if(opt_way_index==4)finish_failure(4'd6);
else if(opt_way_cursor_x==best_end_x[opt_way_index]&&opt_way_cursor_y==best_end_y[opt_way_index])
opt_way_index<=opt_way_index+1'b1;
else begin own_way_x<=best_end_x[opt_way_index];own_way_y<=best_end_y[opt_way_index];own_way_valid<=1;
opt_way_cursor_x<=best_end_x[opt_way_index];opt_way_cursor_y<=best_end_y[opt_way_index];
own_way_last<=last_best_leg(opt_way_index);
opt_way_exhausted<=last_best_leg(opt_way_index);
opt_way_index<=opt_way_index+1'b1;state<=IDLE;end end
default:finish_failure(4'd6);
endcase end end
endmodule
