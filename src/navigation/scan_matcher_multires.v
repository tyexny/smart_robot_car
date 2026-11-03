// Bounded coarse-to-fine SE(2) scan-to-map matcher. Millimetre search windows
// are independent of map resolution. Fine search is 10 mm / 0.2 degree.
// Points are predicted map-frame endpoints; map stays frozen until done.
module scan_matcher_multires #(
    parameter integer CELL_MM=20,GRID_SIZE=210, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y),
    parameter integer MAX_POINTS=128,MIN_POINTS=24,
    parameter integer COARSE_STEP_MM=60,COARSE_RADIUS=2,COARSE_YAW_RADIUS=3,
    parameter integer FINE_STEP_MM=10,FINE_YAW_STEP=2,FINE_RADIUS=3,
    parameter integer WIDE_RADIUS=10,WIDE_YAW_RADIUS=6,
    parameter integer MIN_SUPPORT_PERCENT=75,MIN_HIT_PERCENT=40,
    parameter integer IMPROVEMENT_PERCENT=3
)(
    input wire clk,reset_n,cancel,load_valid,
    input wire [6:0] load_index,input wire signed [15:0] load_x_mm,load_y_mm,
    input wire start,search_wide,input wire [7:0] point_count,
    input wire signed [15:0] pivot_x_mm,pivot_y_mm,
    output wire busy,output reg done,matched,ambiguous,
    output reg signed [15:0] delta_x_mm,delta_y_mm,
    output reg signed [12:0] delta_yaw_tenths,
    output reg [15:0] best_score,baseline_score,
    output reg [7:0] hits,support,output reg [3:0] reject_reason,
    output wire map_req,output wire [2*GRID_BITS-1:0] map_addr,
    input wire map_req_ready,map_rsp_valid,input wire [1:0] map_rsp_data
);
    localparam MAX_RADIUS=WIDE_RADIUS>COARSE_RADIUS?WIDE_RADIUS:COARSE_RADIUS;
    localparam MAX_YAW=WIDE_YAW_RADIUS>COARSE_YAW_RADIUS?WIDE_YAW_RADIUS:COARSE_YAW_RADIUS;
    localparam COARSE_COUNT=(2*MAX_RADIUS+1)*(2*MAX_RADIUS+1)*(2*MAX_YAW+1);
    localparam IDLE=0,BEGIN_POINT=1,WAIT_TRANSFORM=2,WAIT_SCORE=3,END_CANDIDATE=4,
        COMPARE=5,FINE_BEGIN=6,FINISH=7,COMPARE_READ=8;
    reg [3:0] state;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [31:0] points[0:MAX_POINTS-1];
    reg [31:0] point_data;
    reg [6:0] point_index;
    reg [7:0] count;
    reg signed [15:0] pivot_x,pivot_y,tx,ty,bx,by,fine_x,fine_y;
    reg signed [12:0] yaw,byaw,fine_yaw;
    reg signed [4:0] ix,iy,ia,cmp_x,cmp_y,cmp_a;
    reg fine;
    reg wide;
    wire [4:0] radius=wide?WIDE_RADIUS:COARSE_RADIUS;
    wire [4:0] yaw_radius=wide?WIDE_YAW_RADIUS:COARSE_YAW_RADIUS;
    wire [15:0] coarse_total=(2*radius+1)*(2*radius+1)*(2*yaw_radius+1);
    reg [15:0] accumulator,peak,zero_score;
    reg [7:0] hit_acc,sup_acc,peak_hits,peak_sup,zero_hits,zero_sup;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [15:0] coarse_scores[0:COARSE_COUNT-1];
    reg [15:0] coarse_compare_score;
    reg [15:0] candidate_index,compare_index;
    reg signed [4:0] coarse_best_x,coarse_best_y,coarse_best_a;
    reg transform_start,score_start;
    wire transform_busy,transform_done,score_busy,score_done,point_hit,point_support;
    wire signed [31:0] wx,wy;
    wire [8:0] point_score;
    wire [15:0] penalty=(tx<0?-tx:tx)+(ty<0?-ty:ty)+(yaw<0?-yaw:yaw)*2;
    wire [15:0] best_penalty=(bx<0?-bx:bx)+(by<0?-by:by)+(byaw<0?-byaw:byaw)*2;
    wire better=accumulator>peak || (accumulator==peak && penalty<best_penalty);
    wire nonzero=bx!=0||by!=0||byaw!=0;
    wire correction_ok=nonzero&&!ambiguous&&peak_sup*100>=count*MIN_SUPPORT_PERCENT&&
        peak_hits*100>=count*MIN_HIT_PERCENT&&peak>=zero_score+32&&
        peak*100>=zero_score*(100+IMPROVEMENT_PERCENT);
    wire zero_ok=!ambiguous&&zero_sup*100>=count*85&&zero_hits*100>=count*30;
    wire separated=(cmp_x-coarse_best_x>1)||(coarse_best_x-cmp_x>1)||
        (cmp_y-coarse_best_y>1)||(coarse_best_y-cmp_y>1)||
        (cmp_a-coarse_best_a>1)||(coarse_best_a-cmp_a>1);
    assign busy=state!=IDLE||score_busy||transform_busy;
    always @(posedge clk)begin
        if(load_valid&&state==IDLE)points[load_index]<={load_x_mm,load_y_mm};
        point_data<=points[point_index];
        if(state==END_CANDIDATE&&!fine)coarse_scores[candidate_index]<=accumulator;
        coarse_compare_score<=coarse_scores[compare_index];
    end
    slam_rigid_transform transform(.clk(clk),.reset_n(reset_n),.start(transform_start),.cancel(cancel),
        .point_x(point_data[31:16]),.point_y(point_data[15:0]),.pivot_x(pivot_x),.pivot_y(pivot_y),
        .translation_x(tx),.translation_y(ty),.angle_tenths(yaw),
        .busy(transform_busy),.done(transform_done),.result_x(wx),.result_y(wy));
    slam_endpoint_score #(.CELL_MM(CELL_MM),.GRID_SIZE(GRID_SIZE),.GRID_X(GRID_X),.GRID_Y(GRID_Y),.GRID_BITS(GRID_BITS)) scorer(
        .clk(clk),.reset_n(reset_n),.start(score_start),.cancel(cancel),.x_mm(wx),.y_mm(wy),
        .busy(score_busy),.done(score_done),.score(point_score),.hit(point_hit),.supported(point_support),
        .map_req(map_req),.map_addr(map_addr),.map_req_ready(map_req_ready),
        .map_rsp_valid(map_rsp_valid),.map_rsp_data(map_rsp_data));
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;done<=0;matched<=0;ambiguous<=0;delta_x_mm<=0;delta_y_mm<=0;delta_yaw_tenths<=0;
            best_score<=0;baseline_score<=0;hits<=0;support<=0;reject_reason<=0;
            point_index<=0;count<=0;pivot_x<=0;pivot_y<=0;tx<=0;ty<=0;yaw<=0;bx<=0;by<=0;byaw<=0;
            fine_x<=0;fine_y<=0;fine_yaw<=0;fine<=0;wide<=0;ix<=0;iy<=0;ia<=0;cmp_x<=0;cmp_y<=0;cmp_a<=0;
            accumulator<=0;peak<=0;zero_score<=0;hit_acc<=0;sup_acc<=0;peak_hits<=0;peak_sup<=0;
            zero_hits<=0;zero_sup<=0;candidate_index<=0;compare_index<=0;coarse_best_x<=0;coarse_best_y<=0;coarse_best_a<=0;
            transform_start<=0;score_start<=0;
        end else begin
            done<=0;transform_start<=0;score_start<=0;
            if(cancel)begin state<=IDLE;matched<=0;end
            else case(state)
                IDLE:if(start&&!score_busy)begin
                    matched<=0;ambiguous<=0;delta_x_mm<=0;delta_y_mm<=0;delta_yaw_tenths<=0;reject_reason<=0;
                    if(point_count<MIN_POINTS)begin done<=1;reject_reason<=2;end
                    else begin
                        count<=point_count;pivot_x<=pivot_x_mm;pivot_y<=pivot_y_mm;point_index<=0;
                        wide<=search_wide;
                        ix<=search_wide?-WIDE_RADIUS:-COARSE_RADIUS;iy<=search_wide?-WIDE_RADIUS:-COARSE_RADIUS;
                        ia<=search_wide?-WIDE_YAW_RADIUS:-COARSE_YAW_RADIUS;
                        tx<=-(search_wide?WIDE_RADIUS:COARSE_RADIUS)*COARSE_STEP_MM;
                        ty<=-(search_wide?WIDE_RADIUS:COARSE_RADIUS)*COARSE_STEP_MM;
                        yaw<=-(search_wide?WIDE_YAW_RADIUS:COARSE_YAW_RADIUS)*10;
                        bx<=0;by<=0;byaw<=0;peak<=0;zero_score<=0;zero_hits<=0;zero_sup<=0;
                        accumulator<=0;hit_acc<=0;sup_acc<=0;fine<=0;candidate_index<=0;
                        state<=BEGIN_POINT;
                    end
                end
                BEGIN_POINT:begin transform_start<=1;state<=WAIT_TRANSFORM;end
                WAIT_TRANSFORM:if(transform_done)begin score_start<=1;state<=WAIT_SCORE;end
                WAIT_SCORE:if(score_done)begin
                    accumulator<=accumulator+point_score;hit_acc<=hit_acc+point_hit;sup_acc<=sup_acc+point_support;
                    if(point_index+1>=count)state<=END_CANDIDATE;
                    else begin point_index<=point_index+1'b1;state<=BEGIN_POINT;end
                end
                END_CANDIDATE:begin
                    if(tx==0&&ty==0&&yaw==0)begin zero_score<=accumulator;zero_hits<=hit_acc;zero_sup<=sup_acc;end
                    if(better)begin
                        peak<=accumulator;bx<=tx;by<=ty;byaw<=yaw;peak_hits<=hit_acc;peak_sup<=sup_acc;
                        if(!fine)begin coarse_best_x<=ix;coarse_best_y<=iy;coarse_best_a<=ia;end
                    end
                    accumulator<=0;hit_acc<=0;sup_acc<=0;point_index<=0;candidate_index<=candidate_index+1'b1;
                    if(ix<(fine?FINE_RADIUS:$signed({1'b0,radius})))begin
                        ix<=ix+1'b1;tx<=tx+(fine?FINE_STEP_MM:COARSE_STEP_MM);state<=BEGIN_POINT;
                    end else begin
                        ix<=fine?-FINE_RADIUS:-$signed({1'b0,radius});
                        tx<=fine?fine_x-FINE_RADIUS*FINE_STEP_MM:-$signed({1'b0,radius})*COARSE_STEP_MM;
                        if(iy<(fine?FINE_RADIUS:$signed({1'b0,radius})))begin
                            iy<=iy+1'b1;ty<=ty+(fine?FINE_STEP_MM:COARSE_STEP_MM);state<=BEGIN_POINT;
                        end else begin
                            iy<=fine?-FINE_RADIUS:-$signed({1'b0,radius});
                            ty<=fine?fine_y-FINE_RADIUS*FINE_STEP_MM:-$signed({1'b0,radius})*COARSE_STEP_MM;
                            if(ia<(fine?FINE_RADIUS:$signed({1'b0,yaw_radius})))begin
                                ia<=ia+1'b1;yaw<=yaw+(fine?FINE_YAW_STEP:10);state<=BEGIN_POINT;
                            end else if(fine)state<=FINISH;
                            else begin
                                compare_index<=0;cmp_x<=-$signed({1'b0,radius});cmp_y<=-$signed({1'b0,radius});cmp_a<=-$signed({1'b0,yaw_radius});state<=COMPARE_READ;
                            end
                        end
                    end
                end
                COMPARE_READ:state<=COMPARE;
                COMPARE:begin
                    if(separated&&coarse_compare_score+32>=peak&&coarse_compare_score!=0)ambiguous<=1;
                    if(compare_index==coarse_total-1)state<=FINE_BEGIN;
                    else begin
                        compare_index<=compare_index+1'b1;
                        state<=COMPARE_READ;
                        if(cmp_x<$signed({1'b0,radius}))cmp_x<=cmp_x+1'b1;
                        else begin cmp_x<=-$signed({1'b0,radius});
                            if(cmp_y<$signed({1'b0,radius}))cmp_y<=cmp_y+1'b1;
                            else begin cmp_y<=-$signed({1'b0,radius});cmp_a<=cmp_a+1'b1;end
                        end
                    end
                end
                FINE_BEGIN:begin
                    fine<=1;fine_x<=bx;fine_y<=by;fine_yaw<=byaw;
                    ix<=-FINE_RADIUS;iy<=-FINE_RADIUS;ia<=-FINE_RADIUS;
                    tx<=bx-FINE_RADIUS*FINE_STEP_MM;ty<=by-FINE_RADIUS*FINE_STEP_MM;yaw<=byaw-FINE_RADIUS*FINE_YAW_STEP;
                    state<=BEGIN_POINT;
                end
                FINISH:begin
                    matched<=correction_ok||zero_ok;best_score<=correction_ok?peak:zero_score;baseline_score<=zero_score;
                    hits<=correction_ok?peak_hits:zero_hits;support<=correction_ok?peak_sup:zero_sup;
                    reject_reason<=correction_ok||zero_ok?0:ambiguous?8:7;
                    if(correction_ok)begin delta_x_mm<=bx;delta_y_mm<=by;delta_yaw_tenths<=byaw;end
                    done<=1;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
