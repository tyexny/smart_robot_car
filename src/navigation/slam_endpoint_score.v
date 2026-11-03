// Smooth scan-to-grid score. Occupied cells are sampled over a 3x3 stencil;
// squared distance to their centres retains sub-cell translation information.
// One outstanding read, with cancellation drain. FREE/UNKNOWN never attract.
module slam_endpoint_score #(
    parameter integer CELL_MM=20, GRID_SIZE=210, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y)
)(
    input wire clk,reset_n,start,cancel,
    input wire signed [31:0] x_mm,y_mm,
    output wire busy,output reg done,
    output reg [8:0] score,output reg hit,supported,
    output reg map_req,output reg [2*GRID_BITS-1:0] map_addr,
    input wire map_req_ready,map_rsp_valid,input wire [1:0] map_rsp_data
);
    localparam IDLE=0,QUERY=1,WAIT_MAP=2,NEXT=3,DRAIN=4,
        OFFSET=5,DIVIDE=6,CELL=7,FINISH=8;
    localparam MAP_HALF_X_MM=GRID_X*CELL_MM/2, MAP_HALF_Y_MM=GRID_Y*CELL_MM/2;
    localparam GRID_RECIPROCAL_SHIFT=CELL_MM==40?21:20;
    localparam [15:0] GRID_RECIPROCAL_MULTIPLIER=CELL_MM==30?16'd34953:16'd52429;
    localparam OFFSET_BITS=$clog2((GRID_X>GRID_Y?GRID_X:GRID_Y)*CELL_MM+1);
    localparam DELTA_BITS=$clog2(3*CELL_MM)+1;
    reg [3:0] state;
    reg signed [31:0] px,py,cx,cy,qx,qy;
    reg signed [DELTA_BITS-1:0] dx,dy;
    reg [OFFSET_BITS-1:0] offset_x,offset_y;
    reg [OFFSET_BITS+15:0] cell_product_x,cell_product_y;
    reg signed [2:0] nx,ny;
    reg [2*DELTA_BITS-1:0] distance2;
    wire signed [31:0] query_x=cx+nx,query_y=cy+ny;
    localparam integer SCORE_RECIPROCAL_Q16=(128*65536+CELL_MM*CELL_MM/2)/(CELL_MM*CELL_MM);
    wire [63:0] weighted_distance=distance2*SCORE_RECIPROCAL_Q16;
    wire [31:0] weight=distance2>=2*CELL_MM*CELL_MM ? 0 :
        256-(weighted_distance>>16);
    assign busy=state!=IDLE;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;done<=0;score<=0;hit<=0;supported<=0;map_req<=0;map_addr<=0;
            px<=0;py<=0;cx<=0;cy<=0;qx<=0;qy<=0;dx<=0;dy<=0;distance2<=0;nx<=0;ny<=0;
            offset_x<=0;offset_y<=0;cell_product_x<=0;cell_product_y<=0;
        end else begin
            done<=0;
            if(cancel) begin
                map_req<=0;
                if((state==WAIT_MAP || state==DRAIN || (map_req&&map_req_ready))&&!map_rsp_valid) state<=DRAIN;
                else state<=IDLE;
            end else case(state)
                IDLE:if(start) begin
                    score<=0;hit<=0;supported<=0;px<=x_mm;py<=y_mm;
                    nx<=-1;ny<=-1;state<=OFFSET;
                end
                // Shift into a nonnegative map coordinate before division.
                // The 20 mm reciprocal is exact for unsigned 16-bit inputs;
                // separate stages avoid a signed divide on the input path.
                OFFSET:if(px<-MAP_HALF_X_MM||px>=MAP_HALF_X_MM||
                          py<-MAP_HALF_Y_MM||py>=MAP_HALF_Y_MM)state<=FINISH;
                    else begin
                        offset_x<=px+MAP_HALF_X_MM;offset_y<=py+MAP_HALF_Y_MM;state<=DIVIDE;
                    end
                DIVIDE:begin
                    if((CELL_MM==20||CELL_MM==30||CELL_MM==40)&&OFFSET_BITS<=16)begin
                        cell_product_x<=offset_x*GRID_RECIPROCAL_MULTIPLIER;cell_product_y<=offset_y*GRID_RECIPROCAL_MULTIPLIER;
                    end else begin cx<=offset_x/CELL_MM;cy<=offset_y/CELL_MM;end
                    state<=CELL;
                end
                CELL:begin
                    if((CELL_MM==20||CELL_MM==30||CELL_MM==40)&&OFFSET_BITS<=16)begin cx<=cell_product_x>>GRID_RECIPROCAL_SHIFT;cy<=cell_product_y>>GRID_RECIPROCAL_SHIFT;end
                    state<=QUERY;
                end
                QUERY:begin
                    if(cx<0||cx>=GRID_X||cy<0||cy>=GRID_Y||query_x<0||query_x>=GRID_X||query_y<0||query_y>=GRID_Y)
                        state<=NEXT;
                    else if(!map_req) begin
                        map_addr<={query_y[GRID_BITS-1:0],query_x[GRID_BITS-1:0]};map_req<=1;
                        qx<=query_x;qy<=query_y;
                        dx<=px-(query_x*CELL_MM-MAP_HALF_X_MM+CELL_MM/2);
                        dy<=py-(query_y*CELL_MM-MAP_HALF_Y_MM+CELL_MM/2);
                    end else if(map_req_ready) begin
                        map_req<=0;distance2<=dx*dx+dy*dy;state<=WAIT_MAP;
                    end
                end
                WAIT_MAP:if(map_rsp_valid) begin
                    if(map_rsp_data==2) begin
                        if(weight>score)score<=weight[8:0];
                        if(weight!=0)supported<=1;
                        if(nx==0&&ny==0)hit<=1;
                    end
                    state<=NEXT;
                end
                NEXT:if(nx==1&&ny==1)begin done<=1;state<=IDLE;end
                    else begin
                        if(nx==1)begin nx<=-1;ny<=ny+1'b1;end else nx<=nx+1'b1;
                        state<=QUERY;
                    end
                DRAIN:if(map_rsp_valid)state<=IDLE;
                FINISH:begin done<=1;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
