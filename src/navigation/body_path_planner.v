`timescale 1ns/1ps
// Optional turn/forward navigation uses directional body and turn-clearance masks.
// Fixed-heading translation remains available to legacy callers. One frozen map is
// shared by HDMI publication and the motor waypoints. Endpoint checks default on.
// Compact (GRID_SIZE squared) RAMs avoid the padded 256x256 planner storage.
module body_path_planner #(
    parameter integer GRID_SIZE=140, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y), CELL_MM=30,
    parameter integer FRONT_MM=140, REAR_MM=260, HALF_WIDTH_MM=140,
    parameter integer MARGIN_MM=40, SOFT_CELLS=15, TURN_COST=6,
    parameter integer ALLOW_UNKNOWN=1, MAP_TIMEOUT_CLKS=500000,
    parameter integer TURN_IN_PLACE=0, TRACK_YAW_TENTHS=0,
    parameter integer CHECK_START_CLEARANCE=1, CHECK_GOAL_CLEARANCE=1
)(
    input wire clk,reset_n,start_plan,abort,
    input wire [GRID_BITS-1:0] start_x,start_y,goal_x,goal_y,
    input wire signed [31:0] start_heading_tenths,
    output wire map_req, output wire [2*GRID_BITS-1:0] map_addr,
    input wire map_req_ready,map_rsp_valid,input wire [1:0] map_rsp_data,
    output wire busy,output reg done,success,output reg [3:0] fail_code,
    input wire path_rewind,waypoint_next,
    output wire waypoint_busy,output reg waypoint_valid,
    output reg [GRID_BITS-1:0] waypoint_x,waypoint_y,output reg waypoint_last,
    output reg path_clear,path_cell_valid,output reg [2*GRID_BITS-1:0] path_cell_addr,
    output reg route_uses_unknown,output reg signed [12:0] body_heading,
    output reg [2*GRID_BITS:0] route_steps,route_turns
);
    localparam integer N=GRID_X*GRID_Y, AB=$clog2(N), LB=GRID_BITS+3;
    localparam integer F=FRONT_MM+MARGIN_MM,R=REAR_MM+MARGIN_MM,W=HALF_WIDTH_MM+MARGIN_MM;
    localparam [5:0] IDLE=0,SNAP_REQ=1,SNAP_WAIT=2,BOUNDS=3,
        ROT_READ=4,ROT_EVAL=5,PREP=6,FOOT_READ=7,FOOT_EVAL=8,FOOT_WRITE=9,
        END_READ=10,END_EVAL=11,SEED=12,SCAN_READ=13,SCAN_EVAL=14,
        CLOSE=15,NEIGH_READ=16,NEIGH_EVAL=17,NEIGH_WRITE=18,ADVANCE=19,
        TRACE_READ=20,TRACE_EVAL=21,WP_READ=22,WP_EVAL=23,PREP_READ=24,DRAIN=25,
        BOUND_INDEX=26,BOUND_COMMIT=27,NEIGH_COST=28;
    reg [5:0] state;
    reg [AB-1:0] rd_addr,wr_addr;
    reg node_we,attr_we;
    reg [17:0] node_wdata;
    reg [7:0] attr_wdata;
    // node: parent[17:16], closed[15], distance[14:0].
    // attr: clearance[7:4], blocked[3], unknown-footprint[2], raw[1:0].
    localparam integer NODE_BANKS=(N+1023)/1024, ATTR_BANKS=(N+2047)/2048;
    wire [17:0] node_bank_q[0:NODE_BANKS-1];
    wire [7:0] attr_bank_q[0:ATTR_BANKS-1];
    reg [AB-1:0] read_index;
    wire [17:0] node_q=node_bank_q[read_index/1024];
    wire [7:0] attr_q=attr_bank_q[read_index/2048];
    always @(posedge clk) begin
        read_index<=rd_addr;
    end
    genvar bank;
    generate for(bank=0;bank<NODE_BANKS;bank=bank+1)begin: nb
        reg [17:0] mem[0:1023];reg [17:0] q;
        assign node_bank_q[bank]=q;
        always @(posedge clk)begin
            q<=mem[rd_addr%1024];
            if(node_we&&wr_addr/1024==bank)mem[wr_addr%1024]<=node_wdata;
        end
    end
    for(bank=0;bank<ATTR_BANKS;bank=bank+1)begin: ab
        reg [7:0] mem[0:2047];reg [7:0] q;
        assign attr_bank_q[bank]=q;
        always @(posedge clk)begin
            q<=mem[rd_addr%2048];
            if(attr_we&&wr_addr/2048==bank)mem[wr_addr%2048]<=attr_wdata;
        end
    end endgenerate
    function [AB-1:0] index;
        input [GRID_BITS-1:0] x,y;
        begin index=y*GRID_X+x;end
    endfunction
    function integer ceil_root;
        input integer v; integer k;
        begin ceil_root=0;for(k=0;k<1024;k=k+1)if(k*k<v)ceil_root=k+1;end
    endfunction
    // Bounds for a rotation of at most 45 degrees to the nearest cardinal.
    // Five-degree bins use cos(lower), sin(upper), plus the interior extremum.
    // This deliberately over-approximates the swept rectangle at the start.
    function integer sin5;
        input integer b;
        begin case(b)
            0:sin5=0;1:sin5=1429;2:sin5=2846;3:sin5=4241;
            4:sin5=5604;5:sin5=6925;6:sin5=8192;7:sin5=9398;
            8:sin5=10532;default:sin5=11586;
        endcase end
    endfunction
    function integer cos5lower;
        input integer b;
        begin case(b)
            0,1:cos5lower=16384;2:cos5lower=16322;3:cos5lower=16136;
            4:cos5lower=15826;5:cos5lower=15396;6:cos5lower=14849;
            7:cos5lower=14189;8:cos5lower=13422;default:cos5lower=12552;
        endcase end
    endfunction
    function integer max_i;input integer a,b;begin max_i=a>b?a:b;end endfunction
    // Constant function: every multiplication/division below is evaluated at
    // elaboration, NOT between 50 MHz registers. Runtime uses an 80-word mux.
    function [4*LB-1:0] bounds_word;
        input integer bin,ori,negative;
        integer cs,sn,fx,rx,p,n,lx,hx,ly,hy;
        reg signed [LB-1:0] xl,xh,yl,yh;
        begin
            cs=cos5lower(bin);sn=sin5(bin);
            fx=max_i(F,(F*cs+W*sn+16383)/16384);
            rx=max_i(R,(R*cs+W*sn+16383)/16384);
            if(bin>=9)fx=max_i(fx,ceil_root(F*F+W*W));
            if(bin>=6)rx=max_i(rx,ceil_root(R*R+W*W));
            p=max_i(W,((negative?R:F)*sn+W*cs+16383)/16384);
            n=max_i(W,((negative?F:R)*sn+W*cs+16383)/16384);
            case(ori)
                0:begin lx=rx;hx=fx;ly=n;hy=p;end
                1:begin lx=fx;hx=rx;ly=p;hy=n;end
                2:begin lx=p;hx=n;ly=rx;hy=fx;end
                default:begin lx=n;hx=p;ly=fx;hy=rx;end
            endcase
            xl=-(lx-CELL_MM/2+CELL_MM-1)/CELL_MM;xh=(hx+CELL_MM/2)/CELL_MM;
            yl=-(ly-CELL_MM/2+CELL_MM-1)/CELL_MM;yh=(hy+CELL_MM/2)/CELL_MM;
            bounds_word={xl,xh,yl,yh};
        end
    endfunction
    wire [4*LB-1:0] bound_table[0:79];
    genvar bo,bs,bb;
    generate for(bo=0;bo<4;bo=bo+1)begin: bt_o
        for(bs=0;bs<2;bs=bs+1)begin: bt_s
            for(bb=0;bb<10;bb=bb+1)begin: bt_b
                assign bound_table[bo*20+bs*10+bb]=bounds_word(bb,bo,bs);
            end
        end
    end endgenerate
    reg [3:0] rotation_bin;
    reg [6:0] rotation_index,base_index;
    reg [GRID_BITS-1:0] sx,sy,gx,gy,cx,cy,scan_x,scan_y,tx,ty;
    wire endpoint_cell=(cx==sx&&cy==sy)||(cx==gx&&cy==gy);
    // Exempt only the selected endpoint node. Raw map cells and every
    // intermediate node retain their ordinary body/obstacle checks. If start
    // equals goal, either enabled check still applies to that shared node.
    wire endpoint_exempt=endpoint_cell&&
        (!CHECK_START_CLEARANCE||cx!=sx||cy!=sy)&&
        (!CHECK_GOAL_CLEARANCE||cx!=gx||cy!=gy);
    reg [AB-1:0] cursor,scan_index,current_index,neighbor_index;
    reg [1:0] orientation,neighbor_dir,current_parent,trace_dir,way_dir;
    reg signed [12:0] initial_delta;
    reg signed [GRID_BITS+2:0] lo_x,hi_x,lo_y,hi_y,rot_lo_x,rot_hi_x,rot_lo_y,rot_hi_y,dx,dy;
    reg [7:0] raw_center;
    reg [3:0] clearance;
    reg footprint_unknown,blocked,result_available,way_first,way_exhausted,way_walked,trace_seen;
    reg [31:0] wait_count;
    reg [15:0] bucket,next_bucket,current_distance,proposed_distance;
    reg [14:0] neighbor_distance;
    reg neighbor_eligible;
    reg [17:0] current_node;
    reg [7:0] current_attr,neighbor_attr;
    reg [2*GRID_BITS:0] trace_count,way_count;
    localparam integer TURN_RADIUS=ceil_root((F>R?F:R)*(F>R?F:R)+W*W);
    localparam integer TURN_CELLS=(TURN_RADIUS+CELL_MM/2+CELL_MM-1)/CELL_MM;
    localparam integer COVER_SIDE=2*TURN_CELLS+1;
    localparam integer TRACK_BIN=(TRACK_YAW_TENTHS+49)/50;
    // Constant cell masks: four directional footprints (including tracking
    // yaw) and a full rotation disk for the rear monitor's swept envelope.
    // No runtime square root, multiplication, or trigonometric datapath.
    function [4:0] coverage_word;
        input integer x,y;
        integer o,ax,ay,lx,hx,ly,hy;
        reg [4*LB-1:0] a,b;
        begin
            coverage_word=0;
            for(o=0;o<4;o=o+1)begin
                a=bounds_word(TRACK_BIN,o,0);b=bounds_word(TRACK_BIN,o,1);
                lx=$signed(a[4*LB-1-:LB])<$signed(b[4*LB-1-:LB])?$signed(a[4*LB-1-:LB]):$signed(b[4*LB-1-:LB]);
                hx=$signed(a[3*LB-1-:LB])>$signed(b[3*LB-1-:LB])?$signed(a[3*LB-1-:LB]):$signed(b[3*LB-1-:LB]);
                ly=$signed(a[2*LB-1-:LB])<$signed(b[2*LB-1-:LB])?$signed(a[2*LB-1-:LB]):$signed(b[2*LB-1-:LB]);
                hy=$signed(a[LB-1:0])>$signed(b[LB-1:0])?$signed(a[LB-1:0]):$signed(b[LB-1:0]);
                coverage_word[o]=x>=lx&&x<=hx&&y>=ly&&y<=hy;
            end
            ax=(x<0?-x:x)*CELL_MM-CELL_MM/2;if(ax<0)ax=0;
            ay=(y<0?-y:y)*CELL_MM-CELL_MM/2;if(ay<0)ay=0;
            coverage_word[4]=ax*ax+ay*ay<=TURN_RADIUS*TURN_RADIUS;
        end
    endfunction
    wire [4:0] coverage_table[0:COVER_SIDE*COVER_SIDE-1];
    genvar cov_i;
    generate for(cov_i=0;cov_i<COVER_SIDE*COVER_SIDE;cov_i=cov_i+1)begin: cv
        assign coverage_table[cov_i]=coverage_word(cov_i%COVER_SIDE-TURN_CELLS,cov_i/COVER_SIDE-TURN_CELLS);
    end endgenerate
    wire [GRID_BITS+3:0] cover_x=dx+TURN_CELLS,cover_y=dy+TURN_CELLS;
    wire [4:0] sample_coverage=dx>=-TURN_CELLS&&dx<=TURN_CELLS&&dy>=-TURN_CELLS&&dy<=TURN_CELLS ?
        coverage_table[cover_y*COVER_SIDE+cover_x]:5'd0;
    reg [3:0] directional_blocked,directional_unknown;
    reg rotation_blocked;
    reg [8:0] current_motion_mask;
    wire [8:0] motion_q;
    generate if(TURN_IN_PLACE)begin: spin_clearance
        localparam integer BANKS=(N+2047)/2048;
        wire [8:0] bank_q[0:BANKS-1];
        assign motion_q=bank_q[read_index/2048];
        for(bank=0;bank<BANKS;bank=bank+1)begin: mb
            reg [8:0] mem[0:2047];reg [8:0] q;
            assign bank_q[bank]=q;
            always @(posedge clk)begin
                q<=mem[rd_addr%2048];
                if(state==FOOT_WRITE&&cursor/2048==bank)
                    mem[cursor%2048]<={endpoint_exempt?1'b0:rotation_blocked,directional_unknown,
                        endpoint_exempt?4'b0:directional_blocked};
            end
        end
    end else begin: no_spin_clearance
        assign motion_q=0;
    end endgenerate
    wire [8:0] rotation_magnitude=initial_delta[12]?-initial_delta:initial_delta;
    wire signed [GRID_BITS+3:0] sample_x=$signed({1'b0,cx})+dx;
    wire signed [GRID_BITS+3:0] sample_y=$signed({1'b0,cy})+dy;
    wire sample_inside=sample_x>=0&&sample_x<GRID_X&&sample_y>=0&&sample_y<GRID_Y;
    wire signed [GRID_BITS+3:0] ex=dx<lo_x?lo_x-dx:dx>hi_x?dx-hi_x:0;
    wire signed [GRID_BITS+3:0] ey=dy<lo_y?lo_y-dy:dy>hi_y?dy-hi_y:0;
    wire [GRID_BITS+3:0] shell=ex>ey?ex:ey;
    wire sample_obstacle=!sample_inside||attr_q[1:0]==2||attr_q[1:0]==3||(!ALLOW_UNKNOWN&&attr_q[1:0]==0);
    wire [GRID_BITS-1:0] nx=neighbor_dir==0?scan_x-1'b1:neighbor_dir==1?scan_x+1'b1:scan_x;
    wire [GRID_BITS-1:0] ny=neighbor_dir==2?scan_y-1'b1:neighbor_dir==3?scan_y+1'b1:scan_y;
    wire neighbor_inside=neighbor_dir==0?scan_x>0:neighbor_dir==1?scan_x<GRID_X-1:
                               neighbor_dir==2?scan_y>0:scan_y<GRID_Y-1;
    wire [16:0] candidate_cost={1'b0,current_distance}+17'd1+
        (SOFT_CELLS-attr_q[7:4])+ (attr_q[2]?17'd8:17'd0)+
        ((current_distance!=0&&current_parent!=neighbor_dir)?TURN_COST:0);
    wire [GRID_BITS-1:0] step_x=node_q[17:16]==0?tx+1'b1:node_q[17:16]==1?tx-1'b1:tx;
    wire [GRID_BITS-1:0] step_y=node_q[17:16]==2?ty+1'b1:node_q[17:16]==3?ty-1'b1:ty;
    wire trace_step_ok=node_q[17:16]==0?tx<GRID_X-1:node_q[17:16]==1?tx>0:
                       node_q[17:16]==2?ty<GRID_Y-1:ty>0;
    assign map_req=reset_n&&!abort&&state==SNAP_REQ;
    assign map_addr={cy,cx};
    assign waypoint_busy=state==WP_READ||state==WP_EVAL;
    assign busy=state!=IDLE&&!waypoint_busy;
    // Explicit RAM controls, no array resets. Addresses settle in READ states.
    always @* begin
        rd_addr=cursor;wr_addr=cursor;node_we=0;attr_we=0;
        node_wdata=18'h07fff;attr_wdata=0;
        case(state)
            SNAP_WAIT:if(map_rsp_valid)begin node_we=1;attr_we=1;attr_wdata={6'd0,map_rsp_data};end
            ROT_READ,ROT_EVAL,FOOT_READ,FOOT_EVAL:
                rd_addr=sample_inside?index(sample_x[GRID_BITS-1:0],sample_y[GRID_BITS-1:0]):{AB{1'b0}};
            PREP:rd_addr=cursor;
            FOOT_WRITE:begin attr_we=1;attr_wdata={clearance,
                endpoint_exempt?1'b0:(TURN_IN_PLACE?(&directional_blocked):blocked),
                TURN_IN_PLACE?(|directional_unknown):footprint_unknown,raw_center[1:0]};end
            END_READ,END_EVAL:rd_addr=index(cx,cy);
            SEED:begin node_we=1;wr_addr=index(gx,gy);node_wdata=0;end
            SCAN_READ,SCAN_EVAL:rd_addr=scan_index;
            CLOSE:begin node_we=1;wr_addr=current_index;node_wdata={current_parent,1'b1,current_distance[14:0]};end
            NEIGH_READ,NEIGH_EVAL:rd_addr=neighbor_inside?index(nx,ny):{AB{1'b0}};
            NEIGH_WRITE:begin node_we=1;wr_addr=neighbor_index;node_wdata={neighbor_dir,1'b0,proposed_distance[14:0]};end
            TRACE_READ,TRACE_EVAL,WP_READ,WP_EVAL:rd_addr=index(tx,ty);
            default:begin end
        endcase
    end
    task fail;
        input [3:0] code;
        begin state<=IDLE;done<=1;success<=0;fail_code<=code;result_available<=0;
            waypoint_valid<=0;way_exhausted<=1;path_clear<=1;route_steps<=0;route_turns<=0;end
    endtask
    task advance_scan;
        begin state<=ADVANCE;end
    endtask
    task emit_way;
        input last;
        begin waypoint_valid<=1;waypoint_x<=tx;waypoint_y<=ty;waypoint_last<=last;
            way_exhausted<=last;state<=IDLE;end
    endtask
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin
            state<=IDLE;done<=0;success<=0;fail_code<=0;path_clear<=0;path_cell_valid<=0;path_cell_addr<=0;
            waypoint_valid<=0;waypoint_x<=0;waypoint_y<=0;waypoint_last<=0;route_uses_unknown<=0;
            body_heading<=0;route_steps<=0;route_turns<=0;result_available<=0;
            sx<=0;sy<=0;gx<=0;gy<=0;cx<=0;cy<=0;cursor<=0;scan_x<=0;scan_y<=0;scan_index<=0;
            tx<=0;ty<=0;orientation<=0;initial_delta<=0;lo_x<=0;hi_x<=0;lo_y<=0;hi_y<=0;
            rotation_bin<=0;rotation_index<=0;base_index<=0;
            rot_lo_x<=0;rot_hi_x<=0;rot_lo_y<=0;rot_hi_y<=0;dx<=0;dy<=0;
            wait_count<=0;raw_center<=0;clearance<=0;footprint_unknown<=0;blocked<=0;
            directional_blocked<=0;directional_unknown<=0;rotation_blocked<=0;current_motion_mask<=0;
            bucket<=0;next_bucket<=65535;current_distance<=0;current_parent<=0;current_index<=0;
            neighbor_distance<=0;neighbor_eligible<=0;
            current_node<=0;current_attr<=0;neighbor_dir<=0;neighbor_index<=0;neighbor_attr<=0;proposed_distance<=0;
            trace_dir<=0;trace_seen<=0;trace_count<=0;way_dir<=0;way_first<=1;way_exhausted<=1;way_walked<=0;way_count<=0;
        end else begin
            done<=0;path_clear<=0;path_cell_valid<=0;waypoint_valid<=0;
            if(abort)begin
                // An accepted map read still owns its late response. Drain it
                // before allowing another plan to issue the first snapshot read.
                if(state==SNAP_WAIT&&!map_rsp_valid)begin state<=DRAIN;wait_count<=0;end
                else if(state!=DRAIN||map_rsp_valid)state<=IDLE;
                result_available<=0;success<=0;way_exhausted<=1;path_clear<=1;
            end else if(start_plan&&state==IDLE)begin
                path_clear<=1;success<=0;fail_code<=0;result_available<=0;way_exhausted<=1;
                route_uses_unknown<=0;route_steps<=0;route_turns<=0;
                sx<=start_x;sy<=start_y;gx<=goal_x;gy<=goal_y;cx<=0;cy<=0;cursor<=0;wait_count<=0;
                if(start_x>=GRID_X||start_y>=GRID_Y||goal_x>=GRID_X||goal_y>=GRID_Y||
                    start_heading_tenths>1800||start_heading_tenths < -1800)fail(4'd6);
                else begin
                    if(start_heading_tenths>=-450&&start_heading_tenths<450)begin
                        orientation<=0;body_heading<=0;initial_delta<=start_heading_tenths;
                    end else if(start_heading_tenths>=450&&start_heading_tenths<1350)begin
                        orientation<=2;body_heading<=900;initial_delta<=start_heading_tenths-900;
                    end else if(start_heading_tenths>=-1350&&start_heading_tenths < -450)begin
                        orientation<=3;body_heading<=-900;initial_delta<=start_heading_tenths+900;
                    end else begin
                        orientation<=1;body_heading<=1800;
                        initial_delta<=start_heading_tenths>=0?start_heading_tenths-1800:start_heading_tenths+1800;
                    end
                    state<=SNAP_REQ;
                end
            end else case(state)
                DRAIN:if(map_rsp_valid)state<=IDLE;
                IDLE:if(result_available)begin
                    if(path_rewind)begin tx<=sx;ty<=sy;way_first<=1;way_exhausted<=0;way_count<=0;end
                    else if(waypoint_next&&!way_exhausted)begin
                        if(way_first)begin tx<=sx;ty<=sy;way_first<=0;waypoint_x<=sx;waypoint_y<=sy;
                            waypoint_last<=sx==gx&&sy==gy;way_exhausted<=sx==gx&&sy==gy;waypoint_valid<=1;end
                        else begin way_walked<=0;state<=WP_READ;end
                    end
                end
                SNAP_REQ:if(map_req_ready)begin wait_count<=0;state<=SNAP_WAIT;end
                    else if(wait_count>=MAP_TIMEOUT_CLKS-1)fail(4'd4);else wait_count<=wait_count+1'b1;
                SNAP_WAIT:if(map_rsp_valid)begin
                    wait_count<=0;
                    if(cursor==N-1)state<=BOUNDS;
                    else begin cursor<=cursor+1'b1;if(cx==GRID_X-1)begin cx<=0;cy<=cy+1'b1;end
                        else cx<=cx+1'b1;state<=SNAP_REQ;end
                end else if(wait_count>=MAP_TIMEOUT_CLKS-1)begin fail(4'd4);state<=DRAIN;end
                    else wait_count<=wait_count+1'b1;
                BOUNDS:begin
                    rotation_bin<=rotation_magnitude>400?9:rotation_magnitude>350?8:
                        rotation_magnitude>300?7:rotation_magnitude>250?6:
                        rotation_magnitude>200?5:rotation_magnitude>150?4:
                        rotation_magnitude>100?3:rotation_magnitude>50?2:rotation_magnitude>0?1:0;
                    state<=BOUND_INDEX;
                end
                BOUND_INDEX:begin
                    base_index<=orientation*7'd20;
                    rotation_index<=orientation*7'd20+(initial_delta[12]?7'd10:7'd0)+rotation_bin;
                    state<=BOUND_COMMIT;
                end
                BOUND_COMMIT:begin
                    {lo_x,hi_x,lo_y,hi_y}<=bound_table[base_index];
                    {rot_lo_x,rot_hi_x,rot_lo_y,rot_hi_y}<=bound_table[rotation_index];
                    wait_count<=0;
                    if(CHECK_START_CLEARANCE)begin cx<=sx;cy<=sy;state<=ROT_READ;end
                    else begin cx<=0;cy<=0;cursor<=0;state<=PREP_READ;end
                end
                ROT_READ:begin
                    if(wait_count==0)begin dx<=rot_lo_x;dy<=rot_lo_y;wait_count<=1;end
                    else state<=ROT_EVAL;
                end
                ROT_EVAL:if(sample_obstacle)fail(4'd1);
                    else if(dx==rot_hi_x&&dy==rot_hi_y)begin
                        cx<=0;cy<=0;cursor<=0;state<=PREP_READ;
                    end else begin
                        if(dx==rot_hi_x)begin dx<=rot_lo_x;dy<=dy+1'b1;end else dx<=dx+1'b1;
                        state<=ROT_READ;
                    end
                PREP_READ:state<=PREP;
                PREP:begin raw_center<=attr_q;clearance<=SOFT_CELLS;footprint_unknown<=0;blocked<=0;
                    directional_blocked<=0;directional_unknown<=0;rotation_blocked<=0;
                    dx<=TURN_IN_PLACE?-(TURN_CELLS+SOFT_CELLS):lo_x-SOFT_CELLS;
                    dy<=TURN_IN_PLACE?-(TURN_CELLS+SOFT_CELLS):lo_y-SOFT_CELLS;state<=FOOT_READ;end
                FOOT_READ:state<=FOOT_EVAL;
                FOOT_EVAL:begin
                    if(TURN_IN_PLACE)begin
                        if(sample_obstacle)begin
                            directional_blocked<=directional_blocked|sample_coverage[3:0];
                            if(sample_coverage[4])rotation_blocked<=1;
                        end
                        if(sample_inside&&attr_q[1:0]==0)directional_unknown<=directional_unknown|sample_coverage[3:0];
                    end
                    if(!TURN_IN_PLACE&&sample_obstacle&&shell==0)begin blocked<=1;clearance<=0;state<=FOOT_WRITE;end
                    else begin
                        if(sample_obstacle&&shell<clearance)clearance<=shell[3:0];
                        if(sample_inside&&shell==0&&attr_q[1:0]==0)footprint_unknown<=1;
                        if(dx==(TURN_IN_PLACE?TURN_CELLS+SOFT_CELLS:hi_x+SOFT_CELLS)&&
                           dy==(TURN_IN_PLACE?TURN_CELLS+SOFT_CELLS:hi_y+SOFT_CELLS))state<=FOOT_WRITE;
                        else begin
                            if(dx==(TURN_IN_PLACE?TURN_CELLS+SOFT_CELLS:hi_x+SOFT_CELLS))begin
                                dx<=TURN_IN_PLACE?-(TURN_CELLS+SOFT_CELLS):lo_x-SOFT_CELLS;dy<=dy+1'b1;
                            end else dx<=dx+1'b1;
                            state<=FOOT_READ;
                        end
                    end
                end
                FOOT_WRITE:begin
                    if(cursor==N-1)begin cx<=sx;cy<=sy;wait_count<=0;state<=END_READ;end
                    else begin cursor<=cursor+1'b1;
                        if(cx==GRID_X-1)begin cx<=0;cy<=cy+1'b1;end else cx<=cx+1'b1;
                        state<=PREP_READ;end
                end
                END_READ:state<=END_EVAL;
                END_EVAL:if((wait_count==0?CHECK_START_CLEARANCE:CHECK_GOAL_CLEARANCE)&&
                    (attr_q[3]||(TURN_IN_PLACE&&wait_count==0&&motion_q[orientation])))fail(wait_count==0?4'd1:4'd2);
                    else if(wait_count==0)begin cx<=gx;cy<=gy;wait_count<=1;state<=END_READ;end
                    else state<=SEED;
                SEED:begin bucket<=0;next_bucket<=65535;scan_index<=0;scan_x<=0;scan_y<=0;state<=SCAN_READ;end
                SCAN_READ:state<=SCAN_EVAL;
                SCAN_EVAL:begin
                    if(!attr_q[3]&&!node_q[15]&&node_q[14:0]!=32767)begin
                        if(node_q[14:0]==bucket)begin
                            if(scan_x==sx&&scan_y==sy)begin
                                tx<=sx;ty<=sy;trace_seen<=0;trace_count<=0;state<=TRACE_READ;
                            end else begin
                                current_distance<=node_q[14:0];current_parent<=node_q[17:16];
                                current_motion_mask<=motion_q;
                                current_index<=scan_index;current_attr<=attr_q;neighbor_dir<=0;state<=CLOSE;
                            end
                        end else begin if(node_q[14:0]<next_bucket)next_bucket<=node_q[14:0];advance_scan;end
                    end else advance_scan;
                end
                CLOSE:state<=NEIGH_READ;
                NEIGH_READ:state<=NEIGH_EVAL;
                NEIGH_EVAL:begin
                    // Isolate bank selection + weighted addition from distance
                    // comparison and the write/next-bucket control fanout.
                    proposed_distance<=candidate_cost[15:0];neighbor_distance<=node_q[14:0];
                    neighbor_eligible<=neighbor_inside&&!attr_q[3]&&!node_q[15]&&
                        (!TURN_IN_PLACE||(!motion_q[neighbor_dir]&&!current_motion_mask[neighbor_dir]&&
                        (current_distance==0||current_parent==neighbor_dir||!current_motion_mask[8])&&
                        (nx!=sx||ny!=sy||orientation==neighbor_dir||!motion_q[8])));
                    neighbor_index<=index(nx,ny);state<=NEIGH_COST;
                end
                NEIGH_COST:begin
                    if(neighbor_eligible&&proposed_distance<32767&&proposed_distance<neighbor_distance)begin
                        state<=NEIGH_WRITE;
                        if(proposed_distance<next_bucket)next_bucket<=proposed_distance;
                    end else if(neighbor_dir==3)advance_scan;
                    else begin neighbor_dir<=neighbor_dir+1'b1;state<=NEIGH_READ;end
                end
                NEIGH_WRITE:if(neighbor_dir==3)advance_scan;
                    else begin neighbor_dir<=neighbor_dir+1'b1;state<=NEIGH_READ;end
                ADVANCE:if(scan_index==N-1)begin
                    if(next_bucket==65535)fail(4'd3);
                    else begin bucket<=next_bucket;next_bucket<=65535;scan_index<=0;scan_x<=0;scan_y<=0;state<=SCAN_READ;end
                end else begin scan_index<=scan_index+1'b1;
                    if(scan_x==GRID_X-1)begin scan_x<=0;scan_y<=scan_y+1'b1;end else scan_x<=scan_x+1'b1;
                    state<=SCAN_READ;end
                TRACE_READ:state<=TRACE_EVAL;
                TRACE_EVAL:begin
                    if(trace_count>=N||attr_q[3]||node_q[14:0]==32767)fail(4'd6);
                    else if(TURN_IN_PLACE&&((tx!=gx||ty!=gy)&&motion_q[node_q[17:16]]||
                        (trace_seen&&trace_dir!=node_q[17:16]&&(tx!=gx||ty!=gy)&&motion_q[8])))fail(4'd7);
                    else begin
                        path_cell_valid<=1;path_cell_addr<={ty,tx};
                        if(attr_q[2])route_uses_unknown<=1;
                        if(tx==gx&&ty==gy)begin
                            state<=IDLE;success<=1;done<=1;result_available<=1;
                            route_steps<=trace_count;tx<=sx;ty<=sy;way_first<=1;way_exhausted<=0;way_count<=0;
                        end else if(!trace_step_ok)fail(4'd6);
                        else begin
                            if(trace_seen&&trace_dir!=node_q[17:16])route_turns<=route_turns+1'b1;
                            trace_seen<=1;trace_dir<=node_q[17:16];trace_count<=trace_count+1'b1;
                            tx<=step_x;ty<=step_y;state<=TRACE_READ;
                        end
                    end
                end
                WP_READ:state<=WP_EVAL;
                WP_EVAL:begin
                    if(way_count>=N||attr_q[3]||node_q[14:0]==32767)fail(4'd6);
                    else if(tx==gx&&ty==gy)emit_way(1);
                    else if(way_walked&&way_dir!=node_q[17:16])emit_way(0);
                    else if(!trace_step_ok)fail(4'd6);
                    else begin way_walked<=1;way_dir<=node_q[17:16];way_count<=way_count+1'b1;
                        tx<=step_x;ty<=step_y;state<=WP_READ;end
                end
                default:fail(4'd6);
            endcase
        end
    end
endmodule
