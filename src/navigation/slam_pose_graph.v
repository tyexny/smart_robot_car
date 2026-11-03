// Bounded SE(2) graph with serialized synchronous RAM accesses. Node zero is
// fixed. Q8 state, relative constraints and rollback objective are retained.
module slam_pose_graph #(
    parameter integer MAX_NODES=64,MAX_EDGES=128,ITERATIONS=48,
    parameter integer NODE_BITS=$clog2(MAX_NODES)
)(
    input wire clk,reset_n,clear,
    input wire add_node,input wire signed [15:0] node_x,node_y,
    input wire signed [12:0] node_yaw,
    input wire add_edge,input wire [NODE_BITS-1:0] edge_i,edge_j,
    input wire signed [15:0] edge_dx,edge_dy,input wire signed [12:0] edge_dyaw,
    input wire edge_loop,start_optimize,
    output wire busy,output reg done,accepted,capacity_fault,
    output reg [NODE_BITS:0] node_count,
    input wire [NODE_BITS-1:0] query_node,
    output wire signed [15:0] query_x,query_y,output wire signed [12:0] query_yaw,
    output reg [47:0] initial_error,final_error
);
    localparam EB=$clog2(MAX_EDGES),EW=46+2*NODE_BITS;
    localparam IDLE=0,BACKUP_READ=1,BACKUP_WAIT=2,BACKUP_WRITE=3,
        EDGE_READ=4,EDGE_WAIT=5,EDGE_LATCH=6,I_READ=7,I_WAIT=8,I_LATCH=9,
        J_READ=10,J_WAIT=11,J_LATCH=12,TRIG=13,MULTIPLY=14,RESIDUAL=15,
        ERROR_PARTS=16,ERROR_SUM=17,APPLY=18,WRITE_I=19,WRITE_J=20,NEXT=21,
        CHECK=22,RESTORE_READ=23,RESTORE_WAIT=24,RESTORE_WRITE=25;
    reg [4:0] state;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [95:0] nodes[0:MAX_NODES-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [95:0] backup[0:MAX_NODES-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [EW-1:0] edges[0:MAX_EDGES-1];
    // Small integer mirror retains the asynchronous public query interface;
    // the optimizer's full-precision multi-access arrays no longer use muxes.
    (* ram_style="distributed",syn_ramstyle="distributed_ram" *) reg [44:0] published[0:MAX_NODES-1];
    reg [95:0] node_data,backup_data,cached_i,cached_j,updated_i,updated_j;
    reg [EW-1:0] edge_data;
    reg [EB:0] edge_count;
    reg [EB-1:0] cursor;
    reg [NODE_BITS-1:0] node_cursor,ni,nj;
    reg signed [15:0] mx,my;
    reg signed [12:0] ma,angle;
    reg loop_weight,trig_start,measure_only;
    reg [7:0] iteration;
    reg signed [31:0] pc,ps,qc,qs,rx,ry,ra;
    reg [47:0] error_sum;
    reg [31:0] error_x,error_y,error_a;
    reg [47:0] measured_edge_error;
    wire trig_done;
    wire signed [15:0] cosine,sine;
    wire adding_node=state==IDLE&&add_node&&node_count<MAX_NODES;
    wire adding_edge=state==IDLE&&add_edge&&edge_count<MAX_EDGES&&
        edge_i<node_count&&edge_j<node_count&&edge_i!=edge_j;
    wire node_write=reset_n&&!clear&&(adding_node||state==RESTORE_WRITE||
        (!measure_only&&((state==WRITE_I&&ni!=0)||(state==WRITE_J&&nj!=0))));
    wire [NODE_BITS-1:0] write_addr=adding_node?node_count[NODE_BITS-1:0]:
        state==RESTORE_WRITE?node_cursor:state==WRITE_I?ni:nj;
    wire [95:0] initial_node={{8{node_x[15]}},node_x,8'd0,
        {8{node_y[15]}},node_y,8'd0,{11{node_yaw[12]}},node_yaw,8'd0};
    wire [95:0] write_data=adding_node?initial_node:state==RESTORE_WRITE?backup_data:
        state==WRITE_I?updated_i:updated_j;
    wire [NODE_BITS-1:0] read_addr=
        (state==BACKUP_READ||state==BACKUP_WAIT||state==BACKUP_WRITE)?node_cursor:
        (state==J_READ||state==J_WAIT||state==J_LATCH)?nj:ni;
    wire signed [31:0] ix=cached_i[95:64],iy=cached_i[63:32],ia=cached_i[31:0];
    wire signed [31:0] jx=cached_j[95:64],jy=cached_j[63:32],ja=cached_j[31:0];
    assign busy=state!=IDLE;
    assign query_x=published[query_node][44:29];
    assign query_y=published[query_node][28:13];
    assign query_yaw=published[query_node][12:0];
    function signed [31:0] wrap_q8;
        input signed [31:0] a;
        begin wrap_q8=a>1800*256?a-3600*256:a< -1800*256?a+3600*256:a;end
    endfunction
    function [31:0] magnitude;
        input signed [31:0] a;
        begin magnitude=a<0?-a:a;end
    endfunction
    always @(posedge clk)begin
        if(node_write)begin
            nodes[write_addr]<=write_data;
            published[write_addr]<={write_data[87:72],write_data[55:40],write_data[20:8]};
        end
        node_data<=nodes[read_addr];
        // GW5AT single-port RAM supports write-through, not read-before-write.
        // Restoration only reads this RAM after the entire backup is complete.
        if(reset_n&&!clear&&state==BACKUP_WRITE)begin
            backup[node_cursor]<=node_data;
            backup_data<=node_data;
        end else backup_data<=backup[node_cursor];
        if(reset_n&&!clear&&adding_edge)
            edges[edge_count[EB-1:0]]<={edge_i,edge_j,edge_dx,edge_dy,edge_dyaw,edge_loop};
        edge_data<=edges[cursor];
    end
    heading_lut_fine_pipeline trig(.clk(clk),.reset_n(reset_n),.start(trig_start),.cancel(clear),
        .angle_tenths(angle),.done(trig_done),.cos_q14(cosine),.sin_q14(sine));
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;done<=0;accepted<=0;capacity_fault<=0;node_count<=0;edge_count<=0;
            cursor<=0;node_cursor<=0;ni<=0;nj<=0;mx<=0;my<=0;ma<=0;angle<=0;
            loop_weight<=0;trig_start<=0;iteration<=0;measure_only<=0;
            pc<=0;ps<=0;qc<=0;qs<=0;rx<=0;ry<=0;ra<=0;error_sum<=0;initial_error<=0;final_error<=0;
            error_x<=0;error_y<=0;error_a<=0;measured_edge_error<=0;
            cached_i<=0;cached_j<=0;updated_i<=0;updated_j<=0;
        end else begin
            done<=0;trig_start<=0;
            if(clear)begin state<=IDLE;node_count<=0;edge_count<=0;capacity_fault<=0;accepted<=0;end
            else case(state)
                IDLE:begin
                    if(add_node)begin
                        if(adding_node)node_count<=node_count+1'b1;else capacity_fault<=1;
                    end
                    if(add_edge)begin
                        if(adding_edge)edge_count<=edge_count+1'b1;else capacity_fault<=1;
                    end
                    if(start_optimize&&node_count>1&&edge_count>0)begin
                        node_cursor<=0;accepted<=0;state<=BACKUP_READ;
                    end
                end
                BACKUP_READ:state<=BACKUP_WAIT;
                BACKUP_WAIT:state<=BACKUP_WRITE;
                BACKUP_WRITE:begin
                    if(node_cursor+1>=node_count)begin
                        cursor<=0;iteration<=0;measure_only<=1;error_sum<=0;state<=EDGE_READ;
                    end else begin node_cursor<=node_cursor+1'b1;state<=BACKUP_READ;end
                end
                EDGE_READ:state<=EDGE_WAIT;
                EDGE_WAIT:state<=EDGE_LATCH;
                EDGE_LATCH:begin
                    ni<=edge_data[46+NODE_BITS+:NODE_BITS];nj<=edge_data[46+:NODE_BITS];
                    mx<=edge_data[45:30];my<=edge_data[29:14];ma<=edge_data[13:1];
                    loop_weight<=edge_data[0];state<=I_READ;
                end
                I_READ:state<=I_WAIT;
                I_WAIT:state<=I_LATCH;
                I_LATCH:begin cached_i<=node_data;angle<=$signed(node_data[31:0])>>>8;state<=J_READ;end
                J_READ:state<=J_WAIT;
                J_WAIT:state<=J_LATCH;
                J_LATCH:begin cached_j<=node_data;trig_start<=1;state<=TRIG;end
                TRIG:if(trig_done)begin
                    pc<=mx*cosine;ps<=my*sine;qc<=mx*sine;qs<=my*cosine;state<=MULTIPLY;
                end
                MULTIPLY:begin
                    rx<=ix+(($signed(pc)-$signed(ps))>>>6)-jx;
                    ry<=iy+(($signed(qc)+$signed(qs))>>>6)-jy;
                    ra<=wrap_q8(ia+$signed(ma)*256-ja);state<=RESIDUAL;
                end
                RESIDUAL:begin
                    error_x<=magnitude(rx);error_y<=magnitude(ry);error_a<=magnitude(ra);state<=ERROR_PARTS;
                end
                ERROR_PARTS:begin measured_edge_error<=error_x+error_y+(error_a*7/4);state<=ERROR_SUM;end
                ERROR_SUM:begin error_sum<=error_sum+(loop_weight?measured_edge_error*2:measured_edge_error);state<=APPLY;end
                APPLY:begin
                    updated_i<={ix-(rx>>>(loop_weight?2:3)),iy-(ry>>>(loop_weight?2:3)),wrap_q8(ia-(ra>>>(loop_weight?2:3)))};
                    updated_j<={jx+(rx>>>(loop_weight?2:3)),jy+(ry>>>(loop_weight?2:3)),wrap_q8(ja+(ra>>>(loop_weight?2:3)))};
                    state<=WRITE_I;
                end
                WRITE_I:state<=WRITE_J;
                WRITE_J:state<=NEXT;
                NEXT:if(cursor+1>=edge_count)begin
                    cursor<=0;
                    if(measure_only&&iteration==0)begin
                        initial_error<=error_sum;measure_only<=0;iteration<=1;error_sum<=0;state<=EDGE_READ;
                    end else if(measure_only)begin final_error<=error_sum;state<=CHECK;end
                    else begin
                        iteration<=iteration+1'b1;measure_only<=iteration>=ITERATIONS;error_sum<=0;state<=EDGE_READ;
                    end
                end else begin cursor<=cursor+1'b1;state<=EDGE_READ;end
                CHECK:if(final_error<=initial_error)begin accepted<=1;done<=1;state<=IDLE;end
                    else begin node_cursor<=0;cursor<=edge_count-1'b1;state<=RESTORE_READ;end
                RESTORE_READ:state<=RESTORE_WAIT;
                RESTORE_WAIT:state<=RESTORE_WRITE;
                RESTORE_WRITE:begin
                    if(node_cursor+1>=node_count)begin
                        if(edge_count>0&&edge_data[0])edge_count<=edge_count-1'b1;
                        done<=1;accepted<=0;state<=IDLE;
                    end else begin node_cursor<=node_cursor+1'b1;state<=RESTORE_READ;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
