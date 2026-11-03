// Uniform-clock pose history for arrival-latency compensation. A point queries
// its estimated acquisition timestamp, independently of decoder backpressure.
// Wrapped heading interpolation crosses +/-180 degrees on the short arc.
module slam_pose_history #(
    parameter integer CLK_FREQ_HZ=50000000,TICK_US=1000,DEPTH=512,PB=$clog2(DEPTH)
)(
    input wire clk,reset_n,clear,pose_valid,
    input wire signed [15:0] pose_x,pose_y,input wire signed [31:0] pose_yaw,
    input wire query_start,input wire [31:0] query_time,
    output reg done,valid,output reg signed [15:0] x_mm,y_mm,
    output reg signed [12:0] yaw_tenths
);
    localparam RAW_TICK=CLK_FREQ_HZ/1000000*TICK_US+(CLK_FREQ_HZ%1000000)*TICK_US/1000000;
    localparam TICK_CLKS=RAW_TICK>0?RAW_TICK:1;
    (* ram_style="block",syn_ramstyle="block_ram" *)reg [45:0] history[0:DEPTH-1];
    reg [31:0] timestamp,tick,latest_time,age,remainder;
    reg [PB-1:0] write_pointer,read_pointer;
    reg [PB-1:0] query_pointer;
    reg [PB:0] query_stored;
    reg [PB:0] stored;
    reg [45:0] read_data,newer,older;
    reg [3:0] phase;
    reg signed [31:0] dx,dy,da;
    reg signed [63:0] wx,wy,wa;
    reg [63:0] fraction_product;
    reg [63:0] age_product;
    reg [31:0] whole_ticks;
    localparam [32:0] TICK_RECIPROCAL_Q32=64'd4294967296/TICK_CLKS;
    wire [31:0] signed_age=latest_time-query_time;
    always @(posedge clk)begin
        if(!clear&&tick==TICK_CLKS-1)history[write_pointer]<={pose_valid,pose_x,pose_y,pose_yaw[12:0]};
        read_data<=history[read_pointer];
    end
    function signed [31:0] wrap;
        input signed [31:0] a;
        begin wrap=a>1800?a-3600:a< -1800?a+3600:a;end
    endfunction
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            timestamp<=0;tick<=0;latest_time<=0;write_pointer<=0;read_pointer<=0;stored<=0;query_pointer<=0;query_stored<=0;
            phase<=0;done<=0;valid<=0;x_mm<=0;y_mm<=0;yaw_tenths<=0;age<=0;remainder<=0;
            newer<=0;older<=0;dx<=0;dy<=0;da<=0;wx<=0;wy<=0;wa<=0;fraction_product<=0;age_product<=0;whole_ticks<=0;
        end else begin
            timestamp<=timestamp+1'b1;done<=0;
            if(clear)begin tick<=0;stored<=0;phase<=0;valid<=0;end
            else begin
                if(tick==TICK_CLKS-1)begin
                    tick<=0;latest_time<=timestamp;write_pointer<=write_pointer+1'b1;
                    if(stored<DEPTH)stored<=stored+1'b1;
                end else tick<=tick+1'b1;
                case(phase)
                    0:if(query_start)begin
                        valid<=0;
                        // A point newer than the last tick uses the most recent
                        // pair, rather than wrapping the unsigned history age.
                        age<=signed_age[31]?0:signed_age;
                        query_pointer<=write_pointer;query_stored<=stored;
                        phase<=1;
                    end
                    1:if(query_stored<2||age>=(query_stored-1)*TICK_CLKS)begin done<=1;valid<=0;phase<=0;end
                        else begin
                            age_product<=age*TICK_RECIPROCAL_Q32;phase<=2;
                        end
                    2:begin
                        whole_ticks<=age_product[63:32];remainder<=age-age_product[63:32]*TICK_CLKS;phase<=3;
                    end
                    3:begin
                        // The floor reciprocal can underestimate by one tick.
                        read_pointer<=query_pointer-1-whole_ticks-(remainder>=TICK_CLKS);
                        if(remainder>=TICK_CLKS)remainder<=remainder-TICK_CLKS;
                        phase<=4;
                    end
                    4:phase<=5;
                    5:begin newer<=read_data;read_pointer<=read_pointer-1'b1;phase<=6;end
                    6:phase<=7;
                    7:begin older<=read_data;phase<=8;end
                    8:begin
                        dx<=$signed(older[44:29])-$signed(newer[44:29]);
                        dy<=$signed(older[28:13])-$signed(newer[28:13]);
                        da<=wrap($signed(older[12:0])-$signed(newer[12:0]));
                        fraction_product<=remainder*TICK_RECIPROCAL_Q32;phase<=9;
                    end
                    9:begin
                        wx<=$signed(dx[16:0])*$signed({1'b0,fraction_product[31:16]});
                        wy<=$signed(dy[16:0])*$signed({1'b0,fraction_product[31:16]});
                        wa<=$signed(da[13:0])*$signed({1'b0,fraction_product[31:16]});phase<=10;
                    end
                    10:begin
                        x_mm<=$signed(newer[44:29])+(wx>>>16);y_mm<=$signed(newer[28:13])+(wy>>>16);
                        yaw_tenths<=wrap($signed(newer[12:0])+(wa>>>16));
                        valid<=newer[45]&&older[45];done<=1;phase<=0;
                    end
                    default:phase<=0;
                endcase
            end
        end
    end
endmodule
