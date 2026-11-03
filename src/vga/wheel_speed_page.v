`timescale 1ns/1ps
// 1280x720 four-wheel tracking dashboard. Signed motor protocol units are
// LF target/feedback signs are inverted for physical forward-positive display.
// 81 visible samples, 100 ms apart: exactly 8.0 s, at four pixels/interval.
// History writes occur at frame_begin (vertical blank), never during a frame.
// Six registered stages match telemetry_pages and scan_map_page.
module wheel_speed_page #(
    parameter integer CLK_FREQ_HZ=75_000_000,
    parameter integer SAMPLE_MS=100
)(
    input wire clk,reset_n,frame_begin,active,
    input wire [11:0] pixel_x,pixel_y,
    input wire hs_in,vs_in,de_in,
    input wire [63:0] actual_speeds,target_speeds,
    input wire motor_online,
    input wire [215:0] imu_bcd,
    input wire [8:0] imu_negative,imu_valid,
    output reg [23:0] rgb_out,
    output reg hs_out,vs_out,de_out,active_out
);
    localparam [23:0] BG=24'hf3f5f8,CARD=24'hffffff,TEXT=24'h202c3b,
        MUTED=24'h778391,GRID=24'he5eaf0,BLUE=24'h428ff5,ACTUAL_RED=24'he53935,
        RED=24'hcf5050;
    localparam integer SAMPLE_CLKS=(CLK_FREQ_HZ/1000)*SAMPLE_MS;
    localparam integer TICK_BITS=SAMPLE_CLKS<2?1:$clog2(SAMPLE_CLKS);
    reg [TICK_BITS-1:0] tick;
    reg sample_pending,display_active,display_online;
    reg [6:0] write_index;
    reg [7:0] sample_count;
    // First valid chart columns for the current history length. Compute these
    // only when a new sample is committed, outside the raw pixel arithmetic.
    reg [6:0] left_first_column,right_first_column;
    // Bit 64 records whether the feedback was live at this sample. Two
    // synchronous read ports allow interpolation between adjacent samples.
    reg [64:0] history[0:127];
    reg [64:0] hist_left,hist_right;
    reg [63:0] sample_heights;
    integer k,j,digit;
    // Compile-time table: -20..+20 fills the existing 128-pixel plot.
    // The integer division is constant-folded; runtime scaling is a lookup.
    function [511:0] make_height_table;
        input integer unused;
        integer code,speed,height;
        begin
            make_height_table=0;
            for(code=0;code<64;code=code+1)begin
                speed=code<32?code:code-64;
                height=((20-speed)*128+20)/40;
                if(height<0)height=0;
                if(height>128)height=128;
                make_height_table[code*8+:8]=height[7:0];
            end
        end
    endfunction
    localparam [511:0] HEIGHT_TABLE=make_height_table(0);
    function [7:0] speed_height;
        input signed [16:0] speed;
        begin
            if(speed>20) speed_height=0;
            else if(speed< -20) speed_height=128;
            else speed_height=HEIGHT_TABLE[{speed[5:0],3'b0}+:8];
        end
    endfunction
    // Extend before negation so raw -32768 is displayed as +32768.
    // Both traces and their numbers use the same physical-wheel convention.
    function signed [16:0] display_speed;
        input [63:0] speeds;
        input [1:0] wheel;
        reg signed [15:0] raw;
        begin
            raw=speeds[(3-wheel)*16+:16];
            display_speed=wheel==0 ? -$signed({raw[15],raw}) : $signed({raw[15],raw});
        end
    endfunction
    always @* begin
        for(k=0;k<4;k=k+1) begin
            sample_heights[k*16+:8]=speed_height(display_speed(actual_speeds,k));
            sample_heights[k*16+8+:8]=speed_height(display_speed(target_speeds,k));
        end
    end
    wire sample_write=frame_begin && sample_pending;
    // Iterative binary-to-decimal formatting runs only during vertical blank.
    reg [19:0] number_bcd[0:7],number_work[0:7];
    reg [15:0] number_shift[0:7];
    reg [7:0] number_negative;
    reg [4:0] format_step;
    reg [19:0] adjusted[0:7];
    reg signed [16:0] format_value;
    integer n;
    always @* begin
        for(j=0;j<8;j=j+1) begin
            adjusted[j]=number_work[j];
            for(digit=0;digit<5;digit=digit+1)
                if(number_work[j][digit*4+:4]>=5)
                    adjusted[j][digit*4+:4]=number_work[j][digit*4+:4]+4'd3;
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            tick<=0;sample_pending<=0;write_index<=0;sample_count<=0;
            left_first_column<=7'd81;right_first_column<=7'd80;
            display_active<=0;display_online<=0;format_step<=16;number_negative<=0;
            for(n=0;n<8;n=n+1) begin
                number_bcd[n]<=0;number_work[n]<=0;number_shift[n]<=0;
            end
        end else begin
            if(tick==SAMPLE_CLKS-1) begin tick<=0;sample_pending<=1;end
            else tick<=tick+1'b1;
            if(sample_write) begin
                sample_pending<=0;write_index<=write_index+1'b1;
                if(sample_count<128) sample_count<=sample_count+1'b1;
                if(sample_count<80)begin
                    left_first_column<=7'd80-sample_count[6:0];
                    right_first_column<=7'd79-sample_count[6:0];
                end else begin left_first_column<=0;right_first_column<=0;end
            end
            if(frame_begin) begin
                display_active<=active;display_online<=motor_online;
                format_step<=0;
                for(n=0;n<8;n=n+1) begin
                    if(n<4) format_value=display_speed(actual_speeds,n);
                    else format_value=display_speed(target_speeds,n-4);
                    number_negative[n]<=format_value[16];
                    number_shift[n]<=format_value[16]?-format_value:format_value;
                    number_work[n]<=0;
                end
            end else if(format_step<16) begin
                for(n=0;n<8;n=n+1) begin
                    number_work[n]<={adjusted[n][18:0],number_shift[n][15]};
                    number_shift[n]<=number_shift[n]<<1;
                    if(format_step==15) number_bcd[n]<={adjusted[n][18:0],number_shift[n][15]};
                end
                format_step<=format_step+1'b1;
            end
        end
    end
    always @(posedge clk) if(sample_write)
        history[write_index]<={motor_online,sample_heights};

    // Stage 1: local coordinates and synchronous history reads. The page
    // starts with empty history; sample_count masks all uninitialized RAM.
    wire right_card=pixel_x>=448,bottom_card=pixel_y>=400;
    wire [11:0] local_x=right_card?pixel_x-12'd448:pixel_x;
    wire [11:0] local_y=bottom_card?pixel_y-12'd320:pixel_y;
    // Both origins are multiples of four. Seven-bit modular subtraction is
    // exactly the old (local_x-80)[8:2], including pixels outside the charts.
    wire [6:0] column_index=pixel_x[8:2]-(right_card?7'd4:7'd20);
    // The two chart origins are multiples of four pixels. Fold both origin
    // subtractions into pointer bases when a sample is written. Each RAM
    // address then needs one seven-bit add, with no local-X/plot-X carry chain.
    reg [6:0] left_read_base,right_read_base,left_next_base,right_next_base;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)begin
            left_read_base<=7'd27;right_read_base<=7'd43;
            left_next_base<=7'd28;right_next_base<=7'd44;
        end else if(sample_write)begin
            left_read_base<=left_read_base+1'b1;right_read_base<=right_read_base+1'b1;
            left_next_base<=left_next_base+1'b1;right_next_base<=right_next_base+1'b1;
        end
    wire [6:0] read_left=(right_card?right_read_base:left_read_base)+pixel_x[8:2];
    wire [6:0] read_right=(right_card?right_next_base:left_next_base)+pixel_x[8:2];
    reg [11:0] x1,y1,global_x1,global_y1;
    reg [1:0] wheel1,fraction1;
    reg [12:0] text_column1,header_column1;
    reg [6:0] imu_column1;
    reg [3:0] imu_row1;
    reg [4:0] tick_left_column1,tick_middle_column1,tick_now_column1,badge_column1;
    always @(posedge clk)begin
        text_column1<=(right_card?pixel_x>=480:pixel_x>=32) ?
            (pixel_x>>3)-(right_card?13'd60:13'd4) : 13'd8191;
        header_column1<=pixel_x>=32 ? (pixel_x>>3)-13'd4 : 13'd8191;
        imu_column1<=(pixel_x>>4)-7'd58;
        imu_row1<=pixel_y<128 ? 4'd0 : pixel_y[9:6]-4'd1;
        tick_left_column1<=(local_x-12'd80)>>3;
        tick_middle_column1<=(local_x-12'd224)>>3;
        tick_now_column1<=(local_x-12'd376)>>3;
        badge_column1<=(pixel_x-12'd1024)>>3;
    end
    reg plot1,left_valid1,right_valid1,hs1,vs1,de1,active1;
    // Decode text scan lines alongside the coordinate register. Sharing the
    // raw Y register with other pages otherwise puts range comparisons and
    // the entire character priority mux in one font-ASCII cycle.
    reg [12:0] text_y1 /* synthesis syn_preserve=1 */;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)text_y1<=0;
        else begin
            text_y1[0]<=pixel_y>=16 && pixel_y<32;
            text_y1[1]<=pixel_y>=48 && pixel_y<64;
            text_y1[2]<=local_y>=96 && local_y<112;
            text_y1[3]<=local_y>=128 && local_y<144;
            text_y1[4]<=local_y>=192 && local_y<208;
            text_y1[5]<=local_y>=256 && local_y<272;
            text_y1[6]<=local_y>=320 && local_y<336;
            text_y1[7]<=local_y>=336 && local_y<352;
            text_y1[8]<=pixel_y>=704 && pixel_y<720;
            text_y1[9]<=pixel_y>=32 && pixel_y<48;
            text_y1[10]<=pixel_y>=96 && pixel_y<688;
            text_y1[11]<=pixel_y<128 || !pixel_y[5];
            text_y1[12]<=pixel_y>=80 && pixel_y<704;
        end
    always @(posedge clk) begin
        hist_left<=history[read_left];hist_right<=history[read_right];
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            x1<=0;y1<=0;global_x1<=0;global_y1<=0;wheel1<=0;fraction1<=0;
            plot1<=0;left_valid1<=0;right_valid1<=0;hs1<=0;vs1<=0;de1<=0;active1<=0;
        end else begin
            x1<=local_x;y1<=local_y;global_x1<=pixel_x;global_y1<=pixel_y;
            wheel1<={bottom_card,right_card};fraction1<=pixel_x[1:0];
            plot1<=pixel_x<896 && local_x>=80 && local_x<=400 && local_y>=192 && local_y<=320;
            left_valid1<=sample_count!=0 && column_index>=left_first_column;
            right_valid1<=column_index<80 && column_index>=right_first_column;
            hs1<=hs_in;vs1<=vs_in;de1<=de_in;active1<=display_active;
        end
    end
    function [6:0] message_char;
        input [8*80-1:0] message;
        input integer length,column;
        begin
            message_char=7'd32;
            if(column>=0 && column<length) message_char=message[(length-1-column)*8+:7];
        end
    endfunction
    function [6:0] number_char;
        input [19:0] bcd;
        input negative,valid;
        input integer column;
        begin
            number_char=7'd32;
            if(column>=0 && column<6) begin
                if(!valid) number_char="-";
                else if(column==0) number_char=negative?"-":"+";
                else number_char=7'd48+((bcd>>((5-column)*4))&4'hf);
            end
        end
    endfunction
    reg [6:0] ascii;
    wire sidebar=global_x1>=928 && global_x1<1248;
    wire sidebar_font=sidebar && text_y1[10];
    wire [3:0] imu_row=imu_row1;
    wire [6:0] imu_column=imu_column1;
    wire [62:0] imu_fields;
    wire [6:0] imu_header;
    imu_nine_fields u_sidebar_fields(.text_column(imu_column1),.values_bcd(imu_bcd),
        .values_negative(imu_negative),.values_valid(imu_valid),.field_ascii(imu_fields));
    imu_nine_text u_sidebar_header(.text_row(4'd0),.text_column(imu_column1),
        .values_bcd(216'd0),.values_negative(9'd0),.values_valid(9'd0),.ascii(imu_header));
    reg [62:0] imu_fields2;
    reg [6:0] imu_header2;
    reg [3:0] imu_row2;
    reg imu_select2;
    reg [6:0] imu_ascii2;
    integer imu_field;
    always @*begin
        imu_ascii2=0;
        for(imu_field=0;imu_field<9;imu_field=imu_field+1)
            imu_ascii2=imu_ascii2|({7{imu_row2==imu_field+1}}&imu_fields2[imu_field*7+:7]);
        if(imu_row2>9)imu_ascii2=32;
        if(imu_row2==0)imu_ascii2=imu_header2;
    end
    reg [23:0] text_color,base_color;
    integer text_column,header_column;
    always @* begin
        ascii=32;text_color=TEXT;base_color=BG;
        if(global_x1<896 && x1>=16 && x1<432 && y1>=80 && y1<368) base_color=CARD;
        if(global_x1>=912 && global_x1<1264 && global_y1>=80 && global_y1<688) base_color=CARD;
        if(global_x1>=1008 && global_x1<1264 && global_y1>=12 && global_y1<64)
            base_color=display_online?24'he4f3ea:24'hfbe6e6;
        text_column=text_column1;
        header_column=header_column1;
        if(text_y1[0])
            ascii=message_char("FPGA MOTION CONTROL / FOUR-WHEEL SPEED TRACKING",47,header_column);
        else if(text_y1[1]) begin
            ascii=message_char("BLUE: TARGET   RED: ACTUAL   MOTOR UNITS   RANGE: -20..+20",58,header_column);
            text_color=MUTED;
        end else if(text_y1[2]) begin
            case(wheel1)
                0:ascii=message_char("LF / LEFT FRONT",15,text_column);
                1:ascii=message_char("RF / RIGHT FRONT",16,text_column);
                2:ascii=message_char("LB / LEFT BACK",14,text_column);
                3:ascii=message_char("RB / RIGHT BACK",15,text_column);
            endcase
        end else if(text_y1[3]) begin
            if(text_column<7) begin ascii=message_char("ACTUAL ",7,text_column);text_color=ACTUAL_RED;end
            else if(text_column<13) begin
                ascii=number_char(number_bcd[wheel1],number_negative[wheel1],display_online,text_column-7);
                text_color=ACTUAL_RED;
            end else if(text_column>=17 && text_column<24) begin
                ascii=message_char("TARGET ",7,text_column-17);text_color=BLUE;
            end else if(text_column>=24 && text_column<30) begin
                ascii=number_char(number_bcd[4+wheel1],number_negative[4+wheel1],1'b1,text_column-24);
                text_color=BLUE;
            end
        end else if(text_y1[4] && x1<72) ascii=message_char("+20",3,text_column);
        else if(text_y1[5] && x1<72) ascii=message_char("  0",3,text_column);
        else if(text_y1[6] && x1<72) ascii=message_char("-20",3,text_column);
        else if(text_y1[7]) begin
            if(x1>=80 && x1<128) ascii=message_char("-8.0 s",6,{27'd0,tick_left_column1});
            else if(x1>=224 && x1<272) ascii=message_char("-4.0 s",6,{27'd0,tick_middle_column1});
            else if(x1>=376 && x1<400) ascii=message_char("NOW",3,{27'd0,tick_now_column1});
            text_color=MUTED;
        end else if(text_y1[8]) begin
            ascii=message_char("SW3 ON: SPEED PAGE / OFF: RETURN   WINDOW 8.0 s   SAMPLE 100 ms   RANGE +/-20",77,header_column);
            text_color=MUTED;
        end
        if(plot1) begin
            if(y1==192 || y1==224 || y1==256 || y1==288 || y1==320 ||
               x1==80 || x1==160 || x1==240 || x1==320 || x1==400) base_color=GRID;
        end
        // One feedback badge replaces all four per-plot status lines.
        if(global_x1>=1024 && global_x1<1248 && text_y1[9]) begin
            ascii=display_online?message_char("FEEDBACK ONLINE",15,{27'd0,badge_column1}):
                message_char("FEEDBACK OFFLINE",16,{27'd0,badge_column1});
            text_color=display_online?24'h25834b:RED;
        end
        // Sidebar uses doubled native font, independent of the chart columns.
        if(sidebar && text_y1[10]) begin
            ascii=32;
            text_color=TEXT;
        end else if(global_x1>=896 && text_y1[12]) ascii=32;
    end
    // Six real pipeline stages: RAM -> wheel/delta -> interpolation ->
    // raster comparisons -> color -> output. No RAM-to-multiply-to-min/max
    // combinational path remains in a single 75 MHz cycle.
    wire font_pixel;
    // Retiming: use the old font3 delay slot BEFORE the synchronous font
    // ROM, so the long character/size decoder terminates at local FFs.
    // Font ROM is now stage3 and font4 still stage4: HDMI latency stays six.
    reg [6:0] font_ascii2 /* synthesis syn_preserve=1 */;
    reg [6:0] font_ascii3;reg [2:0] font_col3;reg [3:0] font_row3;
    reg [2:0] font_col2 /* synthesis syn_preserve=1 */;
    reg [3:0] font_row2 /* synthesis syn_preserve=1 */;
    always @(posedge clk)begin
        font_ascii2<=ascii;
        imu_fields2<=imu_fields;imu_header2<=imu_header;imu_row2<=imu_row1;
        imu_select2<=sidebar && text_y1[10] && text_y1[11];
        font_ascii3<=imu_select2?imu_ascii2:font_ascii2;
        font_col3<=font_col2;font_row3<=font_row2;
        font_col2<=sidebar_font?global_x1[3:1]:x1[2:0];
        font_row2<=sidebar_font?global_y1[4:1]:y1[3:0];
    end
    telemetry_font8x16 font(.clk(clk),.col(font_col3),.row(font_row3),.ascii(font_ascii3),.pixel(font_pixel));
    wire [7:0] actual_left=hist_left[wheel1*16+:8],actual_right=hist_right[wheel1*16+:8];
    wire [7:0] target_left=hist_left[wheel1*16+8+:8],target_right=hist_right[wheel1*16+8+:8];
    reg [7:0] actual_start2,target_start2;
    reg signed [8:0] actual_delta2,target_delta2;
    reg [1:0] fraction2;
    reg right_valid2,draw_actual2,draw_target2,stripe2;
    reg [23:0] base2,text2,base3,text3,base4,text4;
    reg [7:0] chart_y2,chart_y3;
    reg signed [10:0] actual_line3,actual_next3,target_line3,target_next3;
    reg draw_actual3,draw_target3,stripe3,font3;
    reg actual_hit4,target_hit4,stripe4,font4;
    reg [23:0] rgb5;
    reg [3:0] hs_pipe,vs_pipe,de_pipe,active_pipe;
    function signed [10:0] interpolate;
        input [7:0] origin;
        input signed [8:0] delta;
        input [2:0] fraction;
        reg signed [10:0] extended,product;
        begin
            extended={{2{delta[8]}},delta};
            case(fraction)
                0:product=0;
                1:product=extended;
                2:product=extended<<<1;
                3:product=extended+(extended<<<1);
                default:product=extended<<<2;
            endcase
            interpolate=$signed({3'b0,origin})+(product>>>2);
        end
    endfunction
    function raster_hit;
        input [7:0] ordinate;
        input signed [10:0] first,last;
        reg signed [10:0] point;
        begin
            point=$signed({3'b0,ordinate});
            raster_hit=(point>=first-11'sd1 && point<=last+11'sd1) ||
                       (point>=last-11'sd1 && point<=first+11'sd1);
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            actual_start2<=0;target_start2<=0;actual_delta2<=0;target_delta2<=0;fraction2<=0;
            right_valid2<=0;draw_actual2<=0;draw_target2<=0;stripe2<=0;chart_y2<=0;
            base2<=BG;text2<=TEXT;base3<=BG;text3<=TEXT;base4<=BG;text4<=TEXT;
            chart_y3<=0;actual_line3<=0;actual_next3<=0;target_line3<=0;target_next3<=0;
            draw_actual3<=0;draw_target3<=0;stripe3<=0;font3<=0;
            actual_hit4<=0;target_hit4<=0;stripe4<=0;font4<=0;
            rgb5<=0;rgb_out<=0;hs_pipe<=0;vs_pipe<=0;de_pipe<=0;active_pipe<=0;
            hs_out<=0;vs_out<=0;de_out<=0;active_out<=0;
        end else begin
            // Stage 2: select two endpoints and subtract; no interpolation.
            actual_start2<=actual_left;target_start2<=target_left;
            actual_delta2<=$signed({1'b0,actual_right})-$signed({1'b0,actual_left});
            target_delta2<=$signed({1'b0,target_right})-$signed({1'b0,target_left});
            fraction2<=fraction1;right_valid2<=right_valid1;
            draw_actual2<=plot1 && left_valid1 && hist_left[64] &&
                (fraction1==0 || (right_valid1 && hist_right[64]));
            draw_target2<=plot1 && left_valid1 && (fraction1==0 || right_valid1);
            stripe2<=x1[2];chart_y2<=y1-12'd192;base2<=base_color;text2<=text_color;
            // Stage 3: constant-factor shift/add interpolation, no DSP blocks.
            actual_line3<=interpolate(actual_start2,actual_delta2,{1'b0,fraction2});
            target_line3<=interpolate(target_start2,target_delta2,{1'b0,fraction2});
            actual_next3<=interpolate(actual_start2,actual_delta2,
                right_valid2?{1'b0,fraction2}+3'd1:{1'b0,fraction2});
            target_next3<=interpolate(target_start2,target_delta2,
                right_valid2?{1'b0,fraction2}+3'd1:{1'b0,fraction2});
            chart_y3<=chart_y2;draw_actual3<=draw_actual2;draw_target3<=draw_target2;
            stripe3<=stripe2;font3<=font_pixel;base3<=base2;text3<=text2;
            // Stage 4: compare independently of the interpolation adder chain.
            actual_hit4<=draw_actual3 && raster_hit(chart_y3,actual_line3,actual_next3);
            target_hit4<=draw_target3 && raster_hit(chart_y3,target_line3,target_next3);
            stripe4<=stripe3;font4<=font_pixel;base4<=base3;text4<=text3;
            // Stages 5 and 6: compose colors, then drive aligned HDMI output.
            rgb5<=font_pixel?text4:actual_hit4 && !(target_hit4 && stripe4)?ACTUAL_RED:target_hit4?BLUE:base4;
            rgb_out<=de_pipe[3]?rgb5:24'd0;
            hs_pipe<={hs_pipe[2:0],hs1};vs_pipe<={vs_pipe[2:0],vs1};
            de_pipe<={de_pipe[2:0],de1};active_pipe<={active_pipe[2:0],active1};
            hs_out<=hs_pipe[3];vs_out<=vs_pipe[3];de_out<=de_pipe[3];active_out<=active_pipe[3];
        end
    end
endmodule
