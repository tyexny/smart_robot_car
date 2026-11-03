`timescale 1ns/1ps
// Display-only rolling history: body-edge LiDAR front distance (cm) and
// mean absolute measured wheel speed (motor protocol units, NOT cm/s).
// Separate plots: front 0..200 cm, speed 0..20 motor units.
// 128 points at 100 ms, sampled only in vertical blank. Six pixel stages.
module obstacle_curve_page #(
    parameter integer CLK_FREQ_HZ=75_000_000,
    parameter integer SAMPLE_MS=100,BLINK_MS=400
)(
    input wire clk,reset_n,frame_begin,active,
    input wire [11:0] pixel_x,pixel_y,
    input wire hs_in,vs_in,de_in,
    input wire [15:0] front_cm,
    input wire front_valid,motor_online,avoidance_enabled,
    input wire [63:0] actual_speeds,
    input wire [3:0] motion_command,avoidance_fault,
    input wire [5:0] avoidance_state,
    input wire [1:0] side_clear,
    output reg [23:0] rgb_out,
    output reg hs_out,vs_out,de_out,active_out
);
    localparam [23:0] BG=24'hffffff,TEXT=24'h102b52,MUTED=24'h78838b,
        GRID=24'he9edf1,BLUE=24'h0000ff,GREEN=24'h008800,
        RED=24'hdc0000,PALE_GREEN=24'he1f8dd,PALE_RED=24'hffdfdf,BORDER=24'h7e91a8;
    localparam integer SAMPLE_CLKS=(CLK_FREQ_HZ/1000)*SAMPLE_MS;
    localparam integer TICK_BITS=SAMPLE_CLKS<2?1:$clog2(SAMPLE_CLKS);
    reg [TICK_BITS-1:0] tick;
    reg sample_pending,display_active,display_front_valid,display_online,display_avoidance;
    localparam integer BLINK_CLKS=(CLK_FREQ_HZ/1000)*BLINK_MS<1?1:(CLK_FREQ_HZ/1000)*BLINK_MS;
    reg [31:0] blink_tick;
    reg blink_phase,display_blink;
    reg [3:0] display_motion,display_fault;
    reg [5:0] display_state;
    reg [1:0] display_side_clear;
    reg [6:0] write_index;
    reg [7:0] sample_count;
    // Two 8-bit ordinates and independent validity; never reset the RAM.
    reg [17:0] history[0:127];
    reg [17:0] hist_left,hist_right;
    function [15:0] magnitude;
        input [15:0] speed;
        begin magnitude=speed[15]?(~speed+16'd1):speed;end
    endfunction
    reg [16:0] pair_ab,pair_cd;
    reg [15:0] mean_speed;
    // Widen BEFORE adding, including abs(-32768). Divide by four using wires.
    wire [17:0] speed_sum={1'b0,pair_ab}+{1'b0,pair_cd};
    always @(posedge clk or negedge reset_n)
        if(!reset_n) begin pair_ab<=0;pair_cd<=0;mean_speed<=0;end
        else begin
            pair_ab<={1'b0,magnitude(actual_speeds[63:48])}+{1'b0,magnitude(actual_speeds[47:32])};
            pair_cd<={1'b0,magnitude(actual_speeds[31:16])}+{1'b0,magnitude(actual_speeds[15:0])};
            mean_speed<=speed_sum[17:2];
        end
    function [7:0] front_height;
        input [15:0] value;
        begin front_height=value>=200?8'd0:8'd200-value[7:0];end
    endfunction
    function [7:0] speed_height;
        input [15:0] value;
        // Ten pixels/unit, implemented as constant shifts and add.
        begin speed_height=value>=20?8'd0:8'd200-({value[4:0],3'b000}+{2'b00,value[4:0],1'b0});
        end
    endfunction
    wire sample_write=frame_begin && sample_pending;
    // Format full raw values at frame start; geometry clips only the curves.
    reg [19:0] number_bcd[0:1],number_work[0:1],adjusted[0:1];
    reg [15:0] number_shift[0:1];
    reg [4:0] format_step;
    integer j,digit,n;
    always @* begin
        for(j=0;j<2;j=j+1) begin
            adjusted[j]=number_work[j];
            for(digit=0;digit<5;digit=digit+1)
                if(number_work[j][digit*4+:4]>=5)
                    adjusted[j][digit*4+:4]=number_work[j][digit*4+:4]+4'd3;
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            tick<=0;sample_pending<=0;write_index<=0;sample_count<=0;
            blink_tick<=0;blink_phase<=1;display_blink<=1;
            display_motion<=0;display_fault<=0;display_state<=0;display_side_clear<=0;
            display_active<=0;display_front_valid<=0;display_online<=0;display_avoidance<=0;format_step<=16;
            for(n=0;n<2;n=n+1) begin number_bcd[n]<=0;number_work[n]<=0;number_shift[n]<=0;end
        end else begin
            if(blink_tick==BLINK_CLKS-1)begin blink_tick<=0;blink_phase<=!blink_phase;end
            else blink_tick<=blink_tick+1'b1;
            if(tick==SAMPLE_CLKS-1) begin tick<=0;sample_pending<=1;end
            else tick<=tick+1'b1;
            if(sample_write) begin
                // A coincident tick must remain pending for the next blank.
                sample_pending<=tick==SAMPLE_CLKS-1;
                write_index<=write_index+1'b1;
                if(sample_count<128) sample_count<=sample_count+1'b1;
            end
            if(frame_begin) begin
                display_active<=active;display_front_valid<=front_valid;
                display_online<=motor_online;display_avoidance<=avoidance_enabled;
                display_motion<=motion_command;display_fault<=avoidance_fault;
                display_state<=avoidance_state;display_side_clear<=side_clear;display_blink<=blink_phase;
                number_shift[0]<=front_cm;number_shift[1]<=mean_speed;
                number_work[0]<=0;number_work[1]<=0;format_step<=0;
            end else if(format_step<16) begin
                for(n=0;n<2;n=n+1) begin
                    number_work[n]<={adjusted[n][18:0],number_shift[n][15]};
                    number_shift[n]<=number_shift[n]<<1;
                    if(format_step==15) number_bcd[n]<={adjusted[n][18:0],number_shift[n][15]};
                end
                format_step<=format_step+1'b1;
            end
        end
    end
    always @(posedge clk) if(sample_write)
        history[write_index]<={front_valid,motor_online,front_height(front_cm),speed_height(mean_speed)};

    // Both plots x=128..1144: 127 intervals of 8 pixels.
    // Same 128 samples, 100ms clock and x-axis; only display height changes.
    // Distance y=368..496, speed y=560..688, both 128 pixels high.
    wire [11:0] plot_x=pixel_x-12'd128;
    wire [6:0] column_index=plot_x[9:3];
    wire [6:0] read_left=write_index+column_index,read_right=read_left+7'd1;
    reg [11:0] x1,y1;
    reg [12:0] diagonal_sum1;
    reg signed [12:0] diagonal_difference1;
    reg [13:0] double_x_sum1;
    reg signed [13:0] double_x_difference1;
    always @(posedge clk)begin
        diagonal_sum1<={1'b0,pixel_x}+{1'b0,pixel_y};
        diagonal_difference1<=$signed({1'b0,pixel_x})-$signed({1'b0,pixel_y});
        double_x_sum1<={1'b0,pixel_x,1'b0}+{2'd0,pixel_y};
        double_x_difference1<=($signed({2'd0,pixel_x})<<<1)-$signed({2'd0,pixel_y});
    end
    reg [7:0] chart_y1;
    reg speed_plot1;
    reg [2:0] fraction1;
    reg plot1,left_valid1,right_valid1,hs1,vs1,de1,active1;
    always @(posedge clk) begin hist_left<=history[read_left];hist_right<=history[read_right];end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            x1<=0;y1<=0;chart_y1<=0;speed_plot1<=0;fraction1<=0;plot1<=0;left_valid1<=0;right_valid1<=0;
            hs1<=0;vs1<=0;de1<=0;active1<=0;
        end else begin
            x1<=pixel_x;y1<=pixel_y;fraction1<=plot_x[2:0];
            speed_plot1<=pixel_y>=528;
            chart_y1<=pixel_y>=528?pixel_y-12'd560:pixel_y-12'd368;
            plot1<=pixel_x>=128 && pixel_x<=1144 &&
                ((pixel_y>=368 && pixel_y<=496) || (pixel_y>=560 && pixel_y<=688));
            left_valid1<=sample_count!=0 && {1'b0,column_index}>=8'd128-sample_count;
            right_valid1<=column_index!=127 && {1'b0,column_index}+8'd1>=8'd128-sample_count;
            hs1<=hs_in;vs1<=vs_in;de1<=de_in;active1<=display_active;
        end
    end
    function [6:0] message_char;
        input [8*128-1:0] message;
        input [7:0] length;
        input [11:0] column;
        reg [10:0] offset;
        begin
            offset=({3'd0,length}-11'd1-{3'd0,column[7:0]})<<3;
            message_char=32;
            if(column<length) message_char=message[offset+:7];
        end
    endfunction
    function [6:0] number_char;
        input [19:0] bcd;
        input valid;
        input [11:0] column;
        begin
            number_char=32;
            if(column<5)begin
                if(!valid)number_char=45;
                else case(column)
                    0:number_char=7'd48+{3'd0,bcd[19:16]};
                    1:number_char=7'd48+{3'd0,bcd[15:12]};
                    2:number_char=7'd48+{3'd0,bcd[11:8]};
                    3:number_char=7'd48+{3'd0,bcd[7:4]};
                    4:number_char=7'd48+{3'd0,bcd[3:0]};
                    default:;
                endcase
            end
        end
    endfunction
    reg [255:0] glyph_message;
    reg [7:0] glyph_length;
    reg [11:0] glyph_column;
    reg glyph_numeric,glyph_number_valid;
    reg [19:0] glyph_number;
    task select_text;
        input [1023:0] message;
        input [7:0] length;
        input [11:0] column;
        begin glyph_message=message[255:0];glyph_length=length;
            glyph_column=column;glyph_numeric=0;end
    endtask
    task select_number;
        input [19:0] value;
        input valid;
        input [11:0] column;
        begin glyph_number=value;glyph_number_valid=valid;
            glyph_column=column;glyph_numeric=1;end
    endtask
    reg [23:0] base_color,text_color;
    reg [2:0] font_col;
    reg [3:0] font_row;
    reg [8*128-1:0] alert_message,move_message;
    reg [7:0] alert_length,move_length;
    reg [11:0] alert_x,col,gx,gy,origin_x,origin_y;
    reg [2:0] scale;
    reg signed [12:0] warning_dx,warning_dy;
    reg alert_red;
    reg [1023:0] alert_message_q,move_message_q;
    reg [7:0] alert_length_q,move_length_q;
    reg [11:0] alert_x_q;
    reg alert_red_q;
    reg [9:0] text_region1;
    // All ten visible text regions are disjoint. Decode their bounds beside
    // x1/y1, so the glyph stage selects in parallel rather than traversing
    // the full priority chain before subtracting the selected origin.
    always @(posedge clk)begin
        text_region1[0]<=(pixel_x>=24&&pixel_x<720&&pixel_y>=8&&pixel_y<56);
        text_region1[1]<=(pixel_x>=888&&pixel_x<1176&&pixel_y>=16&&pixel_y<48);
        text_region1[2]<=(pixel_x>=1192&&pixel_y>=8&&pixel_y<56);
        text_region1[3]<=(pixel_y>=143&&pixel_y<191 && ((pixel_x>=83&&pixel_x<179)||(pixel_x>=1089&&pixel_x<1209)));
        text_region1[4]<=(pixel_y>=208&&pixel_y<256&&(pixel_x<258||pixel_x>=1024));
        text_region1[5]<=(pixel_x>=alert_x_q&&pixel_x<alert_x_q+alert_length_q*32&&pixel_y>=101&&pixel_y<165);
        text_region1[6]<=(pixel_x>=464&&pixel_x<464+move_length_q*32&&pixel_y>=220&&pixel_y<284);
        text_region1[7]<=(pixel_x>=94&&((pixel_y>=338&&pixel_y<370)||(pixel_y>=534&&pixel_y<566)));
        text_region1[8]<=((pixel_y>=500&&pixel_y<516)||(pixel_y>=692&&pixel_y<708));
        text_region1[9]<=((pixel_x>=38&&pixel_x<62)&&((pixel_y>=360&&pixel_y<376)||(pixel_y>=392&&pixel_y<408)||(pixel_y>=424&&pixel_y<440)||(pixel_y>=456&&pixel_y<472)||(pixel_y>=488&&pixel_y<504)||(pixel_y>=552&&pixel_y<568)||(pixel_y>=584&&pixel_y<600)||(pixel_y>=616&&pixel_y<632)||(pixel_y>=648&&pixel_y<664)||(pixel_y>=680&&pixel_y<696))&&!((pixel_y>=500&&pixel_y<516)||(pixel_y>=692&&pixel_y<708)));
    end
    // Frame-frozen status is decoded during blanking, before raster use.
    always @(posedge clk)begin
        alert_message_q<=alert_message;move_message_q<=move_message;
        alert_length_q<=alert_length;move_length_q<=move_length;
        alert_x_q<=alert_x;alert_red_q<=alert_red;
    end
    function panel_inside;
        input [11:0] x,y,x0,y0,xm,ym,cut;
        begin panel_inside=x>=x0 && x<=xm && y>=y0 && y<=ym &&
            diagonal_sum1>=({1'b0,x0}+{1'b0,y0}+{1'b0,cut}) &&
            diagonal_sum1<=({1'b0,xm}+{1'b0,ym}-{1'b0,cut}) &&
            diagonal_difference1<=($signed({1'b0,xm})-$signed({1'b0,y0})-$signed({1'b0,cut})) &&
            diagonal_difference1>=($signed({1'b0,x0})-$signed({1'b0,ym})+$signed({1'b0,cut}));end
    endfunction
    function direction_arrow;
        input [11:0] x,y;
        input [3:0] direction;
        reg signed [12:0] dx,dy,a,b;
        begin
            dx=0;dy=0;a=0;b=0;direction_arrow=0;
            case(direction)
              3,5:direction_arrow=y>=228 && y<=288 && (y>=258 ?
                ((diagonal_difference1>=58&&diagonal_difference1<=70)||(diagonal_difference1>=110&&diagonal_difference1<=122)) :
                ((diagonal_sum1>=574&&diagonal_sum1<=586)||(diagonal_sum1>=626&&diagonal_sum1<=638)));
              4,6:direction_arrow=y>=228 && y<=288 && (y>=258 ?
                ((diagonal_sum1>=598&&diagonal_sum1<=610)||(diagonal_sum1>=650&&diagonal_sum1<=662)) :
                ((diagonal_difference1>=82&&diagonal_difference1<=94)||(diagonal_difference1>=134&&diagonal_difference1<=146)));
              1:direction_arrow=x>=332 && x<=388 && (x>=360 ?
                diagonal_difference1>=111&&diagonal_difference1<=123 : diagonal_sum1>=597&&diagonal_sum1<=609);
              2:direction_arrow=x>=332 && x<=388 && (x>=360 ?
                diagonal_sum1>=627&&diagonal_sum1<=639 : diagonal_difference1>=81&&diagonal_difference1<=93);
              default:begin end
            endcase
        end
    endfunction
    // Register independent background regions in S2. Resolve the nested
    // shapes inside each disjoint panel in S3, reusing its former base2->base3
    // delay. A pixel coordinate no longer traverses every panel override.
    wire [15:0] background_regions;
    assign background_regions[0]=y1<72;
    assign background_regions[1]=panel_inside(x1,y1,6,124,256,284,12);
    assign background_regions[2]=panel_inside(x1,y1,10,128,252,280,10);
    assign background_regions[3]=panel_inside(x1,y1,1024,124,1274,284,12);
    assign background_regions[4]=panel_inside(x1,y1,1028,128,1270,280,10);
    assign background_regions[5]=panel_inside(x1,y1,270,84,1008,190,12);
    assign background_regions[6]=!alert_red_q&&panel_inside(x1,y1,274,88,1004,186,10);
    assign background_regions[7]=alert_red_q&&y1>=98&&y1<=174&&(x1>=350?double_x_difference1<=14'sd602:double_x_sum1>=14'd798);
    assign background_regions[8]=alert_red_q&&y1>=113&&y1<=166&&(x1>=350?double_x_difference1<=14'sd587:double_x_sum1>=14'd813);
    assign background_regions[9]=alert_red_q&&x1>=346&&x1<=354&&((y1>=124&&y1<=146)||(y1>=154&&y1<=162));
    assign background_regions[10]=panel_inside(x1,y1,270,202,1008,314,12);
    assign background_regions[11]=panel_inside(x1,y1,274,206,1004,310,10);
    assign background_regions[12]=display_blink&&direction_arrow(x1,y1,display_motion);
    assign background_regions[13]=panel_inside(x1,y1,6,332,1274,524,9)||panel_inside(x1,y1,6,528,1274,718,9);
    assign background_regions[14]=panel_inside(x1,y1,9,335,1271,521,7)||panel_inside(x1,y1,9,531,1271,715,7);
    assign background_regions[15]=plot1&&(chart_y1==0||chart_y1==32||chart_y1==64||chart_y1==96||chart_y1==128)&&x1[2:0]<4;
    reg [15:0] background_regions2;
    reg [3:0] background_status2;
    wire bg_plot=|background_regions2[15:13],bg_move=|background_regions2[12:10];
    wire bg_alert=|background_regions2[9:5],bg_right=|background_regions2[4:3],bg_left=|background_regions2[2:1];
    wire [23:0] plot_background=background_regions2[15]?GRID:background_regions2[14]?BG:BORDER;
    wire [23:0] move_background=background_regions2[12]?GREEN:background_regions2[11]?BG:background_status2[3]?RED:GREEN;
    wire [23:0] alert_background=background_regions2[9]?BG:background_regions2[8]?RED:background_regions2[7]?BG:
        background_regions2[6]?PALE_GREEN:background_status2[2]?RED:GREEN;
    wire [23:0] left_background=background_regions2[2]?(background_status2[0]?PALE_GREEN:PALE_RED):(background_status2[0]?GREEN:RED);
    wire [23:0] right_background=background_regions2[4]?(background_status2[1]?PALE_GREEN:PALE_RED):(background_status2[1]?GREEN:RED);
    wire [23:0] resolved_background3=(bg_plot?plot_background:24'd0)|(bg_move?move_background:24'd0)|
        (bg_alert?alert_background:24'd0)|(bg_right?right_background:24'd0)|(bg_left?left_background:24'd0)|
        (background_regions2[0]?TEXT:24'd0)|((|background_regions2)?24'd0:BG);
    always @* begin
        glyph_message=0;glyph_length=0;glyph_column=0;glyph_numeric=0;
        glyph_number=0;glyph_number_valid=0;base_color=BG;text_color=TEXT;col=0;gx=0;gy=0;
        scale=1;origin_x=0;origin_y=0;font_col=0;font_row=0;
        alert_message="PATH CLEAR";alert_length=10;alert_red=0;
        if(!display_avoidance)begin alert_message="AVOIDANCE OFF";alert_length=13;end
        else if(display_fault!=0)begin
            alert_red=1;
            if(display_fault==9)begin alert_message="NO CLEAR PATH";alert_length=13;end
            else begin alert_message="SENSOR WAIT";alert_length=11;end
        end else if(display_state==39)begin alert_message="RETURN TO LINE";alert_length=14;end
        else if(display_state==38)begin alert_message="PASSING OBSTACLE";alert_length=16;end
        else if(display_state==35 || (display_front_valid && number_bcd[0]<=20'h00038))begin
            alert_message="OBSTACLE AHEAD";alert_length=14;alert_red=1;
        end else if(display_motion==0)begin alert_message="STOPPED";alert_length=7;end
        move_message="STOP";move_length=4;
        case(display_motion)
          1:begin move_message="FORWARD";move_length=7;end
          2:begin move_message="BACKWARD";move_length=8;end
          3:if(display_state==39)begin move_message="RETURN LEFT";move_length=11;end
            else begin move_message="MOVE LEFT";move_length=9;end
          4:if(display_state==39)begin move_message="RETURN RIGHT";move_length=12;end
            else begin move_message="MOVE RIGHT";move_length=10;end
          5:begin move_message="TURN LEFT";move_length=9;end
          6:begin move_message="TURN RIGHT";move_length=10;end
          default:begin end
        endcase
        alert_x=(1278-alert_length*32)/2;
        warning_dx=x1>350?x1-350:350-x1;warning_dy=y1-98;
        if(y1<72)base_color=TEXT;
        if(panel_inside(x1,y1,6,124,256,284,12))base_color=display_side_clear[0]?GREEN:RED;
        if(panel_inside(x1,y1,10,128,252,280,10))base_color=display_side_clear[0]?PALE_GREEN:PALE_RED;
        if(panel_inside(x1,y1,1024,124,1274,284,12))base_color=display_side_clear[1]?GREEN:RED;
        if(panel_inside(x1,y1,1028,128,1270,280,10))base_color=display_side_clear[1]?PALE_GREEN:PALE_RED;
        if(panel_inside(x1,y1,270,84,1008,190,12))base_color=alert_red_q?RED:GREEN;
        if(!alert_red_q && panel_inside(x1,y1,274,88,1004,186,10))base_color=PALE_GREEN;
        if(alert_red_q && y1>=98&&y1<=174 && (x1>=350 ? double_x_difference1<=14'sd602 : double_x_sum1>=14'd798))base_color=BG;
        if(alert_red_q && y1>=113&&y1<=166 && (x1>=350 ? double_x_difference1<=14'sd587 : double_x_sum1>=14'd813))base_color=RED;
        if(alert_red_q&&x1>=346&&x1<=354&&((y1>=124&&y1<=146)||(y1>=154&&y1<=162)))base_color=BG;
        if(panel_inside(x1,y1,270,202,1008,314,12))base_color=display_motion==0?RED:GREEN;
        if(panel_inside(x1,y1,274,206,1004,310,10))base_color=BG;
        if(display_blink && direction_arrow(x1,y1,display_motion))base_color=GREEN;
        if(panel_inside(x1,y1,6,332,1274,524,9)||panel_inside(x1,y1,6,528,1274,718,9))base_color=BORDER;
        if(panel_inside(x1,y1,9,335,1271,521,7)||panel_inside(x1,y1,9,531,1271,715,7))base_color=BG;
        if(plot1 && ((chart_y1==0||chart_y1==32||chart_y1==64||chart_y1==96||chart_y1==128)&&x1[2:0]<4))base_color=GRID;
        // Text uses the original synchronous 8x16 ROM at integer scale.
        // All selector fields and blink state are frozen at vertical blank.
        case(text_region1)
10'b0000000001:begin
            origin_x=24;origin_y=8;scale=3;text_color=BG;
            select_text("TASK 3 : OBSTACLE AVOIDANCE",27,(x1-12'd24)/12'd24);
        end
          10'b0000000010:begin
            origin_x=888;origin_y=16;scale=2;text_color=BG;
            select_text("| AVOIDANCE:",12,(x1-12'd888)/12'd16);
        end
          10'b0000000100:begin
            origin_x=1192;origin_y=8;scale=3;text_color=display_avoidance?24'h00ee00:24'hff6060;
            if(display_avoidance)select_text("ON",2,(x1-12'd1192)/12'd24);else select_text("OFF",3,(x1-12'd1192)/12'd24);
        end
          10'b0000001000:begin
            origin_x=x1<270?83:1089;origin_y=143;scale=3;
            if(x1<270)select_text("LEFT",4,(x1-12'd83)/12'd24);else select_text("RIGHT",5,(x1-12'd1089)/12'd24);
        end
          10'b0000010000:begin
            if(x1<258)begin origin_x=display_side_clear[0]?71:47;text_color=display_side_clear[0]?GREEN:RED;
                if(display_side_clear[0])select_text("CLEAR",5,(x1-origin_x)/12'd24);else select_text("BLOCKED",7,(x1-origin_x)/12'd24);end
            else begin origin_x=display_side_clear[1]?1089:1065;text_color=display_side_clear[1]?GREEN:RED;
                if(display_side_clear[1])select_text("CLEAR",5,(x1-origin_x)/12'd24);else select_text("BLOCKED",7,(x1-origin_x)/12'd24);end
            origin_y=208;scale=3;
        end
          10'b0000100000:begin
            origin_x=alert_x_q;origin_y=101;scale=4;text_color=alert_red_q?BG:GREEN;
            select_text(alert_message_q,alert_length_q,(x1-alert_x_q)/12'd32);
        end
          10'b0001000000:begin
            origin_x=464;origin_y=220;scale=4;text_color=display_motion==0?RED:GREEN;
            select_text(move_message_q,move_length_q,(x1-12'd464)/12'd32);
        end
          10'b0010000000:begin
            origin_y=y1<528?338:534;scale=2;text_color=speed_plot1?GREEN:BLUE;
            if(x1>=94&&x1<900)begin origin_x=94;
                if(speed_plot1)select_text("MEAN WHEEL SPEED (motor units)",30,(x1-12'd94)/12'd16);else select_text("FRONT DISTANCE (cm)",19,(x1-12'd94)/12'd16);end
            else if(x1>=1038&&x1<1102)begin origin_x=1038;select_text("NOW:",4,(x1-12'd1038)/12'd16);end
            else if(x1>=1102&&x1<1182)begin origin_x=1102;
                select_number(number_bcd[speed_plot1],speed_plot1?display_online:display_front_valid,(x1-12'd1102)/12'd16);end
        end
          10'b0100000000:begin
            origin_y=y1<528?500:692;text_color=MUTED;
            if(x1>=128&&x1<184)begin origin_x=128;select_text("-12.7 s",7,(x1-12'd128)/12'd8);end
            else if(x1>=608&&x1<656)begin origin_x=608;select_text("-6.4 s",6,(x1-12'd608)/12'd8);end
            else if(x1>=1120&&x1<1144)begin origin_x=1120;select_text("NOW",3,(x1-12'd1120)/12'd8);end
        end
          10'b1000000000:begin
            origin_x=38;scale=1;
            if(y1>=360&&y1<376)begin origin_y=360;select_text("200",3,(x1-12'd38)/12'd8);end
            else if(y1>=392&&y1<408)begin origin_y=392;select_text("150",3,(x1-12'd38)/12'd8);end
            else if(y1>=424&&y1<440)begin origin_y=424;select_text("100",3,(x1-12'd38)/12'd8);end
            else if(y1>=456&&y1<472)begin origin_y=456;select_text(" 50",3,(x1-12'd38)/12'd8);end
            else if(y1>=488&&y1<504)begin origin_y=488;select_text("  0",3,(x1-12'd38)/12'd8);end
            else if(y1>=552&&y1<568)begin origin_y=552;select_text(" 20",3,(x1-12'd38)/12'd8);end
            else if(y1>=584&&y1<600)begin origin_y=584;select_text(" 15",3,(x1-12'd38)/12'd8);end
            else if(y1>=616&&y1<632)begin origin_y=616;select_text(" 10",3,(x1-12'd38)/12'd8);end
            else if(y1>=648&&y1<664)begin origin_y=648;select_text("  5",3,(x1-12'd38)/12'd8);end
            else if(y1>=680&&y1<696)begin origin_y=680;select_text("  0",3,(x1-12'd38)/12'd8);end
        end
          default:begin end
        endcase
        gx=x1-origin_x;gy=y1-origin_y;
    end
    wire font_pixel;
    // Origin selection/subtraction and division occupy separate stages.
    // Move the former post-ROM font4 register before the ROM, retaining
    // exactly six clocks of pixel/sync latency and identical glyphs.
    reg [6:0] font_ascii3;
    reg [255:0] glyph_message2;
    reg [7:0] glyph_length2;
    reg [11:0] glyph_column2;
    reg glyph_numeric2,glyph_number_valid2;
    reg [19:0] glyph_number2;
    reg [11:0] glyph_x2,glyph_y2;
    reg [2:0] glyph_scale2,font_col3;
    reg [3:0] font_row3;
    reg [11:0] scaled_glyph_x,scaled_glyph_y;
    always @*begin
        case(glyph_scale2)
          2:begin scaled_glyph_x=glyph_x2>>1;scaled_glyph_y=glyph_y2>>1;end
          3:begin scaled_glyph_x=glyph_x2/12'd3;scaled_glyph_y=glyph_y2/12'd3;end
          4:begin scaled_glyph_x=glyph_x2>>2;scaled_glyph_y=glyph_y2>>2;end
          default:begin scaled_glyph_x=glyph_x2;scaled_glyph_y=glyph_y2;end
        endcase
    end
    always @(posedge clk)begin
        glyph_message2<=glyph_message;glyph_length2<=glyph_length;glyph_column2<=glyph_column;
        glyph_numeric2<=glyph_numeric;glyph_number_valid2<=glyph_number_valid;glyph_number2<=glyph_number;
        glyph_x2<=gx;glyph_y2<=gy;glyph_scale2<=scale;
        font_ascii3<=glyph_numeric2?number_char(glyph_number2,glyph_number_valid2,glyph_column2):
            message_char({768'd0,glyph_message2},glyph_length2,glyph_column2);font_col3<=scaled_glyph_x[2:0];font_row3<=scaled_glyph_y[3:0];
    end
    telemetry_font8x16 font(.clk(clk),.col(font_col3),.row(font_row3),.ascii(font_ascii3),.pixel(font_pixel));
    wire [7:0] raw_left=speed_plot1?hist_left[7:0]:hist_left[15:8];
    wire [7:0] raw_right=speed_plot1?hist_right[7:0]:hist_right[15:8];
    // Scale stored 0..200 ordinates to 0..128 without changing RAM samples.
    wire [15:0] scaled_left=raw_left*16'd164,scaled_right=raw_right*16'd164;
    wire [7:0] selected_left=scaled_left[15:8],selected_right=scaled_right[15:8];
    wire selected_valid_left=speed_plot1?hist_left[16]:hist_left[17];
    wire selected_valid_right=speed_plot1?hist_right[16]:hist_right[17];
    reg [7:0] start2,chart_y2,chart_y3;
    reg signed [8:0] delta2;
    reg [2:0] fraction2;
    reg right_valid2,draw2,speed_plot2;
    reg signed [11:0] line3,next3;
    reg draw3,speed_plot3,hit4,speed_plot4,font4;
    wire font3=font_pixel;
    reg [23:0] base2,text2,base3,text3,base4,text4,rgb5;
    reg [3:0] hs_pipe,vs_pipe,de_pipe,active_pipe;
    function signed [11:0] interpolate;
        input [7:0] origin;
        input signed [8:0] delta;
        input [3:0] fraction;
        reg signed [11:0] extended,product;
        begin
            extended={{3{delta[8]}},delta};
            // Parallel constant shifts/adds, at most one addition per choice.
            case(fraction)
                0:product=0;
                1:product=extended;
                2:product=extended<<<1;
                3:product=extended+(extended<<<1);
                4:product=extended<<<2;
                5:product=extended+(extended<<<2);
                6:product=(extended<<<1)+(extended<<<2);
                7:product=(extended<<<3)-extended;
                default:product=extended<<<3;
            endcase
            interpolate=$signed({4'd0,origin})+(product>>>3);
        end
    endfunction
    function raster_hit;
        input [7:0] ordinate;
        input signed [11:0] first,last;
        reg signed [11:0] point;
        begin
            point=$signed({4'd0,ordinate});
            raster_hit=(point>=first-12'sd1 && point<=last+12'sd1) ||
                       (point>=last-12'sd1 && point<=first+12'sd1);
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            start2<=0;delta2<=0;fraction2<=0;
            right_valid2<=0;draw2<=0;speed_plot2<=0;chart_y2<=0;chart_y3<=0;
            line3<=0;next3<=0;draw3<=0;speed_plot3<=0;
            hit4<=0;speed_plot4<=0;font4<=0;
            base2<=BG;text2<=TEXT;base3<=BG;text3<=TEXT;base4<=BG;text4<=TEXT;rgb5<=0;rgb_out<=0;
            background_regions2<=0;background_status2<=0;
            hs_pipe<=0;vs_pipe<=0;de_pipe<=0;active_pipe<=0;hs_out<=0;vs_out<=0;de_out<=0;active_out<=0;
        end else begin
            start2<=selected_left;
            delta2<=$signed({1'b0,selected_right})-$signed({1'b0,selected_left});
            fraction2<=fraction1;right_valid2<=right_valid1 && selected_valid_right;
            draw2<=plot1 && left_valid1 && selected_valid_left && (fraction1==0 || (right_valid1 && selected_valid_right));
            speed_plot2<=speed_plot1;chart_y2<=chart_y1;base2<=base_color;text2<=text_color;
            background_regions2<=background_regions;
            background_status2<={display_motion==0,alert_red_q,display_side_clear};
            line3<=interpolate(start2,delta2,{1'b0,fraction2});
            next3<=interpolate(start2,delta2,right_valid2?{1'b0,fraction2}+4'd1:{1'b0,fraction2});
            chart_y3<=chart_y2;draw3<=draw2;
            speed_plot3<=speed_plot2;base3<=resolved_background3;text3<=text2;
            hit4<=draw3 && raster_hit(chart_y3,line3,next3);
            speed_plot4<=speed_plot3;font4<=font3;base4<=base3;text4<=text3;
            rgb5<=font_pixel?text4:hit4?(speed_plot4?GREEN:BLUE):base4;
            rgb_out<=de_pipe[3]?rgb5:24'd0;
            hs_pipe<={hs_pipe[2:0],hs1};vs_pipe<={vs_pipe[2:0],vs1};
            de_pipe<={de_pipe[2:0],de1};active_pipe<={active_pipe[2:0],active1};
            hs_out<=hs_pipe[3];vs_out<=vs_pipe[3];de_out<=de_pipe[3];active_out<=active_pipe[3];
        end
    end
endmodule
