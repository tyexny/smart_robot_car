`timescale 1ns/1ps
// Final HDMI layer, independent of selected page. RGB and all sync signals
// acquire exactly one matching clock of latency, including the record tap.
module thermal_warning_overlay(
    input wire clk,reset_n,
    input wire [23:0] rgb_in,
    input wire hs_in,vs_in,de_in,
    input wire task_present,sensor_ok,alarm,
    input wire signed [15:0] maximum_centi,
    input wire [6:0] threshold_c,
    output wire [23:0] rgb_out,
    output reg hs_out,vs_out,de_out
);
    reg [11:0] x,y;
    reg de_previous,vs_previous;
    reg display_task,display_sensor,display_alarm,display_negative;
    reg [6:0] display_threshold;
    reg [15:0] number_shift;
    reg [19:0] number_work,number_bcd,number_adjusted;
    reg [4:0] format_step;
    integer digit;
    always @* begin
        number_adjusted=number_work;
        for(digit=0;digit<5;digit=digit+1)
            if(number_work[digit*4+:4]>=5)
                number_adjusted[digit*4+:4]=number_work[digit*4+:4]+4'd3;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            x<=0;y<=0;de_previous<=0;vs_previous<=0;
            display_task<=0;display_sensor<=0;display_alarm<=0;
            display_negative<=0;display_threshold<=40;
            number_shift<=0;number_work<=0;number_bcd<=0;format_step<=16;
        end else begin
            de_previous<=de_in;vs_previous<=vs_in;
            if(!de_in) x<=0;else x<=x+1'b1;
            if(!vs_in) y<=0;
            else if(!de_in && de_previous) y<=y+1'b1;
            // All fields are from one coherent CDC tuple. Apply only in
            // vertical blank; 16 formatting clocks finish before active video.
            if(vs_in && !vs_previous) begin
                display_task<=task_present;display_sensor<=sensor_ok;display_alarm<=alarm;
                display_negative<=maximum_centi<0;display_threshold<=threshold_c;
                number_shift<=maximum_centi<0 ? -maximum_centi : maximum_centi;
                number_work<=0;format_step<=0;
            end else if(format_step<16) begin
                number_work<={number_adjusted[18:0],number_shift[15]};
                number_shift<=number_shift<<1;
                if(format_step==15) number_bcd<={number_adjusted[18:0],number_shift[15]};
                format_step<=format_step+1'b1;
            end
        end
    end
    function [6:0] message_char;
        input [8*32-1:0] message;
        input integer length,column;
        begin
            message_char=32;
            if(column>=0 && column<length)
                message_char=message[(length-1-column)*8+:7];
        end
    endfunction
    // 32x64 px glyphs in a 576x320 panel at (16,16): exactly 20% of
    // the 1280x720 screen. Power-of-two scaling keeps division off the
    // 75 MHz pixel path. LIMIT deliberately remains absent.
    wire [1:0] line=y<96?0:y<168?1:y<240?2:3;
    wire text_region=x>=32 && x<544 && ((y>=32&&y<96)||(y>=104&&y<168)||(y>=176&&y<240)||(y>=248&&y<312));
    wire [11:0] text_x=x-12'd32;
    wire [11:0] text_y=y<96?y-12'd32:y<168?y-12'd104:y<240?y-12'd176:y-12'd248;
    wire [4:0] column=text_x[9:5];
    reg [6:0] ascii;
    always @* begin
        ascii=32;
        if(text_region) begin
            case(line)
                0: if(!display_sensor) ascii=message_char("THERMAL ERROR",13,column);
                   else if(display_alarm) ascii=message_char("WARNING",7,column);
                   else ascii=message_char("THERMAL MONITOR",15,column);
                1: if(!display_sensor) ascii=message_char("CHECK SENSOR",12,column);
                   else if(display_alarm) ascii=message_char("HIGH TEMPERATURE",16,column);
                   else ascii=message_char("TASK TEMPERATURE",16,column);
                2: if(!display_sensor) ascii=message_char("PI CONNECTION",13,column);
                   else if(display_alarm) ascii=message_char("SOURCE DETECTED",15,column);
                   else ascii=message_char("MONITORING",10,column);
                3: begin
                    case(column)
                        0:ascii="M";1:ascii="A";2:ascii="X";3:ascii=":";
                        5:ascii=!display_sensor ? "-" : display_negative ? "-" : "+";
                        6:ascii=display_sensor ? 7'd48+number_bcd[19:16] : "-";
                        7:ascii=display_sensor ? 7'd48+number_bcd[15:12] : "-";
                        8:ascii=display_sensor ? 7'd48+number_bcd[11:8] : "-";
                        9:ascii=".";
                        10:ascii=display_sensor ? 7'd48+number_bcd[7:4] : "-";
                        11:ascii=display_sensor ? 7'd48+number_bcd[3:0] : "-";
                        12:ascii="C";
                        default:ascii=32;
                    endcase
                end
                default:ascii=32;
            endcase
        end
    end
    wire font_pixel;
    telemetry_font8x16 u_font(.clk(clk),.col(text_x[4:2]),.row(text_y[5:2]),
        .ascii(ascii),.pixel(font_pixel));
    reg [23:0] rgb_delayed,background_delayed;
    reg box_delayed,text_delayed;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            rgb_delayed<=0;background_delayed<=0;box_delayed<=0;text_delayed<=0;
            hs_out<=0;vs_out<=0;de_out<=0;
        end else begin
            rgb_delayed<=rgb_in;hs_out<=hs_in;vs_out<=vs_in;de_out<=de_in;
            box_delayed<=de_in && display_task && x>=16 && x<592 && y>=16 && y<336;
            text_delayed<=text_region;
            background_delayed<=!display_sensor ? 24'h704000 : display_alarm ? 24'h980000 : 24'h002830;
        end
    end
    assign rgb_out=box_delayed ? (text_delayed && font_pixel ? 24'hffffff : background_delayed) : rgb_delayed;
endmodule
