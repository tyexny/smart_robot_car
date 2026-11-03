`timescale 1ns/1ps
// 1280x720 scan/navigation page. Pixel/sync inputs are registered first;
// map_rd_addr is combinational from that registered coordinate. Both external
// RAMs sample it one clock after the raw inputs and return data a clock later.
// RGB, HS, VS, DE and active_out use six registers. frame_begin stays raw so
// metadata snapshots agree with the top-level path-bank frame snapshot.
// RAM address {map_y,map_x}; +X is screen-up and +Y is screen-left.
// heading_tenths is relative to the map's initial forward direction, wrapped
// to [-1800,1800]. Metadata and page selection are latched at frame_begin.
module scan_map_page #(parameter integer CELL_SHIFT=4,
    parameter integer CELL_MM=(1<<CELL_SHIFT), GRID_SIZE=128, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y), COMPACT_ADDRESS=0,
    parameter integer ALLOW_UNKNOWN_PATH=0,
    parameter integer ENABLE_MAPPING_DIAGNOSTICS=0) (
    input wire clk, reset_n, frame_begin,
    input wire [11:0] pixel_x, pixel_y,
    input wire hs_in, vs_in, de_in, active,
    input wire [3:0] status,
    input wire [3:0] reject_reason,
    input wire [7:0] diagnostic_points, diagnostic_hits, diagnostic_support,
    input wire [3:0] auto_fault,
    input wire stop_confirmed,
    input wire [63:0] motor_speeds,
    input wire motor_online,
    input wire [3:0] stop_wait_reason,
    input wire signed [31:0] heading_tenths,
    input wire [15:0] distance_mm,
    input wire distance_valid,
    input wire [3:0] nav_status, nav_fault,
    input wire [GRID_BITS-1:0] goal_x_cell, goal_y_cell,
    input wire goal_valid, pose_valid, path_valid,
    input wire signed [15:0] pose_x_mm, pose_y_mm,
    input wire path_rd_data,
    input wire [1:0] map_rd_data,
    output wire [2*GRID_BITS-1:0] map_rd_addr,
    // Same address one pixel clock earlier, for memory-local replicas of
    // the address register. The path-overlay RAM keeps map_rd_addr.
    output wire [2*GRID_BITS-1:0] map_rd_addr_raw,
    output reg [23:0] rgb_out,
    output reg hs_out, vs_out, de_out, active_out
);
    reg display_stop_confirmed;

    function [6:0] mapping_control_char;
        input [3:0] code;
        input [4:0] position;
        begin
            case({code,position})
                9'd193, 9'd225:mapping_control_char=7'd65;
                9'd197:mapping_control_char=7'd68;
                9'd196:mapping_control_char=7'd69;
                9'd230:mapping_control_char=7'd71;
                9'd228:mapping_control_char=7'd73;
                9'd229:mapping_control_char=7'd78;
                9'd192, 9'd224:mapping_control_char=7'd80;
                9'd195, 9'd227:mapping_control_char=7'd83;
                9'd194, 9'd226:mapping_control_char=7'd85;
                default:mapping_control_char=7'd32;
            endcase
        end
    endfunction
    function [6:0] slam_status_char;
        input [3:0] s;
        input [4:0] position;
        begin
            case({s,position})
                9'd416:slam_status_char=7'd77;
                9'd417:slam_status_char=7'd65;
                9'd418:slam_status_char=7'd84;
                9'd419:slam_status_char=7'd67;
                9'd420:slam_status_char=7'd72;
                9'd421:slam_status_char=7'd73;
                9'd422:slam_status_char=7'd78;
                9'd423:slam_status_char=7'd71;
                9'd448:slam_status_char=7'd83;
                9'd449:slam_status_char=7'd67;
                9'd450:slam_status_char=7'd65;
                9'd451:slam_status_char=7'd78;
                9'd453:slam_status_char=7'd83;
                9'd454:slam_status_char=7'd75;
                9'd455:slam_status_char=7'd73;
                9'd456:slam_status_char=7'd80;
                9'd480:slam_status_char=7'd83;
                9'd481:slam_status_char=7'd69;
                9'd482:slam_status_char=7'd69;
                9'd483:slam_status_char=7'd68;
                9'd485:slam_status_char=7'd83;
                9'd486:slam_status_char=7'd84;
                9'd487:slam_status_char=7'd79;
                9'd488:slam_status_char=7'd80;
                default:slam_status_char=7'd32;
            endcase
        end
    endfunction
    function [6:0] status_char;
        input [3:0] s;
        input [4:0] position;
        begin
            if(ENABLE_MAPPING_DIAGNOSTICS && (s==6 || s==7)) status_char=mapping_control_char(s,position);
            else if(s>=13) status_char=slam_status_char(s,position);
            else case({s,display_stop_confirmed,position})
                10'd67, 10'd71, 10'd99, 10'd103, 10'd133, 10'd165, 10'd199, 10'd231,
                10'd322, 10'd354, 10'd385, 10'd392, 10'd417, 10'd424, 10'd449, 10'd481,
                10'd577, 10'd583, 10'd609, 10'd615, 10'd640, 10'd647, 10'd672, 10'd679,
                10'd704, 10'd710, 10'd736, 10'd742, 10'd774, 10'd802, 10'd838, 10'd864,
                10'd902, 10'd928, 10'd934, 10'd961, 10'd993:status_char=7'd65;
                10'd391, 10'd423:status_char=7'd66;
                10'd321, 10'd353, 10'd514, 10'd546, 10'd580, 10'd612, 10'd646, 10'd678,
                10'd801:status_char=7'd67;
                10'd66, 10'd98, 10'd393, 10'd425, 10'd516, 10'd548, 10'd805:status_char=7'd68;
                10'd257, 10'd289, 10'd389, 10'd421, 10'd512, 10'd517, 10'd521, 10'd523,
                10'd544, 10'd549, 10'd553, 10'd555, 10'd582, 10'd614, 10'd714, 10'd746,
                10'd808, 10'd937:status_char=7'd69;
                10'd1, 10'd2, 10'd33, 10'd34:status_char=7'd70;
                10'd263, 10'd295, 10'd327, 10'd359:status_char=7'd71;
                10'd65, 10'd72, 10'd97, 10'd104, 10'd128, 10'd134, 10'd160, 10'd166,
                10'd200, 10'd232, 10'd261, 10'd293, 10'd325, 10'd357, 10'd450, 10'd482,
                10'd775, 10'd839, 10'd903, 10'd962, 10'd994:status_char=7'd73;
                10'd64, 10'd96, 10'd260, 10'd292, 10'd388, 10'd420, 10'd581, 10'd613,
                10'd713, 10'd745:status_char=7'd76;
                10'd129, 10'd161, 10'd192, 10'd224, 10'd386, 10'd418, 10'd576, 10'd608,
                10'd711, 10'd743:status_char=7'd77;
                10'd262, 10'd294, 10'd323, 10'd324, 10'd326, 10'd355, 10'd356, 10'd358,
                10'd513, 10'd545, 10'd648, 10'd680, 10'd803, 10'd807:status_char=7'd78;
                10'd0, 10'd32, 10'd193, 10'd195, 10'd225, 10'd227, 10'd515, 10'd547,
                10'd643, 10'd675, 10'd707, 10'd739, 10'd770, 10'd806, 10'd834, 10'd867,
                10'd871, 10'd898, 10'd931:status_char=7'd79;
                10'd387, 10'd419, 10'd578, 10'd610, 10'd712, 10'd744, 10'd771, 10'd835,
                10'd872, 10'd899, 10'd933:status_char=7'd80;
                10'd68, 10'd100, 10'd196, 10'd228, 10'd518, 10'd520, 10'd550, 10'd552,
                10'd584, 10'd616:status_char=7'd82;
                10'd256, 10'd288, 10'd320, 10'd352, 10'd384, 10'd416, 10'd522, 10'd554,
                10'd645, 10'd677, 10'd709, 10'd741, 10'd768, 10'd800, 10'd832, 10'd869,
                10'd896, 10'd936:status_char=7'd83;
                10'd73, 10'd105, 10'd135, 10'd167, 10'd194, 10'd201, 10'd226, 10'd233,
                10'd258, 10'd259, 10'd290, 10'd291, 10'd451, 10'd483, 10'd524, 10'd556,
                10'd642, 10'd674, 10'd706, 10'd738, 10'd769, 10'd776, 10'd833, 10'd840,
                10'd866, 10'd870, 10'd897, 10'd904, 10'd930, 10'd963, 10'd995:status_char=7'd84;
                10'd130, 10'd162, 10'd641, 10'd673, 10'd705, 10'd737, 10'd865, 10'd929,
                10'd935:status_char=7'd85;
                10'd70, 10'd102, 10'd132, 10'd164, 10'd198, 10'd230, 10'd448, 10'd480,
                10'd773, 10'd837, 10'd901, 10'd960, 10'd992:status_char=7'd87;
                default:status_char=7'd32;
            endcase
        end
    endfunction
    function [6:0] nav_status_char;
        input [3:0] s;
        input [4:0] position;
        begin
            case({s,display_stop_confirmed,position})
                10'd6, 10'd38, 10'd70, 10'd102, 10'd130, 10'd162, 10'd196, 10'd228,
                10'd390, 10'd416, 10'd454, 10'd481, 10'd518, 10'd545, 10'd549, 10'd582,
                10'd614, 10'd646, 10'd678, 10'd710, 10'd742, 10'd774, 10'd806, 10'd838,
                10'd870, 10'd902, 10'd934, 10'd966, 10'd998:nav_status_char=7'd65;
                10'd422:nav_status_char=7'd68;
                10'd65, 10'd97, 10'd421:nav_status_char=7'd69;
                10'd548:nav_status_char=7'd70;
                10'd68, 10'd100, 10'd135, 10'd167, 10'd262, 10'd294, 10'd325, 10'd357:nav_status_char=7'd71;
                10'd198, 10'd230:nav_status_char=7'd72;
                10'd7, 10'd39, 10'd133, 10'd165, 10'd260, 10'd292, 10'd323, 10'd355,
                10'd391, 10'd419, 10'd455, 10'd519, 10'd583, 10'd615, 10'd647, 10'd679,
                10'd711, 10'd743, 10'd775, 10'd807, 10'd839, 10'd871, 10'd903, 10'd935,
                10'd967, 10'd999:nav_status_char=7'd73;
                10'd71, 10'd103, 10'd129, 10'd161, 10'd551:nav_status_char=7'd76;
                10'd320, 10'd352:nav_status_char=7'd77;
                10'd131, 10'd132, 10'd134, 10'd163, 10'd164, 10'd166, 10'd192, 10'd224,
                10'd259, 10'd261, 10'd291, 10'd293, 10'd324, 10'd356, 10'd480, 10'd544:nav_status_char=7'd78;
                10'd2, 10'd34, 10'd69, 10'd101, 10'd193, 10'd225, 10'd321, 10'd353,
                10'd386, 10'd450, 10'd486, 10'd514, 10'd578, 10'd610, 10'd642, 10'd674,
                10'd706, 10'd738, 10'd770, 10'd802, 10'd834, 10'd866, 10'd898, 10'd930,
                10'd962, 10'd994:nav_status_char=7'd79;
                10'd3, 10'd35, 10'd128, 10'd160, 10'd195, 10'd227, 10'd387, 10'd451,
                10'd487, 10'd515, 10'd579, 10'd611, 10'd643, 10'd675, 10'd707, 10'd739,
                10'd771, 10'd803, 10'd835, 10'd867, 10'd899, 10'd931, 10'd963, 10'd995:nav_status_char=7'd80;
                10'd258, 10'd290, 10'd417, 10'd418:nav_status_char=7'd82;
                10'd0, 10'd32, 10'd64, 10'd96, 10'd384, 10'd448, 10'd484, 10'd512,
                10'd576, 10'd608, 10'd640, 10'd672, 10'd704, 10'd736, 10'd768, 10'd800,
                10'd832, 10'd864, 10'd896, 10'd928, 10'd960, 10'd992:nav_status_char=7'd83;
                10'd1, 10'd8, 10'd33, 10'd40, 10'd66, 10'd98, 10'd197, 10'd229,
                10'd256, 10'd288, 10'd385, 10'd392, 10'd449, 10'd456, 10'd485, 10'd513,
                10'd520, 10'd552, 10'd577, 10'd584, 10'd609, 10'd616, 10'd641, 10'd648,
                10'd673, 10'd680, 10'd705, 10'd712, 10'd737, 10'd744, 10'd769, 10'd776,
                10'd801, 10'd808, 10'd833, 10'd840, 10'd865, 10'd872, 10'd897, 10'd904,
                10'd929, 10'd936, 10'd961, 10'd968, 10'd993, 10'd1000:nav_status_char=7'd84;
                10'd257, 10'd289, 10'd550:nav_status_char=7'd85;
                10'd322, 10'd354, 10'd420, 10'd482, 10'd546:nav_status_char=7'd86;
                10'd5, 10'd37, 10'd389, 10'd453, 10'd517, 10'd581, 10'd613, 10'd645,
                10'd677, 10'd709, 10'd741, 10'd773, 10'd805, 10'd837, 10'd869, 10'd901,
                10'd933, 10'd965, 10'd997:nav_status_char=7'd87;
                default:nav_status_char=7'd32;
            endcase
        end
    endfunction
    function [2:0] heading_direction;
        input signed [31:0] h;
        begin
            if(h>=-225 && h<225) heading_direction=0;
            else if(h>=225 && h<675) heading_direction=1;
            else if(h>=675 && h<1125) heading_direction=2;
            else if(h>=1125 && h<1575) heading_direction=3;
            else if(h>=1575 || h<-1575) heading_direction=4;
            else if(h>=-1575 && h<-1125) heading_direction=5;
            else if(h>=-1125 && h<-675) heading_direction=6;
            else heading_direction=7;
        end
    endfunction

    // Reference UI: a metric rectangle on the left, five live cards on the
    // right, and a compact legend. Six pixel cycles match every HDMI page.
    localparam [23:0] BG=24'h061424,PANEL=24'h0b2036,UNKNOWN=24'h304c68,
        FREE=24'hdceaf3,OCCUPIED=24'hff365e,CAR=24'h00dc62,PATH=24'h00c9ed,
        GOAL=24'hffbe13,TEXT=24'heaf8ff,MUTED=24'h7295b3,EDGE=24'h1272a0;
    localparam integer MAP_X=128,MAP_Y=84,MAP_W=728,MAP_H=588;
    localparam integer MAP_HALF_X=GRID_X*CELL_MM/2,MAP_HALF_Y=GRID_Y*CELL_MM/2;
    localparam integer SB=$clog2(GRID_X*GRID_Y);
    localparam [7:0] WX=48+(GRID_Y*CELL_MM/1000)%10,WT=48+(GRID_Y*CELL_MM/100)%10;
    localparam [7:0] HX=48+(GRID_X*CELL_MM/1000)%10,HT=48+(GRID_X*CELL_MM/100)%10;
    localparam [8*15-1:0] MAP_LABEL={"MAP ",WX,".",WT," x ",HX,".",HT," m"};
    reg display_active,display_distance_valid,display_motor_online;
    reg [3:0] display_status,display_nav_status,display_fault;
    reg display_goal_valid,display_path_valid,display_pose_valid;
    reg signed [15:0] display_pose_x,display_pose_y;
    reg signed [31:0] display_heading;
    reg [2:0] display_direction;
    reg [GRID_BITS-1:0] display_goal_x,display_goal_y;
    reg [11:0] car_center_x,car_center_y,goal_center_x,goal_center_y;
    reg car_visible,goal_visible;
    // Formatting runs in blanking, independently of all pixel decoding.
    function [15:0] magnitude;
        input [15:0] v;
        begin magnitude=v[15]?~v+16'd1:v;end
    endfunction
    reg [16:0] mean_pair_a,mean_pair_b;
    reg [15:0] mean_speed;
    always @(posedge clk)begin
        mean_pair_a<={1'b0,magnitude(motor_speeds[63:48])}+{1'b0,magnitude(motor_speeds[47:32])};
        mean_pair_b<={1'b0,magnitude(motor_speeds[31:16])}+{1'b0,magnitude(motor_speeds[15:0])};
        mean_speed<=({1'b0,mean_pair_a}+{1'b0,mean_pair_b})>>2;
    end
    reg [19:0] number_bcd[0:2],number_work[0:2],number_adjust[0:2];
    reg [2:0] number_digits[0:2];
    function [2:0] significant_digits;
        input [19:0] bcd;
        begin significant_digits=bcd[19:16]!=0?5:bcd[15:12]!=0?4:bcd[11:8]!=0?3:bcd[7:4]!=0?2:1;end
    endfunction
    reg [15:0] number_shift[0:2];
    reg [4:0] format_step;reg heading_negative;
    integer f,d;
    always @*begin
        for(f=0;f<3;f=f+1)begin
            number_adjust[f]=number_work[f];
            for(d=0;d<5;d=d+1)
                if(number_work[f][d*4+:4]>=5)number_adjust[f][d*4+:4]=number_work[f][d*4+:4]+4'd3;
        end
    end
    always @(posedge clk or negedge reset_n)
        if(!reset_n)begin
            display_active<=0;display_status<=0;display_nav_status<=0;display_fault<=0;
            display_distance_valid<=0;display_motor_online<=0;display_stop_confirmed<=0;
            display_goal_valid<=0;display_path_valid<=0;display_pose_valid<=0;
            display_goal_x<=0;display_goal_y<=0;display_pose_x<=0;display_pose_y<=0;
            display_heading<=0;display_direction<=0;format_step<=16;heading_negative<=0;
            for(f=0;f<3;f=f+1)begin number_bcd[f]<=0;number_work[f]<=0;number_shift[f]<=0;number_digits[f]<=1;end
        end else if(frame_begin)begin
            display_active<=active;display_status<=status;display_nav_status<=nav_status;
            display_fault<=nav_status!=0?nav_fault:auto_fault;
            display_distance_valid<=distance_valid;display_motor_online<=motor_online;
            display_stop_confirmed<=stop_confirmed;display_goal_valid<=goal_valid;
            display_path_valid<=path_valid;display_pose_valid<=pose_valid;
            display_goal_x<=goal_x_cell;display_goal_y<=goal_y_cell;
            display_pose_x<=pose_x_mm;display_pose_y<=pose_y_mm;
            display_heading<=heading_tenths;display_direction<=heading_direction(heading_tenths);
            heading_negative<=heading_tenths<0;
            number_shift[0]<=heading_tenths<0?-heading_tenths:heading_tenths;
            number_shift[1]<=distance_mm;number_shift[2]<=mean_speed;
            for(f=0;f<3;f=f+1)number_work[f]<=0;
            format_step<=0;
        end else if(format_step<16)begin
            for(f=0;f<3;f=f+1)begin
                number_work[f]<={number_adjust[f][18:0],number_shift[f][15]};
                number_shift[f]<=number_shift[f]<<1;
                if(format_step==15)begin
                    number_bcd[f]<={number_adjust[f][18:0],number_shift[f][15]};
                    number_digits[f]<=significant_digits({number_adjust[f][18:0],number_shift[f][15]});
                end
            end
            format_step<=format_step+1'b1;
        end

    // Map and car use the same millimetre coordinate transform. Register the
    // arithmetic away from pixels; all inputs are frame-latched above.
    wire pose_inside=display_pose_x>=-MAP_HALF_X && display_pose_x<MAP_HALF_X &&
        display_pose_y>=-MAP_HALF_Y && display_pose_y<MAP_HALF_Y;
    // 728/5200 == 588/4200 == 7/50 for the production rectangle.
    // Pipeline the fixed-point projection; blanking is far longer than these
    // four metadata cycles. No pixel passes through a variable divider.
    localparam integer SCALE_X_Q20=(MAP_W*1048576+GRID_Y*CELL_MM-1)/(GRID_Y*CELL_MM);
    localparam integer SCALE_Y_Q20=(MAP_H*1048576+GRID_X*CELL_MM-1)/(GRID_X*CELL_MM);
    reg [15:0] car_offset_x,car_offset_y;
    reg [35:0] car_product_x,car_product_y;
    wire [11:0] goal_screen_x[0:GRID_Y-1],goal_screen_y[0:GRID_X-1];
    genvar goal_index;
    generate for(goal_index=0;goal_index<GRID_Y;goal_index=goal_index+1)begin:goal_x_pixels
        assign goal_screen_x[goal_index]=MAP_X+((GRID_Y-1-goal_index)*2+1)*MAP_W/(GRID_Y*2);
    end
    for(goal_index=0;goal_index<GRID_X;goal_index=goal_index+1)begin:goal_y_pixels
        assign goal_screen_y[goal_index]=MAP_Y+((GRID_X-1-goal_index)*2+1)*MAP_H/(GRID_X*2);
    end endgenerate
    always @(posedge clk or negedge reset_n)
        if(!reset_n)begin
            car_offset_x<=0;car_offset_y<=0;car_product_x<=0;car_product_y<=0;
            car_center_x<=0;car_center_y<=0;goal_center_x<=0;goal_center_y<=0;car_visible<=0;goal_visible<=0;
        end else begin
            car_offset_x<=MAP_HALF_Y-$signed(display_pose_y);
            car_offset_y<=MAP_HALF_X-$signed(display_pose_x);
            car_product_x<=car_offset_x*SCALE_X_Q20;
            car_product_y<=car_offset_y*SCALE_Y_Q20;
            car_center_x<=MAP_X+((car_product_x>>20)>=MAP_W?MAP_W-1:(car_product_x>>20));
            car_center_y<=MAP_Y+((car_product_y>>20)>=MAP_H?MAP_H-1:(car_product_y>>20));
            goal_center_x<=goal_screen_x[display_goal_y];
            goal_center_y<=goal_screen_y[display_goal_x];
            car_visible<=display_pose_valid&&pose_inside;
            goal_visible<=display_goal_valid && display_goal_x<GRID_X && display_goal_y<GRID_Y;
        end

    // LUTs contain elaboration constants. Compact row bases are looked up
    // directly, so pixels never traverse a run-time row-index multiplier.
    wire raw_map=de_in&&pixel_x>=MAP_X&&pixel_x<MAP_X+MAP_W&&pixel_y>=MAP_Y&&pixel_y<MAP_Y+MAP_H;
    wire [9:0] raw_lx=pixel_x-MAP_X,raw_ly=pixel_y-MAP_Y;
    wire [GRID_BITS-1:0] x_table[0:MAP_H-1],y_table[0:MAP_W-1];
    wire [SB-1:0] row_table[0:MAP_W-1];
    genvar k;
    generate for(k=0;k<MAP_H;k=k+1)begin:gx
        assign x_table[k]=GRID_X-1-k*GRID_X/MAP_H;
    end
    for(k=0;k<MAP_W;k=k+1)begin:gy
        assign y_table[k]=GRID_Y-1-k*GRID_Y/MAP_W;
        assign row_table[k]=(GRID_Y-1-k*GRID_Y/MAP_W)*GRID_X;
    end endgenerate
    wire [GRID_BITS-1:0] cell_x_raw=x_table[raw_ly],cell_y_raw=y_table[raw_lx];
    wire [SB-1:0] compact_raw=row_table[raw_lx]+cell_x_raw;
    wire [2*GRID_BITS-1:0] raw_address=COMPACT_ADDRESS?compact_raw:{cell_y_raw,cell_x_raw};
    assign map_rd_addr_raw=raw_map?raw_address:0;
    reg [SB-1:0] map_row_base1;
    reg [GRID_BITS-1:0] map_cell_x1,map_cell_y1;
    reg map_address_valid1;
    wire [SB-1:0] compact_address1=map_row_base1+map_cell_x1;
    assign map_rd_addr=map_address_valid1 ? (COMPACT_ADDRESS?compact_address1:{map_cell_y1,map_cell_x1}) : 0;
    reg [11:0] x1,y1;
    reg [4:0] title_column1,map_label_column1,card_label_column1,card_value_column1;
    reg [6:0] card_label_ascii1;
    wire [4:0] card_label_column_raw=(pixel_x-12'd1032)>>3;
    reg [4:0] legend_column1[0:5];
    reg signed [3:0] card_number1;
    reg [7:0] card_local_y1;
    // These Y bands never overlap. Decode each independently and OR its
    // eight-bit modular offset instead of cascading five priority subtractors.
    wire [4:0] card_region_raw={pixel_y>=544&&pixel_y<648,
        pixel_y>=428&&pixel_y<532,pixel_y>=312&&pixel_y<416,
        pixel_y>=196&&pixel_y<300,pixel_y>=80&&pixel_y<184};
    wire card_region_any=|card_region_raw;
    wire [3:0] card_number_raw={4{!card_region_any}}|
        {1'b0,card_region_raw[4],card_region_raw[2]|card_region_raw[3],card_region_raw[1]|card_region_raw[3]};
    wire [7:0] card_offset_raw=({8{card_region_raw[0]}}&(pixel_y[7:0]-8'd80))|
        ({8{card_region_raw[1]}}&(pixel_y[7:0]-8'd196))|
        ({8{card_region_raw[2]}}&(pixel_y[7:0]-8'd56))|
        ({8{card_region_raw[3]}}&(pixel_y[7:0]-8'd172))|
        ({8{card_region_raw[4]}}&(pixel_y[7:0]-8'd32));
    integer prelegend;
    // Prepare region coordinates with the raw pixel. Their S1 registers line
    // up with x1/y1, removing coordinate subtraction from font selection S2.
    always @(posedge clk)begin
        title_column1<=(pixel_x-12'd48)>>4;
        map_label_column1<=(pixel_x-12'd1136)>>3;
        card_label_column1<=(pixel_x-12'd1032)>>3;
        card_value_column1<=(pixel_x-12'd1032)>>4;
        card_number1<=card_number_raw;card_local_y1<=card_offset_raw;
        case(card_number_raw)
            0:card_label_ascii1<=message_char("MODE",4,{2'b0,card_label_column_raw});
            1:card_label_ascii1<=message_char("STATUS",6,{2'b0,card_label_column_raw});
            2:card_label_ascii1<=message_char("YAW",3,{2'b0,card_label_column_raw});
            3:card_label_ascii1<=message_char("FRONT",5,{2'b0,card_label_column_raw});
            4:card_label_ascii1<=message_char("MEAN SPEED",10,{2'b0,card_label_column_raw});
            default:card_label_ascii1<=32;
        endcase
        for(prelegend=0;prelegend<6;prelegend=prelegend+1)
            legend_column1[prelegend]<=(pixel_x-(80+prelegend*200))>>3;
    end
    reg [5:0] sync_pipe,vs_pipe,de_pipe,active_pipe;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)begin
            map_row_base1<=0;map_cell_x1<=0;map_cell_y1<=0;map_address_valid1<=0;
            x1<=0;y1<=0;sync_pipe<=0;vs_pipe<=0;de_pipe<=0;active_pipe<=0;
        end else begin
            map_row_base1<=row_table[raw_lx];map_cell_x1<=cell_x_raw;map_cell_y1<=cell_y_raw;
            map_address_valid1<=raw_map;x1<=pixel_x;y1<=pixel_y;
            sync_pipe<={sync_pipe[4:0],hs_in};vs_pipe<={vs_pipe[4:0],vs_in};
            de_pipe<={de_pipe[4:0],de_in};active_pipe<={active_pipe[4:0],display_active};
        end
    always @*begin hs_out=sync_pipe[5];vs_out=vs_pipe[5];de_out=de_pipe[5];active_out=active_pipe[5];end

    function [6:0] message_char;
        input [8*80-1:0] s;
        input [6:0] length;
        input signed [6:0] column;
        reg [6:0] selected_char;
        reg matched;
        integer k;
        begin
            selected_char=0;matched=0;
            // Every call supplies an elaboration-constant string/length.
            // Decode its constant positions directly instead of subtracting
            // the pixel column and barrel-selecting a 640-bit vector.
            for(k=0;k<80;k=k+1)if(k<length)begin
                selected_char=selected_char|((column==k)?s[(length-1-k)*8+:7]:7'd0);
                matched=matched|(column==k);
            end
            message_char=matched?selected_char:7'd32;
        end
    endfunction
    function [6:0] heading_value_char;
        input [4:0] c;reg [2:0] count;
        begin
            count=number_digits[0]<2?2:number_digits[0];
            if(c==0)heading_value_char=heading_negative?"-":"+";
            else if(c<count)heading_value_char=decimal_char(number_bcd[0],display_pose_valid,c+4-count);
            else if(c==count)heading_value_char=".";
            else if(c==count+1)heading_value_char=decimal_char(number_bcd[0],display_pose_valid,4);
            else heading_value_char=message_char(" deg",4,c-count-2);
        end
    endfunction
    function [6:0] front_value_char;
        input [4:0] c;
        begin front_value_char=c<number_digits[1]?
            decimal_char(number_bcd[1],display_distance_valid,c+5-number_digits[1]):
            message_char(" mm",3,c-number_digits[1]);end
    endfunction
    function [6:0] mean_value_char;
        input [4:0] c;
        begin mean_value_char=c<number_digits[2]?
            decimal_char(number_bcd[2],display_motor_online,c+5-number_digits[2]):7'd32;end
    endfunction
    reg [6:0] heading_text[0:15],front_text[0:15],mean_text[0:15];
    reg [6:0] mode_text[0:15],status_text[0:15];
    genvar value_position;
    generate for(value_position=0;value_position<16;value_position=value_position+1)begin:g_value_text
        // Formatting is metadata work, completed during frame blanking.
        // The pixel path selects a prepared character; its six cycles stay.
        always @(posedge clk)begin
            heading_text[value_position]<=heading_value_char(value_position);
            front_text[value_position]<=front_value_char(value_position);
            mean_text[value_position]<=mean_value_char(value_position);
            mode_text[value_position]<=display_nav_status!=0?message_char("AUTO",4,value_position):
                (display_status>=1&&display_status!=6&&display_status!=7?
                 message_char("MAP",3,value_position):message_char("MANUAL",6,value_position));
            status_text[value_position]<=display_nav_status==14?message_char("MANUAL CTRL",11,value_position):
                display_nav_status!=0?nav_status_char(display_nav_status,value_position):
                status_char(display_status,value_position);
        end
    end endgenerate
    function [6:0] decimal_char;
        input [19:0] bcd;input valid;input signed [6:0] column;
        reg [3:0] digit;
        begin
            case(column)
                0:digit=bcd[19:16];1:digit=bcd[15:12];2:digit=bcd[11:8];
                3:digit=bcd[7:4];default:digit=bcd[3:0];
            endcase
            decimal_char=column>=0&&column<5 ? (valid?7'd48+digit:7'd45):7'd32;
        end
    endfunction
    function [23:0] accent;
        input integer card;
        begin case(card)0:accent=PATH;1:accent=CAR;2:accent=24'h368dff;3:accent=24'hb27bff;default:accent=GOAL;endcase end
    endfunction
    wire map_here=x1>=MAP_X&&x1<MAP_X+MAP_W&&y1>=MAP_Y&&y1<MAP_Y+MAP_H;
    wire left_panel=x1>=24&&x1<968&&y1>=68&&y1<684;
    wire [11:0] cx_abs=x1>car_center_x?x1-car_center_x:car_center_x-x1;
    wire [11:0] cy_abs=y1>car_center_y?y1-car_center_y:car_center_y-y1;
    wire signed [12:0] car_dx=$signed({1'b0,x1})-$signed({1'b0,car_center_x});
    wire signed [12:0] car_dy=$signed({1'b0,y1})-$signed({1'b0,car_center_y});
    wire [11:0] goal_ax=x1>goal_center_x?x1-goal_center_x:goal_center_x-x1;
    wire [11:0] goal_ay=y1>goal_center_y?y1-goal_center_y:goal_center_y-y1;
    wire [11:0] goal_limit=goal_ay<4?11:goal_ay<7?10:goal_ay<9?8:goal_ay<11?6:3;
    wire goal_dot=goal_visible&&map_here&&goal_ay<=11&&goal_ax<=goal_limit;
    // Each cardinal triangle is at distance 14..23 from the car. Derive its
    // half width from UNSIGNED absolute distance, bounded to five bits.
    // A signed negative car_dy/car_dx inside an unsigned comparison otherwise
    // gets zero-extended and can turn the upward/leftward arrow into a stripe.
    wire [4:0] arrow_half_width_x=(5'd23-cx_abs[4:0])>>1;
    wire [4:0] arrow_half_width_y=(5'd23-cy_abs[4:0])>>1;
    function arrow_shape;
        input [2:0] direction;
        input signed [12:0] dx,dy;
        input [11:0] ax,ay;
        reg [4:0] half_x,half_y;
        begin
            half_x=(5'd23-ax[4:0])>>1;half_y=(5'd23-ay[4:0])>>1;
            case(direction)
                0:arrow_shape=dy>=-23&&dy<=-14&&ax<=half_y;
                1:arrow_shape=dx<=-12&&dx>=-21&&dy<=-12&&dy>=-21&&dx+dy>=-36;
                2:arrow_shape=dx>=-23&&dx<=-14&&ay<=half_x;
                3:arrow_shape=dx<=-12&&dx>=-21&&dy>=12&&dy<=21&&dx-dy>=-36;
                4:arrow_shape=dy<=23&&dy>=14&&ax<=half_y;
                5:arrow_shape=dx>=12&&dx<=21&&dy>=12&&dy<=21&&dx+dy<=36;
                6:arrow_shape=dx<=23&&dx>=14&&ay<=half_x;
                default:arrow_shape=dx>=12&&dx<=21&&dy<=-12&&dy>=-21&&dx-dy<=36;
            endcase
        end
    endfunction
    // Live geometry remains available for diagnostics; rendering below uses
    // the registered deltas so two coordinate subtractors are never chained
    // with diagonal-arrow sums and the final RGB priority in one pixel clock.
    wire car_body=car_visible&&map_here&&cx_abs<=9&&cy_abs<=11;
    wire car_arrow=car_visible&&map_here&&arrow_shape(display_direction,car_dx,car_dy,cx_abs,cy_abs);

    wire [6:0] ascii;
    reg [6:0] header_ascii,map_ascii,card_ascii,legend_ascii,goal_ascii;reg [2:0] font_col;reg [3:0] font_row;
    reg [23:0] base,text_color;reg text_here;
    // All text coordinates are bounded by their region. Keeping them narrow
    // avoids 32-bit integer subtractors in the pixel-to-font path.
    reg signed [3:0] card;
    reg [7:0] local_y;
    reg signed [12:0] column;
    integer legend_scan;
    always @*begin
        header_ascii=0;map_ascii=0;card_ascii=0;legend_ascii=0;goal_ascii=0;font_col=x1[2:0];font_row=y1[3:0];base=BG;text_color=TEXT;text_here=0;
        card=-1;local_y=0;column=0;
        if(left_panel)begin
            base=PANEL;
            if(x1<26||x1>=966||y1<70||y1>=682)base=PATH;
        end
        if(y1>=58&&y1<60&&x1>=24&&x1<1256)base=EDGE;
        if(x1>=48&&x1<336&&y1>=12&&y1<44)begin
            header_ascii=message_char("SLAM NAVIGATION",15,{2'b0,title_column1});font_col=x1[3:1];font_row=(y1-12)>>1;
            text_here=1;text_color=TEXT;
        end
        if(x1>=1256-8*15&&x1<1256&&y1>=24&&y1<40)begin
            map_ascii=message_char(MAP_LABEL,15,{2'b0,map_label_column1});font_col=x1[2:0];font_row=y1-24;text_here=1;text_color=PATH;
        end
        if(x1>=992&&x1<1256&&y1>=80&&y1<648)begin
            card=card_number1;local_y=card_local_y1;
            if(card>=0)begin
                base=PANEL;
                if(x1<994||x1>=1254||local_y<2||local_y>=102)base=EDGE;
                if(x1>=1008&&x1<1014&&local_y>=18&&local_y<86)base=accent(card);
                if(x1>=1032&&local_y>=16&&local_y<32)begin
                    card_ascii=card_label_ascii1;
                    if(card_label_column1[4])card_ascii=32;
                    text_here=1;text_color=MUTED;font_col=x1[2:0];font_row=local_y-16;
                end
                if(x1>=1032&&local_y>=44&&local_y<76)begin
                    column={8'd0,card_value_column1};text_here=1;font_col=x1[3:1]^3'b100;font_row=(local_y-44)>>1;
                    case(card)
                        0:card_ascii=mode_text[card_value_column1[3:0]];
                        1:card_ascii=status_text[card_value_column1[3:0]];
                        2:card_ascii=heading_text[card_value_column1[3:0]];
                        3:card_ascii=front_text[card_value_column1[3:0]];
                        4:card_ascii=mean_text[card_value_column1[3:0]];
                    endcase
                    if(card_value_column1[4])card_ascii=32;
                end
                if(card==4&&local_y>=82&&local_y<98&&x1>=1032)begin
                    card_ascii=message_char("MOTOR UNITS",11,{2'b0,card_label_column1});font_col=x1[2:0];font_row=local_y-82;
                    text_here=1;text_color=MUTED;
                end
                if(card==1&&display_fault!=0&&local_y>=82&&local_y<98&&x1>=1032)begin
                    column={8'd0,card_label_column1};card_ascii=column==0?"F":column==1?7'd48+display_fault/10:column==2?7'd48+display_fault%10:7'd32;
                    font_col=x1[2:0];font_row=local_y-82;text_here=1;text_color=OCCUPIED;
                end
            end
        end
        if(x1>=24&&x1<1256&&y1>=692&&y1<720)begin
            base=PANEL;
            if(y1==692||y1==719||x1==24||x1==1255)base=EDGE;
            // All six intervals are elaboration constants. Never multiply
            // the late selected legend number on the 75 MHz pixel path.
            for(legend_scan=0;legend_scan<6;legend_scan=legend_scan+1)
            if(y1>=698&&y1<714&&x1>=40+legend_scan*200&&x1<240+legend_scan*200)begin
                column=x1-(40+legend_scan*200);
                if(column>=0&&column<24)begin
                    case(legend_scan)0:base=UNKNOWN;1:base=FREE;2:base=OCCUPIED;3:base=CAR;4:base=PATH;5:base=GOAL;endcase
                end else if(column>=40)begin
                    case(legend_scan)
                        0:legend_ascii=message_char("UNKNOWN",7,{2'b0,legend_column1[legend_scan]});
                        1:legend_ascii=message_char("FREE",4,{2'b0,legend_column1[legend_scan]});
                        2:legend_ascii=message_char("OCCUPIED",8,{2'b0,legend_column1[legend_scan]});
                        3:legend_ascii=message_char("CAR",3,{2'b0,legend_column1[legend_scan]});
                        4:legend_ascii=message_char("PATH",4,{2'b0,legend_column1[legend_scan]});
                        5:legend_ascii=message_char("GOAL",4,{2'b0,legend_column1[legend_scan]});
                    endcase
                    text_here=1;font_col=x1[2:0];font_row=y1-698;
                end
            end
        end
        // Gold goal dot and its G label are visible above path/occupancy.
        if(goal_visible&&left_panel&&x1>=goal_center_x+16&&x1<goal_center_x+32&&
            y1+24>=goal_center_y&&y1<goal_center_y+8)begin
            goal_ascii="G";font_col=(x1-goal_center_x-16)>>1;font_row=(y1+24-goal_center_y)>>1;
            text_here=1;text_color=GOAL;
        end
    end
    // These text regions cannot overlap: title/map header, right cards,
    // bottom legend and goal inside the left map panel. Select their glyphs
    // in parallel, rather than routing card text through six legend muxes.
    assign ascii=text_here ? (header_ascii|map_ascii|card_ascii|legend_ascii|goal_ascii) : 7'd32;
    reg [6:0] font_ascii2;reg [2:0] font_col2;reg [3:0] font_row2;
    wire font_pixel;
    telemetry_font8x16 font(.clk(clk),.ascii(font_ascii2),.col(font_col2),.row(font_row2),.pixel(font_pixel));
    reg map2,path2,goal2,text2,text3,goal3,car3;
    reg signed [12:0] car_dx2,car_dy2;
    reg [11:0] car_ax2,car_ay2;
    reg [2:0] car_direction2;
    reg car_visible2;
    reg [23:0] base2,color2,color3,rgb3,rgb4,rgb5;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)begin
            font_ascii2<=32;font_col2<=0;font_row2<=0;
            map2<=0;path2<=0;goal2<=0;text2<=0;text3<=0;goal3<=0;car3<=0;
            car_dx2<=0;car_dy2<=0;car_ax2<=0;car_ay2<=0;car_direction2<=0;car_visible2<=0;
            base2<=0;color2<=0;color3<=0;rgb3<=0;rgb4<=0;rgb5<=0;rgb_out<=0;
        end else begin
            font_ascii2<=ascii;font_col2<=font_col;font_row2<=font_row;
            map2<=map_here;path2<=display_path_valid;goal2<=goal_dot;
            car_dx2<=car_dx;car_dy2<=car_dy;car_ax2<=cx_abs;car_ay2<=cy_abs;
            car_direction2<=display_direction;car_visible2<=car_visible;
            text2<=text_here;base2<=base;color2<=text_color;
            color3<=color2;text3<=text2;
            goal3<=goal2;
            car3<=de_pipe[1]&&car_visible2&&map2&&((car_ax2<=9&&car_ay2<=11)||
                arrow_shape(car_direction2,car_dx2,car_dy2,car_ax2,car_ay2));
            if(!de_pipe[1])rgb3<=0;
            else if(goal2)rgb3<=GOAL;
            else if(map2)begin
                if(map_rd_data==2||map_rd_data==3)rgb3<=OCCUPIED;
                else if(path2&&path_rd_data&&(ALLOW_UNKNOWN_PATH||map_rd_data==1))rgb3<=PATH;
                else rgb3<=map_rd_data==1?FREE:UNKNOWN;
            end else rgb3<=base2;
            // Preserve goal > car > map/path and text > graphics exactly,
            // reusing S3/S4 rather than adding a seventh video/sync clock.
            rgb4<=text3&&font_pixel?color3:(car3&&!goal3)?CAR:rgb3;
            rgb5<=rgb4;rgb_out<=rgb5;
        end
endmodule
