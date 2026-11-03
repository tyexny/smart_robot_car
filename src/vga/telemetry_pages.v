`timescale 1ns/1ps
// SW7=0: camera dashboard. SW7=1: full sensor page.
// Values update at a video frame boundary, during vertical back porch.
// Main: 304x252 panel, native 8x16 font. Full page: 1216x648, 16x32 font.
// Raw pixel/RGB/sync inputs are registered before region decoding. RGB and
// all syncs have identical six-register latency; frame_begin stays raw.
// Optional object-box comparison runs between the existing first/second
// registers, keeping driver coordinate arithmetic off its comparison path.
module telemetry_pages #(parameter [11:0] OSD_X=16, OSD_Y=16,
 parameter PERSON_BOXES_ENABLE=0, TRACK_ID_LABEL_ENABLE=0)(
 input wire clk,rst_n,frame_begin,
 input wire [11:0] pixel_x,pixel_y,
 input wire [23:0] rgb_in,
 input wire hs_in,vs_in,de_in,page_select,telemetry_valid,
 input wire imu_panel_select,
 output wire record_screen_page,
 input wire [671:0] imu_values,
 input wire [3:0] imu_known,
 input wire [63:0] motor_speeds,motor_counts,
 input wire motor_online,
 input wire [3:0] motion_command,
 input wire [15:0] front_distance_cm,rear_distance_cm,
 input wire front_distance_valid,rear_distance_valid,
 input wire [15:0] left_distance_cm,right_distance_cm,
 input wire left_distance_valid,right_distance_valid,
 input wire left_distance_measured_valid,right_distance_measured_valid,
 input wire [1:0] left_distance_status,right_distance_status,
 input wire [5:0] avoidance_state,
 input wire [3:0] avoidance_fault,
 input wire avoidance_enabled,
 input wire [2:0] follow_status,
 input wire [3:0] follow_selected_slot,
 input wire [31:0] follow_frame_id,
 input wire [31:0] follow_target_id,
 input wire [3:0] follow_reason,
 input wire recognition_active_video, // existing display-domain SW4 session level
 input wire person_boxes_reset_n,person_boxes_enable,person_boxes_valid,
 input wire [31:0] person_boxes_frame_id,
 input wire [3:0] person_boxes_count,
 input wire [511:0] person_boxes_coords,
 input wire [55:0] person_boxes_classes,
 input wire [119:0] person_boxes_label_text,
 input wire [3:0] person_boxes_label_length,
 output wire [215:0] imu_nine_bcd,
 output wire [8:0] imu_nine_negative,imu_nine_valid,
 output reg [23:0] rgb_out,
 output reg hs_out,vs_out,de_out
);
 // SUG550-2.4E section5.13: preserve equivalent registers as physical copies.
 // Keep box comparisons off the shared dashboard/map coordinate fanout.
 // Both banks sample the same raw pixel on the same edge; video latency is unchanged.
 reg [11:0] coord_x /* synthesis syn_preserve = 1 */;
 reg [11:0] coord_y /* synthesis syn_preserve = 1 */;
 reg [11:0] box_coord_x /* synthesis syn_preserve = 1 */;
 reg [11:0] box_coord_y /* synthesis syn_preserve = 1 */;
 reg hs_input_r,vs_input_r,de_input_r;
 reg [23:0] rgb_input_r;
 reg person_enable_input_r;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   coord_x<=0;coord_y<=0;box_coord_x<=0;box_coord_y<=0;hs_input_r<=0;vs_input_r<=0;de_input_r<=0;
   rgb_input_r<=0;person_enable_input_r<=0;
  end else begin
   coord_x<=pixel_x;coord_y<=pixel_y;box_coord_x<=pixel_x;box_coord_y<=pixel_y;
   hs_input_r<=hs_in;vs_input_r<=vs_in;de_input_r<=de_in;
   rgb_input_r<=rgb_in;person_enable_input_r<=person_boxes_enable;
  end
 end
 // Parallel per-slot comparison results are captured before priority/color decode.
 wire [7:0] object_label_candidates,object_badge_candidates,object_selected_boxes;
 wire [7:0] object_label_inks,object_badge_inks,object_box_borders,object_box_bold;
 wire [55:0] object_candidate_classes,object_candidate_local_x;
 wire [119:0] object_custom_text;
 wire [3:0] object_custom_length;
 wire [31:0] object_candidate_local_y,object_badge_local_y;
 wire [23:0] object_badge_local_x;
 wire [2:0] object_follow_status,object_banner_local_x;
 wire [3:0] object_banner_local_y;
 wire object_banner_candidate,object_banner_ink;
 wire [6:0] object_banner_ascii;
 generate
  if(PERSON_BOXES_ENABLE) begin: person_detection
   person_box_overlay #(.CLASS_LABELS_ENABLE(1),.TRACK_ID_LABEL_ENABLE(TRACK_ID_LABEL_ENABLE)) u_person_boxes(
    .clk(clk),.reset_n(rst_n && person_boxes_reset_n),.frame_begin(frame_begin),
    .boxes_valid(person_boxes_valid),.boxes_frame_id(person_boxes_frame_id),
    .boxes_count(person_boxes_count),.boxes_coords(person_boxes_coords),
    .boxes_classes(person_boxes_classes),
    .boxes_label_text(person_boxes_label_text),.boxes_label_length(person_boxes_label_length),
    .label_custom_text(object_custom_text),.label_custom_length(object_custom_length),
    .follow_status(follow_status),.follow_selected_slot(follow_selected_slot),.follow_frame_id(follow_frame_id),
    .follow_target_id(follow_target_id),.follow_reason(follow_reason),
    .enable(person_enable_input_r),.pixel_x(box_coord_x),.pixel_y(box_coord_y),
    .de_in(de_input_r),.rgb_in(rgb_input_r),.rgb_out(),.active_box_count(),
    .label_hit(),.label_text_hit(),.label_literal_valid(),.label_literal_ascii(),
    .label_color(),.label_banner(),.label_class(),.label_char_index(),.label_col(),.label_row(),
    .label_origin_x(),.label_origin_y(),
    .label_color_candidates(object_label_candidates),.badge_color_candidates(object_badge_candidates),
    .label_color_selected(object_selected_boxes),.label_color_status(object_follow_status),
    .label_text_candidates(object_label_inks),.badge_text_candidates(object_badge_inks),
    .box_border_candidates(object_box_borders),.box_bold_candidates(object_box_bold),
    .candidate_classes(object_candidate_classes),.candidate_local_x(object_candidate_local_x),
    .candidate_local_y(object_candidate_local_y),.badge_local_x(object_badge_local_x),.badge_local_y(object_badge_local_y),
    .banner_candidate(object_banner_candidate),.banner_text_candidate(object_banner_ink),
    .banner_ascii(object_banner_ascii),.banner_local_x(object_banner_local_x),.banner_local_y(object_banner_local_y));
  end else begin: no_person_detection
   assign object_custom_text=0;assign object_custom_length=0;
   assign object_label_candidates=0;assign object_badge_candidates=0;assign object_selected_boxes=0;
   assign object_label_inks=0;assign object_badge_inks=0;assign object_box_borders=0;assign object_box_bold=0;
   assign object_candidate_classes=0;assign object_candidate_local_x=0;assign object_candidate_local_y=0;
   assign object_badge_local_x=0;assign object_badge_local_y=0;
   assign object_follow_status=0;assign object_banner_candidate=0;assign object_banner_ink=0;
   assign object_banner_ascii=7'd32;assign object_banner_local_x=0;assign object_banner_local_y=0;
  end
 endgenerate

 reg [31:0] latest[0:32],snapshot[0:32];
 reg [3:0] latest_distance_known,display_distance_known;
 reg [3:0] latest_side_status,display_side_status;
 reg [1:0] latest_side_measured,display_side_measured;
 reg [5:0] latest_avoidance_state,display_avoidance_state;
 reg [3:0] latest_avoidance_fault,display_avoidance_fault;
 reg latest_avoidance_enabled,display_avoidance_enabled;
 reg [15:0] display_av_ascii,display_fault_ascii;
 reg [3:0] latest_known,display_known,latest_motion,display_motion;
 reg latest_online,display_online,page_meta,page_sync,display_page;
 reg display_recognition,display_imu_panel;
 assign record_screen_page=display_page || display_imu_panel || !display_recognition;
 // Change the camera panel only at frame boundaries, even while telemetry
 // conversion is busy. The top reuses the existing local session level.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin display_recognition<=0;display_imu_panel<=0;end
  else if(frame_begin) begin
    display_recognition<=PERSON_BOXES_ENABLE && recognition_active_video;
    display_imu_panel<=imu_panel_select;
  end
 end
 reg [23:0] bcd[0:32],work_bcd,adjusted;
 reg [32:0] negative;
 assign imu_nine_bcd={bcd[20],bcd[14],bcd[13],bcd[5],bcd[4],bcd[3],bcd[2],bcd[1],bcd[0]};
 assign imu_nine_negative={negative[20],negative[14],negative[13],negative[5:0]};
 assign imu_nine_valid={{3{display_known[2]}},{6{display_known[0]}}};
 reg [31:0] magnitude, selected_value;
 // For a negative two's-complement word, bits above its lowest set bit
 // invert; that bit and every lower bit stay unchanged. Compute that test
 // with a bounded-depth prefix tree instead of a 32-bit increment chain.
 // Kept radix-four boundaries prevent synthesis sharing adjacent prefixes
 // into a serial carry-like chain. No formatter or pixel stage is added.
 wire [30:0] magnitude_prefix4 /* synthesis syn_keep = 1 */;
 wire [30:0] magnitude_prefix16 /* synthesis syn_keep = 1 */;
 wire [31:0] selected_magnitude;
 genvar magnitude_bit;
 generate
   for(magnitude_bit=0;magnitude_bit<31;magnitude_bit=magnitude_bit+1) begin: magnitude_prefix
     if(magnitude_bit<4)
       assign magnitude_prefix4[magnitude_bit]=|selected_value[magnitude_bit:0];
     else
       assign magnitude_prefix4[magnitude_bit]=|selected_value[magnitude_bit-:4];
     if(magnitude_bit<4)
       assign magnitude_prefix16[magnitude_bit]=magnitude_prefix4[magnitude_bit];
     else if(magnitude_bit<8)
       assign magnitude_prefix16[magnitude_bit]=magnitude_prefix4[magnitude_bit] | magnitude_prefix4[magnitude_bit-4];
     else if(magnitude_bit<12)
       assign magnitude_prefix16[magnitude_bit]=magnitude_prefix4[magnitude_bit] | magnitude_prefix4[magnitude_bit-4] | magnitude_prefix4[magnitude_bit-8];
     else
       assign magnitude_prefix16[magnitude_bit]=magnitude_prefix4[magnitude_bit] | magnitude_prefix4[magnitude_bit-4] | magnitude_prefix4[magnitude_bit-8] | magnitude_prefix4[magnitude_bit-12];
     if(magnitude_bit<16)
       assign selected_magnitude[magnitude_bit+1]=selected_value[magnitude_bit+1] ^ (selected_value[31] & magnitude_prefix16[magnitude_bit]);
     else
       assign selected_magnitude[magnitude_bit+1]=selected_value[magnitude_bit+1] ^ (selected_value[31] & (magnitude_prefix16[magnitude_bit] | magnitude_prefix16[magnitude_bit-16]));
   end
 endgenerate
 assign selected_magnitude[0]=selected_value[0];
 reg [5:0] field_index;
 reg [5:0] bit_count;
 reg [2:0] state;
 integer n,digit;
 // Convert the small control debug codes only at a frame boundary. Their
 // decimal ASCII bytes are registered, keeping decimal arithmetic away from
 // the pixel character-selection path.
 function [15:0] decimal_pair;
   input [5:0] value;
   reg [3:0] tens;
   reg [5:0] ones;
   reg [7:0] high_ascii,low_ascii;
   begin
     if(value>=60) begin tens=6;ones=value-60;end
     else if(value>=50) begin tens=5;ones=value-50;end
     else if(value>=40) begin tens=4;ones=value-40;end
     else if(value>=30) begin tens=3;ones=value-30;end
     else if(value>=20) begin tens=2;ones=value-20;end
     else if(value>=10) begin tens=1;ones=value-10;end
     else begin tens=0;ones=value;end
     high_ascii=8'd48+tens;low_ascii=8'd48+ones;
     decimal_pair={high_ascii,low_ascii};
   end
 endfunction
 always @* begin
   adjusted=work_bcd;
   for(digit=0;digit<6;digit=digit+1)
     if(work_bcd[digit*4+:4]>=5) adjusted[digit*4+:4]=work_bcd[digit*4+:4]+3;
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     latest_known<=0;display_known<=0;latest_online<=0;display_online<=0;
     latest_distance_known<=0;display_distance_known<=0;
     latest_side_status<=0;display_side_status<=0;
     latest_side_measured<=0;display_side_measured<=0;
     latest_avoidance_state<=0;display_avoidance_state<=0;
     latest_avoidance_fault<=0;display_avoidance_fault<=0;
     latest_avoidance_enabled<=0;display_avoidance_enabled<=0;
     display_av_ascii<="00";display_fault_ascii<="00";
     latest_motion<=0;display_motion<=0;page_meta<=0;page_sync<=0;display_page<=0;
     state<=0;field_index<=0;bit_count<=0;magnitude<=0;selected_value<=0;work_bcd<=0;negative<=0;
     for(n=0;n<33;n=n+1) begin latest[n]<=0;snapshot[n]<=0;bcd[n]<=0;end
   end else begin
     page_meta<=page_select;page_sync<=page_meta;
     if(telemetry_valid) begin
       for(n=0;n<21;n=n+1) latest[n]<=imu_values[n*32+:32];
       for(n=0;n<4;n=n+1) begin
         latest[21+n]<={{16{motor_speeds[63-n*16]}},motor_speeds[48-n*16+:16]};
         latest[25+n]<={16'd0,motor_counts[48-n*16+:16]};
       end
       latest_known<=imu_known;latest_online<=motor_online;latest_motion<=motion_command;
       latest[29]<={16'd0,front_distance_cm};latest[30]<={16'd0,rear_distance_cm};
       latest[31]<={16'd0,left_distance_cm};latest[32]<={16'd0,right_distance_cm};
       latest_distance_known<={right_distance_valid,left_distance_valid,rear_distance_valid,front_distance_valid};
       latest_side_status<={right_distance_status,left_distance_status};
       latest_side_measured<={right_distance_measured_valid,left_distance_measured_valid};
       latest_avoidance_state<=avoidance_state;latest_avoidance_fault<=avoidance_fault;
       latest_avoidance_enabled<=avoidance_enabled;
     end
     case(state)
       0:if(frame_begin) begin
         for(n=0;n<33;n=n+1) snapshot[n]<=latest[n];
         display_known<=latest_known;display_online<=latest_online;
         display_distance_known<=latest_distance_known;
         display_side_status<=latest_side_status;display_side_measured<=latest_side_measured;
         display_avoidance_state<=latest_avoidance_state;display_avoidance_fault<=latest_avoidance_fault;
         display_avoidance_enabled<=latest_avoidance_enabled;
         display_av_ascii<=decimal_pair(latest_avoidance_state);
         display_fault_ascii<=decimal_pair({2'b00,latest_avoidance_fault});
         display_motion<=latest_motion;display_page<=page_sync;field_index<=0;state<=1;
       end
       1:begin
         // Separate the 33-field mux from the signed magnitude carry chain.
         selected_value<=snapshot[field_index];state<=4;
       end
       4:begin
         negative[field_index]<=selected_value[31];
         // The four unsigned 16-bit sonars need only 16 double-dabble
         // shifts. 29*35 + 4*19 = 1091 conversion clocks, fitting the
         // existing 1100-clock vertical-porch budget with no extra pipeline.
         if(field_index>=29) begin
           magnitude<={selected_value[15:0],16'd0};bit_count<=16;
         end else begin
           magnitude<=selected_magnitude;
           bit_count<=0;
         end
         work_bcd<=0;state<=2;
       end
       2:begin
         work_bcd<={adjusted[22:0],magnitude[31]};magnitude<={magnitude[30:0],1'b0};
         if(bit_count==31) state<=3;else bit_count<=bit_count+1'b1;
       end
       3:begin
         bcd[field_index]<=work_bcd;
         if(field_index==32) state<=0;
         else begin field_index<=field_index+1'b1;state<=1;end
       end
       default:state<=0;
     endcase
   end
 end
 function [2:0] decimals;
   input [5:0] field;
   begin
     if(field<3) decimals=3;
     else if(field<9) decimals=1;
     else if(field<13) decimals=4;
     else if(field<16 || field==17 || field==20) decimals=1;
     else if(field==16) decimals=2;
     else decimals=0;
   end
 endfunction
 function known;
   input [5:0] field;
   begin
     if(field<9) known=display_known[0];
     else if(field<13) known=display_known[1];
     else if(field<16 || field==20) known=display_known[2];
     else if(field<20) known=display_known[3];
     else if(field<29) known=display_online;
     else begin
       case(field)
         29:known=display_distance_known[0];
         30:known=display_distance_known[1];
         31:known=display_distance_known[2];
         32:known=display_distance_known[3];
         default:known=0;
       endcase
     end
   end
 endfunction
 function [6:0] number_char;
   input [5:0] field;
   input [3:0] position;
   reg [2:0] frac;
   integer dp,digit_position;
   begin
     frac=decimals(field);dp=7-frac;number_char=" ";
     // Sonars are unsigned centimetres, right aligned in three columns.
     // Front/rear retain their raw-display blank-for-invalid convention.
     // Side values show the control-normalized range; NOE/FAR markers
     // distinguish a clear 500 cm limit from a measured return.
     if(field>=29) begin
       if(known(field)) begin
         case(position)
           0:if(bcd[field][11:8]!=0) number_char=7'd48+bcd[field][11:8];
           1:if(bcd[field][11:4]!=0) number_char=7'd48+bcd[field][7:4];
           2:number_char=7'd48+bcd[field][3:0];
           default:number_char=" ";
         endcase
       end else if(field>=31 && position<3) number_char="-";
     end
     else if(!known(field)) number_char=position<7 ? "-" : " ";
     else if(position==0) number_char=negative[field] ? "-" : "+";
     else if(frac!=0 && position==dp) number_char=".";
     else begin
       digit_position=position-1;
       if(frac!=0 && position>dp) digit_position=digit_position-1;
       if(digit_position>=0 && digit_position<6)
         number_char=7'd48+bcd[field][(5-digit_position)*4+:4];
     end
   end
 endfunction
 function [119:0] mode_text;
   input [3:0] mode;
   begin
     case(mode)
       0:mode_text="STOP           ";
       1:mode_text="FORWARD        ";
       2:mode_text="BACKWARD       ";
       3:mode_text="STRAFE LEFT    ";
       4:mode_text="STRAFE RIGHT   ";
       5:mode_text="ROTATE LEFT    ";
       6:mode_text="ROTATE RIGHT   ";
       7:mode_text="ARC LEFT       ";
       8:mode_text="ARC RIGHT      ";
       default:mode_text="UNKNOWN        ";
     endcase
   end
 endfunction
 function [31:0] direction_text;
   input [5:0] field;
   begin
     if(!display_online) direction_text="----";
     else if(snapshot[field]==0) direction_text="STOP";
     else if(negative[field] ^ (field==21)) direction_text="REV ";
     else direction_text="FWD ";
   end
 endfunction
 wire [11:0] lx=coord_x-OSD_X,ly=coord_y-OSD_Y;
 // Predecode the 3x SW1 glyph coordinates alongside the input registers.
 // Constant ROMs avoid divide-by-three logic on the live font path and keep
 // the existing six-cycle RGB/sync alignment. Blank/porch ROM data is unused.
 reg [7:0] imu_x_geometry[0:1279];
 reg [8:0] imu_y_geometry[0:719];
 reg [7:0] imu_x_coord;
 reg [8:0] imu_y_coord;
 integer imu_px,imu_py,imu_line,imu_offset,imu_start;
 initial begin
   for(imu_px=0;imu_px<1280;imu_px=imu_px+1)begin
     imu_x_geometry[imu_px]=8'hf8;
     imu_offset=imu_px-OSD_X-16;
     if(imu_offset>=0 && imu_offset<20*28 && imu_offset%28<24)
       imu_x_geometry[imu_px]=((imu_offset/28)<<3)|((imu_offset%28)/3);
   end
   for(imu_py=0;imu_py<720;imu_py=imu_py+1)begin
     imu_y_geometry[imu_py]=0;
     for(imu_line=0;imu_line<12;imu_line=imu_line+1)begin
       imu_start=OSD_Y+(imu_line/4)*232+(imu_line%4)*52+8;
       if(imu_py>=imu_start && imu_py<imu_start+48)
         imu_y_geometry[imu_py]=256+(imu_line<<4)+(imu_py-imu_start)/3;
     end
   end
 end
 always @(posedge clk)begin
   imu_x_coord<=imu_x_geometry[pixel_x[10:0]];
   imu_y_coord<=imu_y_geometry[pixel_y[9:0]];
 end
 wire current_panel=de_input_r && (display_page || !display_recognition || display_imu_panel) &&
   coord_x>=OSD_X && coord_y>=OSD_Y &&
   coord_x<OSD_X+(display_page ? 1216 : display_imu_panel ? 608 : 304) &&
   coord_y<OSD_Y+(display_page ? 648 : display_imu_panel ? 688 : 252) &&
   (display_page || !display_imu_panel || ly<224 || (ly>=232 && ly<456) || ly>=464);
 wire imu_border=current_panel && display_imu_panel && !display_page &&
   (lx<2 || lx>=606 || ly<2 || (ly>=222 && ly<224) || (ly>=232 && ly<234) ||
    (ly>=454 && ly<456) || (ly>=464 && ly<466) || ly>=686);
 wire [23:0] imu_border_color=ly<232?24'h37c4ff:ly<464?24'hb58aff:24'h3ee9a3;
 reg border_coord,border_s0,border_s1,border_s2;
 reg [23:0] border_color_coord,border_color_s0,border_color_s1,border_color_s2;
 reg panel,glyph_valid;
 reg [6:0] column;
 reg [4:0] row;
 reg [11:0] gy;
 wire [4:0] calculated_row;
 wire [11:0] calculated_gy;
 wire calculated_valid;
 // Decode constant vertical geometry beside the input coordinate register.
 // Both page formats travel with the same pixel, including 12-bit wrap and
 // the unbounded final row. No row compare/subtract/OR tree on the font stage.
 (* ram_style="block",syn_romstyle="block_rom" *) reg [35:0] text_y_geometry[0:4095];
 reg [35:0] text_geometry_coord;
 function [35:0] text_geometry_value;
   input integer value_y;
   integer text_offset,text_full_row,text_main_row,text_full_gy,text_main_gy;
   begin
     text_offset=(value_y-OSD_Y)&4095;
     text_full_row=text_offset/36;if(text_full_row>17)text_full_row=17;
     text_main_row=text_offset/24;if(text_main_row>9)text_main_row=9;
     text_full_gy=text_offset-text_full_row*36;text_main_gy=text_offset-text_main_row*24;
     text_geometry_value={text_full_gy<32,text_full_row[4:0],text_full_gy[11:0],
                          text_main_gy<16,text_main_row[4:0],text_main_gy[11:0]};
   end
 endfunction
 integer text_y;
 initial begin
   // Keep each constant-initialization loop below Gowin's 2000-iteration
   // limit, without changing a global synthesis limit or ROM contents.
   for(text_y=0;text_y<1024;text_y=text_y+1)text_y_geometry[text_y]=text_geometry_value(text_y);
   for(text_y=1024;text_y<2048;text_y=text_y+1)text_y_geometry[text_y]=text_geometry_value(text_y);
   for(text_y=2048;text_y<3072;text_y=text_y+1)text_y_geometry[text_y]=text_geometry_value(text_y);
   for(text_y=3072;text_y<4096;text_y=text_y+1)text_y_geometry[text_y]=text_geometry_value(text_y);
 end
 always @(posedge clk)text_geometry_coord<=text_y_geometry[pixel_y];
 assign calculated_valid=display_page ? text_geometry_coord[35] : text_geometry_coord[17];
 assign calculated_row=display_page ? text_geometry_coord[34:30] : text_geometry_coord[16:12];
 assign calculated_gy=display_page ? text_geometry_coord[29:18] : text_geometry_coord[11:0];
 reg [23:0] rgb_coord;
 reg hs_coord,vs_coord,de_coord;
 reg [2:0] col_coord;
 reg [3:0] row_coord;
 reg [2:0] status_reverse;
 reg [1:0] direction_reverse;
 reg [3:0] mode_reverse;
 reg [119:0] mode_label;
 reg [55:0] imu_label,motor_label;
 reg [31:0] dir_label;
 reg [6:0] ascii;
 // Decode independent dynamic fields beside the static ROM. Their final
 // selection uses the next existing character stage instead of a long chain
 // of overrides on the column-to-label_s0 path; pixel latency stays six.
 reg [34:0] dynamic_ascii;
 reg [4:0] dynamic_hit;
 reg [5:0] ni;
 reg [3:0] np;
 reg [6:0] static_row_coord[0:63];
 reg [3:0] number_offset_coord[0:15];
 reg select_number;
 function [6:0] static_label_character;
   input [12:0] address; // {display_page,row[4:0],column[6:0]}
   begin
     // Direct sparse character ROM removes the wide row-string mux followed
     // by a per-column barrel select from the label_s0 pixel path. Each source
     // row is exactly 76 bytes; padding and unlisted rows/columns are spaces.
     static_label_character=7'd32;
     case(address)
       // Full sensor row 0: ALL SENSOR DATA  SW7=1
       13'h1000: static_label_character=7'd65; // A
       13'h1001: static_label_character=7'd76; // L
       13'h1002: static_label_character=7'd76; // L
       13'h1004: static_label_character=7'd83; // S
       13'h1005: static_label_character=7'd69; // E
       13'h1006: static_label_character=7'd78; // N
       13'h1007: static_label_character=7'd83; // S
       13'h1008: static_label_character=7'd79; // O
       13'h1009: static_label_character=7'd82; // R
       13'h100b: static_label_character=7'd68; // D
       13'h100c: static_label_character=7'd65; // A
       13'h100d: static_label_character=7'd84; // T
       13'h100e: static_label_character=7'd65; // A
       13'h1011: static_label_character=7'd83; // S
       13'h1012: static_label_character=7'd87; // W
       13'h1013: static_label_character=7'd55; // 7
       13'h1014: static_label_character=7'd61; // =
       13'h1015: static_label_character=7'd49; // 1
       // Full sensor row 1: IMU:          MOTOR:          CMD:
       13'h1080: static_label_character=7'd73; // I
       13'h1081: static_label_character=7'd77; // M
       13'h1082: static_label_character=7'd85; // U
       13'h1083: static_label_character=7'd58; // :
       13'h108e: static_label_character=7'd77; // M
       13'h108f: static_label_character=7'd79; // O
       13'h1090: static_label_character=7'd84; // T
       13'h1091: static_label_character=7'd79; // O
       13'h1092: static_label_character=7'd82; // R
       13'h1093: static_label_character=7'd58; // :
       13'h109e: static_label_character=7'd67; // C
       13'h109f: static_label_character=7'd77; // M
       13'h10a0: static_label_character=7'd68; // D
       13'h10a1: static_label_character=7'd58; // :
       // Full sensor row 2: ACC (g)       X:             Y:             Z:
       13'h1100: static_label_character=7'd65; // A
       13'h1101: static_label_character=7'd67; // C
       13'h1102: static_label_character=7'd67; // C
       13'h1104: static_label_character=7'd40; // (
       13'h1105: static_label_character=7'd103; // g
       13'h1106: static_label_character=7'd41; // )
       13'h110e: static_label_character=7'd88; // X
       13'h110f: static_label_character=7'd58; // :
       13'h111d: static_label_character=7'd89; // Y
       13'h111e: static_label_character=7'd58; // :
       13'h112c: static_label_character=7'd90; // Z
       13'h112d: static_label_character=7'd58; // :
       // Full sensor row 3: GYRO (d/s)    X:             Y:             Z:
       13'h1180: static_label_character=7'd71; // G
       13'h1181: static_label_character=7'd89; // Y
       13'h1182: static_label_character=7'd82; // R
       13'h1183: static_label_character=7'd79; // O
       13'h1185: static_label_character=7'd40; // (
       13'h1186: static_label_character=7'd100; // d
       13'h1187: static_label_character=7'd47; // /
       13'h1188: static_label_character=7'd115; // s
       13'h1189: static_label_character=7'd41; // )
       13'h118e: static_label_character=7'd88; // X
       13'h118f: static_label_character=7'd58; // :
       13'h119d: static_label_character=7'd89; // Y
       13'h119e: static_label_character=7'd58; // :
       13'h11ac: static_label_character=7'd90; // Z
       13'h11ad: static_label_character=7'd58; // :
       // Full sensor row 4: MAG (uT)      X:             Y:             Z:
       13'h1200: static_label_character=7'd77; // M
       13'h1201: static_label_character=7'd65; // A
       13'h1202: static_label_character=7'd71; // G
       13'h1204: static_label_character=7'd40; // (
       13'h1205: static_label_character=7'd117; // u
       13'h1206: static_label_character=7'd84; // T
       13'h1207: static_label_character=7'd41; // )
       13'h120e: static_label_character=7'd88; // X
       13'h120f: static_label_character=7'd58; // :
       13'h121d: static_label_character=7'd89; // Y
       13'h121e: static_label_character=7'd58; // :
       13'h122c: static_label_character=7'd90; // Z
       13'h122d: static_label_character=7'd58; // :
       // Full sensor row 5: EULER (deg)   R:             P:             Y:
       13'h1280: static_label_character=7'd69; // E
       13'h1281: static_label_character=7'd85; // U
       13'h1282: static_label_character=7'd76; // L
       13'h1283: static_label_character=7'd69; // E
       13'h1284: static_label_character=7'd82; // R
       13'h1286: static_label_character=7'd40; // (
       13'h1287: static_label_character=7'd100; // d
       13'h1288: static_label_character=7'd101; // e
       13'h1289: static_label_character=7'd103; // g
       13'h128a: static_label_character=7'd41; // )
       13'h128e: static_label_character=7'd82; // R
       13'h128f: static_label_character=7'd58; // :
       13'h129d: static_label_character=7'd80; // P
       13'h129e: static_label_character=7'd58; // :
       13'h12ac: static_label_character=7'd89; // Y
       13'h12ad: static_label_character=7'd58; // :
       // Full sensor row 6: YAW REL (deg):
       13'h1300: static_label_character=7'd89; // Y
       13'h1301: static_label_character=7'd65; // A
       13'h1302: static_label_character=7'd87; // W
       13'h1304: static_label_character=7'd82; // R
       13'h1305: static_label_character=7'd69; // E
       13'h1306: static_label_character=7'd76; // L
       13'h1308: static_label_character=7'd40; // (
       13'h1309: static_label_character=7'd100; // d
       13'h130a: static_label_character=7'd101; // e
       13'h130b: static_label_character=7'd103; // g
       13'h130c: static_label_character=7'd41; // )
       13'h130d: static_label_character=7'd58; // :
       // Full sensor row 7: QUAT          W:             X:
       13'h1380: static_label_character=7'd81; // Q
       13'h1381: static_label_character=7'd85; // U
       13'h1382: static_label_character=7'd65; // A
       13'h1383: static_label_character=7'd84; // T
       13'h138e: static_label_character=7'd87; // W
       13'h138f: static_label_character=7'd58; // :
       13'h139d: static_label_character=7'd88; // X
       13'h139e: static_label_character=7'd58; // :
       // Full sensor row 8: QUAT          Y:             Z:
       13'h1400: static_label_character=7'd81; // Q
       13'h1401: static_label_character=7'd85; // U
       13'h1402: static_label_character=7'd65; // A
       13'h1403: static_label_character=7'd84; // T
       13'h140e: static_label_character=7'd89; // Y
       13'h140f: static_label_character=7'd58; // :
       13'h141d: static_label_character=7'd90; // Z
       13'h141e: static_label_character=7'd58; // :
       // Full sensor row 9: TEMP(C):                    HEIGHT(m):
       13'h1480: static_label_character=7'd84; // T
       13'h1481: static_label_character=7'd69; // E
       13'h1482: static_label_character=7'd77; // M
       13'h1483: static_label_character=7'd80; // P
       13'h1484: static_label_character=7'd40; // (
       13'h1485: static_label_character=7'd67; // C
       13'h1486: static_label_character=7'd41; // )
       13'h1487: static_label_character=7'd58; // :
       13'h149c: static_label_character=7'd72; // H
       13'h149d: static_label_character=7'd69; // E
       13'h149e: static_label_character=7'd73; // I
       13'h149f: static_label_character=7'd71; // G
       13'h14a0: static_label_character=7'd72; // H
       13'h14a1: static_label_character=7'd84; // T
       13'h14a2: static_label_character=7'd40; // (
       13'h14a3: static_label_character=7'd109; // m
       13'h14a4: static_label_character=7'd41; // )
       13'h14a5: static_label_character=7'd58; // :
       // Full sensor row 10: PRESS(Pa):                  REF PRESS(Pa):
       13'h1500: static_label_character=7'd80; // P
       13'h1501: static_label_character=7'd82; // R
       13'h1502: static_label_character=7'd69; // E
       13'h1503: static_label_character=7'd83; // S
       13'h1504: static_label_character=7'd83; // S
       13'h1505: static_label_character=7'd40; // (
       13'h1506: static_label_character=7'd80; // P
       13'h1507: static_label_character=7'd97; // a
       13'h1508: static_label_character=7'd41; // )
       13'h1509: static_label_character=7'd58; // :
       13'h151c: static_label_character=7'd82; // R
       13'h151d: static_label_character=7'd69; // E
       13'h151e: static_label_character=7'd70; // F
       13'h1520: static_label_character=7'd80; // P
       13'h1521: static_label_character=7'd82; // R
       13'h1522: static_label_character=7'd69; // E
       13'h1523: static_label_character=7'd83; // S
       13'h1524: static_label_character=7'd83; // S
       13'h1525: static_label_character=7'd40; // (
       13'h1526: static_label_character=7'd80; // P
       13'h1527: static_label_character=7'd97; // a
       13'h1528: static_label_character=7'd41; // )
       13'h1529: static_label_character=7'd58; // :
       // Full sensor row 11: WHEELS: RAW SPEED AND ENCODER COUNT16
       13'h1580: static_label_character=7'd87; // W
       13'h1581: static_label_character=7'd72; // H
       13'h1582: static_label_character=7'd69; // E
       13'h1583: static_label_character=7'd69; // E
       13'h1584: static_label_character=7'd76; // L
       13'h1585: static_label_character=7'd83; // S
       13'h1586: static_label_character=7'd58; // :
       13'h1588: static_label_character=7'd82; // R
       13'h1589: static_label_character=7'd65; // A
       13'h158a: static_label_character=7'd87; // W
       13'h158c: static_label_character=7'd83; // S
       13'h158d: static_label_character=7'd80; // P
       13'h158e: static_label_character=7'd69; // E
       13'h158f: static_label_character=7'd69; // E
       13'h1590: static_label_character=7'd68; // D
       13'h1592: static_label_character=7'd65; // A
       13'h1593: static_label_character=7'd78; // N
       13'h1594: static_label_character=7'd68; // D
       13'h1596: static_label_character=7'd69; // E
       13'h1597: static_label_character=7'd78; // N
       13'h1598: static_label_character=7'd67; // C
       13'h1599: static_label_character=7'd79; // O
       13'h159a: static_label_character=7'd68; // D
       13'h159b: static_label_character=7'd69; // E
       13'h159c: static_label_character=7'd82; // R
       13'h159e: static_label_character=7'd67; // C
       13'h159f: static_label_character=7'd79; // O
       13'h15a0: static_label_character=7'd85; // U
       13'h15a1: static_label_character=7'd78; // N
       13'h15a2: static_label_character=7'd84; // T
       13'h15a3: static_label_character=7'd49; // 1
       13'h15a4: static_label_character=7'd54; // 6
       // Full sensor row 12: LF:                          RF:
       13'h1600: static_label_character=7'd76; // L
       13'h1601: static_label_character=7'd70; // F
       13'h1602: static_label_character=7'd58; // :
       13'h161d: static_label_character=7'd82; // R
       13'h161e: static_label_character=7'd70; // F
       13'h161f: static_label_character=7'd58; // :
       // Full sensor row 13: LB:                          RB:
       13'h1680: static_label_character=7'd76; // L
       13'h1681: static_label_character=7'd66; // B
       13'h1682: static_label_character=7'd58; // :
       13'h169d: static_label_character=7'd82; // R
       13'h169e: static_label_character=7'd66; // B
       13'h169f: static_label_character=7'd58; // :
       // Full sensor row 14: LF COUNT:                    RF COUNT:
       13'h1700: static_label_character=7'd76; // L
       13'h1701: static_label_character=7'd70; // F
       13'h1703: static_label_character=7'd67; // C
       13'h1704: static_label_character=7'd79; // O
       13'h1705: static_label_character=7'd85; // U
       13'h1706: static_label_character=7'd78; // N
       13'h1707: static_label_character=7'd84; // T
       13'h1708: static_label_character=7'd58; // :
       13'h171d: static_label_character=7'd82; // R
       13'h171e: static_label_character=7'd70; // F
       13'h1720: static_label_character=7'd67; // C
       13'h1721: static_label_character=7'd79; // O
       13'h1722: static_label_character=7'd85; // U
       13'h1723: static_label_character=7'd78; // N
       13'h1724: static_label_character=7'd84; // T
       13'h1725: static_label_character=7'd58; // :
       // Full sensor row 15: LB COUNT:                    RB COUNT:
       13'h1780: static_label_character=7'd76; // L
       13'h1781: static_label_character=7'd66; // B
       13'h1783: static_label_character=7'd67; // C
       13'h1784: static_label_character=7'd79; // O
       13'h1785: static_label_character=7'd85; // U
       13'h1786: static_label_character=7'd78; // N
       13'h1787: static_label_character=7'd84; // T
       13'h1788: static_label_character=7'd58; // :
       13'h179d: static_label_character=7'd82; // R
       13'h179e: static_label_character=7'd66; // B
       13'h17a0: static_label_character=7'd67; // C
       13'h17a1: static_label_character=7'd79; // O
       13'h17a2: static_label_character=7'd85; // U
       13'h17a3: static_label_character=7'd78; // N
       13'h17a4: static_label_character=7'd84; // T
       13'h17a5: static_label_character=7'd58; // :
       // Full sensor row 16: AV:    ERR:            ULTRASONIC
       13'h1800: static_label_character=7'd65; // A
       13'h1801: static_label_character=7'd86; // V
       13'h1802: static_label_character=7'd58; // :
       13'h1807: static_label_character=7'd69; // E
       13'h1808: static_label_character=7'd82; // R
       13'h1809: static_label_character=7'd82; // R
       13'h180a: static_label_character=7'd58; // :
       13'h1817: static_label_character=7'd85; // U
       13'h1818: static_label_character=7'd76; // L
       13'h1819: static_label_character=7'd84; // T
       13'h181a: static_label_character=7'd82; // R
       13'h181b: static_label_character=7'd65; // A
       13'h181c: static_label_character=7'd83; // S
       13'h181d: static_label_character=7'd79; // O
       13'h181e: static_label_character=7'd78; // N
       13'h181f: static_label_character=7'd73; // I
       13'h1820: static_label_character=7'd67; // C
       // Full sensor row 17: FRONT:        cm LEFT:    cm     REAR:     cm     RIGHT:    cm              
       13'h1880: static_label_character=7'd70; // F
       13'h1881: static_label_character=7'd82; // R
       13'h1882: static_label_character=7'd79; // O
       13'h1883: static_label_character=7'd78; // N
       13'h1884: static_label_character=7'd84; // T
       13'h1885: static_label_character=7'd58; // :
       13'h188e: static_label_character=7'd99; // c
       13'h188f: static_label_character=7'd109; // m
       13'h1891: static_label_character=7'd76; // L
       13'h1892: static_label_character=7'd69; // E
       13'h1893: static_label_character=7'd70; // F
       13'h1894: static_label_character=7'd84; // T
       13'h1895: static_label_character=7'd58; // :
       13'h189a: static_label_character=7'd99; // c
       13'h189b: static_label_character=7'd109; // m
       13'h18a1: static_label_character=7'd82; // R
       13'h18a2: static_label_character=7'd69; // E
       13'h18a3: static_label_character=7'd65; // A
       13'h18a4: static_label_character=7'd82; // R
       13'h18a5: static_label_character=7'd58; // :
       13'h18ab: static_label_character=7'd99; // c
       13'h18ac: static_label_character=7'd109; // m
       13'h18b2: static_label_character=7'd82; // R
       13'h18b3: static_label_character=7'd73; // I
       13'h18b4: static_label_character=7'd71; // G
       13'h18b5: static_label_character=7'd72; // H
       13'h18b6: static_label_character=7'd84; // T
       13'h18b7: static_label_character=7'd58; // :
       13'h18bc: static_label_character=7'd99; // c
       13'h18bd: static_label_character=7'd109; // m
       // Camera row 0: IMU:          MOTOR:
       13'h0000: static_label_character=7'd73; // I
       13'h0001: static_label_character=7'd77; // M
       13'h0002: static_label_character=7'd85; // U
       13'h0003: static_label_character=7'd58; // :
       13'h000e: static_label_character=7'd77; // M
       13'h000f: static_label_character=7'd79; // O
       13'h0010: static_label_character=7'd84; // T
       13'h0011: static_label_character=7'd79; // O
       13'h0012: static_label_character=7'd82; // R
       13'h0013: static_label_character=7'd58; // :
       // Camera row 1: CMD:
       13'h0080: static_label_character=7'd67; // C
       13'h0081: static_label_character=7'd77; // M
       13'h0082: static_label_character=7'd68; // D
       13'h0083: static_label_character=7'd58; // :
       // Camera row 2: ROLL:             PITCH:
       13'h0100: static_label_character=7'd82; // R
       13'h0101: static_label_character=7'd79; // O
       13'h0102: static_label_character=7'd76; // L
       13'h0103: static_label_character=7'd76; // L
       13'h0104: static_label_character=7'd58; // :
       13'h0112: static_label_character=7'd80; // P
       13'h0113: static_label_character=7'd73; // I
       13'h0114: static_label_character=7'd84; // T
       13'h0115: static_label_character=7'd67; // C
       13'h0116: static_label_character=7'd72; // H
       13'h0117: static_label_character=7'd58; // :
       // Camera row 3: YAW REL:          YAW ABS:
       13'h0180: static_label_character=7'd89; // Y
       13'h0181: static_label_character=7'd65; // A
       13'h0182: static_label_character=7'd87; // W
       13'h0184: static_label_character=7'd82; // R
       13'h0185: static_label_character=7'd69; // E
       13'h0186: static_label_character=7'd76; // L
       13'h0187: static_label_character=7'd58; // :
       13'h0192: static_label_character=7'd89; // Y
       13'h0193: static_label_character=7'd65; // A
       13'h0194: static_label_character=7'd87; // W
       13'h0196: static_label_character=7'd65; // A
       13'h0197: static_label_character=7'd66; // B
       13'h0198: static_label_character=7'd83; // S
       13'h0199: static_label_character=7'd58; // :
       // Camera row 4: GZ(d/s):          TEMP(C):
       13'h0200: static_label_character=7'd71; // G
       13'h0201: static_label_character=7'd90; // Z
       13'h0202: static_label_character=7'd40; // (
       13'h0203: static_label_character=7'd100; // d
       13'h0204: static_label_character=7'd47; // /
       13'h0205: static_label_character=7'd115; // s
       13'h0206: static_label_character=7'd41; // )
       13'h0207: static_label_character=7'd58; // :
       13'h0212: static_label_character=7'd84; // T
       13'h0213: static_label_character=7'd69; // E
       13'h0214: static_label_character=7'd77; // M
       13'h0215: static_label_character=7'd80; // P
       13'h0216: static_label_character=7'd40; // (
       13'h0217: static_label_character=7'd67; // C
       13'h0218: static_label_character=7'd41; // )
       13'h0219: static_label_character=7'd58; // :
       // Camera row 5: LF:              RF:
       13'h0280: static_label_character=7'd76; // L
       13'h0281: static_label_character=7'd70; // F
       13'h0282: static_label_character=7'd58; // :
       13'h0291: static_label_character=7'd82; // R
       13'h0292: static_label_character=7'd70; // F
       13'h0293: static_label_character=7'd58; // :
       // Camera row 6: LB:              RB:
       13'h0300: static_label_character=7'd76; // L
       13'h0301: static_label_character=7'd66; // B
       13'h0302: static_label_character=7'd58; // :
       13'h0311: static_label_character=7'd82; // R
       13'h0312: static_label_character=7'd66; // B
       13'h0313: static_label_character=7'd58; // :
       // Camera row 7: FRONT:     cm    REAR:     cm
       13'h0380: static_label_character=7'd70; // F
       13'h0381: static_label_character=7'd82; // R
       13'h0382: static_label_character=7'd79; // O
       13'h0383: static_label_character=7'd78; // N
       13'h0384: static_label_character=7'd84; // T
       13'h0385: static_label_character=7'd58; // :
       13'h038b: static_label_character=7'd99; // c
       13'h038c: static_label_character=7'd109; // m
       13'h0391: static_label_character=7'd82; // R
       13'h0392: static_label_character=7'd69; // E
       13'h0393: static_label_character=7'd65; // A
       13'h0394: static_label_character=7'd82; // R
       13'h0395: static_label_character=7'd58; // :
       13'h039b: static_label_character=7'd99; // c
       13'h039c: static_label_character=7'd109; // m
       // Camera row 8: LEFT:     cm     RIGHT:    cm         
       13'h0400: static_label_character=7'd76; // L
       13'h0401: static_label_character=7'd69; // E
       13'h0402: static_label_character=7'd70; // F
       13'h0403: static_label_character=7'd84; // T
       13'h0404: static_label_character=7'd58; // :
       13'h040a: static_label_character=7'd99; // c
       13'h040b: static_label_character=7'd109; // m
       13'h0411: static_label_character=7'd82; // R
       13'h0412: static_label_character=7'd73; // I
       13'h0413: static_label_character=7'd71; // G
       13'h0414: static_label_character=7'd72; // H
       13'h0415: static_label_character=7'd84; // T
       13'h0416: static_label_character=7'd58; // :
       13'h041b: static_label_character=7'd99; // c
       13'h041c: static_label_character=7'd109; // m
       // Camera row 9: AV:    ERR:
       13'h0480: static_label_character=7'd65; // A
       13'h0481: static_label_character=7'd86; // V
       13'h0482: static_label_character=7'd58; // :
       13'h0487: static_label_character=7'd69; // E
       13'h0488: static_label_character=7'd82; // R
       13'h0489: static_label_character=7'd82; // R
       13'h048a: static_label_character=7'd58; // :
       default:static_label_character=7'd32;
     endcase
   end
 endfunction
 // The status belongs to the same frame snapshot as its numeric value.
 // Raw driver errors take precedence over stale/unknown, and measured_valid
 // distinguishes a normal reading from the saturated far-range convention.
 function [6:0] side_status_character;
   input right_side;
   input [1:0] position;
   reg [1:0] sensor_status;
   reg sensor_known,sensor_measured;
   reg [23:0] text;
   begin
     sensor_status=right_side ? display_side_status[3:2] : display_side_status[1:0];
     sensor_known=right_side ? display_distance_known[3] : display_distance_known[2];
     sensor_measured=right_side ? display_side_measured[1] : display_side_measured[0];
     if(sensor_status>=2) text="ERR";
     else if(!sensor_known) text="OLD";
     else if(sensor_status==1) text="NOE";
     else if(!sensor_measured) text="FAR";
     else text="OK ";
     case(position)
       0:side_status_character=text[22:16];
       1:side_status_character=text[14:8];
       2:side_status_character=text[6:0];
       default:side_status_character=" ";
     endcase
   end
 endfunction
 function [6:0] avoidance_enable_character;
   input [1:0] position;
   begin
     case(position)
       0:avoidance_enable_character="O";
       1:avoidance_enable_character=display_avoidance_enabled ? "N" : "F";
       2:avoidance_enable_character=display_avoidance_enabled ? " " : "F";
       default:avoidance_enable_character=" ";
     endcase
   end
 endfunction
 // All numeric windows are mutually exclusive. Masked OR keeps their
 // formatter/index selection parallel instead of inferring 47 priority muxes.
 wire [46:0] number_window;
 assign number_window[0]=display_page && (row==2 && column>=17 && column<25);
 assign number_window[1]=display_page && (row==2 && column>=32 && column<40);
 assign number_window[2]=display_page && (row==2 && column>=47 && column<55);
 assign number_window[3]=display_page && (row==3 && column>=17 && column<25);
 assign number_window[4]=display_page && (row==3 && column>=32 && column<40);
 assign number_window[5]=display_page && (row==3 && column>=47 && column<55);
 assign number_window[6]=display_page && (row==4 && column>=17 && column<25);
 assign number_window[7]=display_page && (row==4 && column>=32 && column<40);
 assign number_window[8]=display_page && (row==4 && column>=47 && column<55);
 assign number_window[9]=display_page && (row==5 && column>=17 && column<25);
 assign number_window[10]=display_page && (row==5 && column>=32 && column<40);
 assign number_window[11]=display_page && (row==5 && column>=47 && column<55);
 assign number_window[12]=display_page && (row==6 && column>=15 && column<23);
 assign number_window[13]=display_page && (row==7 && column>=17 && column<25);
 assign number_window[14]=display_page && (row==7 && column>=32 && column<40);
 assign number_window[15]=display_page && (row==8 && column>=17 && column<25);
 assign number_window[16]=display_page && (row==8 && column>=32 && column<40);
 assign number_window[17]=display_page && (row==9 && column>=9 && column<17);
 assign number_window[18]=display_page && (row==9 && column>=39 && column<47);
 assign number_window[19]=display_page && (row==10 && column>=11 && column<19);
 assign number_window[20]=display_page && (row==10 && column>=43 && column<51);
 assign number_window[21]=display_page && (row==12 && column>=4 && column<12);
 assign number_window[22]=display_page && (row==12 && column>=33 && column<41);
 assign number_window[23]=display_page && (row==13 && column>=4 && column<12);
 assign number_window[24]=display_page && (row==13 && column>=33 && column<41);
 assign number_window[25]=display_page && (row==14 && column>=10 && column<18);
 assign number_window[26]=display_page && (row==14 && column>=39 && column<47);
 assign number_window[27]=display_page && (row==15 && column>=10 && column<18);
 assign number_window[28]=display_page && (row==15 && column>=39 && column<47);
 assign number_window[29]=display_page && (row==17 && column>=10 && column<13);
 assign number_window[30]=display_page && (row==17 && column>=39 && column<42);
 assign number_window[31]=display_page && (row==17 && column>=22 && column<25);
 assign number_window[32]=display_page && (row==17 && column>=56 && column<59);
 assign number_window[33]=!display_page && (row==2 && column>=6 && column<14);
 assign number_window[34]=!display_page && (row==2 && column>=25 && column<33);
 assign number_window[35]=!display_page && (row==3 && column>=9 && column<17);
 assign number_window[36]=!display_page && (row==3 && column>=27 && column<35);
 assign number_window[37]=!display_page && (row==4 && column>=9 && column<17);
 assign number_window[38]=!display_page && (row==4 && column>=27 && column<35);
 assign number_window[39]=!display_page && (row==5 && column>=4 && column<12);
 assign number_window[40]=!display_page && (row==5 && column>=21 && column<29);
 assign number_window[41]=!display_page && (row==6 && column>=4 && column<12);
 assign number_window[42]=!display_page && (row==6 && column>=21 && column<29);
 assign number_window[43]=!display_page && (row==7 && column>=7 && column<10);
 assign number_window[44]=!display_page && (row==7 && column>=23 && column<26);
 assign number_window[45]=!display_page && (row==8 && column>=6 && column<9);
 assign number_window[46]=!display_page && (row==8 && column>=23 && column<26);
 always @* begin
   status_reverse=0;direction_reverse=0;mode_reverse=0;
   mode_label=mode_text(display_motion);
   imu_label=display_known[2] ? "ONLINE " : "WAIT   ";
   motor_label=display_online ? "ONLINE " : "WAIT   ";
   dir_label="    ";ascii=static_row_coord[{display_page,row}];ni=0;np=0;select_number=0;
   dynamic_ascii=0;dynamic_hit=0;
   ni=
     ({6{number_window[0]}} & 6'd0) |
     ({6{number_window[1]}} & 6'd1) |
     ({6{number_window[2]}} & 6'd2) |
     ({6{number_window[3]}} & 6'd3) |
     ({6{number_window[4]}} & 6'd4) |
     ({6{number_window[5]}} & 6'd5) |
     ({6{number_window[6]}} & 6'd6) |
     ({6{number_window[7]}} & 6'd7) |
     ({6{number_window[8]}} & 6'd8) |
     ({6{number_window[9]}} & 6'd13) |
     ({6{number_window[10]}} & 6'd14) |
     ({6{number_window[11]}} & 6'd15) |
     ({6{number_window[12]}} & 6'd20) |
     ({6{number_window[13]}} & 6'd9) |
     ({6{number_window[14]}} & 6'd10) |
     ({6{number_window[15]}} & 6'd11) |
     ({6{number_window[16]}} & 6'd12) |
     ({6{number_window[17]}} & 6'd17) |
     ({6{number_window[18]}} & 6'd16) |
     ({6{number_window[19]}} & 6'd18) |
     ({6{number_window[20]}} & 6'd19) |
     ({6{number_window[21]}} & 6'd21) |
     ({6{number_window[22]}} & 6'd22) |
     ({6{number_window[23]}} & 6'd23) |
     ({6{number_window[24]}} & 6'd24) |
     ({6{number_window[25]}} & 6'd25) |
     ({6{number_window[26]}} & 6'd26) |
     ({6{number_window[27]}} & 6'd27) |
     ({6{number_window[28]}} & 6'd28) |
     ({6{number_window[29]}} & 6'd29) |
     ({6{number_window[30]}} & 6'd30) |
     ({6{number_window[31]}} & 6'd31) |
     ({6{number_window[32]}} & 6'd32) |
     ({6{number_window[33]}} & 6'd13) |
     ({6{number_window[34]}} & 6'd14) |
     ({6{number_window[35]}} & 6'd20) |
     ({6{number_window[36]}} & 6'd15) |
     ({6{number_window[37]}} & 6'd5) |
     ({6{number_window[38]}} & 6'd17) |
     ({6{number_window[39]}} & 6'd21) |
     ({6{number_window[40]}} & 6'd22) |
     ({6{number_window[41]}} & 6'd23) |
     ({6{number_window[42]}} & 6'd24) |
     ({6{number_window[43]}} & 6'd29) |
     ({6{number_window[44]}} & 6'd30) |
     ({6{number_window[45]}} & 6'd31) |
     ({6{number_window[46]}} & 6'd32);
   np=
     ({4{number_window[0]}} & number_offset_coord[1]) |
     ({4{number_window[1]}} & number_offset_coord[0]) |
     ({4{number_window[2]}} & number_offset_coord[15]) |
     ({4{number_window[3]}} & number_offset_coord[1]) |
     ({4{number_window[4]}} & number_offset_coord[0]) |
     ({4{number_window[5]}} & number_offset_coord[15]) |
     ({4{number_window[6]}} & number_offset_coord[1]) |
     ({4{number_window[7]}} & number_offset_coord[0]) |
     ({4{number_window[8]}} & number_offset_coord[15]) |
     ({4{number_window[9]}} & number_offset_coord[1]) |
     ({4{number_window[10]}} & number_offset_coord[0]) |
     ({4{number_window[11]}} & number_offset_coord[15]) |
     ({4{number_window[12]}} & number_offset_coord[15]) |
     ({4{number_window[13]}} & number_offset_coord[1]) |
     ({4{number_window[14]}} & number_offset_coord[0]) |
     ({4{number_window[15]}} & number_offset_coord[1]) |
     ({4{number_window[16]}} & number_offset_coord[0]) |
     ({4{number_window[17]}} & number_offset_coord[9]) |
     ({4{number_window[18]}} & number_offset_coord[7]) |
     ({4{number_window[19]}} & number_offset_coord[11]) |
     ({4{number_window[20]}} & number_offset_coord[11]) |
     ({4{number_window[21]}} & number_offset_coord[4]) |
     ({4{number_window[22]}} & number_offset_coord[1]) |
     ({4{number_window[23]}} & number_offset_coord[4]) |
     ({4{number_window[24]}} & number_offset_coord[1]) |
     ({4{number_window[25]}} & number_offset_coord[10]) |
     ({4{number_window[26]}} & number_offset_coord[7]) |
     ({4{number_window[27]}} & number_offset_coord[10]) |
     ({4{number_window[28]}} & number_offset_coord[7]) |
     ({4{number_window[29]}} & number_offset_coord[10]) |
     ({4{number_window[30]}} & number_offset_coord[7]) |
     ({4{number_window[31]}} & number_offset_coord[6]) |
     ({4{number_window[32]}} & number_offset_coord[8]) |
     ({4{number_window[33]}} & number_offset_coord[6]) |
     ({4{number_window[34]}} & number_offset_coord[9]) |
     ({4{number_window[35]}} & number_offset_coord[9]) |
     ({4{number_window[36]}} & number_offset_coord[11]) |
     ({4{number_window[37]}} & number_offset_coord[9]) |
     ({4{number_window[38]}} & number_offset_coord[11]) |
     ({4{number_window[39]}} & number_offset_coord[4]) |
     ({4{number_window[40]}} & number_offset_coord[5]) |
     ({4{number_window[41]}} & number_offset_coord[4]) |
     ({4{number_window[42]}} & number_offset_coord[5]) |
     ({4{number_window[43]}} & number_offset_coord[7]) |
     ({4{number_window[44]}} & number_offset_coord[7]) |
     ({4{number_window[45]}} & number_offset_coord[6]) |
     ({4{number_window[46]}} & number_offset_coord[7]);
   select_number=|number_window;
   if(panel && glyph_valid) begin
     if((display_page && row==16) || (!display_page && row==9)) begin
       if(column==3) begin dynamic_hit[0]=1;dynamic_ascii[0+:7]=display_av_ascii[14:8];end
       if(column==4) begin dynamic_hit[0]=1;dynamic_ascii[0+:7]=display_av_ascii[6:0];end
       if(column==11) begin dynamic_hit[0]=1;dynamic_ascii[0+:7]=display_fault_ascii[14:8];end
       if(column==12) begin dynamic_hit[0]=1;dynamic_ascii[0+:7]=display_fault_ascii[6:0];end
       if(column>=16 && column<19) begin dynamic_hit[0]=1;dynamic_ascii[0+:7]=avoidance_enable_character(column-16);end
     end
     if(!display_page && row==8) begin
       if(column>=13 && column<16) begin dynamic_hit[1]=1;dynamic_ascii[7+:7]=side_status_character(0,column-13);end
       if(column>=30 && column<33) begin dynamic_hit[1]=1;dynamic_ascii[7+:7]=side_status_character(1,column-30);end
     end
     if(display_page && row==17) begin
       if(column>=29 && column<32) begin dynamic_hit[1]=1;dynamic_ascii[7+:7]=side_status_character(0,column-29);end
       if(column>=63 && column<66) begin dynamic_hit[1]=1;dynamic_ascii[7+:7]=side_status_character(1,column-63);end
     end
     if((display_page && row==1) || (!display_page && row==0)) begin
       if(column>=5 && column<12) begin dynamic_hit[2]=1;status_reverse=3'd3-column[2:0];dynamic_ascii[14+:7]=imu_label[{status_reverse,3'b000}+:7];end
       if(column>=20 && column<27) begin dynamic_hit[2]=1;status_reverse=3'd2-column[2:0];dynamic_ascii[14+:7]=motor_label[{status_reverse,3'b000}+:7];end
     end
     if((!display_page && row==1 && column>=5 && column<20) ||
        (display_page && row==1 && column>=35 && column<50))
       begin dynamic_hit[3]=1;mode_reverse=(display_page ? 4'd1 : 4'd3)-column[3:0];dynamic_ascii[21+:7]=mode_label[{mode_reverse,3'b000}+:7];end
     if(!display_page && (row==5 || row==6)) begin
       if(column>=12 && column<16) begin
         dynamic_hit[4]=1;dir_label=direction_text(row==5 ? 21 : 23);direction_reverse=2'd3-column[1:0];dynamic_ascii[28+:7]=dir_label[{direction_reverse,3'b000}+:7];
       end
       if(column>=29 && column<33) begin
         dynamic_hit[4]=1;dir_label=direction_text(row==5 ? 22 : 24);direction_reverse=2'd0-column[1:0];dynamic_ascii[28+:7]=dir_label[{direction_reverse,3'b000}+:7];
       end
     end
   end else ascii=" ";
 end
 // Prefetch each fixed row beside the existing column register. Each ROM
 // has a constant page and row, so X sees only a small per-row character LUT;
 // the next stage selects a row from these already encoded characters.
 wire [11:0] text_input_lx=pixel_x-OSD_X;
 localparam [11:0] TEXT_RESET_LX=12'd0-OSD_X;
 reg [6:0] text_full_column_coord,text_main_column_coord;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin text_full_column_coord<=TEXT_RESET_LX[10:4];text_main_column_coord<=TEXT_RESET_LX[9:3];end
  else begin text_full_column_coord<=text_input_lx[10:4];text_main_column_coord<=text_input_lx[9:3];end
 wire [6:0] next_column=display_page ? text_full_column_coord : text_main_column_coord;
 genvar fixed_text_row,number_offset;
 generate
  for(fixed_text_row=0;fixed_text_row<64;fixed_text_row=fixed_text_row+1) begin: static_row_prefetch
   localparam [5:0] ADDRESS_ROW=fixed_text_row;
   // ADDRESS_ROW already contains this ROM's constant page. Select its
   // horizontal scale at elaboration, so the live display_page flag cannot
   // traverse the column mux and all 64 character ROMs in one pixel clock.
   wire [6:0] row_column=ADDRESS_ROW[5] ? text_full_column_coord : text_main_column_coord;
   always @(posedge clk)
    static_row_coord[fixed_text_row]<=static_label_character({ADDRESS_ROW,row_column});
  end
  for(number_offset=0;number_offset<16;number_offset=number_offset+1) begin: number_offset_prefetch
   localparam [3:0] OFFSET=number_offset;
   always @(posedge clk)
    number_offset_coord[number_offset]<=next_column[3:0]-OFFSET;
  end
 endgenerate
 wire [6:0] imu_ascii;
 reg [6:0] imu_ascii_s0;
 imu_nine_text #(.GROUPED(1)) u_main_imu_text(.text_row(row[3:0]),.text_column(column),
   .values_bcd(imu_nine_bcd),.values_negative(imu_nine_negative),.values_valid(imu_nine_valid),.ascii(imu_ascii));
 reg [6:0] label_s0,ascii_s1;
 reg [34:0] dynamic_ascii_s0;
 reg [4:0] dynamic_hit_s0;
 reg [5:0] ni_s0;
 reg [3:0] np_s0;
 reg number_s0;
 reg [2:0] col_s0;
 reg [3:0] row_s0;
 reg [23:0] rgb_s0;
 reg hs_s0,vs_s0,de_s0,panel_s0;
 wire [6:0] dynamic_label_s0=dynamic_ascii_s0[0+:7] | dynamic_ascii_s0[7+:7] |
   dynamic_ascii_s0[14+:7] | dynamic_ascii_s0[21+:7] | dynamic_ascii_s0[28+:7];
 wire [6:0] final_ascii=display_imu_panel && !display_page ? imu_ascii_s0 : number_s0 ? number_char(ni_s0,np_s0) :
   (|dynamic_hit_s0 ? dynamic_label_s0 : label_s0);
 reg [2:0] col_s1;
 reg [3:0] row_s1;
 reg [23:0] rgb_s1,rgb_s2;
 reg hs_s1,vs_s1,de_s1,hs_s2,vs_s2,de_s2,panel_s1,panel_s2;
 wire font_pixel;
 telemetry_font8x16 font(.clk(clk),.col(col_s1),.row(row_s1),.ascii(ascii_s1),.pixel(font_pixel));
 // Object pixels retain the same six register stages as RGB/HS/VS/DE:
 // input -> parallel comparisons/local offsets -> winner -> ROM/color -> font -> RGB.
 // Capture metadata on the comparison edge so a frame-bank swap cannot pair
 // the last pixel of one frame with the next frame's class or text origin.
 reg object_hit_s0,object_text_s0;
 reg object_hit_s1,object_text_s1,object_hit_s2,object_text_s2;
 wire [6:0] object_class_s0;
 reg [6:0] object_ascii_s1;
 reg [119:0] object_custom_coord,object_custom_s0;
 reg [3:0] object_length_coord,object_length_s0;
 always @(posedge clk)begin
   object_custom_coord<=object_custom_text;object_length_coord<=object_custom_length;
   object_custom_s0<=object_custom_coord;object_length_s0<=object_length_coord;
 end
 wire [3:0] object_char_s0,object_row_s0;
 reg [3:0] object_row_s1;
 wire [2:0] object_col_s0;
 reg [2:0] object_col_s1;
 wire [6:0] object_ascii;
 reg object_literal_s0;
 wire [6:0] object_literal_ascii_s0;
 reg [7:0] object_label_candidates_coord,object_badge_candidates_coord,object_selected_boxes_coord;
 // Candidate geometry is registered before the independent earlier-slot
 // occupancy reductions and winner qualification. Pixel latency is unchanged.
 reg [7:0] object_label_inks_coord,object_badge_inks_coord,object_box_borders_coord,object_box_bold_coord;
 reg [55:0] object_candidate_classes_coord,object_candidate_local_x_coord;
 reg [31:0] object_candidate_local_y_coord,object_badge_local_y_coord;
 reg [23:0] object_badge_local_x_coord;
 reg object_page_coord,object_banner_ink_coord;
 reg [6:0] object_banner_ascii_coord;
 reg [2:0] object_banner_col_coord;
 reg [3:0] object_banner_row_coord;
 reg [2:0] object_follow_coord,object_follow_s0;
 reg object_selected_s0;
 wire [7:0] object_label_winners,object_badge_winners;
 genvar color_slot;
 generate for(color_slot=0;color_slot<8;color_slot=color_slot+1) begin: object_color_winner
  // Compare/capture geometry first. Parallel constant-mask priority reductions
  // then feed the parallel payload reduction, never the raw pixel comparator.
  // This represents the identical winning mask and the same pixel interval.
  assign object_label_winners[color_slot]=object_label_candidates_coord[color_slot] &&
      !(object_label_candidates_coord & ((1<<color_slot)-1));
  assign object_badge_winners[color_slot]=object_badge_candidates_coord[color_slot] &&
      !(object_badge_candidates_coord & ((1<<color_slot)-1)) && !(|object_label_candidates_coord);
 end endgenerate
 wire [6:0] object_winner_class,object_winner_x,object_winner_badge_ascii;
 wire [3:0] object_winner_y;
 // These ARE the s0 payload registers, parallel with hit/text/selection.
 // Do not register them again: that pairs the preceding pixel's payload
 // with this pixel's hit mask. Banner muxing follows these local registers.
 reg [6:0] object_winner_class_q,object_winner_x_q,object_winner_badge_ascii_q;
 reg [3:0] object_winner_y_q;
 reg [6:0] object_banner_ascii_s0;
 reg object_banner_s0;
 reg [2:0] object_banner_col_s0;
 reg [3:0] object_banner_row_s0;
 assign object_class_s0=object_winner_class_q;
 assign object_char_s0=object_winner_x_q[6:3];
 assign object_literal_ascii_s0=object_banner_s0 ? object_banner_ascii_s0 : object_winner_badge_ascii_q;
 assign object_col_s0=object_banner_s0 ? object_banner_col_s0 : object_winner_x_q[2:0];
 assign object_row_s0=object_banner_s0 ? object_banner_row_s0 : object_winner_y_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   object_winner_class_q<=0;object_winner_x_q<=0;object_winner_badge_ascii_q<=32;object_winner_y_q<=0;
   object_banner_ascii_s0<=32;object_banner_col_s0<=0;object_banner_row_s0<=0;
  end else begin
   object_winner_class_q<=object_winner_class;object_winner_x_q<=object_winner_x;
   object_winner_badge_ascii_q<=object_winner_badge_ascii;object_winner_y_q<=object_winner_y;
   object_banner_ascii_s0<=object_banner_ascii_coord;object_banner_col_s0<=object_banner_col_coord;
   object_banner_row_s0<=object_banner_row_coord;
  end
 end
 // Explicit parallel masked terms plus reductions avoid procedural serial
 // OR chains on the priority-selected label/badge coordinates. Same pixel.
 wire [7:0] object_class_terms[0:6],object_x_terms[0:6],object_ascii_terms[0:6];
 wire [7:0] object_y_terms[0:3];
 genvar winner_bit,winner_item;
 generate for(winner_bit=0;winner_bit<7;winner_bit=winner_bit+1)begin:g_object_winner_bit
   for(winner_item=0;winner_item<8;winner_item=winner_item+1)begin:g_item
     localparam [6:0] BADGE_ASCII=49+winner_item;
     wire [6:0] badge_x={4'b0,object_badge_local_x_coord[winner_item*3+:3]};
     assign object_class_terms[winner_bit][winner_item]=object_label_winners[winner_item]&&object_candidate_classes_coord[winner_item*7+winner_bit];
     assign object_x_terms[winner_bit][winner_item]=(object_label_winners[winner_item]&&object_candidate_local_x_coord[winner_item*7+winner_bit])||
       (object_badge_winners[winner_item]&&badge_x[winner_bit]);
     assign object_ascii_terms[winner_bit][winner_item]=object_badge_winners[winner_item]&&BADGE_ASCII[winner_bit];
   end
   assign object_winner_class[winner_bit]=|object_class_terms[winner_bit];
   assign object_winner_x[winner_bit]=|object_x_terms[winner_bit];
   assign object_winner_badge_ascii[winner_bit]=|object_ascii_terms[winner_bit];
 end
 for(winner_bit=0;winner_bit<4;winner_bit=winner_bit+1)begin:g_object_winner_y
   for(winner_item=0;winner_item<8;winner_item=winner_item+1)begin:g_item
     assign object_y_terms[winner_bit][winner_item]=(object_label_winners[winner_item]&&object_candidate_local_y_coord[winner_item*4+winner_bit])||
       (object_badge_winners[winner_item]&&object_badge_local_y_coord[winner_item*4+winner_bit]);
   end
   assign object_winner_y[winner_bit]=|object_y_terms[winner_bit];
 end endgenerate
 wire object_selected_border=|(object_box_bold_coord & object_selected_boxes_coord);
 wire object_border=|object_box_borders_coord || object_selected_border;
 function [23:0] detection_color;
   input [6:0] category;
   begin
     case(category)
       80:detection_color=24'hff4040;
       81:detection_color=24'hffff00;
       82:detection_color=24'h00ff00;
       83:detection_color=24'h4080ff;
       default:detection_color=24'h00ff00;
     endcase
   end
 endfunction
 wire [7:0] object_border_winners;
 wire [23:0] object_border_terms[0:7];
 genvar ball_slot;
 generate for(ball_slot=0;ball_slot<8;ball_slot=ball_slot+1)begin:g_ball_border
   wire eligible=object_box_borders_coord[ball_slot] ||
     (object_box_bold_coord[ball_slot] && object_selected_boxes_coord[ball_slot]);
   wire [7:0] eligible_mask=object_box_borders_coord |
     (object_box_bold_coord & object_selected_boxes_coord);
   wire [6:0] category=object_candidate_classes_coord[ball_slot*7+:7];
   wire ball=category>=80 && category<=83;
   assign object_border_winners[ball_slot]=eligible && !(eligible_mask & ((1<<ball_slot)-1));
   assign object_border_terms[ball_slot]={24{object_border_winners[ball_slot]}} &
     (ball ? detection_color(category) : object_selected_border ?
       (object_follow_coord==1 ? 24'hffff00 : 24'h00ffff) : 24'h00ff00);
 end endgenerate
 wire [23:0] object_border_color=object_border_terms[0] | object_border_terms[1] |
   object_border_terms[2] | object_border_terms[3] | object_border_terms[4] |
   object_border_terms[5] | object_border_terms[6] | object_border_terms[7];
 reg [23:0] object_color_s1,object_color_s2;
 reg object_banner_coord,object_banner_s1,object_banner_s2;
 wire object_font_pixel;
 detection_class_label object_label_rom(.class_id(object_class_s0),
   .custom_text(object_custom_s0),.custom_length(object_length_s0),
   .char_index(object_char_s0),.supported(),.text_length(),.label_width(),.ascii(object_ascii));
 telemetry_font8x16 object_font(.clk(clk),.col(object_col_s1),.row(object_row_s1),
   .ascii(object_ascii_s1),.pixel(object_font_pixel));
 wire [23:0] source_rgb=display_page ? 24'h101c2c : rgb_input_r;
 wire [11:0] main_text_lx=display_imu_panel?lx-12'd8:lx;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)begin
     border_coord<=0;border_s0<=0;border_s1<=0;border_s2<=0;
     border_color_coord<=0;border_color_s0<=0;border_color_s1<=0;border_color_s2<=0;
   end else begin
     border_coord<=imu_border;border_s0<=border_coord;border_s1<=border_s0;border_s2<=border_s1;
     border_color_coord<=imu_border_color;border_color_s0<=border_color_coord;
     border_color_s1<=border_color_s0;border_color_s2<=border_color_s1;
   end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     panel<=0;glyph_valid<=0;column<=0;row<=0;gy<=0;
     object_literal_s0<=0;
     object_label_candidates_coord<=0;object_badge_candidates_coord<=0;object_selected_boxes_coord<=0;
     object_label_inks_coord<=0;object_badge_inks_coord<=0;object_box_borders_coord<=0;object_box_bold_coord<=0;
     object_candidate_classes_coord<=0;object_candidate_local_x_coord<=0;object_candidate_local_y_coord<=0;
     object_badge_local_x_coord<=0;object_badge_local_y_coord<=0;object_page_coord<=0;
     object_banner_ink_coord<=0;object_banner_ascii_coord<=7'd32;object_banner_col_coord<=0;object_banner_row_coord<=0;
     object_follow_coord<=0;object_follow_s0<=0;object_selected_s0<=0;
     object_color_s1<=24'h00ff00;object_color_s2<=24'h00ff00;
     object_banner_coord<=0;object_banner_s0<=0;object_banner_s1<=0;object_banner_s2<=0;
     object_ascii_s1<=7'd32;
     object_hit_s0<=0;object_text_s0<=0;
     object_hit_s1<=0;object_text_s1<=0;object_col_s1<=0;object_row_s1<=0;
     object_hit_s2<=0;object_text_s2<=0;
     rgb_coord<=0;hs_coord<=0;vs_coord<=0;de_coord<=0;col_coord<=0;row_coord<=0;
     label_s0<=" ";imu_ascii_s0<=32;dynamic_ascii_s0<=0;dynamic_hit_s0<=0;
     ni_s0<=0;np_s0<=0;number_s0<=0;col_s0<=0;row_s0<=0;
     rgb_s0<=0;hs_s0<=0;vs_s0<=0;de_s0<=0;panel_s0<=0;
     ascii_s1<=" ";col_s1<=0;row_s1<=0;rgb_s1<=0;rgb_s2<=0;rgb_out<=0;
     hs_s1<=0;vs_s1<=0;de_s1<=0;hs_s2<=0;vs_s2<=0;de_s2<=0;
     hs_out<=0;vs_out<=0;de_out<=0;panel_s1<=0;panel_s2<=0;
   end else begin
     panel<=current_panel;glyph_valid<=display_imu_panel && !display_page ? imu_y_coord[8] : calculated_valid;
     // SW7 replaces the whole camera image, including areas outside its panel.
     object_label_candidates_coord<=object_label_candidates;object_badge_candidates_coord<=object_badge_candidates;
     object_label_inks_coord<=object_label_inks;object_badge_inks_coord<=object_badge_inks;
     object_box_borders_coord<=object_box_borders;object_box_bold_coord<=object_box_bold;
     object_selected_boxes_coord<=object_selected_boxes;object_follow_coord<=object_follow_status;
     object_candidate_classes_coord<=object_candidate_classes;object_candidate_local_x_coord<=object_candidate_local_x;
     object_candidate_local_y_coord<=object_candidate_local_y;object_badge_local_x_coord<=object_badge_local_x;
     object_badge_local_y_coord<=object_badge_local_y;object_page_coord<=display_page;
     object_banner_coord<=object_banner_candidate;object_banner_ink_coord<=object_banner_ink;
     object_banner_ascii_coord<=object_banner_ascii;object_banner_col_coord<=object_banner_local_x;object_banner_row_coord<=object_banner_local_y;
     object_hit_s0<=object_banner_coord || ((!object_page_coord) && (|object_label_candidates_coord || |object_badge_candidates_coord));
     object_text_s0<=object_banner_coord ? object_banner_ink_coord : ((!object_page_coord) &&
       (|(object_label_winners & object_label_inks_coord) || |(object_badge_winners & object_badge_inks_coord)));
     object_literal_s0<=object_banner_coord || |object_badge_winners;
     object_selected_s0<=|((object_label_winners | object_badge_winners) & object_selected_boxes_coord);
     object_follow_s0<=object_follow_coord;object_banner_s0<=object_banner_coord;
     object_hit_s1<=object_hit_s0;object_text_s1<=object_text_s0;
     object_ascii_s1<=object_literal_s0 ? object_literal_ascii_s0 : object_ascii;object_col_s1<=object_col_s0;object_row_s1<=object_row_s0;
     object_color_s1<=object_banner_s0 ? (object_follow_s0==1 ? 24'hffff00 : object_follow_s0==2 ? 24'h00ffff : 24'hff4040) :
       (object_class_s0>=80 && object_class_s0<=83) ? detection_color(object_class_s0) :
       object_selected_s0 ? (object_follow_s0==1 ? 24'hffff00 : 24'h00ffff) : 24'h00ff00;
     object_banner_s1<=object_banner_s0;
     object_hit_s2<=object_hit_s1;object_text_s2<=object_text_s1;
     object_color_s2<=object_color_s1;object_banner_s2<=object_banner_s1;
     column<=display_page ? lx[10:4] : display_imu_panel ? {2'd0,imu_x_coord[7:3]} : main_text_lx[9:3];
     row<=display_imu_panel && !display_page ? {1'b0,imu_y_coord[7:4]} : calculated_row;gy<=calculated_gy;
     col_coord<=display_page ? lx[3:1] : display_imu_panel ? imu_x_coord[2:0] : main_text_lx[2:0];
     row_coord<=display_page ? calculated_gy[4:1] : display_imu_panel ? imu_y_coord[3:0] : calculated_gy[3:0];
     rgb_coord<=source_rgb;hs_coord<=hs_input_r;vs_coord<=vs_input_r;de_coord<=de_input_r;
     label_s0<=ascii;imu_ascii_s0<=glyph_valid ? imu_ascii : 7'd32;
     dynamic_ascii_s0<=dynamic_ascii;dynamic_hit_s0<=dynamic_hit;
     ni_s0<=ni;np_s0<=np;number_s0<=select_number && panel && glyph_valid;
     col_s0<=col_coord;row_s0<=row_coord;
     rgb_s0<=(!object_page_coord && object_border) ? object_border_color : rgb_coord;
     hs_s0<=hs_coord;vs_s0<=vs_coord;de_s0<=de_coord;panel_s0<=panel;
     ascii_s1<=final_ascii;col_s1<=col_s0;row_s1<=row_s0;
     rgb_s1<=rgb_s0;hs_s1<=hs_s0;vs_s1<=vs_s0;de_s1<=de_s0;panel_s1<=panel_s0;
     rgb_s2<=rgb_s1;hs_s2<=hs_s1;vs_s2<=vs_s1;de_s2<=de_s1;panel_s2<=panel_s1;
     hs_out<=hs_s2;vs_out<=vs_s2;de_out<=de_s2;
     if(!de_s2) rgb_out<=0;
     else if(object_hit_s2 && object_banner_s2) rgb_out<=(object_text_s2 && object_font_pixel) ? object_color_s2 : 24'h000000;
     else if(panel_s2) rgb_out<=border_s2 ? border_color_s2 : font_pixel ? 24'hffffff :
       {2'b00,rgb_s2[23:18],2'b00,rgb_s2[15:10],2'b00,rgb_s2[7:2]};
     else if(object_hit_s2) rgb_out<=(object_text_s2 && object_font_pixel) ? object_color_s2 : 24'h000000;
     else rgb_out<=rgb_s2;
   end
 end
endmodule
