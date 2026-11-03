// Coordinates use the full camera image and half-open rectangles:
// x1 <= x < x2, y1 <= y < y2. Each packed 64-bit box is {y2,x2,y1,x1}.
// Detection packets are staged, then swapped into the active bank only at a
// display frame boundary. This prevents a result from tearing midway through
// a displayed frame. A prepared zero-count result clears at that boundary.
// RGB is combinational so the existing telemetry RGB/timing pipeline is kept.
// Precompute inset edges when a result arrives. The pixel path performs only
// bounded coordinate comparisons. Label-local coordinates are evaluated only
// after a deterministic box selection, then registered in the OSD pipeline.
// Incoming coordinates have already been validated by detection_box_parser.
module person_box_overlay #(
    parameter MAX_BOXES = 8,
    parameter STALE_FRAMES = 30,
    parameter BORDER_PIXELS = 3,
    parameter CLASS_LABELS_ENABLE = 0,
    parameter TRACK_ID_LABEL_ENABLE = 0,
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter [23:0] BOX_COLOR = 24'h00ff00
)(
    input wire clk,
    input wire reset_n,
    input wire frame_begin,
    input wire boxes_valid,
    input wire [31:0] boxes_frame_id,
    input wire [3:0] boxes_count,
    input wire [MAX_BOXES*64-1:0] boxes_coords,
    input wire [MAX_BOXES*7-1:0] boxes_classes,
    input wire [119:0] boxes_label_text,
    input wire [3:0] boxes_label_length,
    input wire enable,
    input wire [2:0] follow_status,
    input wire [3:0] follow_selected_slot,
    input wire [31:0] follow_frame_id,
    input wire [31:0] follow_target_id,
    input wire [3:0] follow_reason,
    input wire [11:0] pixel_x,
    input wire [11:0] pixel_y,
    input wire de_in,
    input wire [23:0] rgb_in,
    output reg [23:0] rgb_out,
    output reg [3:0] active_box_count,
    output reg label_hit,label_text_hit,
    output reg [6:0] label_class,
    output wire [119:0] label_custom_text,
    output wire [3:0] label_custom_length,
    output reg [3:0] label_char_index,
    output reg [2:0] label_col,
    output reg [3:0] label_row,
    output reg label_literal_valid,label_banner,
    output reg [6:0] label_literal_ascii,
    output reg [23:0] label_color,
    // The board registers these parallel candidates before winner/color decode.
    output wire [MAX_BOXES-1:0] label_color_candidates,badge_color_candidates,label_color_selected,
    output wire [2:0] label_color_status,
    // Parallel board metadata. No priority encoder, selected-origin mux, or
    // color decoder is allowed before the OSD captures these candidates.
    output wire [MAX_BOXES-1:0] label_text_candidates,badge_text_candidates,
    output reg [MAX_BOXES-1:0] box_border_candidates,box_bold_candidates,
    output wire [MAX_BOXES*7-1:0] candidate_classes,candidate_local_x,
    output wire [MAX_BOXES*4-1:0] candidate_local_y,badge_local_y,
    output wire [MAX_BOXES*3-1:0] badge_local_x,
    output wire banner_candidate,banner_text_candidate,
    output wire [6:0] banner_ascii,
    output wire [2:0] banner_local_x,
    output wire [3:0] banner_local_y,
    output reg [$clog2(IMAGE_WIDTH+1)-1:0] label_origin_x,
    output reg [$clog2(IMAGE_HEIGHT+1)-1:0] label_origin_y
);
    localparam X_BITS = $clog2(IMAGE_WIDTH+1);
    localparam Y_BITS = $clog2(IMAGE_HEIGHT+1);
    localparam [15:0] BORDER_EDGE = BORDER_PIXELS;
    localparam [15:0] IMAGE_RIGHT = IMAGE_WIDTH;
    localparam [15:0] IMAGE_BOTTOM = IMAGE_HEIGHT;

    // Follow metadata is committed only with a complete display-frame snapshot.
    // Omitted/unknown ports in legacy consumers fail closed to the OFF mode.
    reg [31:0] pending_frame_id,active_frame_id;
    reg [2:0] display_follow_status;
    reg [MAX_BOXES-1:0] display_selected;
    reg [31:0] display_target_id;
    reg [3:0] display_reason;
    function [2:0] safe_follow_status;
        input [2:0] value;
        begin case(value)
            3'd1,3'd2,3'd3,3'd4:safe_follow_status=value;
            3'd5:safe_follow_status=TRACK_ID_LABEL_ENABLE ? value : 3'd0;
            default:safe_follow_status=0;
        endcase end
    endfunction
    function [6:0] hex_character;
        input [3:0] nibble;
        begin hex_character=nibble<10 ? 7'd48+nibble : 7'd65+nibble-7'd10;end
    endfunction
    // A permanent target ID replaces the transient slot in active tracking.
    // The one 32-character banner uses the existing pixel/font pipeline.
    function [6:0] tracking_banner_character;
        input [2:0] status;
        input [4:0] position;
        input [31:0] identity;
        input [3:0] why;
        reg [4:0] id_start,reason_start,id_offset;
        reg [3:0] identity_nibble;
        begin
            tracking_banner_character=7'd32;id_start=10;reason_start=19;
            case(status)
                2:case(position)
                    0:tracking_banner_character="F";1:tracking_banner_character="O";
                    2:tracking_banner_character="L";3:tracking_banner_character="L";
                    4:tracking_banner_character="O";5:tracking_banner_character="W";
                    7:tracking_banner_character="I";8:tracking_banner_character="D";9:tracking_banner_character=":";
                endcase
                3,5:begin
                    id_start=13;reason_start=22;
                    case(position)
                        0:tracking_banner_character=status==5 ? "W" : "L";
                        1:tracking_banner_character=status==5 ? "A" : "O";
                        2:tracking_banner_character=status==5 ? "I" : "S";
                        3:tracking_banner_character="T";
                        5:tracking_banner_character="S";6:tracking_banner_character="T";
                        7:tracking_banner_character="O";8:tracking_banner_character="P";
                        10:tracking_banner_character="I";11:tracking_banner_character="D";12:tracking_banner_character=":";
                    endcase
                end
                4:begin
                    id_start=14;reason_start=23;
                    case(position)
                        0:tracking_banner_character="B";1:tracking_banner_character="L";2:tracking_banner_character="O";
                        3:tracking_banner_character="C";4:tracking_banner_character="K";
                        6:tracking_banner_character="S";7:tracking_banner_character="T";
                        8:tracking_banner_character="O";9:tracking_banner_character="P";
                        11:tracking_banner_character="I";12:tracking_banner_character="D";13:tracking_banner_character=":";
                    endcase
                end
            endcase
            id_offset=position-id_start;identity_nibble=0;
            case(id_offset)
                0:identity_nibble=identity[31:28];1:identity_nibble=identity[27:24];
                2:identity_nibble=identity[23:20];3:identity_nibble=identity[19:16];
                4:identity_nibble=identity[15:12];5:identity_nibble=identity[11:8];
                6:identity_nibble=identity[7:4];7:identity_nibble=identity[3:0];
            endcase
            if(position>=id_start && position<id_start+5'd8)
                tracking_banner_character=hex_character(identity_nibble);
            if(position==reason_start) tracking_banner_character="R";
            if(position==reason_start+1) tracking_banner_character=hex_character(why);
        end
    endfunction
    wire [2:0] safe_status=safe_follow_status(follow_status);
    reg [3:0] display_slot;
    function [6:0] banner_character;
        input [2:0] status;
        input [4:0] position;
        input [3:0] slot;
        begin
            banner_character=7'd32;
            case(status)
                1:case(position)
                    0:banner_character="S";1:banner_character="E";2:banner_character="L";
                    3:banner_character="E";4:banner_character="C";5:banner_character="T";
                    7:banner_character=slot<8 ? 7'd49+slot : "-";
                    10:banner_character="0";11:banner_character="8";12:banner_character="/";
                    13:banner_character="5";14:banner_character="A";
                    17:banner_character="1";18:banner_character="C";
                endcase
                2:case(position)
                    0:banner_character="F";1:banner_character="O";2:banner_character="L";
                    3:banner_character="L";4:banner_character="O";5:banner_character="W";
                    7:banner_character=slot<8 ? 7'd49+slot : "-";
                    10:banner_character="4";11:banner_character="2";
                    13:banner_character="S";14:banner_character="E";15:banner_character="L";
                    16:banner_character="E";17:banner_character="C";18:banner_character="T";
                endcase
                3:case(position)
                    0:banner_character="L";1:banner_character="O";2:banner_character="S";3:banner_character="T";
                    5:banner_character="/";7:banner_character="S";8:banner_character="T";
                    9:banner_character="O";10:banner_character="P";
                    13:banner_character="4";14:banner_character="2";
                    16:banner_character="S";17:banner_character="E";18:banner_character="L";
                    19:banner_character="E";20:banner_character="C";21:banner_character="T";
                endcase
                4:case(position)
                    0:banner_character="B";1:banner_character="L";2:banner_character="O";
                    3:banner_character="C";4:banner_character="K";5:banner_character="E";6:banner_character="D";
                    8:banner_character="/";10:banner_character="S";11:banner_character="T";
                    12:banner_character="O";13:banner_character="P";
                    16:banner_character="4";17:banner_character="2";
                    19:banner_character="S";20:banner_character="E";21:banner_character="L";
                    22:banner_character="E";23:banner_character="C";24:banner_character="T";
                endcase
            endcase
        end
    endfunction
    reg pending_valid;
    reg [3:0] pending_count;
    reg [MAX_BOXES*64-1:0] pending_coords,active_coords;
    reg [MAX_BOXES*64-1:0] pending_inner,active_inner;
    reg [MAX_BOXES*64-1:0] pending_bold,active_bold,pending_badges,active_badges;
    reg [MAX_BOXES*64-1:0] active_badge_ink;
    reg [MAX_BOXES*7-1:0] pending_classes,active_classes;
    reg [MAX_BOXES*64-1:0] pending_labels,active_labels;
    reg [MAX_BOXES*64-1:0] pending_label_ink,active_label_ink;

    function [MAX_BOXES*64-1:0] inner_edges;
        input [MAX_BOXES*64-1:0] coords;
        integer index;
        reg [15:0] left_edge,top_edge,right_edge,bottom_edge;
        reg [16:0] inset_left,inset_top;
        begin
            inner_edges=0;
            for(index=0;index<MAX_BOXES;index=index+1) begin
                left_edge=coords[index*64+:16];
                top_edge=coords[index*64+16+:16];
                right_edge=coords[index*64+32+:16];
                bottom_edge=coords[index*64+48+:16];
                inset_left={1'b0,left_edge}+{1'b0,BORDER_EDGE};
                inset_top={1'b0,top_edge}+{1'b0,BORDER_EDGE};
                inner_edges[index*64+:16]=inset_left>{1'b0,IMAGE_RIGHT} ? IMAGE_RIGHT:inset_left[15:0];
                inner_edges[index*64+16+:16]=inset_top>{1'b0,IMAGE_BOTTOM} ? IMAGE_BOTTOM:inset_top[15:0];
                // Saturation also makes rectangles thinner than the border
                // render solid, without unsigned subtraction wrapping around.
                inner_edges[index*64+32+:16]=right_edge>BORDER_EDGE ? right_edge-BORDER_EDGE:16'b0;
                inner_edges[index*64+48+:16]=bottom_edge>BORDER_EDGE ? bottom_edge-BORDER_EDGE:16'b0;
            end
        end
    endfunction


    function [MAX_BOXES*64-1:0] bold_edges;
        input [MAX_BOXES*64-1:0] coords;
        integer i;
        reg [15:0] l,t,r,b;
        begin bold_edges=0;
            for(i=0;i<MAX_BOXES;i=i+1) begin
                l=coords[i*64+:16];t=coords[i*64+16+:16];r=coords[i*64+32+:16];b=coords[i*64+48+:16];
                bold_edges[i*64+:16]=l<IMAGE_WIDTH-6 ? l+16'd6 : IMAGE_WIDTH;
                bold_edges[i*64+16+:16]=t<IMAGE_HEIGHT-6 ? t+16'd6 : IMAGE_HEIGHT;
                bold_edges[i*64+32+:16]=r>6 ? r-16'd6 : 16'd0;
                bold_edges[i*64+48+:16]=b>6 ? b-16'd6 : 16'd0;
            end
        end
    endfunction
    // Prepare the inset rectangle once when committing its badge snapshot.
    // The pixel path then uses only comparisons, with no padding additions.
    function [MAX_BOXES*64-1:0] badge_ink_edges;
        input [MAX_BOXES*64-1:0] badges;
        integer i;
        reg [15:0] l,t;
        begin badge_ink_edges=0;
            for(i=0;i<MAX_BOXES;i=i+1)begin
                l=badges[i*64+:X_BITS];t=badges[i*64+16+:Y_BITS];
                badge_ink_edges[i*64+:16]=l+16'd2;
                badge_ink_edges[i*64+16+:16]=t+16'd2;
                badge_ink_edges[i*64+32+:16]=l+16'd10;
                badge_ink_edges[i*64+48+:16]=t+16'd18;
            end
        end
    endfunction
    function [MAX_BOXES*64-1:0] badge_edges;
        input [MAX_BOXES*64-1:0] coords;
        integer i;
        reg [15:0] l,t,b;
        begin badge_edges=0;
            for(i=0;i<MAX_BOXES;i=i+1) begin
                l=coords[i*64+:16];t=coords[i*64+16+:16];b=coords[i*64+48+:16];
                l=l<IMAGE_WIDTH-20 ? l+16'd8 : IMAGE_WIDTH-12;
                t=(b>24 && b-24>=t) ? b-16'd24 : t;
                if(t>IMAGE_HEIGHT-20) t=IMAGE_HEIGHT-20;
                badge_edges[i*64+:16]=l;badge_edges[i*64+16+:16]=t;
                badge_edges[i*64+32+:16]=l+16'd12;badge_edges[i*64+48+:16]=t+16'd20;
            end
        end
    endfunction

    // Validate sequence ordering once at receipt. This relatively long 32-bit
    // comparison drives only small control registers, never a wide payload CE.
    reg sequence_known;
    reg [31:0] latest_frame_id;
    reg [31:0] age_frames;
    // Parallel carry prefixes keep the exact 32-bit age increment off a
    // full-width ripple chain. Age still advances only at the original frame
    // boundary and retains the original saturation/expiry decision.
    wire [7:0] age_full4 /* synthesis syn_keep = 1 */;
    wire [1:0] age_full16 /* synthesis syn_keep = 1 */;
    wire [31:0] age_increment;
    genvar age_group,age_bit;
    generate
        for(age_group=0;age_group<8;age_group=age_group+1) begin: age_prefix4
            assign age_full4[age_group]=&age_frames[age_group*4+:4];
        end
        for(age_group=0;age_group<2;age_group=age_group+1) begin: age_prefix16
            assign age_full16[age_group]=&age_full4[age_group*4+:4];
        end
        for(age_bit=0;age_bit<32;age_bit=age_bit+1) begin: age_add_one
            if(age_bit==0) assign age_increment[age_bit]=~age_frames[age_bit];
            else if(age_bit<4)
                assign age_increment[age_bit]=age_frames[age_bit] ^ (&age_frames[age_bit-1:0]);
            else if(age_bit<16) begin
                if(age_bit%4==0)
                    assign age_increment[age_bit]=age_frames[age_bit] ^ (&age_full4[age_bit/4-1:0]);
                else
                    assign age_increment[age_bit]=age_frames[age_bit] ^
                        ((&age_full4[age_bit/4-1:0]) && (&age_frames[age_bit-1:age_bit/4*4]));
            end else begin
                if(age_bit==16)
                    assign age_increment[age_bit]=age_frames[age_bit] ^ age_full16[0];
                else if(age_bit<20)
                    assign age_increment[age_bit]=age_frames[age_bit] ^
                        (age_full16[0] && (&age_frames[age_bit-1:16]));
                else if(age_bit%4==0)
                    assign age_increment[age_bit]=age_frames[age_bit] ^
                        (age_full16[0] && (&age_full4[age_bit/4-1:4]));
                else
                    assign age_increment[age_bit]=age_frames[age_bit] ^
                        (age_full16[0] && (&age_full4[age_bit/4-1:4]) &&
                         (&age_frames[age_bit-1:age_bit/4*4]));
            end
        end
    endgenerate
    wire [31:0] sequence_delta=boxes_frame_id-latest_frame_id;
    wire raw_fresh=boxes_valid && boxes_count<=MAX_BOXES &&
        (!sequence_known || (sequence_delta!=0 && !sequence_delta[31]));
    wire fresh_result;
    wire [31:0] prepared_frame_id;
    wire [3:0] prepared_count;
    wire [MAX_BOXES*64-1:0] prepared_coords,prepared_inner,prepared_bold,prepared_badges;
    wire [MAX_BOXES*7-1:0] prepared_classes;
    wire [MAX_BOXES*64-1:0] prepared_labels,prepared_label_ink;
    reg [119:0] custom_s0,custom_s1,custom_s2,custom_s3,pending_custom,active_custom;
    reg [3:0] length_s0,length_s1,length_s2,length_s3,pending_length,active_length;
    wire [119:0] prepared_custom=CLASS_LABELS_ENABLE ? custom_s3 : boxes_label_text;
    wire [3:0] prepared_length=CLASS_LABELS_ENABLE ? length_s3 : boxes_label_length;
    assign label_custom_text=active_custom;
    assign label_custom_length=active_length;
    // Match the existing four preparation stages exactly; text, class,
    // coordinates and width change together at the display frame boundary.
    always @(posedge clk)begin
        custom_s0<=boxes_label_text;custom_s1<=custom_s0;custom_s2<=custom_s1;custom_s3<=custom_s2;
        length_s0<=boxes_label_length;length_s1<=length_s0;length_s2<=length_s1;length_s3<=length_s2;
        if(fresh_result)begin pending_custom<=prepared_custom;pending_length<=prepared_length;end
        if(frame_begin)begin
            if(fresh_result)begin active_custom<=prepared_custom;active_length<=prepared_length;end
            else if(pending_valid)begin active_custom<=pending_custom;active_length<=pending_length;end
        end
    end
    wire [31:0] commit_frame_id=fresh_result ? prepared_frame_id : pending_valid ? pending_frame_id : active_frame_id;
    wire [3:0] commit_count=fresh_result ? prepared_count : pending_valid ? pending_count :
        age_frames>=STALE_FRAMES-1 ? 4'd0 : active_box_count;
    wire commit_selected_valid=(safe_status==1 || safe_status==2) &&
        follow_selected_slot<MAX_BOXES && follow_selected_slot<commit_count &&
        follow_frame_id==commit_frame_id;
    wire [3:0] commit_slot=commit_selected_valid ? follow_selected_slot : 4'd15;
    // Encode each character from the SAME incoming frame snapshot as the
    // display metadata. The pixel path only selects an already encoded ASCII
    // value; target-ID nibble selection and hexadecimal conversion no longer
    // follow the pixel-coordinate subtractor. No video pipeline stage is added.
    reg [6:0] display_banner_ascii [0:31];
    genvar banner_position;
    generate for(banner_position=0;banner_position<32;banner_position=banner_position+1) begin: banner_snapshot
        always @(posedge clk or negedge reset_n) begin
            if(!reset_n) display_banner_ascii[banner_position]<=7'd32;
            else if(frame_begin)
                display_banner_ascii[banner_position]<=TRACK_ID_LABEL_ENABLE && safe_status>=2 ?
                    tracking_banner_character(safe_status,banner_position,follow_target_id,follow_reason) :
                    banner_character(safe_status,banner_position,commit_slot);
        end
    end endgenerate
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            display_follow_status<=0;display_selected<=0;display_slot<=15;display_target_id<=0;display_reason<=0;
        end
        else if(frame_begin) begin
            display_follow_status<=safe_status;display_selected<=0;display_slot<=15;
            display_target_id<=TRACK_ID_LABEL_ENABLE ? follow_target_id : 32'd0;
            display_reason<=TRACK_ID_LABEL_ENABLE ? follow_reason : 4'd0;
            if(commit_selected_valid) begin
                display_selected[follow_selected_slot]<=1;display_slot<=follow_selected_slot;
            end
        end
    end


    generate
        if(CLASS_LABELS_ENABLE) begin: prepare_labels
            // Four preparation registers: capture/class width, limit/Y,
            // clamped X, then complete outer/ink rectangles. Payload registers
            // run every clock without reset or clock enable. Only the four
            // valid bits reset, so an interrupted session cannot replay data.
            reg [3:0] valid;
            reg [31:0] frame_s0,frame_s1,frame_s2,frame_s3;
            reg [3:0] count_s0,count_s1,count_s2,count_s3;
            reg [MAX_BOXES*64-1:0] coords_s0,coords_s1,coords_s2,coords_s3;
            reg [MAX_BOXES*7-1:0] classes_s0,classes_s1,classes_s2,classes_s3;
            reg [MAX_BOXES*7-1:0] widths_s0,widths_s1,widths_s2;
            reg [MAX_BOXES*X_BITS-1:0] limits_s1,left_s2;
            reg [MAX_BOXES*64-1:0] outer_y_s1,outer_y_s2,ink_y_s1,ink_y_s2;
            reg [MAX_BOXES*64-1:0] inner_s1,inner_s2,inner_s3;
            reg [MAX_BOXES*64-1:0] bold_s1,bold_s2,bold_s3,badges_s1,badges_s2,badges_s3;
            reg [MAX_BOXES*64-1:0] labels_s3,ink_s3;
            wire [MAX_BOXES*7-1:0] incoming_widths;
            localparam [X_BITS-1:0] RIGHT_MINUS_PAD=IMAGE_WIDTH-4;
            localparam [X_BITS-1:0] X_PAD=2;
            localparam [X_BITS-1:0] X_BORDER=BORDER_PIXELS;
            localparam [Y_BITS-1:0] Y_BORDER=BORDER_PIXELS;
            localparam [X_BITS:0] X_LIMIT=IMAGE_WIDTH;
            localparam [Y_BITS:0] Y_LIMIT=IMAGE_HEIGHT;
            localparam [Y_BITS-1:0] LABEL_HEIGHT=20;
            localparam [Y_BITS-1:0] Y_PAD=2;
            localparam [Y_BITS-1:0] LABEL_HEIGHT_MINUS_PAD=18;

            genvar slot;
            for(slot=0;slot<MAX_BOXES;slot=slot+1) begin: widths
                detection_class_label size_rom(
                    .class_id(boxes_classes[slot*7+:7]),.char_index(4'd0),
                    .custom_text(boxes_label_text),.custom_length(boxes_label_length),
                    .supported(),.text_length(),
                    .label_width(incoming_widths[slot*7+:7]),.ascii());
            end

            // Parser-validated camera coordinates need only 11 horizontal and
            // 10 vertical bits on this board. Keep the packed 16-bit ABI, but
            // make unused high bits constant through every preparation stage.
            function [MAX_BOXES*64-1:0] narrow_coords;
                input [MAX_BOXES*64-1:0] coords;
                integer index;
                begin
                    narrow_coords=0;
                    for(index=0;index<MAX_BOXES;index=index+1) begin
                        narrow_coords[index*64+:X_BITS]=coords[index*64+:X_BITS];
                        narrow_coords[index*64+16+:Y_BITS]=coords[index*64+16+:Y_BITS];
                        narrow_coords[index*64+32+:X_BITS]=coords[index*64+32+:X_BITS];
                        narrow_coords[index*64+48+:Y_BITS]=coords[index*64+48+:Y_BITS];
                    end
                end
            endfunction
            function [MAX_BOXES*64-1:0] narrow_inner;
                input [MAX_BOXES*64-1:0] coords;
                integer index;
                reg [X_BITS-1:0] left_edge,right_edge;
                reg [Y_BITS-1:0] top_edge,bottom_edge;
                reg [X_BITS:0] inset_left;
                reg [Y_BITS:0] inset_top;
                begin
                    narrow_inner=0;
                    for(index=0;index<MAX_BOXES;index=index+1) begin
                        left_edge=coords[index*64+:X_BITS];
                        top_edge=coords[index*64+16+:Y_BITS];
                        right_edge=coords[index*64+32+:X_BITS];
                        bottom_edge=coords[index*64+48+:Y_BITS];
                        inset_left={1'b0,left_edge}+{1'b0,X_BORDER};
                        inset_top={1'b0,top_edge}+{1'b0,Y_BORDER};
                        narrow_inner[index*64+:X_BITS]=inset_left>X_LIMIT ? X_LIMIT[X_BITS-1:0] : inset_left[X_BITS-1:0];
                        narrow_inner[index*64+16+:Y_BITS]=inset_top>Y_LIMIT ? Y_LIMIT[Y_BITS-1:0] : inset_top[Y_BITS-1:0];
                        narrow_inner[index*64+32+:X_BITS]=right_edge>X_BORDER ? right_edge-X_BORDER : {X_BITS{1'b0}};
                        narrow_inner[index*64+48+:Y_BITS]=bottom_edge>Y_BORDER ? bottom_edge-Y_BORDER : {Y_BITS{1'b0}};
                    end
                end
            endfunction
            function [MAX_BOXES*X_BITS-1:0] right_limits;
                input [MAX_BOXES*7-1:0] widths;
                integer index;
                begin
                    right_limits=0;
                    for(index=0;index<MAX_BOXES;index=index+1)
                        right_limits[index*X_BITS+:X_BITS]=RIGHT_MINUS_PAD-widths[index*7+:7];
                end
            endfunction
            function [MAX_BOXES*64-1:0] label_y;
                input [MAX_BOXES*64-1:0] coords;
                input ink;
                integer index;
                reg [Y_BITS-1:0] top_edge,top_out,bottom_out;
                begin
                    label_y=0;
                    for(index=0;index<MAX_BOXES;index=index+1) begin
                        top_edge=coords[index*64+16+:Y_BITS];
                        if(top_edge>=LABEL_HEIGHT) begin
                            top_out=ink ? top_edge-LABEL_HEIGHT_MINUS_PAD : top_edge-LABEL_HEIGHT;
                            bottom_out=ink ? top_edge-Y_PAD : top_edge;
                        end else begin
                            top_out=ink ? top_edge+Y_PAD : top_edge;
                            bottom_out=ink ? top_edge+LABEL_HEIGHT_MINUS_PAD : top_edge+LABEL_HEIGHT;
                        end
                        label_y[index*64+16+:Y_BITS]=top_out;
                        label_y[index*64+48+:Y_BITS]=bottom_out;
                    end
                end
            endfunction
            function [MAX_BOXES*X_BITS-1:0] clamp_left;
                input [MAX_BOXES*64-1:0] coords;
                input [MAX_BOXES*X_BITS-1:0] limits;
                integer index;
                reg [X_BITS-1:0] left_edge,right_limit;
                begin
                    clamp_left=0;
                    for(index=0;index<MAX_BOXES;index=index+1) begin
                        left_edge=coords[index*64+:X_BITS];
                        right_limit=limits[index*X_BITS+:X_BITS];
                        clamp_left[index*X_BITS+:X_BITS]=left_edge>right_limit ? right_limit : left_edge;
                    end
                end
            endfunction
            function [MAX_BOXES*64-1:0] finish_labels;
                input [MAX_BOXES*X_BITS-1:0] lefts;
                input [MAX_BOXES*7-1:0] widths;
                input [MAX_BOXES*64-1:0] y_edges;
                input ink;
                integer index;
                reg [X_BITS-1:0] left_edge,right_offset;
                begin
                    finish_labels=0;
                    for(index=0;index<MAX_BOXES;index=index+1) begin
                        left_edge=lefts[index*X_BITS+:X_BITS];
                        // width=len*8: +2/+4 changes only the low three bits.
                        // No second adder follows the left+width carry chain.
                        right_offset={widths[index*7+3+:4],ink ? 3'b010 : 3'b100};
                        if(widths[index*7+:7]!=0) begin
                            finish_labels[index*64+:X_BITS]=ink ? left_edge+X_PAD : left_edge;
                            finish_labels[index*64+32+:X_BITS]=left_edge+right_offset;
                            finish_labels[index*64+16+:Y_BITS]=y_edges[index*64+16+:Y_BITS];
                            finish_labels[index*64+48+:Y_BITS]=y_edges[index*64+48+:Y_BITS];
                        end
                    end
                end
            endfunction

            always @(posedge clk or negedge reset_n) begin
                if(!reset_n) valid<=0;
                else valid<={valid[2:0],raw_fresh};
            end
            always @(posedge clk) begin
                frame_s0<=boxes_frame_id;count_s0<=boxes_count;
                coords_s0<=narrow_coords(boxes_coords);classes_s0<=boxes_classes;
                widths_s0<=incoming_widths;
                frame_s1<=frame_s0;count_s1<=count_s0;coords_s1<=coords_s0;classes_s1<=classes_s0;
                widths_s1<=widths_s0;limits_s1<=right_limits(widths_s0);
                outer_y_s1<=label_y(coords_s0,1'b0);ink_y_s1<=label_y(coords_s0,1'b1);
                inner_s1<=narrow_inner(coords_s0);
                bold_s1<=bold_edges(coords_s0);badges_s1<=badge_edges(coords_s0);
                frame_s2<=frame_s1;count_s2<=count_s1;coords_s2<=coords_s1;classes_s2<=classes_s1;
                widths_s2<=widths_s1;left_s2<=clamp_left(coords_s1,limits_s1);
                outer_y_s2<=outer_y_s1;ink_y_s2<=ink_y_s1;inner_s2<=inner_s1;
                bold_s2<=bold_s1;badges_s2<=badges_s1;
                frame_s3<=frame_s2;count_s3<=count_s2;coords_s3<=coords_s2;classes_s3<=classes_s2;
                inner_s3<=inner_s2;bold_s3<=bold_s2;badges_s3<=badges_s2;
                labels_s3<=finish_labels(left_s2,widths_s2,outer_y_s2,1'b0);
                ink_s3<=finish_labels(left_s2,widths_s2,ink_y_s2,1'b1);
            end
            assign fresh_result=valid[3];
            assign prepared_frame_id=frame_s3;assign prepared_count=count_s3;
            assign prepared_coords=coords_s3;assign prepared_classes=classes_s3;
            assign prepared_bold=bold_s3;assign prepared_badges=badges_s3;
            assign prepared_inner=inner_s3;assign prepared_labels=labels_s3;
            assign prepared_label_ink=ink_s3;
        end else begin: immediate_boxes
            // Legacy border-only consumers retain zero preparation latency.
            assign fresh_result=raw_fresh;
            assign prepared_frame_id=boxes_frame_id;assign prepared_count=boxes_count;
            assign prepared_coords=boxes_coords;assign prepared_classes=boxes_classes;
            assign prepared_bold=bold_edges(boxes_coords);assign prepared_badges=badge_edges(boxes_coords);
            assign prepared_inner=inner_edges(boxes_coords);
            assign prepared_labels=0;assign prepared_label_ink=0;
        end
    endgenerate

    // Commit only complete prepared snapshots. The last preparation register
    // can arrive at the same frame boundary and takes priority over pending.
    // A raw result in the final four clocks before a boundary waits for the
    // following display frame; coordinates and names always switch together.
    // Wide payload banks have no reset. Their validity/count banks alone gate
    // visibility, avoiding reset and raw sequence fanout into the data path.
    always @(posedge clk) begin
        if(fresh_result) begin
            pending_frame_id<=prepared_frame_id;pending_bold<=prepared_bold;pending_badges<=prepared_badges;
            pending_coords<=prepared_coords;pending_inner<=prepared_inner;
            pending_classes<=prepared_classes;pending_labels<=prepared_labels;
            pending_label_ink<=prepared_label_ink;
        end
        if(frame_begin) begin
            if(fresh_result) begin
                active_frame_id<=prepared_frame_id;active_bold<=prepared_bold;active_badges<=prepared_badges;
                active_badge_ink<=badge_ink_edges(prepared_badges);
                active_coords<=prepared_coords;active_inner<=prepared_inner;
                active_classes<=prepared_classes;active_labels<=prepared_labels;
                active_label_ink<=prepared_label_ink;
            end else if(pending_valid) begin
                active_frame_id<=pending_frame_id;active_bold<=pending_bold;active_badges<=pending_badges;
                active_badge_ink<=badge_ink_edges(pending_badges);
                active_coords<=pending_coords;active_inner<=pending_inner;
                active_classes<=pending_classes;active_labels<=pending_labels;
                active_label_ink<=pending_label_ink;
            end
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            pending_valid<=0;pending_count<=0;active_box_count<=0;
            sequence_known<=0;latest_frame_id<=0;age_frames<=0;
        end else begin
            if(raw_fresh) begin
                sequence_known<=1;latest_frame_id<=boxes_frame_id;
            end
            if(fresh_result) begin
                pending_valid<=1;pending_count<=prepared_count;
            end
            if(frame_begin) begin
                if(fresh_result) begin
                    active_box_count<=prepared_count;pending_valid<=0;age_frames<=0;
                end else if(pending_valid) begin
                    active_box_count<=pending_count;pending_valid<=0;age_frames<=0;
                end else if(age_frames>=STALE_FRAMES-1) active_box_count<=0;
                else age_frames<=age_increment;
            end
        end
    end

    integer box_index;
    reg [X_BITS-1:0] x1,x2,inner_x1,inner_x2;
    reg [Y_BITS-1:0] y1,y2,inner_y1,inner_y2;
    reg border_hit,selected_border_hit;
    reg [23:0] border_color;
    reg [X_BITS-1:0] badge_x;
    reg [Y_BITS-1:0] badge_y;
    reg [MAX_BOXES-1:0] label_candidates,ink_candidates,label_selected;
    reg [MAX_BOXES-1:0] badge_candidates,badge_ink_candidates,badge_selected;
    reg [6:0] badge_ascii;
    reg selected_text;
    assign label_color_candidates=label_candidates;
    assign badge_color_candidates=badge_candidates;
    assign label_color_selected=display_selected;
    assign label_color_status=display_follow_status;
    assign label_text_candidates=ink_candidates;
    assign badge_text_candidates=badge_ink_candidates;
    assign candidate_classes=active_classes;
    genvar metadata_slot;
    generate for(metadata_slot=0;metadata_slot<MAX_BOXES;metadata_slot=metadata_slot+1) begin: board_candidates
        // Only these low bits address the 8x16 font. Region/ink masks enforce
        // clipping separately, including pixels before the text origin.
        assign candidate_local_x[metadata_slot*7+:7]=pixel_x[6:0]-active_label_ink[metadata_slot*64+:7];
        assign candidate_local_y[metadata_slot*4+:4]=pixel_y[3:0]-active_label_ink[metadata_slot*64+16+:4];
        assign badge_local_x[metadata_slot*3+:3]=pixel_x[2:0]-active_badges[metadata_slot*64+:3]-3'd2;
        assign badge_local_y[metadata_slot*4+:4]=pixel_y[3:0]-active_badges[metadata_slot*64+16+:4]-4'd2;
    end endgenerate
    assign banner_candidate=CLASS_LABELS_ENABLE && enable && de_in && display_follow_status!=0 &&
        pixel_x>=768 && pixel_x<1024 && pixel_y<20;
    assign banner_text_candidate=pixel_x>=770 && pixel_x<1022 && pixel_y>=2 && pixel_y<18;
    wire [4:0] banner_character_index=(pixel_x-12'd770)>>3;
    assign banner_ascii=display_banner_ascii[banner_character_index];
    assign banner_local_x=pixel_x[2:0]-3'd2;
    assign banner_local_y=pixel_y[3:0]-4'd2;
    reg [X_BITS-1:0] text_x;
    reg [Y_BITS-1:0] text_y;
    always @* begin
        border_hit=0;selected_border_hit=0;border_color=BOX_COLOR;badge_x=0;badge_y=0;
        box_border_candidates=0;box_bold_candidates=0;
        label_literal_valid=0;label_banner=0;label_literal_ascii=7'd32;label_color=BOX_COLOR;
        x1=0;y1=0;x2=0;y2=0;
        inner_x1=0;inner_y1=0;inner_x2=0;inner_y2=0;
        label_candidates=0;ink_candidates=0;label_selected=0;
        badge_candidates=0;badge_ink_candidates=0;badge_selected=0;badge_ascii=0;selected_text=0;
        label_hit=0;label_text_hit=0;label_class=0;
        label_origin_x=0;label_origin_y=0;
        label_char_index=0;label_col=0;label_row=0;text_x=0;text_y=0;
        for(box_index=0;box_index<MAX_BOXES;box_index=box_index+1) begin
            x1=active_coords[box_index*64+:X_BITS];
            y1=active_coords[box_index*64+16+:Y_BITS];
            x2=active_coords[box_index*64+32+:X_BITS];
            y2=active_coords[box_index*64+48+:Y_BITS];
            inner_x1=active_inner[box_index*64+:X_BITS];
            inner_y1=active_inner[box_index*64+16+:Y_BITS];
            inner_x2=active_inner[box_index*64+32+:X_BITS];
            inner_y2=active_inner[box_index*64+48+:Y_BITS];
            box_border_candidates[box_index]=enable && de_in &&
               ((pixel_x>>X_BITS)==0) && ((pixel_y>>Y_BITS)==0) && box_index<active_box_count &&
               pixel_x[X_BITS-1:0]>=x1 && pixel_x[X_BITS-1:0]<x2 &&
               pixel_y[Y_BITS-1:0]>=y1 && pixel_y[Y_BITS-1:0]<y2 &&
               (pixel_x[X_BITS-1:0]<inner_x1 || pixel_x[X_BITS-1:0]>=inner_x2 ||
                pixel_y[Y_BITS-1:0]<inner_y1 || pixel_y[Y_BITS-1:0]>=inner_y2);
            if(box_border_candidates[box_index]) border_hit=1;
            // Selected thickness compares in parallel with the ordinary border;
            // no selected-coordinate mux precedes the pixel comparators.
            box_bold_candidates[box_index]=enable && de_in &&
               ((pixel_x>>X_BITS)==0) && ((pixel_y>>Y_BITS)==0) && box_index<active_box_count &&
               pixel_x[X_BITS-1:0]>=x1 && pixel_x[X_BITS-1:0]<x2 &&
               pixel_y[Y_BITS-1:0]>=y1 && pixel_y[Y_BITS-1:0]<y2 &&
               (pixel_x[X_BITS-1:0]<active_bold[box_index*64+:X_BITS] ||
                pixel_x[X_BITS-1:0]>=active_bold[box_index*64+32+:X_BITS] ||
                pixel_y[Y_BITS-1:0]<active_bold[box_index*64+16+:Y_BITS] ||
                pixel_y[Y_BITS-1:0]>=active_bold[box_index*64+48+:Y_BITS]);
            if(box_bold_candidates[box_index] && display_selected[box_index])
                begin border_hit=1;selected_border_hit=1;end
            // All eight outer/ink comparisons run in parallel. The selected
            // label uses a one-hot mask; there is no selected-coordinate ->
            // padding arithmetic -> second comparison on this pixel path.
            label_candidates[box_index]=CLASS_LABELS_ENABLE && enable && de_in &&
                ((pixel_x>>X_BITS)==0) && ((pixel_y>>Y_BITS)==0) &&
                box_index<active_box_count &&
                pixel_x[X_BITS-1:0]>=active_labels[box_index*64+:X_BITS] &&
                pixel_x[X_BITS-1:0]<active_labels[box_index*64+32+:X_BITS] &&
                pixel_y[Y_BITS-1:0]>=active_labels[box_index*64+16+:Y_BITS] &&
                pixel_y[Y_BITS-1:0]<active_labels[box_index*64+48+:Y_BITS];
            ink_candidates[box_index]=
                pixel_x[X_BITS-1:0]>=active_label_ink[box_index*64+:X_BITS] &&
                pixel_x[X_BITS-1:0]<active_label_ink[box_index*64+32+:X_BITS] &&
                pixel_y[Y_BITS-1:0]>=active_label_ink[box_index*64+16+:Y_BITS] &&
                pixel_y[Y_BITS-1:0]<active_label_ink[box_index*64+48+:Y_BITS];
        end
        // Fixed-size badge comparisons run in parallel, just like class labels.
        // Neither a hit nor a color from an earlier slot feeds the next slot.
        for(box_index=0;box_index<MAX_BOXES;box_index=box_index+1) begin
            badge_x=active_badges[box_index*64+:X_BITS];badge_y=active_badges[box_index*64+16+:Y_BITS];
            badge_candidates[box_index]=CLASS_LABELS_ENABLE && enable && de_in && display_follow_status!=0 &&
                box_index<active_box_count &&
                pixel_x>=active_badges[box_index*64+:16] && pixel_x<active_badges[box_index*64+32+:16] &&
                pixel_y>=active_badges[box_index*64+16+:16] && pixel_y<active_badges[box_index*64+48+:16];
            badge_ink_candidates[box_index]=pixel_x>=active_badge_ink[box_index*64+:16] &&
                pixel_x<active_badge_ink[box_index*64+32+:16] &&
                pixel_y>=active_badge_ink[box_index*64+16+:16] &&
                pixel_y<active_badge_ink[box_index*64+48+:16];
        end
        label_hit=|label_candidates || |badge_candidates;
        for(box_index=0;box_index<MAX_BOXES;box_index=box_index+1) begin
            // Constant prefix masks select the lowest eligible slot in parallel.
            // Class labels retain priority over every numeric badge.
            label_selected[box_index]=label_candidates[box_index] &&
                !(label_candidates & ((1<<box_index)-1));
            badge_selected[box_index]=badge_candidates[box_index] &&
                !(badge_candidates & ((1<<box_index)-1)) && !(|label_candidates);
            label_text_hit=label_text_hit | (label_selected[box_index] && ink_candidates[box_index]) |
                (badge_selected[box_index] && badge_ink_candidates[box_index]);
            label_class=label_class | ({7{label_selected[box_index]}} & active_classes[box_index*7+:7]);
            selected_text=selected_text | ((label_selected[box_index] || badge_selected[box_index]) && display_selected[box_index]);
            label_origin_x=label_origin_x | ({X_BITS{label_selected[box_index]}} & active_label_ink[box_index*64+:X_BITS]) |
                ({X_BITS{badge_selected[box_index]}} & (active_badges[box_index*64+:X_BITS]+2));
            label_origin_y=label_origin_y | ({Y_BITS{label_selected[box_index]}} & active_label_ink[box_index*64+16+:Y_BITS]) |
                ({Y_BITS{badge_selected[box_index]}} & (active_badges[box_index*64+16+:Y_BITS]+2));
            badge_ascii=badge_ascii | ({7{badge_selected[box_index]}} & (7'd49+box_index));
        end
        label_literal_valid=|badge_selected;
        label_literal_ascii=label_literal_valid ? badge_ascii : 7'd32;
        label_color=selected_text ? (display_follow_status==1 ? 24'hffff00 : 24'h00ffff) : BOX_COLOR;
        // One fixed banner, including on the SW7 page, uses the same font chain.
        if(CLASS_LABELS_ENABLE && enable && de_in && display_follow_status!=0 &&
            pixel_x>=768 && pixel_x<1024 && pixel_y<20) begin
            label_hit=1;label_banner=1;label_literal_valid=1;
            label_origin_x=770;label_origin_y=2;
            label_text_hit=pixel_x>=770 && pixel_x<1022 && pixel_y>=2 && pixel_y<18;
            label_literal_ascii=banner_ascii;
            label_color=display_follow_status==1 ? 24'hffff00 : display_follow_status==2 ? 24'h00ffff : 24'hff4040;
        end
        border_color=selected_border_hit ? (display_follow_status==1 ? 24'hffff00 : 24'h00ffff) : BOX_COLOR;

        // Optional combinational metadata for standalone consumers. The board
        // OSD registers the origins first and subtracts in its next stage.
        text_x=pixel_x[X_BITS-1:0]-label_origin_x;
        text_y=pixel_y[Y_BITS-1:0]-label_origin_y;
        label_char_index=text_x[6:3];label_col=text_x[2:0];label_row=text_y[3:0];
        rgb_out=(enable && de_in && border_hit &&
                 ((pixel_x>>X_BITS)==0) && ((pixel_y>>Y_BITS)==0)) ? border_color : rgb_in;
    end
endmodule
