// FBOX v1/v2, all multibyte fields are network byte order:
//  0..3 "FBOX", 4 version=1, 5 count, 6..7 header length=16,
//  8..11 camera frame_id, 12..13 width=640, 14..15 height=360.
// Each v1 12-byte COCO record: x1,y1,x2,y2,confidence(0..1000),class(0..79 COCO/VLM, 80..83 HSV balls).
// With TRACKING_ENABLE, v2 has a 20-byte header (nonzero tracker session at
// bytes 16..19) and 18-byte records: the v1 fields, nonzero u32 track ID and
// u16 flags (only bit0, observed in this frame, is permitted). A v1 packet
// remains a real observation but has no tracked identity (session/ID zero).
// Class IDs are validated here and committed atomically with coordinates.
// All fields are uint16. Rectangles are half-open; display coordinates are multiplied
// by two. An Ethernet payload_end never commits: packet_done MUST be the
// CRC-validated packet_good pulse, at least one clock after payload_end.
// Validation is registered before the wide result bank is committed. The
// boxes_valid pulse follows packet_done by two clocks. A small registered
// decision and per-slot active mask isolate validation from the wide-bank
// enable/reset fanout; no payload field changes before the atomic publication.
// packet_start may coincide with the first valid byte. A new start discards
// an unvalidated previous packet, including a bad Ethernet frame. Once a
// CRC-good packet enters the commit pipeline, the next header cannot cancel
// it: frame/count/mask are frozen before that header changes staged metadata.
module detection_box_parser #(
    parameter MAX_BOXES = 8,
    parameter INPUT_WIDTH = 640,
    parameter INPUT_HEIGHT = 360,
    parameter MAX_FRAME_LAG = 15,
    parameter REQUIRE_VIDEO_FRAME = 0,
    parameter TRACKING_ENABLE = 0
)(
    input wire clk,
    input wire reset_n,
    input wire packet_start,
    input wire payload_valid,
    input wire [7:0] payload_data,
    input wire payload_end,
    input wire packet_done,
    input wire packet_error,
    input wire [15:0] payload_length,
    input wire boxes_ready,
    // Synchronize this complete frame-id snapshot into clk before connecting.
    // latest_video_valid=0 disables the optional maximum-age/future check.
    input wire [31:0] latest_video_frame_id,
    input wire latest_video_valid,
    // Optional first transmitted frame of the current recognition session.
    // Serial-number subtraction accepts equality and works across u32 wrap.
    input wire [31:0] minimum_frame_id,
    input wire minimum_frame_valid,
    output reg boxes_valid,
    output reg [31:0] frame_id,
    output reg [3:0] box_count,
    output reg [MAX_BOXES*64-1:0] box_coords,
    output reg [MAX_BOXES*7-1:0] box_classes,
    output reg [119:0] box_label_text,
    output reg [3:0] box_label_length,
    output wire [31:0] tracker_session,
    output wire [MAX_BOXES*32-1:0] box_track_ids,
    output wire [MAX_BOXES*10-1:0] box_confidences,
    output wire [MAX_BOXES-1:0] box_observed,
    output reg packet_reject
);
    reg receiving, awaiting_good, malformed;
    reg records_phase;
    // One-hot selection makes each 64-bit record a fixed local register bank.
    // It advances only with a valid final record byte, so payload bubbles do
    // not change the selected bank or the saved high byte.
    reg [MAX_BOXES-1:0] record_bank;
    // Legal v2 FBOX payloads stop at byte count 164. Saturate at 255 rather than
    // wrapping, so any oversized packet remains malformed until next start.
    // Keeping this counter bounded removes a 16-bit position decode from the
    // wide coordinate-bank enables.
    localparam BYTE_COUNT_BITS=TRACKING_ENABLE ? 8 : 7;
    reg [BYTE_COUNT_BITS-1:0] byte_count;
    reg staged_version2,staged_version3;
    reg [119:0] staged_label_text;
    reg [3:0] staged_label_length,publish_label_length;
    reg [31:0] staged_tracker_session;
    reg [MAX_BOXES*32-1:0] staged_track_ids;
    reg [MAX_BOXES*10-1:0] staged_confidences;
    reg [MAX_BOXES-1:0] staged_observed;
    reg [23:0] record_track_high;
    reg [31:0] published_tracker_session;
    reg [MAX_BOXES*32-1:0] published_track_ids;
    reg [MAX_BOXES*10-1:0] published_confidences;
    reg [MAX_BOXES-1:0] published_observed;
    reg [7:0] staged_count;
    reg [31:0] staged_frame_id;
    reg [MAX_BOXES*64-1:0] staged_coords;
    reg [MAX_BOXES*7-1:0] staged_classes;
    reg [7:0] word_high;
    reg [3:0] record_index;
    reg [4:0] record_offset;
    reg [15:0] box_x1, box_y1, box_x2;
    reg coordinate_bad_r;
    reg sequence_known;
    reg [31:0] previous_frame_id;
    // The record count is registered before the constant length lookup:
    // the MAC RX byte used to launch that lookup directly.
    reg [7:0] header_count_r;
    reg [7:0] expected_length_c;
    always @(*) begin
        if (staged_version3) case (header_count_r)
            0: expected_length_c=36;1: expected_length_c=54;2: expected_length_c=72;
            3: expected_length_c=90;4: expected_length_c=108;5: expected_length_c=126;
            6: expected_length_c=144;7: expected_length_c=162;8: expected_length_c=180;
            default: expected_length_c=0;
        endcase else if (staged_version2) case (header_count_r)
            0: expected_length_c=20;1: expected_length_c=38;2: expected_length_c=56;
            3: expected_length_c=74;4: expected_length_c=92;5: expected_length_c=110;
            6: expected_length_c=128;7: expected_length_c=146;8: expected_length_c=164;
            default: expected_length_c=0;
        endcase else case (header_count_r)
            0: expected_length_c=16;1: expected_length_c=28;2: expected_length_c=40;
            3: expected_length_c=52;4: expected_length_c=64;5: expected_length_c=76;
            6: expected_length_c=88;7: expected_length_c=100;8: expected_length_c=112;
            default: expected_length_c=0;
        endcase
    end
    reg [31:0] sequence_delta_r, video_age_r, minimum_delta_r;
    reg sequence_ok_r, video_age_ok_r, minimum_ok_r, format_ok_r, commit_pending;
    reg publish_pending;
    reg [MAX_BOXES-1:0] publish_write,publish_zero;
    // These snapshots need no asynchronous clear: publish_pending is reset
    // and can assert only after all metadata/mask fields have been captured.
    reg [31:0] publish_frame_id;
    reg [31:0] publish_tracker_session;
    reg [3:0] publish_box_count;
    reg [MAX_BOXES-1:0] publish_mask;
    // Decode only the registered count/format, independently of the
    // packet-start position-zero mux. Exact original cycle and bubble rules.
    wire record_header_complete=(byte_count==15&&!staged_version2)||
        (byte_count==19&&!staged_version3)||byte_count==35;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin records_phase<=0;record_bank<=0;end
        else if(packet_start)begin records_phase<=0;record_bank<=0;end
        else if(payload_valid&&receiving)begin
            if(records_phase)begin
                if(record_offset==(staged_version2?17:11))record_bank<=record_bank<<1;
            end else if(record_header_complete)begin
                records_phase<=1;record_bank<={{(MAX_BOXES-1){1'b0}},1'b1};
            end
        end
    end
    wire [BYTE_COUNT_BITS-1:0] position = packet_start ? 0 : byte_count;
    // Start loads a constant AFTER the incrementer. It must not select the
    // adder input or the saturation reduction on the 125MHz counter path.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)byte_count<=0;
        else if(packet_start)byte_count<=payload_valid ? 1 : 0;
        else if(payload_valid && receiving && !(&byte_count))byte_count<=byte_count+1'b1;
    end
    // A 16-bit field consumes the immediately preceding valid payload byte.
    // Capture every valid byte, including across payload bubbles; header and
    // record offsets only decide when to use it. This removes packet-start
    // and record/header selection from this eight-bit register's D path.
    always @(posedge clk)
        if(payload_valid) word_high<=payload_data;
    wire [15:0] word_value = {word_high,payload_data};
    wire [15:0] expected_length = {8'd0,expected_length_c};
    // Byte rejection terms are independent. A single sticky update below
    // avoids a long priority chain through every header/record case arm.
    wire [10:0] header_bad_terms={
        position>=(staged_version3 ? 36 : 20),
        (position==19 && {staged_tracker_session[31:8],payload_data}==0),
        (position==15 && word_value!=INPUT_HEIGHT),
        (position==13 && word_value!=INPUT_WIDTH),
        (position==7 && payload_data!=(staged_version3 ? 36 : staged_version2 ? 20 : 16)),
        (position==6 && payload_data!=0),
        (position==5 && payload_data>MAX_BOXES),
        (position==4 && payload_data!=1 && !(TRACKING_ENABLE && (payload_data==2 || payload_data==3))),
        (position==3 && payload_data!=8'h58),
        (position==2 && payload_data!=8'h4f),
        (position==1 && payload_data!=8'h42)};
    wire [7:0] label_end_position=8'd21+{4'd0,staged_label_length};
    wire label_byte_bad=staged_version3 && ((position==20 && (payload_data==0 || payload_data>15)) ||
        (position>=21 && position<36 && ((position<label_end_position) ?
            (payload_data<32 || payload_data>126) : payload_data!=0)));
    wire header_byte_bad=(|header_bad_terms) || label_byte_bad || (position==0 && payload_data!=8'h46);
    wire [6:0] record_bad_terms={
        record_offset>=18,
        (record_offset==17 && |word_value[15:1]),
        (record_offset==15 && {record_track_high,payload_data}==0),
        (record_offset==11 && ((|word_value[15:7]) || (word_value[6:0]>7'd83))),
        (record_offset==9 && word_value>1000),
        (record_offset==8 && coordinate_bad_r),
        record_index>=MAX_BOXES};
    wire byte_malformed=
        (position >= (staged_version3 ? 36+MAX_BOXES*18 : staged_version2 ? 20+MAX_BOXES*18 : 16+MAX_BOXES*12)) ||
        ((records_phase && !packet_start) ? (|record_bad_terms) : header_byte_bad);

    // Register byte rejection before the sticky accumulator's CE. Include
    // this pending bit in format validation so even an erroneous final byte
    // followed immediately by CRC-good rejects on the original public cycle.
    // Parallel rejection registers replace the final header/record/length
    // mux on the 125 MHz byte path. Their OR is consumed on the same edge
    // as the old single bit, including immediate end/CRC-good cases.
    // Each header rejection term has its own register. The former eleven-
    // term reduction before bit2 joined payload comparison with header phase
    // decode on the 125MHz path; reduce the already-registered terms instead.
    reg [14:0] bad_parts_pending;
    wire bad_byte_pending=|bad_parts_pending;
    wire accepting_byte=payload_valid&&(receiving||packet_start);
    wire header_phase=!records_phase || packet_start;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)bad_parts_pending<=0;
        else begin
            bad_parts_pending[0]<=accepting_byte &&
                (position >= (staged_version3 ? 36+MAX_BOXES*18 : staged_version2 ? 20+MAX_BOXES*18 : 16+MAX_BOXES*12));
            bad_parts_pending[1]<=accepting_byte && !header_phase && (|record_bad_terms);
            bad_parts_pending[12:2]<=header_bad_terms & {11{accepting_byte && header_phase}};
            bad_parts_pending[13]<=accepting_byte && header_phase && (position==0 && payload_data!=8'h46);
            bad_parts_pending[14]<=accepting_byte && header_phase && label_byte_bad;
        end

    assign tracker_session=TRACKING_ENABLE ? published_tracker_session : 32'd0;
    assign box_track_ids=TRACKING_ENABLE ? published_track_ids : {MAX_BOXES*32{1'b0}};
    assign box_confidences=TRACKING_ENABLE ? published_confidences : {MAX_BOXES*10{1'b0}};
    assign box_observed=TRACKING_ENABLE ? published_observed : {MAX_BOXES{1'b0}};
    wire commit_accepted=commit_pending && !packet_error &&
        format_ok_r && sequence_ok_r && video_age_ok_r && minimum_ok_r &&
        boxes_ready && !boxes_valid;
    // No coordinate/class data is visible until a complete CRC-good commit. Staged
    // banks need no packet-start clear: every accepted active bank has been
    // fully overwritten, and the commit explicitly zeros inactive banks.
    // Keep byte_count/position decode and packet_start out of the wide-bank
    // write enables. A start that aborts an unfinished record may write that
    // abandoned staged bank; the next accepted packet fully overwrites it.
    // A CRC-good packet has receiving=0 before the next start, so its staged
    // bank remains fixed through publication.
    // A subsequent packet cannot rewrite a staged coordinate/class until
    // after its complete 16-byte header. Publication occurs one edge after
    // this decision, even when packet_start/first byte arrives at the same
    // edge as the decision. No extra wide coordinate/class shadow is needed.
    always @(posedge clk) begin
        // commit_pending is asynchronously cleared. These internal snapshots
        // are invisible until publish_pending, so a second live reset guard
        // only adds session-reset fanout to their data/CE/RESET implementation.
        if(commit_accepted) begin
            publish_frame_id <= staged_frame_id;
            publish_box_count <= staged_count[3:0];
            publish_tracker_session <= staged_version2 ? staged_tracker_session : 32'd0;
            publish_label_length<=staged_version3 ? staged_label_length : 4'd0;
        end
    end
    // Snapshot the complete ID write transaction. The last ID byte precedes
    // flags/packet validation, so this stage finishes before publication. The
    // high-fanout 32-bit banks see only their local registered write enable.
    reg [MAX_BOXES-1:0] id_write_mask;
    reg [31:0] id_write_data;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) id_write_mask<=0;
        else id_write_mask<=record_bank & {MAX_BOXES{TRACKING_ENABLE && payload_valid &&
            receiving && records_phase && ((staged_version2 && record_offset==15) ||
                                           (!staged_version2 && record_offset==11))}};
    end
    always @(posedge clk)
        id_write_data<=staged_version2 ? {record_track_high,payload_data} : 32'd0;
    // Fan out only registered per-bank decisions. Inactive-bank clearing
    // must not synthesize a global pending/mask cone onto hundreds of RESETs.
    integer publish_bank;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin publish_write<=0;publish_zero<=0;end
        else for(publish_bank=0;publish_bank<MAX_BOXES;publish_bank=publish_bank+1) begin
            publish_write[publish_bank]<=commit_accepted && staged_count>publish_bank;
            publish_zero[publish_bank]<=commit_accepted && !(staged_count>publish_bank);
        end
    end
    genvar bank;
    // Each ASCII byte is a local bank. Every accepted v3 header overwrites
    // all 15 bytes, including zero padding, before CRC-good publication.
    genvar label_byte;
    generate for(label_byte=0;label_byte<15;label_byte=label_byte+1)begin:g_label_byte
        // Register a local publication decision, just as for coordinate banks.
        // The next header cannot reach its text bytes before this edge, so
        // the staged byte remains the accepted packet's byte. No global
        // ready/CRC cone drives the 120-bit label bank's CE or RESET pins.
        reg publish_text_write /* synthesis syn_preserve=1 */;
        reg publish_text_zero /* synthesis syn_preserve=1 */;
        always @(posedge clk or negedge reset_n)
            if(!reset_n)begin publish_text_write<=0;publish_text_zero<=0;end
            else begin
                publish_text_write<=commit_accepted && staged_version3;
                publish_text_zero<=commit_accepted && !staged_version3;
            end
        always @(posedge clk)
            if(publish_text_write)
                box_label_text[119-label_byte*8-:8]<=staged_label_text[119-label_byte*8-:8];
            else if(publish_text_zero)
                box_label_text[119-label_byte*8-:8]<=0;
        always @(posedge clk)
            if(payload_valid && receiving && !records_phase && byte_count==21+label_byte)
                staged_label_text[119-label_byte*8-:8]<=payload_data;
    end endgenerate
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)staged_label_length<=0;
        else if(packet_start)staged_label_length<=0;
        else if(payload_valid && receiving && !records_phase && byte_count==20)
            staged_label_length<=payload_data[3:0];
    end
    generate for(bank=0; bank<MAX_BOXES; bank=bank+1) begin: coordinate_bank
        always @(posedge clk) begin
            // These wide data banks are qualified by receiving/publish_pending,
            // both asynchronously cleared by reset_n. Their contents are invalid
            // while boxes_valid is reset, so an extra reset_n write-enable guard
            // is redundant. Keeping it off the banks also avoids distributing
            // the session-reset synchronizer through every data bit's CE/reset.
            if(payload_valid && receiving &&
               records_phase && record_bank[bank] && record_offset==7)
                staged_coords[bank*64 +: 64] <=
                    {word_value << 1,box_x2 << 1,box_y1 << 1,box_x1 << 1};
            if(payload_valid && receiving &&
               records_phase && record_bank[bank] && record_offset==11)
                staged_classes[bank*7 +: 7] <= word_value[6:0];
            if(TRACKING_ENABLE && payload_valid && receiving &&
               records_phase && record_bank[bank]) begin
                if(record_offset==9)
                    staged_confidences[bank*10 +: 10] <= word_value[9:0];
                if(!staged_version2 && record_offset==11) begin
                    staged_observed[bank] <= 1;
                end
                if(staged_version2 && record_offset==17)
                    staged_observed[bank] <= word_value[0];
            end
            if(id_write_mask[bank])
                staged_track_ids[bank*32 +: 32] <= id_write_data;
            if(commit_accepted)
                publish_mask[bank] <= staged_count > bank;
            if(publish_write[bank]) begin
                box_coords[bank*64 +: 64] <= staged_coords[bank*64 +: 64];
                box_classes[bank*7 +: 7] <= staged_classes[bank*7 +: 7];
                if(TRACKING_ENABLE) begin
                    published_track_ids[bank*32 +: 32] <= staged_track_ids[bank*32 +: 32];
                    published_confidences[bank*10 +: 10] <= staged_confidences[bank*10 +: 10];
                    published_observed[bank] <= staged_observed[bank];
                end
            end else if(publish_zero[bank]) begin
                box_coords[bank*64 +: 64] <= 64'd0;
                box_classes[bank*7 +: 7] <= 7'd0;
                if(TRACKING_ENABLE) begin
                    published_track_ids[bank*32 +: 32] <= 0;
                    published_confidences[bank*10 +: 10] <= 0;
                    published_observed[bank] <= 0;
                end
            end
        end
    end endgenerate

    // Header fields are fully overwritten before a packet can commit.
    // Decode the registered byte count, keeping MAC packet_start and its
    // position-zero mux off the 72-bit metadata bank's clock enables.
    // An aborted header's staged bytes are never published. V1 has no session.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin staged_count<=0;staged_frame_id<=0;staged_tracker_session<=0;end
        else if(payload_valid && receiving && !records_phase)begin
            case(byte_count)
                5: staged_count<=payload_data;
                8: staged_frame_id[31:24]<=payload_data;
                9: staged_frame_id[23:16]<=payload_data;
                10: staged_frame_id[15:8]<=payload_data;
                11: staged_frame_id[7:0]<=payload_data;
                16: if(staged_version2)staged_tracker_session[31:24]<=payload_data;
                17: if(staged_version2)staged_tracker_session[23:16]<=payload_data;
                18: if(staged_version2)staged_tracker_session[15:8]<=payload_data;
                19: if(staged_version2)staged_tracker_session[7:0]<=payload_data;
                default: begin end
            endcase
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            receiving <= 0; awaiting_good <= 0; malformed <= 0;
            
            staged_version2 <= 0;staged_version3<=0;
            box_label_length<=0;
            record_track_high <= 0; published_tracker_session <= 0;
            coordinate_bad_r <= 0;
            record_index <= 0; record_offset <= 0;
            box_x1 <= 0; box_y1 <= 0; box_x2 <= 0;
            sequence_known <= 0; previous_frame_id <= 0;
            header_count_r <= 0;
            sequence_delta_r <= 0; video_age_r <= 0;
            minimum_delta_r <= 0; minimum_ok_r <= 0;
            sequence_ok_r <= 0; video_age_ok_r <= 0;
            format_ok_r <= 0; commit_pending <= 0; publish_pending <= 0;
            boxes_valid <= 0; frame_id <= 0; box_count <= 0;
            packet_reject <= 0;
        end else begin
            boxes_valid <= 0;
            packet_reject <= 0;
            commit_pending <= 0;
            publish_pending <= 0;
            // The frame id is complete at header byte 11, at least four byte
            // clocks before even an empty packet ends. Keep the 32-bit carry
            // chains and comparisons out of the result-bank write enable.
            sequence_delta_r <= staged_frame_id-previous_frame_id;
            video_age_r <= latest_video_frame_id-staged_frame_id;
            minimum_delta_r <= staged_frame_id-minimum_frame_id;
            minimum_ok_r <= !minimum_frame_valid || !minimum_delta_r[31];
            sequence_ok_r <= !sequence_known ||
                             ((sequence_delta_r != 0) && !sequence_delta_r[31]);
            video_age_ok_r <= (!REQUIRE_VIDEO_FRAME && !latest_video_valid) ||
                              (latest_video_valid && !video_age_r[31] && video_age_r <= MAX_FRAME_LAG);
            format_ok_r <= !malformed && !bad_byte_pending &&
                           (byte_count == expected_length) &&
                           (payload_length == expected_length);
            if(bad_byte_pending && !packet_start)malformed<=1;
            if (packet_start) begin
                receiving <= 1; awaiting_good <= 0; malformed <= 0;
                
                staged_version2 <= 0;staged_version3<=0;
                coordinate_bad_r <= 0;
                record_index <= 0; record_offset <= 0;
                // Coordinate assembly is overwritten by every record before
                // validation. A new packet need not clear these private
                // registers through the MAC start's high-fanout reset mux.
                header_count_r <= 0;
            end
            if (payload_valid && (receiving || packet_start)) begin
                if(records_phase && !packet_start) begin
                    if (record_index < MAX_BOXES) begin
                        case (record_offset)
                            0,2,4,6,10,16: begin end
                            1: box_x1 <= word_value;
                            3: box_y1 <= word_value;
                            5: box_x2 <= word_value;
                            7: coordinate_bad_r <= (box_x1 >= box_x2) ||
                                (box_y1 >= word_value) || (box_x2 > INPUT_WIDTH) ||
                                (word_value > INPUT_HEIGHT);
                            8: begin


                            end
                            9: begin end
                            11: begin end
                            12: record_track_high[23:16] <= payload_data;
                            13: record_track_high[15:8] <= payload_data;
                            14: record_track_high[7:0] <= payload_data;
                            15: begin end
                            17: begin end
                            default: begin end
                        endcase
                    end
                    if(record_offset == (staged_version2 ? 17 : 11)) begin
                        record_offset <= 0;
                        
                        if(record_index < 15) record_index <= record_index+1'b1;
                    end else record_offset <= record_offset+1'b1;
                end else begin
                case (position)
                    0: begin end
                    1: begin end
                    2: begin end
                    3: begin end
                    4: begin
                        staged_version2 <= TRACKING_ENABLE && (payload_data==2 || payload_data==3);
                        staged_version3 <= TRACKING_ENABLE && payload_data==3;

                    end
                    5: begin
                        header_count_r <= payload_data;
                        // Constant lookup moved into expected_length_c.
                    end
                    6: begin end
                    7: begin end
                    8: begin end
                    9: begin end
                    10: begin end
                    11: begin end
                    12: begin end
                    13: begin end
                    14: begin end
                    15: begin

                        if(!staged_version2) begin
                            
                        end
                    end
                    16: begin end
                    17: begin end
                    18: begin end
                    19: begin


                        if(!staged_version3)begin
                            
                        end
                    end
                    35:begin
                        
                    end
                    default: begin end
                endcase
                end
            end
            if (payload_end && (receiving || packet_start)) begin
                receiving <= 0;
                awaiting_good <= 1;
            end
            if (packet_error) begin
                receiving <= 0;
                awaiting_good <= 0;
                packet_reject <= 1;
            end else if (packet_done && !packet_start) begin
                awaiting_good <= 0;
                if(awaiting_good) commit_pending <= 1;
                else packet_reject <= 1;
            end
            // The first edge after packet_good sees the final byte's registered
            // syntax/length checks. It reserves this sole-producer mailbox and
            // captures only small decision/metadata/mask registers. A next
            // packet_start is safe here: NBA writes leave the prior staged
            // metadata available for this snapshot until the edge completes.
            if (commit_pending && !packet_error) begin
                if (commit_accepted) begin
                    publish_pending <= 1;
                    sequence_known <= 1;
                    previous_frame_id <= staged_frame_id;
                end else packet_reject <= 1;
            end
            // Publication uses only the registered decision. The mailbox
            // cannot become busy before this parser's own valid transfer, so
            // readiness sampled at the decision remains reserved through it.
            if (publish_pending) begin
                frame_id <= publish_frame_id;
                box_count <= publish_box_count;
                box_label_length<=publish_label_length;
                if(TRACKING_ENABLE) published_tracker_session <= publish_tracker_session;
                boxes_valid <= 1;
            end
        end
    end
endmodule
