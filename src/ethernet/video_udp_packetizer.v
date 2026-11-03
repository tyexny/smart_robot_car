// RGB565 input -> half-size RGB565BE or full-size screen RGB332 UDP.
// Both board formats use 1280 pixel bytes/row, below the existing MAC MTU.
// Two dual-clock row RAMs are owned by the camera until their request toggle
// changes, then owned by Ethernet until tx_done acknowledges that toggle.
// A busy link drops rows; the receiver never presents an incomplete frame.
module video_udp_packetizer #(
    parameter integer IMAGE_WIDTH=1280,
    parameter integer IMAGE_HEIGHT=720,
    parameter integer FRAME_STRIDE=1,
    parameter integer TRACK_CONTROL_ENABLE=0,
    parameter integer SCREEN_RECORD=0,
    parameter integer FULL_RES=0,
    parameter integer CAMERA_FULL_RES=0,
    // CH59 adapter exposes the next unconsumed address throughout idle gaps.
    // Opt in only with that read-ahead contract; generic request ports keep
    // the original synchronous response implementation by default.
    parameter integer REGISTER_BYTE_OUTPUT=0
)(
    input wire camera_clk, tx_clk, reset_n,
    input wire camera_vsync, capture_enable,
    input wire pixel_valid,
    input wire [15:0] pixel_rgb565,
    input wire tx_busy, tx_done, tx_data_req,
    input wire [10:0] tx_payload_addr,
    input wire [31:0] lock_session, lock_id,
    input wire lock_valid,
    output reg tx_start,
    output wire [15:0] tx_length,
    output wire [7:0] tx_data,
    output reg [31:0] latest_frame_id,
    output reg latest_frame_valid,
    output reg [31:0] packets_sent,
    output reg [31:0] rows_dropped
);
    localparam integer FULL=SCREEN_RECORD && FULL_RES;
    localparam integer FULL_COLOR=!SCREEN_RECORD && CAMERA_FULL_RES;
    localparam integer SAMPLE_SHIFT=(FULL || FULL_COLOR) ? 0 : 1;
    localparam integer WIDTH=IMAGE_WIDTH>>SAMPLE_SHIFT;
    localparam integer HEIGHT=IMAGE_HEIGHT>>SAMPLE_SHIFT;
    // Native HDMI uses an 8-bit row store; other formats retain RGB565 words.
    localparam integer RAM_BITS=16;
    localparam integer RAM_DEPTH=FULL ? WIDTH/2 : WIDTH;
    localparam integer PIXEL_BYTES=FULL ? 1 : 2;
    // FVID v3 preserves RGB565 at native resolution. A row is split into
    // two equal packets; bank ownership returns only after BOTH complete.
    localparam integer HEADER_BYTES=FULL_COLOR ? 36 : TRACK_CONTROL_ENABLE ? 32 : 24;
    localparam integer SEGMENT_PIXELS=FULL_COLOR ? WIDTH/2 : WIDTH;
    localparam integer PACKET_BYTES=HEADER_BYTES+SEGMENT_PIXELS*PIXEL_BYTES;
    localparam [15:0] HALF_WIDTH=WIDTH/2;
    localparam integer HEADER_ADDR_BITS=HEADER_BYTES>32 ? 6 : 5;
    localparam [15:0] WIRE_WIDTH=WIDTH;
    localparam [15:0] WIRE_HEIGHT=HEIGHT;
    localparam integer RAM_ADDR_BITS=$clog2(RAM_DEPTH);
    reg [1:0] request_toggle, acknowledge_toggle;
    reg [1:0] ack_meta, ack_sync, req_meta, req_sync;
    reg [31:0] frame0, frame1, capture_frame_id;
    reg [15:0] line0, line1;
    reg [11:0] capture_x, capture_y;
    reg vsync_d, enable_meta, enable_sync;
    reg row_accept, row_bank, preferred_bank;
    reg [15:0] stride_count;
    reg frame_selected;
    wire free0=request_toggle[0]==ack_sync[0];
    wire free1=request_toggle[1]==ack_sync[1];
    wire first_bank=preferred_bank ? (free1 ? 1'b1 : 1'b0) :
                                                   (free0 ? 1'b0 : 1'b1);
    wire selected_position=(FULL || FULL_COLOR || !capture_y[0]) && capture_y<IMAGE_HEIGHT &&
        (FULL || FULL_COLOR || !capture_x[0]);
    wire sample_selected=pixel_valid && !camera_vsync && selected_position && enable_sync && frame_selected;
    wire write_enable=reset_n && sample_selected &&
        (capture_x==0 ? (free0 || free1) : row_accept);
    wire write_bank=capture_x==0 ? first_bank : row_bank;
    wire [RAM_ADDR_BITS-1:0] write_address=FULL_COLOR ? capture_x : capture_x>>1;
    wire [7:0] rgb332_pixel={pixel_rgb565[15:13],pixel_rgb565[10:8],pixel_rgb565[4:3]};
    wire [RAM_BITS-1:0] write_pixel=pixel_rgb565;
    // Keep RAM writes outside the asynchronous control-reset block. Gowin then
    // infers true dual-clock BSRAM rather than 20,480 pixel flip-flops and muxes.
    always @(posedge camera_clk or negedge reset_n) begin
        if(!reset_n) begin
            request_toggle<=0; ack_meta<=0; ack_sync<=0;
            frame0<=0; frame1<=0; line0<=0; line1<=0;
            capture_frame_id<=0; capture_x<=0; capture_y<=0;
            vsync_d<=0; enable_meta<=0; enable_sync<=0;
            row_accept<=0; row_bank<=0; preferred_bank<=0;
            rows_dropped<=0; stride_count<=0; frame_selected<=0;
        end else begin
            ack_meta<=acknowledge_toggle; ack_sync<=ack_meta;
            vsync_d<=camera_vsync;
            enable_meta<=capture_enable; enable_sync<=enable_meta;
            if(camera_vsync && !vsync_d) begin
                capture_frame_id<=capture_frame_id+1'b1;
                capture_x<=0; capture_y<=0; row_accept<=0;
                frame_selected<=stride_count==0;
                if(stride_count==FRAME_STRIDE-1) stride_count<=0;
                else stride_count<=stride_count+1'b1;
            end else if(pixel_valid && !camera_vsync) begin
                if(capture_x==IMAGE_WIDTH-1) begin
                    capture_x<=0;
                    if(capture_y<IMAGE_HEIGHT) capture_y<=capture_y+1'b1;
                end else capture_x<=capture_x+1'b1;
                if(selected_position && enable_sync && frame_selected) begin
                    if(capture_x==0) begin
                        row_accept<=free0 || free1;
                        row_bank<=first_bank;
                        if(free0 || free1) begin
                            preferred_bank<=~first_bank;
                        end else rows_dropped<=rows_dropped+1'b1;
                    end else if(row_accept) begin
                        if(capture_x==IMAGE_WIDTH-(1<<SAMPLE_SHIFT)) begin
                            if(row_bank) begin
                                frame1<=capture_frame_id; line1<=capture_y>>SAMPLE_SHIFT;
                                request_toggle[1]<=~request_toggle[1];
                            end else begin
                                frame0<=capture_frame_id; line0<=capture_y>>SAMPLE_SHIFT;
                                request_toggle[0]<=~request_toggle[0];
                            end
                            row_accept<=0;
                        end
                    end
                end
            end
        end
    end

    reg sending, active_bank;
    reg active_half, second_pending;
    reg [31:0] active_frame;
    reg [15:0] active_line;
    reg [31:0] active_lock_session, active_lock_id;
    reg active_lock_valid;
    wire pending0=req_sync[0]!=acknowledge_toggle[0];
    wire pending1=req_sync[1]!=acknowledge_toggle[1];
    // Prepare launch decisions before they reach the active/latest metadata
    // enables. The frame-difference carry chains end at dedicated registers.
    localparam [1:0] LAUNCH_IDLE=2'd0,LAUNCH_DIFF=2'd1,
                     LAUNCH_CHOOSE=2'd2,LAUNCH_READY=2'd3;
    reg [1:0] launch_state;
    reg [1:0] acquired_pending;
    reg acquired_latest_valid;
    reg [31:0] snapshot_frame0,snapshot_frame1;
    reg [15:0] snapshot_line0,snapshot_line1;
    reg [31:0] delta01,delta0_latest,delta1_latest;
    reg prepared_bank,prepared_advance_latest;
    reg [31:0] prepared_frame;
    reg [15:0] prepared_line;
    wire acquire_launch=launch_state==LAUNCH_IDLE && !sending && !second_pending &&
        !tx_busy && (pending0 || pending1);
    wire choose_bank0=acquired_pending[0] &&
        (!acquired_pending[1] || (delta01==0 ?
            snapshot_line0<snapshot_line1 : delta01[31]));
    wire advance_bank0=!acquired_latest_valid ||
        (delta0_latest!=0 && !delta0_latest[31]);
    wire advance_bank1=!acquired_latest_valid ||
        (delta1_latest!=0 && !delta1_latest[31]);

    // Only these three small acquisition qualifiers need a clock enable.
    // Pending banks remain Ethernet-owned, so their frame/line payloads cannot
    // change during preparation or a READY busy wait. An unselected free bank
    // may change, but its acquired_pending bit is zero and it cannot be chosen.
    always @(posedge tx_clk) begin
        if(acquire_launch) begin
            acquired_pending<={pending1,pending0};
            acquired_latest_valid<=latest_frame_valid;
        end
    end
    // Payload registers run unconditionally without reset or enables. Their
    // visibility is qualified only by the resettable launch state. No snapshot,
    // difference, candidate frame/line, or candidate decision drives a reset.
    always @(posedge tx_clk) begin
        snapshot_frame0<=frame0;snapshot_frame1<=frame1;
        snapshot_line0<=line0;snapshot_line1<=line1;
        delta01<=snapshot_frame0-snapshot_frame1;
        delta0_latest<=snapshot_frame0-latest_frame_id;
        delta1_latest<=snapshot_frame1-latest_frame_id;
        prepared_bank<=!choose_bank0;
        prepared_frame<=choose_bank0 ? snapshot_frame0 : snapshot_frame1;
        prepared_line<=choose_bank0 ? snapshot_line0 : snapshot_line1;
        prepared_advance_latest<=choose_bank0 ? advance_bank0 : advance_bank1;
    end

    // Each RAM has an independent synchronous read port, to infer block RAM.
    wire [10:0] pixel_byte_addr=tx_payload_addr-HEADER_BYTES;
    wire [10:0] ram_fetch_addr=REGISTER_BYTE_OUTPUT ? tx_payload_addr+{10'd0,tx_data_req} : tx_payload_addr;
    wire [10:0] ram_pixel_byte_addr=ram_fetch_addr-HEADER_BYTES;
    wire [RAM_ADDR_BITS-1:0] ram_address=ram_fetch_addr<HEADER_BYTES ?
        {RAM_ADDR_BITS{1'b0}} : ((ram_pixel_byte_addr>>1)+(FULL_COLOR && active_half ? WIDTH/2 : 0));
    wire [15:0] read_word0, read_word1;
    generate if(FULL)begin:g_screen_rows
        // 1280x8 fits one 2048x9 BSRAM per bank, just as the previous
        // packed 640x16 bank used one 1024x18 block. Native RGB332 pixels
        // leave RAM directly, eliminating the late high/low-byte mux.
        reg [7:0] row_ram0[0:WIDTH-1],row_ram1[0:WIDTH-1];
        reg [7:0] byte0,byte1;
        wire [$clog2(WIDTH)-1:0] read_address=ram_fetch_addr-HEADER_BYTES;
        wire [$clog2(WIDTH)-1:0] write_column=capture_x;
        always @(posedge camera_clk)begin
            if(write_enable && !write_bank)row_ram0[write_column]<=rgb332_pixel;
            if(write_enable && write_bank)row_ram1[write_column]<=rgb332_pixel;
        end
        always @(posedge tx_clk)begin
            byte0<=row_ram0[read_address];byte1<=row_ram1[read_address];
        end
        assign read_word0={8'd0,byte0};assign read_word1={8'd0,byte1};
    end else if(FULL_COLOR)begin:g_native_rows
        // Four half-row blocks retain the proven 640x16 RAM aspect ratio.
        // Packet reads use a local address; no +640 carry chain precedes RAM.
        localparam integer HALF_ADDR_BITS=$clog2(WIDTH/2);
        reg [15:0] row0_left[0:WIDTH/2-1],row0_right[0:WIDTH/2-1];
        reg [15:0] row1_left[0:WIDTH/2-1],row1_right[0:WIDTH/2-1];
        wire right_half=capture_x>=WIDTH/2;
        wire [HALF_ADDR_BITS-1:0] column_address=right_half ? capture_x-WIDTH/2 : capture_x;
        // HEADER_BYTES is even. Ignore unused RAM data during header reads;
        // only the registered header decoder selects the transmitted byte.
        // Thus no header comparator/mux precedes the RAM address carry chain.
        wire [HALF_ADDR_BITS-1:0] read_address=(ram_fetch_addr>>1)-(HEADER_BYTES/2);
        reg [15:0] read0_left,read0_right,read1_left,read1_right;
        always @(posedge camera_clk)begin
            if(write_enable && !write_bank && !right_half)row0_left[column_address]<=pixel_rgb565;
            if(write_enable && !write_bank && right_half)row0_right[column_address]<=pixel_rgb565;
            if(write_enable && write_bank && !right_half)row1_left[column_address]<=pixel_rgb565;
            if(write_enable && write_bank && right_half)row1_right[column_address]<=pixel_rgb565;
        end
        // The MAC consumes data only for a requested byte, one clock later.
        // Reading on idle cycles is harmless and removes its long request
        // decode from the high-fanout RAM clock enables.
        always @(posedge tx_clk)begin
            read0_left<=row0_left[read_address];read0_right<=row0_right[read_address];
            read1_left<=row1_left[read_address];read1_right<=row1_right[read_address];
        end
        assign read_word0=active_half ? read0_right : read0_left;
        assign read_word1=active_half ? read1_right : read1_left;
    end else begin:g_legacy_rows
        reg [RAM_BITS-1:0] row_ram0 [0:RAM_DEPTH-1];
        reg [RAM_BITS-1:0] row_ram1 [0:RAM_DEPTH-1];
        reg [15:0] word0,word1;
        always @(posedge camera_clk)begin
            if(write_enable && !write_bank)row_ram0[write_address]<=write_pixel;
            if(write_enable && write_bank)row_ram1[write_address]<=write_pixel;
        end
        always @(posedge tx_clk)begin
            word0<=row_ram0[ram_address];word1<=row_ram1[ram_address];
        end
        assign read_word0=word0;assign read_word1=word1;
    end endgenerate
    reg read_low_byte;
    reg read_is_pixel;
    reg [HEADER_ADDR_BITS-1:0] read_header_addr;
    // Hold the primed byte selector during the MAC preamble/header wait.
    // Row RAMs still read continuously; their output is unused in this gap.
    always @(posedge tx_clk) if(tx_data_req) begin
            read_low_byte<=pixel_byte_addr[0];
            // Resolve header/pixel membership on the same edge as the RAM
            // read. Both headers fit in five bits; the output path then needs
            // only a small header decoder and this one-bit selection.
            read_is_pixel<=tx_payload_addr>=HEADER_BYTES;
            read_header_addr<=tx_payload_addr[HEADER_ADDR_BITS-1:0];
    end
    wire [15:0] read_word=active_bank ? read_word1 : read_word0;
    reg [7:0] formatted_byte;
    wire format_is_pixel=REGISTER_BYTE_OUTPUT ? tx_payload_addr>=HEADER_BYTES : read_is_pixel;
    wire format_low_byte=REGISTER_BYTE_OUTPUT ? pixel_byte_addr[0] : read_low_byte;
    wire [HEADER_ADDR_BITS-1:0] format_header_addr=REGISTER_BYTE_OUTPUT ? tx_payload_addr[HEADER_ADDR_BITS-1:0] : read_header_addr;
    // RAM data becomes valid immediately after the request edge. A combinational
    // mux adds no extra cycle: the MAC consumes it on the following edge.
    always @* begin
        if(format_is_pixel)
            formatted_byte=FULL ? read_word[7:0] : format_low_byte ? read_word[7:0] : read_word[15:8];
        else begin
        case(format_header_addr)
            // Distinct magic prevents the YOLO receiver using a rendered map
            // as camera input. The screen recorder accepts FREC v1 only.
            0:formatted_byte=8'h46; 1:formatted_byte=SCREEN_RECORD ? 8'h52 : 8'h56;
            2:formatted_byte=SCREEN_RECORD ? 8'h45 : 8'h49;
            3:formatted_byte=SCREEN_RECORD ? 8'h43 : 8'h44;
            4:formatted_byte=FULL_COLOR ? 3 : TRACK_CONTROL_ENABLE ? 2 : 1;
            5:formatted_byte=FULL ? 2 : 1; 6:formatted_byte=0; 7:formatted_byte=HEADER_BYTES;
            8:formatted_byte=active_frame[31:24]; 9:formatted_byte=active_frame[23:16];
            10:formatted_byte=active_frame[15:8]; 11:formatted_byte=active_frame[7:0];
            12:formatted_byte=WIRE_WIDTH[15:8]; 13:formatted_byte=WIRE_WIDTH[7:0];
            14:formatted_byte=WIRE_HEIGHT[15:8]; 15:formatted_byte=WIRE_HEIGHT[7:0];
            16:formatted_byte=active_line[15:8]; 17:formatted_byte=active_line[7:0];
            18:formatted_byte=0; 19:formatted_byte=1;
            20:formatted_byte=TRACK_CONTROL_ENABLE ? active_lock_session[31:24] : 0;
            21:formatted_byte=TRACK_CONTROL_ENABLE ? active_lock_session[23:16] : 0;
            22:formatted_byte=TRACK_CONTROL_ENABLE ? active_lock_session[15:8] : 0;
            23:formatted_byte=TRACK_CONTROL_ENABLE ? active_lock_session[7:0] : 0;
            24:formatted_byte=active_lock_id[31:24]; 25:formatted_byte=active_lock_id[23:16];
            26:formatted_byte=active_lock_id[15:8]; 27:formatted_byte=active_lock_id[7:0];
            28,29,30:formatted_byte=0; 31:formatted_byte={7'd0,active_lock_valid};
            32:formatted_byte=active_half ? HALF_WIDTH[15:8] : 0;
            33:formatted_byte=active_half ? HALF_WIDTH[7:0] : 0;
            34:formatted_byte=HALF_WIDTH[15:8];35:formatted_byte=HALF_WIDTH[7:0];
            default:formatted_byte=0;
        endcase
        end
    end
    generate if(REGISTER_BYTE_OUTPUT)begin:g_registered_byte
        reg [7:0] byte_q;
        always @(posedge tx_clk)if(tx_data_req)byte_q<=formatted_byte;
        assign tx_data=byte_q;
    end else begin:g_original_byte
        assign tx_data=formatted_byte;
    end endgenerate
    assign tx_length=PACKET_BYTES;
    always @(posedge tx_clk or negedge reset_n) begin
        if(!reset_n) begin
            req_meta<=0; req_sync<=0; acknowledge_toggle<=0;
            tx_start<=0; sending<=0; active_bank<=0;launch_state<=LAUNCH_IDLE;
            active_half<=0;second_pending<=0;
            active_frame<=0; active_line<=0;
            active_lock_session<=0; active_lock_id<=0; active_lock_valid<=0;
            latest_frame_id<=0; latest_frame_valid<=0; packets_sent<=0;
        end else begin
            req_meta<=request_toggle; req_sync<=req_meta; tx_start<=0;
            // READY owns an immutable row candidate. Prepare the wide active
            // tuple throughout a busy wait, including the actual launch edge;
            // the MAC cannot consume it before tx_start. Keep the busy gate
            // on launch/publication only, outside these wide metadata enables.
            if(launch_state==LAUNCH_READY)begin
                active_bank<=prepared_bank;active_frame<=prepared_frame;
                active_half<=0;active_line<=prepared_line;
                if(TRACK_CONTROL_ENABLE)begin
                    active_lock_session<=lock_valid ? lock_session : 0;
                    active_lock_id<=lock_valid ? lock_id : 0;
                    active_lock_valid<=lock_valid;
                end
            end
            case(launch_state)
                LAUNCH_IDLE:if(acquire_launch) launch_state<=LAUNCH_DIFF;
                LAUNCH_DIFF:launch_state<=LAUNCH_CHOOSE;
                LAUNCH_CHOOSE:launch_state<=LAUNCH_READY;
                LAUNCH_READY:if(!tx_busy) begin
                    // Keep latest metadata and start on the same edge. The
                    // bridge samples tx_start and this ID on its next edge.
                    // Control is a coherent TX-clock snapshot. Freeze it at
                    // actual packet launch, including any READY/busy delay.
                    // Rows in one frame may carry different newer controls;
                    // every individual UDP packet has one atomic snapshot.
                    if(prepared_advance_latest) latest_frame_id<=prepared_frame;
                    latest_frame_valid<=1;sending<=1;tx_start<=1;
                    launch_state<=LAUNCH_IDLE;
                end
                default:launch_state<=LAUNCH_IDLE;
            endcase
            if(second_pending && !tx_busy) begin
                second_pending<=0;sending<=1;tx_start<=1;
            end
            if(sending && tx_done) begin
                packets_sent<=packets_sent+1'b1; sending<=0;
                if(FULL_COLOR && !active_half) begin
                    active_half<=1;second_pending<=1;
                end else acknowledge_toggle[active_bank]<=req_sync[active_bank];
            end
        end
    end
endmodule
