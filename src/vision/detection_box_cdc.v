// Acknowledged mailbox for a complete detection result. The source payload
// remains unchanged until the destination has captured it. No individual
// coordinate or class ID is synchronized separately. Both domains share reset_n.
module detection_box_cdc #(
    parameter MAX_BOXES = 8,
    parameter RESET_SYNC_RELEASE = 0,
    parameter TRACK_METADATA_ENABLE = 0
)(
    input wire src_clk,
    input wire dst_clk,
    input wire reset_n,
    input wire src_valid,
    output wire src_ready,
    input wire [31:0] src_frame_id,
    input wire [3:0] src_box_count,
    input wire [MAX_BOXES*64-1:0] src_box_coords,
    input wire [MAX_BOXES*7-1:0] src_box_classes,
    input wire [119:0] src_label_text,
    input wire [3:0] src_label_length,
    input wire [31:0] src_tracker_session,
    input wire [MAX_BOXES*32-1:0] src_box_track_ids,
    input wire [MAX_BOXES*10-1:0] src_box_confidences,
    input wire [MAX_BOXES-1:0] src_box_observed,
    output reg dst_valid,
    output reg [31:0] dst_frame_id,
    output reg [3:0] dst_box_count,
    output reg [MAX_BOXES*64-1:0] dst_box_coords,
    output reg [MAX_BOXES*7-1:0] dst_box_classes,
    output wire [119:0] dst_label_text,
    output wire [3:0] dst_label_length,
    output wire [31:0] dst_tracker_session,
    output wire [MAX_BOXES*32-1:0] dst_box_track_ids,
    output wire [MAX_BOXES*10-1:0] dst_box_confidences,
    output wire [MAX_BOXES-1:0] dst_box_observed
);
    reg [31:0] held_frame_id;
    reg [3:0] held_box_count;
    reg [MAX_BOXES*64-1:0] held_box_coords;
    reg [MAX_BOXES*7-1:0] held_box_classes;
    reg [119:0] held_label_text,dst_label_text_r;
    reg [3:0] held_label_length,dst_label_length_r;
    assign dst_label_text=TRACK_METADATA_ENABLE ? dst_label_text_r : 120'd0;
    assign dst_label_length=TRACK_METADATA_ENABLE ? dst_label_length_r : 4'd0;
    reg [31:0] held_tracker_session, dst_tracker_session_r;
    reg [MAX_BOXES*32-1:0] held_box_track_ids, dst_box_track_ids_r;
    reg [MAX_BOXES*10-1:0] held_box_confidences, dst_box_confidences_r;
    reg [MAX_BOXES-1:0] held_box_observed, dst_box_observed_r;
    // An old caller may leave all new inputs unconnected. The default-off
    // metadata outputs are constants and cannot propagate those unknowns.
    assign dst_tracker_session=TRACK_METADATA_ENABLE ? dst_tracker_session_r : 32'd0;
    assign dst_box_track_ids=TRACK_METADATA_ENABLE ? dst_box_track_ids_r : {MAX_BOXES*32{1'b0}};
    assign dst_box_confidences=TRACK_METADATA_ENABLE ? dst_box_confidences_r : {MAX_BOXES*10{1'b0}};
    assign dst_box_observed=TRACK_METADATA_ENABLE ? dst_box_observed_r : {MAX_BOXES{1'b0}};
    reg request_toggle, acknowledge_toggle;
    (* ASYNC_REG = "TRUE" *) reg ack_meta, ack_sync;
    (* ASYNC_REG = "TRUE" *) reg req_meta, req_sync;
    wire src_domain_reset_n,dst_domain_reset_n;
    generate if(RESET_SYNC_RELEASE) begin:g_reset_release
        (* ASYNC_REG="TRUE" *) reg src_reset_meta,src_reset_sync;
        (* ASYNC_REG="TRUE" *) reg dst_reset_meta,dst_reset_sync;
        always @(posedge src_clk or negedge reset_n)
            if(!reset_n) begin src_reset_meta<=1;src_reset_sync<=1;end
            else begin src_reset_meta<=0;src_reset_sync<=src_reset_meta;end
        always @(posedge dst_clk or negedge reset_n)
            if(!reset_n) begin dst_reset_meta<=1;dst_reset_sync<=1;end
            else begin dst_reset_meta<=0;dst_reset_sync<=dst_reset_meta;end
        assign src_domain_reset_n=~src_reset_sync;
        assign dst_domain_reset_n=~dst_reset_sync;
    end else begin:g_common_reset
        assign src_domain_reset_n=reset_n;
        assign dst_domain_reset_n=reset_n;
    end endgenerate

    assign src_ready = (request_toggle == ack_sync);

    // Each local phase is an exact source-clock replica of request_toggle.
    // Preserving these small registers lets each tuple bank use a nearby
    // enable instead of routing one acceptance LUT to the entire mailbox.
    // The public handshake and the sole cross-domain request stay unchanged.
    reg header_capture_phase /* synthesis syn_preserve = 1 */;
    always @(posedge src_clk or negedge src_domain_reset_n) begin
        if(!src_domain_reset_n) header_capture_phase<=0;
        else if(src_valid && src_ready) header_capture_phase<=~request_toggle;
    end

    // Payload writes need no asynchronous clear or reset-qualified enable.
    // Request/ack/valid/count controls prevent any reset-time or uninitialized
    // payload from becoming visible. The first accepted post-reset request
    // captures its complete tuple on the same edge as the request toggle.
    // Keep reset-release out of the payload enables. Local phases are equal
    // to request_toggle on every edge, so all fields still capture together
    // exactly when src_valid && src_ready and freeze until acknowledgment.
    always @(posedge src_clk) begin
        if(src_valid && header_capture_phase==ack_sync) begin
            held_frame_id<=src_frame_id;held_box_count<=src_box_count;
            if(TRACK_METADATA_ENABLE) begin
                held_tracker_session<=src_tracker_session;
                held_label_text<=src_label_text;held_label_length<=src_label_length;
            end
        end
    end
    genvar box_bank;
    generate for(box_bank=0;box_bank<MAX_BOXES;box_bank=box_bank+1) begin:g_source_bank
        reg coordinate_capture_phase /* synthesis syn_preserve = 1 */;
        reg metadata_capture_phase /* synthesis syn_preserve = 1 */;
        always @(posedge src_clk or negedge src_domain_reset_n) begin
            if(!src_domain_reset_n) begin
                coordinate_capture_phase<=0;
                metadata_capture_phase<=0;
            end else if(src_valid && src_ready) begin
                coordinate_capture_phase<=~request_toggle;
                metadata_capture_phase<=~request_toggle;
            end
        end
        always @(posedge src_clk) begin
            if(src_valid && coordinate_capture_phase==ack_sync)
                held_box_coords[box_bank*64+:64]<=src_box_coords[box_bank*64+:64];
            if(src_valid && metadata_capture_phase==ack_sync) begin
                held_box_classes[box_bank*7+:7]<=src_box_classes[box_bank*7+:7];
                if(TRACK_METADATA_ENABLE) begin
                    held_box_track_ids[box_bank*32+:32]<=src_box_track_ids[box_bank*32+:32];
                    held_box_confidences[box_bank*10+:10]<=src_box_confidences[box_bank*10+:10];
                    held_box_observed[box_bank]<=src_box_observed[box_bank];
                end
            end
        end
    end endgenerate
    always @(posedge dst_clk) begin
        if(dst_domain_reset_n && req_sync!=acknowledge_toggle) begin
            dst_frame_id<=held_frame_id;dst_box_coords<=held_box_coords;
            dst_box_classes<=held_box_classes;
            if(TRACK_METADATA_ENABLE) begin
                dst_tracker_session_r<=held_tracker_session;
                dst_label_text_r<=held_label_text;dst_label_length_r<=held_label_length;
                dst_box_track_ids_r<=held_box_track_ids;
                dst_box_confidences_r<=held_box_confidences;
                dst_box_observed_r<=held_box_observed;
            end
        end
    end
    always @(posedge src_clk or negedge src_domain_reset_n) begin
        if (!src_domain_reset_n) begin
            request_toggle <= 0;
            ack_meta <= 0;
            ack_sync <= 0;
        end else begin
            ack_meta <= acknowledge_toggle;
            ack_sync <= ack_meta;
            if (src_valid && src_ready) begin
                request_toggle <= ~request_toggle;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_domain_reset_n) begin
        if (!dst_domain_reset_n) begin
            req_meta <= 0;
            req_sync <= 0;
            acknowledge_toggle <= 0;
            dst_valid <= 0;
            dst_box_count <= 0;
        end else begin
            req_meta <= request_toggle;
            req_sync <= req_meta;
            dst_valid <= 0;
            if (req_sync != acknowledge_toggle) begin
                dst_box_count <= held_box_count;
                dst_valid <= 1;
                acknowledge_toggle <= req_sync;
            end
        end
    end
endmodule
