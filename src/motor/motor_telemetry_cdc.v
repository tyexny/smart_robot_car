`timescale 1ns/1ps

// Single-entry mailbox. The source holds all 64 bits until the destination
// acknowledges the toggle, so the four speeds cross as one coherent group.
module motor_telemetry_cdc (
    input wire src_clk,
    input wire dst_clk,
    input wire reset_n,
    input wire src_valid,
    input wire [63:0] src_speeds,
    output reg dst_valid,
    output reg [63:0] dst_speeds
);
    reg [63:0] held_speeds;
    reg request_toggle, acknowledge_toggle;
    reg ack_meta, ack_sync;
    reg req_meta, req_sync;

    always @(posedge src_clk or negedge reset_n) begin
        if (!reset_n) begin
            held_speeds <= 0;
            request_toggle <= 0;
            ack_meta <= 0;
            ack_sync <= 0;
        end else begin
            ack_meta <= acknowledge_toggle;
            ack_sync <= ack_meta;
            if (src_valid && request_toggle == ack_sync) begin
                held_speeds <= src_speeds;
                request_toggle <= ~request_toggle;
            end
        end
    end

    always @(posedge dst_clk or negedge reset_n) begin
        if (!reset_n) begin
            req_meta <= 0;
            req_sync <= 0;
            acknowledge_toggle <= 0;
            dst_valid <= 0;
            dst_speeds <= 0;
        end else begin
            req_meta <= request_toggle;
            req_sync <= req_meta;
            dst_valid <= 0;
            if (req_sync != acknowledge_toggle) begin
                dst_speeds <= held_speeds;
                dst_valid <= 1;
                acknowledge_toggle <= req_sync;
            end
        end
    end
endmodule
