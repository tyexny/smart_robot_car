`timescale 1ns/1ps
// Fixed 14-byte UDP payload: "TCMD", version 1, command, session32, seq32.
// Publish only after the Ethernet endpoint has checked length, checksum and FCS.
module task_command_parser(
    input wire clk, reset_n,
    input wire packet_start, payload_valid,
    input wire [7:0] payload_data,
    input wire packet_good, packet_error,
    input wire [15:0] payload_length,
    output reg command_valid,
    output reg [7:0] command
);
    reg [111:0] packet_bits;
    reg [3:0] byte_count;
    reg session_seen;
    reg [31:0] last_session,last_sequence;
    reg candidate_valid,candidate_ready,candidate_new_session,candidate_different;
    reg [7:0] candidate_command;
    reg [31:0] candidate_session,candidate_sequence,candidate_delta;
    reg [7:0] publish_command;
    reg [31:0] publish_session,publish_sequence;
    wire known_command=packet_bits[71:64]==8'h40 || // forward
        packet_bits[71:64]==8'h19 || // backward
        packet_bits[71:64]==8'h15 || // stop
        packet_bits[71:64]==8'h44 || // left arc
        packet_bits[71:64]==8'h43 || // right arc
        packet_bits[71:64]==8'h16 || // rotate left
        packet_bits[71:64]==8'h0d || // rotate right
        packet_bits[71:64]==8'h07 || // strafe left
        packet_bits[71:64]==8'h09 || // strafe right
        packet_bits[71:64]==8'h45 || // speed up
        packet_bits[71:64]==8'h47 || // speed down
        packet_bits[71:64]==8'h46 || // follow select
        packet_bits[71:64]==8'h1c || // follow confirm
        packet_bits[71:64]==8'ha1 || // approach to 60 cm
        packet_bits[71:64]==8'hf0;   // manual motion lease refresh (no new direction)
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            packet_bits<=0;byte_count<=0;session_seen<=0;
            last_session<=0;last_sequence<=0;
            candidate_valid<=0;candidate_ready<=0;candidate_new_session<=0;candidate_different<=0;
            candidate_command<=0;candidate_session<=0;candidate_sequence<=0;candidate_delta<=0;
            publish_command<=0;publish_session<=0;publish_sequence<=0;
            command_valid<=0;command<=0;
        end else begin
            command_valid<=0;
            candidate_valid<=0;
            candidate_ready<=candidate_valid;
            // Snapshot raw fields every clock; the long packet validation
            // cone drives one valid bit instead of 72 register enables.
            candidate_command<=packet_bits[71:64];
            candidate_session<=packet_bits[63:32];
            candidate_sequence<=packet_bits[31:0];
            if(candidate_valid) begin
                publish_command<=candidate_command;
                publish_session<=candidate_session;
                publish_sequence<=candidate_sequence;
                candidate_delta<=candidate_sequence-last_sequence;
                candidate_new_session<=!session_seen || candidate_session!=last_session;
                // (a-b) mod 2^32 is nonzero exactly when a!=b. Register this
                // predicate in parallel with subtraction; do not reduce all
                // 32 delta bits on the high-fanout publication enables.
                candidate_different<=candidate_sequence!=last_sequence;
            end
            if(candidate_ready && (candidate_new_session ||
               (candidate_different && !candidate_delta[31]))) begin
                command<=publish_command;
                command_valid<=1;
                session_seen<=1;
                last_session<=publish_session;
                last_sequence<=publish_sequence;
            end
            if(packet_start) begin
                packet_bits<={104'd0,payload_data};
                byte_count<=payload_valid ? 4'd1 : 4'd0;
            end else if(payload_valid) begin
                packet_bits<={packet_bits[103:0],payload_data};
                if(byte_count!=4'd15) byte_count<=byte_count+1'b1;
            end
            if(packet_error) byte_count<=0;
            if(packet_good && payload_length==16'd14 && byte_count==4'd14 &&
               packet_bits[111:80]==32'h54434d44 && packet_bits[79:72]==8'd1 &&
               packet_bits[63:32]!=0 && known_command) begin
                candidate_valid<=1;
            end
        end
    end
endmodule
