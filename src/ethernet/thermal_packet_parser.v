`timescale 1ns/1ps
// FTHM v1, 16 bytes, network order: magic, version, flags, length,
// sequence32, signed maximum temperature in 0.01 C, threshold40/50/60 C.
// The Ethernet receiver supplies packet_good only after UDP/IP/FCS checks.
module thermal_packet_parser(
    input wire clk, reset_n,
    input wire packet_start, payload_valid,
    input wire [7:0] payload_data,
    input wire packet_good, packet_error,
    input wire [15:0] payload_length,
    output reg sample_valid,
    output reg sensor_valid, task_active,
    output reg signed [15:0] maximum_centi,
    output reg [6:0] threshold_c
);
    reg [127:0] packet_bits;
    reg [4:0] byte_count;
    reg errored;
    reg eligible,good_pending;
    wire signed [15:0] temperature=packet_bits[31:16];
    wire valid_threshold=packet_bits[15:0]==16'd40 ||
        packet_bits[15:0]==16'd50 || packet_bits[15:0]==16'd60;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            packet_bits<=0;byte_count<=0;errored<=0;eligible<=0;good_pending<=0;sample_valid<=0;
            sensor_valid<=0;task_active<=0;maximum_centi<=0;threshold_c<=40;
        end else begin
            sample_valid<=0;
            good_pending<=packet_good && !packet_error;
            // Decode the held payload while Ethernet checks its FCS/tail.
            // The final packet_good pulse then drives only this short enable.
            eligible<=!errored && payload_length==16 && byte_count==16 &&
                packet_bits[127:96]==32'h4654484d && packet_bits[95:88]==1 &&
                packet_bits[87:82]==0 && packet_bits[79:64]==16 && valid_threshold &&
                (!packet_bits[80] || (temperature>=-16'sd4000 && temperature<=16'sd30000));
            if(packet_start) begin
                eligible<=0;
                packet_bits<={120'd0,payload_data};
                byte_count<=payload_valid ? 5'd1 : 5'd0;
                errored<=0;
            end else if(payload_valid) begin
                packet_bits<={packet_bits[119:0],payload_data};
                if(byte_count!=31) byte_count<=byte_count+1'b1;
            end
            if(packet_error) errored<=1;
            if(good_pending && !packet_start && !packet_error && !errored && eligible) begin
                sample_valid<=1;
                sensor_valid<=packet_bits[80];task_active<=packet_bits[81];
                maximum_centi<=temperature;threshold_c<=packet_bits[6:0];
            end
        end
    end
endmodule
