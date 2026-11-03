`timescale 1ns/1ps

// Reply: 0A 03 10, four big-endian counts, four big-endian signed
// speeds (A/B/C/D), Modbus CRC16 low byte then high byte: 21 bytes total.
module motor_telemetry_rx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115_200
)(
    input wire clk,
    input wire reset_n,
    input wire uart_rx,
    output reg data_valid,
    output reg signed [15:0] speed_a,
    output reg signed [15:0] speed_b,
    output reg signed [15:0] speed_c,
    output reg signed [15:0] speed_d,
    output reg [63:0] counts
);
    wire [7:0] rx_byte;
    wire rx_valid, rx_error;
    uart_rx_byte #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) u_rx (
        .clk(clk), .reset_n(reset_n), .uart_rx(uart_rx),
        .data(rx_byte), .valid(rx_valid), .framing_error(rx_error)
    );

    function [15:0] crc_next;
        input [15:0] previous;
        input [7:0] byte_value;
        reg [15:0] value;
        integer bit_number;
        begin
            value = previous ^ {8'd0, byte_value};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
                value = value[0] ? ((value >> 1) ^ 16'hA001) : (value >> 1);
            crc_next = value;
        end
    endfunction

    localparam [1:0] HEADER_0 = 0, HEADER_1 = 1,
                     HEADER_2 = 2, PAYLOAD = 3;
    localparam integer BYTE_TIMEOUT = CLK_FREQ_HZ / 500; // 2 ms
    reg [1:0] state;
    reg [4:0] byte_index;
    reg [31:0] timeout_count;
    reg [15:0] crc;
    reg [7:0] high_byte, crc_low_byte;
    reg signed [15:0] pending_a, pending_b, pending_c, pending_d;
    reg [63:0] pending_counts;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= HEADER_0;
            byte_index <= 0;
            timeout_count <= 0;
            crc <= 16'hFFFF;
            high_byte <= 0;
            crc_low_byte <= 0;
            pending_a <= 0; pending_b <= 0; pending_c <= 0; pending_d <= 0;
            speed_a <= 0; speed_b <= 0; speed_c <= 0; speed_d <= 0;
            pending_counts <= 0; counts <= 0;
            data_valid <= 0;
        end else begin
            data_valid <= 0;
            if (rx_error || (state != HEADER_0 && timeout_count == BYTE_TIMEOUT - 1)) begin
                state <= HEADER_0;
                timeout_count <= 0;
            end else if (rx_valid) begin
                timeout_count <= 0;
                case (state)
                    HEADER_0: if (rx_byte == 8'h0A) begin
                        crc <= crc_next(16'hFFFF, 8'h0A);
                        state <= HEADER_1;
                    end
                    HEADER_1: if (rx_byte == 8'h03) begin
                        crc <= crc_next(crc, rx_byte);
                        state <= HEADER_2;
                    end else if (rx_byte == 8'h0A) begin
                        crc <= crc_next(16'hFFFF, 8'h0A);
                    end else state <= HEADER_0;
                    HEADER_2: if (rx_byte == 8'h10) begin
                        crc <= crc_next(crc, rx_byte);
                        byte_index <= 0;
                        state <= PAYLOAD;
                    end else if (rx_byte == 8'h0A) begin
                        crc <= crc_next(16'hFFFF, 8'h0A);
                        state <= HEADER_1;
                    end else state <= HEADER_0;
                    PAYLOAD: begin
                        if (byte_index < 5'd16) begin
                            crc <= crc_next(crc, rx_byte);
                            case (byte_index)
                                5'd0,5'd2,5'd4,5'd6: high_byte <= rx_byte;
                                5'd1: pending_counts[63:48] <= {high_byte,rx_byte};
                                5'd3: pending_counts[47:32] <= {high_byte,rx_byte};
                                5'd5: pending_counts[31:16] <= {high_byte,rx_byte};
                                5'd7: pending_counts[15:0] <= {high_byte,rx_byte};
                                5'd8, 5'd10, 5'd12, 5'd14: high_byte <= rx_byte;
                                5'd9: pending_a <= {high_byte, rx_byte};
                                5'd11: pending_b <= {high_byte, rx_byte};
                                5'd13: pending_c <= {high_byte, rx_byte};
                                5'd15: pending_d <= {high_byte, rx_byte};
                                default: ;
                            endcase
                        end else if (byte_index == 5'd16)
                            crc_low_byte <= rx_byte;
                        else begin
                            if (crc_low_byte == crc[7:0] && rx_byte == crc[15:8]) begin
                                speed_a <= pending_a;
                                speed_b <= pending_b;
                                speed_c <= pending_c;
                                speed_d <= pending_d;
                                counts <= pending_counts;
                                data_valid <= 1;
                            end
                            state <= HEADER_0;
                        end
                        byte_index <= byte_index + 1'b1;
                    end
                    default: state <= HEADER_0;
                endcase
            end else if (state != HEADER_0)
                timeout_count <= timeout_count + 1'b1;
        end
    end
endmodule
