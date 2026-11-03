`timescale 1ns/1ps

// 8 data bits, no parity, one stop bit. The input is asynchronous to clk.
module uart_rx_byte #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115_200
)(
    input wire clk,
    input wire reset_n,
    input wire uart_rx,
    output reg [7:0] data,
    output reg valid,
    output reg framing_error
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer HALF_BIT = CLKS_PER_BIT / 2;
    localparam [1:0] IDLE = 0, START = 1, DATA = 2, STOP = 3;

    reg rx_meta, rx_sync;
    reg [1:0] state;
    reg [31:0] count;
    reg [2:0] bit_index;
    reg [7:0] shift;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            state <= IDLE;
            count <= 0;
            bit_index <= 0;
            shift <= 0;
            data <= 0;
            valid <= 0;
            framing_error <= 0;
        end else begin
            rx_meta <= uart_rx;
            rx_sync <= rx_meta;
            valid <= 0;
            framing_error <= 0;
            case (state)
                IDLE: if (!rx_sync) begin
                    count <= 0;
                    state <= START;
                end
                START: if (count == HALF_BIT - 1) begin
                    count <= 0;
                    if (!rx_sync) begin
                        bit_index <= 0;
                        state <= DATA;
                    end else
                        state <= IDLE;
                end else count <= count + 1'b1;
                DATA: if (count == CLKS_PER_BIT - 1) begin
                    count <= 0;
                    shift[bit_index] <= rx_sync;
                    if (bit_index == 3'd7)
                        state <= STOP;
                    else
                        bit_index <= bit_index + 1'b1;
                end else count <= count + 1'b1;
                STOP: if (count == CLKS_PER_BIT - 1) begin
                    count <= 0;
                    if (rx_sync) begin
                        data <= shift;
                        valid <= 1;
                    end else
                        framing_error <= 1;
                    state <= IDLE;
                end else count <= count + 1'b1;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
