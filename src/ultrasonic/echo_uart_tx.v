// 8N1, LSB first. Based on fpga_HT6221_car/src/uart_tx_byte.v.
// Pulse start for one clock only when busy is low.
module echo_uart_tx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115_200
)(
    input wire clk, reset_n, start,
    input wire [7:0] data,
    output reg uart_tx, busy, done
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    reg [31:0] count;
    reg [3:0] bit_index;
    reg [9:0] frame;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
            bit_index <= 0;
            frame <= 10'h3ff;
            uart_tx <= 1;
            busy <= 0;
            done <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                uart_tx <= 1;
                if (start) begin
                    frame <= {1'b1, data, 1'b0};
                    uart_tx <= 0;
                    busy <= 1;
                    count <= 0;
                    bit_index <= 0;
                end
            end else if (count == CLKS_PER_BIT - 1) begin
                count <= 0;
                if (bit_index == 9) begin
                    uart_tx <= 1;
                    busy <= 0;
                    done <= 1;
                end else begin
                    bit_index <= bit_index + 1'b1;
                    frame <= {1'b1, frame[9:1]};
                    uart_tx <= frame[1];
                end
            end else count <= count + 1'b1;
        end
    end
endmodule
