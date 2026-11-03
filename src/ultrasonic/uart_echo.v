// Reuses uart_echo/src/uart_echo.v with a configurable measurement scheduler.
// Defaults retain the original one-second, MSB-first two-byte UART reports.
// FFF1: no rising ECHO; FFF2: no falling ECHO; FFF3: ECHO initially high.
// The distance/status outputs must be sampled with measure_done, including errors.
// CLK_FREQ_HZ >= 1 MHz; 0 <= START_OFFSET_MS < MEASURE_PERIOD_MS.
module uart_echo #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer UART_TEST_MODE = 0,
    parameter integer MEASURE_PERIOD_MS = 1000,
    parameter integer START_OFFSET_MS = 0
)(
    input wire clk,
    input wire reset_n,
    input wire echo,
    output wire trig,
    output wire uart_tx,
    output wire [15:0] distance_cm,
    output wire [1:0] measure_status,
    output wire measure_done,
    output wire measure_busy
);
    // Split the constant calculation to avoid 32-bit CLK_FREQ_HZ * 1000 overflow.
    localparam integer PERIOD_CLKS = (CLK_FREQ_HZ / 1000) * MEASURE_PERIOD_MS
                                  + ((CLK_FREQ_HZ % 1000) * MEASURE_PERIOD_MS) / 1000;
    localparam integer OFFSET_CLKS = (CLK_FREQ_HZ / 1000) * START_OFFSET_MS
                                  + ((CLK_FREQ_HZ % 1000) * START_OFFSET_MS) / 1000;
    localparam integer START_COUNT = OFFSET_CLKS + 10;
    reg [31:0] period_count;
    reg [15:0] result_word, tx_word;
    reg [7:0] tx_data;
    reg [1:0] tx_state;
    reg tx_start;
    wire tx_busy, tx_done;
    wire measure_start = (period_count == START_COUNT) && !measure_busy;

    cs100a_measure #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) sensor (
        .clk(clk), .reset_n(reset_n), .start(measure_start), .echo(echo),
        .trig(trig), .busy(measure_busy), .done(measure_done),
        .distance_cm(distance_cm), .status(measure_status)
    );
    echo_uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) pc_tx (
        .clk(clk), .reset_n(reset_n), .start(tx_start), .data(tx_data),
        .uart_tx(uart_tx), .busy(tx_busy), .done(tx_done)
    );
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            period_count <= 1;
            result_word <= 16'hffff;
        end else begin
            if (period_count == PERIOD_CLKS - 1) period_count <= 0;
            else period_count <= period_count + 1'b1;
            if (measure_start) result_word <= 16'hffff;
            else if (measure_done) begin
                case (measure_status)
                    0: result_word <= distance_cm;
                    1: result_word <= 16'hfff1;
                    2: result_word <= 16'hfff2;
                    3: result_word <= 16'hfff3;
                endcase
            end
        end
    end

    // Snapshot the complete word so a new result cannot alter the second byte.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            tx_word <= 0;
            tx_data <= 0;
            tx_start <= 0;
            tx_state <= 0;
        end else begin
            tx_start <= 0;
            case (tx_state)
                0: if (period_count == PERIOD_CLKS - 1 && !tx_busy) begin
                    tx_word <= UART_TEST_MODE ? 16'ha55a : result_word;
                    tx_data <= UART_TEST_MODE ? 8'ha5 : result_word[15:8];
                    tx_start <= 1;
                    tx_state <= 1;
                end
                1: if (tx_done) begin
                    tx_data <= tx_word[7:0];
                    tx_start <= 1;
                    tx_state <= 2;
                end
                2: if (tx_done) tx_state <= 0;
                default: tx_state <= 0;
            endcase
        end
    end
endmodule
