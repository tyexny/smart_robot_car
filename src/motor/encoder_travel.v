// Convert four modulo-16-bit wheel encoder counters into unsigned travel steps.
// Callers decide which motion phases accumulate these steps; every fresh frame
// still advances the baseline so turns and idle motion cannot leak into travel.
module encoder_travel #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer STALE_MS = 500,
    parameter integer MAX_STEP = 2048
)(
    input wire clk,
    input wire reset_n,
    input wire [63:0] counts,
    input wire counts_update,
    output wire valid,
    output reg step_valid,
    output reg [17:0] step_sum,
    output reg fault
);
    localparam integer RAW_STALE_CLKS = (CLK_FREQ_HZ / 1000) * STALE_MS +
                                       ((CLK_FREQ_HZ % 1000) * STALE_MS) / 1000;
    localparam integer STALE_CLKS = (RAW_STALE_CLKS < 1) ? 1 : RAW_STALE_CLKS;

    reg [63:0] previous_counts;
    reg baseline_valid;
    reg healthy;
    reg [31:0] age;

    // Truncating subtraction to 16 bits handles both signed crossings and wrap.
    wire [15:0] delta_a = counts[15:0]  - previous_counts[15:0];
    wire [15:0] delta_b = counts[31:16] - previous_counts[31:16];
    wire [15:0] delta_c = counts[47:32] - previous_counts[47:32];
    wire [15:0] delta_d = counts[63:48] - previous_counts[63:48];

    function [15:0] absolute_delta;
        input [15:0] delta;
        begin
            absolute_delta = delta[15] ? (~delta + 16'd1) : delta;
        end
    endfunction

    wire [15:0] step_a = absolute_delta(delta_a);
    wire [15:0] step_b = absolute_delta(delta_b);
    wire [15:0] step_c = absolute_delta(delta_c);
    wire [15:0] step_d = absolute_delta(delta_d);
    wire jump = (step_a > MAX_STEP) || (step_b > MAX_STEP) ||
                (step_c > MAX_STEP) || (step_d > MAX_STEP);
    wire [17:0] total_step = {2'b00, step_a} + {2'b00, step_b} +
                             {2'b00, step_c} + {2'b00, step_d};

    assign valid = baseline_valid && healthy && (age < STALE_CLKS);

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            previous_counts <= 64'd0;
            baseline_valid <= 1'b0;
            healthy <= 1'b0;
            age <= STALE_CLKS;
            step_valid <= 1'b0;
            step_sum <= 18'd0;
            fault <= 1'b0;
        end else begin
            step_valid <= 1'b0;
            step_sum <= 18'd0;
            fault <= 1'b0;
            if (age < STALE_CLKS)
                age <= age + 32'd1;

            if (counts_update) begin
                // Rebase even an abnormal frame so its jump is never retried.
                previous_counts <= counts;
                age <= 32'd0;
                baseline_valid <= 1'b1;
                if (!baseline_valid || (age >= STALE_CLKS)) begin
                    healthy <= 1'b1;
                end else if (jump) begin
                    healthy <= 1'b0;
                    fault <= 1'b1;
                end else begin
                    healthy <= 1'b1;
                    step_valid <= 1'b1;
                    step_sum <= total_step;
                end
            end
        end
    end
endmodule
