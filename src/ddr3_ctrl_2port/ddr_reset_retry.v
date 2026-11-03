// Release the DDR3 core only after its source PLL is stable. If calibration
// does not finish, pulse the DDR3 core reset and try again without resetting
// the HDMI or motor logic.
module ddr_reset_retry #(
    parameter integer RESET_HOLD_CYCLES = 500_000,    // 10 ms at 50 MHz
    parameter integer CAL_TIMEOUT_CYCLES = 50_000_000 // 1 s at 50 MHz
)(
    input  wire clk,
    input  wire reset_n,
    input  wire pll_lock,
    input  wire init_calib_complete,
    output reg  ddr_reset_n
);
    localparam [1:0] HOLD = 2'd0, WAIT_CAL = 2'd1, READY = 2'd2;
    reg [1:0] state;
    reg [26:0] ticks;
    reg calib_meta, calib_sync;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            calib_meta <= 1'b0;
            calib_sync <= 1'b0;
            state <= HOLD;
            ticks <= 27'd0;
            ddr_reset_n <= 1'b0;
        end else begin
            calib_meta <= init_calib_complete;
            calib_sync <= calib_meta;
            if (!pll_lock) begin
                state <= HOLD;
                ticks <= 27'd0;
                ddr_reset_n <= 1'b0;
            end else begin
                case (state)
                    HOLD: begin
                        if (ticks == RESET_HOLD_CYCLES - 1) begin
                            ticks <= 27'd0;
                            ddr_reset_n <= 1'b1;
                            state <= WAIT_CAL;
                        end else
                            ticks <= ticks + 27'd1;
                    end
                    WAIT_CAL: begin
                        if (calib_sync) begin
                            ticks <= 27'd0;
                            state <= READY;
                        end else if (ticks == CAL_TIMEOUT_CYCLES - 1) begin
                            ticks <= 27'd0;
                            ddr_reset_n <= 1'b0;
                            state <= HOLD;
                        end else
                            ticks <= ticks + 27'd1;
                    end
                    READY: begin
                        if (!calib_sync) begin
                            ticks <= 27'd0;
                            state <= WAIT_CAL;
                        end
                    end
                    default: begin
                        state <= HOLD;
                        ticks <= 27'd0;
                        ddr_reset_n <= 1'b0;
                    end
                endcase
            end
        end
    end
endmodule
