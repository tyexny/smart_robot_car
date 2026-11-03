// Standalone CS100A controller; no UART or periodic scheduler dependency.
// Pulse start for ONE clock when busy=0; starts while busy are ignored.
// Wait >=3 clocks after reset before start (input synchronization).
// done pulses for one clock on success OR error. Sample outputs with done.
// status: 0=success, 1=no echo, 2=echo timeout, 3=echo initially high.
// distance_cm is valid only when done=1 and status=0; errors return zero.
// Outputs hold until the next accepted start. Clock must be >=1 MHz.
// Caller controls measurement interval; allow module recovery between starts.
module cs100a_measure #(
    parameter integer CLK_FREQ_HZ = 50_000_000
)(
    input wire clk, reset_n, start, echo,
    output reg trig,
    output reg busy, done,
    output reg [15:0] distance_cm,
    output reg [1:0] status
);
    // CS100A manual recommends approximately 50 us; round up to whole clocks.
    localparam integer TRIG_CLKS = (CLK_FREQ_HZ + 19999) / 20000;
    localparam integer TIMEOUT_CLKS = CLK_FREQ_HZ / 25; // 40 ms
    localparam [1:0] IDLE = 0, TRIGGER = 1, WAIT_ECHO = 2, MEASURE = 3;
    reg echo_meta, echo_sync;
    reg [1:0] state;
    reg [31:0] trigger_count, timeout_count, fraction;
    reg [15:0] cm_count;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            echo_meta <= 0;
            echo_sync <= 0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            trigger_count <= 0;
            timeout_count <= 0;
            fraction <= 0;
            cm_count <= 0;
            distance_cm <= 0;
            status <= 0;
            busy <= 0;
            done <= 0;
            trig <= 0;
        end else begin
            done <= 0;
            if (start && !busy) begin
                distance_cm <= 0;
                status <= 0;
                busy <= 1;
                fraction <= 0;
                cm_count <= 0;
                trigger_count <= 0;
                timeout_count <= 0;
                if (!echo_sync) begin
                    trig <= 1;
                    state <= TRIGGER;
                end else begin
                    // ECHO already high: do not interpret a stale pulse.
                    trig <= 0;
                    state <= IDLE;
                    status <= 3;
                    busy <= 0;
                    done <= 1;
                end
            end else begin
                case (state)
                    TRIGGER: begin
                        if (trigger_count == TRIG_CLKS - 1) begin
                            trig <= 0;
                            timeout_count <= 0;
                            state <= WAIT_ECHO;
                        end else trigger_count <= trigger_count + 1'b1;
                    end
                    WAIT_ECHO, MEASURE: begin
                        if (timeout_count == TIMEOUT_CLKS - 1) begin
                            status <= (state == WAIT_ECHO) ? 1 : 2;
                            busy <= 0;
                            done <= 1;
                            state <= IDLE;
                        end else begin
                            timeout_count <= timeout_count + 1'b1;
                            if (echo_sync) begin
                                state <= MEASURE;
                                // Exact rational accumulation, avoiding division
                                // and discarding only the final fractional cm.
                                if (cm_count != 16'hff00) begin
                                    if (fraction >= CLK_FREQ_HZ - 17000) begin
                                        fraction <= fraction - (CLK_FREQ_HZ - 17000);
                                        cm_count <= cm_count + 1'b1;
                                    end else fraction <= fraction + 17000;
                                end
                            end else if (state == MEASURE) begin
                                distance_cm <= cm_count;
                                status <= 0;
                                busy <= 0;
                                done <= 1;
                                state <= IDLE;
                            end
                        end
                    end
                    default: state <= IDLE;
                endcase
            end

        end
    end

endmodule
