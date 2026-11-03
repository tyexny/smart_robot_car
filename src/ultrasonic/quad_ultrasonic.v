// Four fixed-direction CS100A probes share four non-overlapping 60 ms slots.
// A 240 ms cycle leaves the 40 ms timeout plus recovery time in each slot.
// Keep the established clear-space convention: no ECHO/out-of-range => 500 cm;
// stuck-high, high-pulse timeout and stale results are invalid. Real 0 cm is close.
// Measured validity remains independent for the original front/rear display.
// Configure MEASURE_PERIOD_MS >= 240 to retain at least 60 ms between probes.
module quad_ultrasonic #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer MEASURE_PERIOD_MS = 240,
    parameter integer STALE_MS = 350,
    parameter integer MAX_VALID_DISTANCE_CM = 500
)(
    input wire clk,
    input wire reset_n,
    input wire front_echo,
    input wire left_echo,
    input wire rear_echo,
    input wire right_echo,
    output wire front_trig,
    output wire left_trig,
    output wire rear_trig,
    output wire right_trig,
    output reg [15:0] front_distance_cm,
    output reg [15:0] left_distance_cm,
    output reg [15:0] rear_distance_cm,
    output reg [15:0] right_distance_cm,
    output reg front_valid,
    output reg left_valid,
    output reg rear_valid,
    output reg right_valid,
    output reg front_measured_valid,
    output reg left_measured_valid,
    output reg rear_measured_valid,
    output reg right_measured_valid,
    output reg front_update,
    output reg left_update,
    output reg rear_update,
    output reg right_update,
    output reg [1:0] front_status,
    output reg [1:0] left_status,
    output reg [1:0] rear_status,
    output reg [1:0] right_status,
    output wire front_busy,
    output wire left_busy,
    output wire rear_busy,
    output wire right_busy
);
    localparam integer STALE_CLKS = (CLK_FREQ_HZ / 1000) * STALE_MS
                                 + ((CLK_FREQ_HZ % 1000) * STALE_MS) / 1000;
    reg [31:0] front_age;
    wire [15:0] front_raw_cm;
    wire [1:0] front_raw_status;
    wire front_done;
    reg [31:0] left_age;
    wire [15:0] left_raw_cm;
    wire [1:0] left_raw_status;
    wire left_done;
    reg [31:0] rear_age;
    wire [15:0] rear_raw_cm;
    wire [1:0] rear_raw_status;
    wire rear_done;
    reg [31:0] right_age;
    wire [15:0] right_raw_cm;
    wire [1:0] right_raw_status;
    wire right_done;

    uart_echo #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .MEASURE_PERIOD_MS(MEASURE_PERIOD_MS),
        .START_OFFSET_MS((MEASURE_PERIOD_MS * 0) / 4)
    ) front_sensor (
        .clk(clk), .reset_n(reset_n), .echo(front_echo), .trig(front_trig),
        .uart_tx(), .distance_cm(front_raw_cm), .measure_status(front_raw_status),
        .measure_done(front_done), .measure_busy(front_busy)
    );
    uart_echo #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .MEASURE_PERIOD_MS(MEASURE_PERIOD_MS),
        .START_OFFSET_MS((MEASURE_PERIOD_MS * 1) / 4)
    ) left_sensor (
        .clk(clk), .reset_n(reset_n), .echo(left_echo), .trig(left_trig),
        .uart_tx(), .distance_cm(left_raw_cm), .measure_status(left_raw_status),
        .measure_done(left_done), .measure_busy(left_busy)
    );
    uart_echo #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .MEASURE_PERIOD_MS(MEASURE_PERIOD_MS),
        .START_OFFSET_MS((MEASURE_PERIOD_MS * 2) / 4)
    ) rear_sensor (
        .clk(clk), .reset_n(reset_n), .echo(rear_echo), .trig(rear_trig),
        .uart_tx(), .distance_cm(rear_raw_cm), .measure_status(rear_raw_status),
        .measure_done(rear_done), .measure_busy(rear_busy)
    );
    uart_echo #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .MEASURE_PERIOD_MS(MEASURE_PERIOD_MS),
        .START_OFFSET_MS((MEASURE_PERIOD_MS * 3) / 4)
    ) right_sensor (
        .clk(clk), .reset_n(reset_n), .echo(right_echo), .trig(right_trig),
        .uart_tx(), .distance_cm(right_raw_cm), .measure_status(right_raw_status),
        .measure_done(right_done), .measure_busy(right_busy)
    );

    // Update is asserted with latched data, status and validity. A new trigger
    // preserves the prior sample while the next measurement is in progress.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            front_distance_cm <= 0;
            front_valid <= 0;
            front_measured_valid <= 0;
            front_update <= 0;
            front_status <= 1;
            front_age <= STALE_CLKS;
            left_distance_cm <= 0;
            left_valid <= 0;
            left_measured_valid <= 0;
            left_update <= 0;
            left_status <= 1;
            left_age <= STALE_CLKS;
            rear_distance_cm <= 0;
            rear_valid <= 0;
            rear_measured_valid <= 0;
            rear_update <= 0;
            rear_status <= 1;
            rear_age <= STALE_CLKS;
            right_distance_cm <= 0;
            right_valid <= 0;
            right_measured_valid <= 0;
            right_update <= 0;
            right_status <= 1;
            right_age <= STALE_CLKS;
        end else begin
            front_update <= 0;
            if (front_age < STALE_CLKS) front_age <= front_age + 1'b1;
            if (front_age >= STALE_CLKS - 1) begin
                front_valid <= 0;
                front_measured_valid <= 0;
            end
            left_update <= 0;
            if (left_age < STALE_CLKS) left_age <= left_age + 1'b1;
            if (left_age >= STALE_CLKS - 1) begin
                left_valid <= 0;
                left_measured_valid <= 0;
            end
            rear_update <= 0;
            if (rear_age < STALE_CLKS) rear_age <= rear_age + 1'b1;
            if (rear_age >= STALE_CLKS - 1) begin
                rear_valid <= 0;
                rear_measured_valid <= 0;
            end
            right_update <= 0;
            if (right_age < STALE_CLKS) right_age <= right_age + 1'b1;
            if (right_age >= STALE_CLKS - 1) begin
                right_valid <= 0;
                right_measured_valid <= 0;
            end

            if (front_done) begin
                front_update <= 1;
                front_status <= front_raw_status;
                front_measured_valid <= front_raw_status == 0 &&
                                        front_raw_cm <= MAX_VALID_DISTANCE_CM;
                if (front_raw_status == 0 || front_raw_status == 1) begin
                    front_distance_cm <= (front_raw_status == 1 || front_raw_cm > MAX_VALID_DISTANCE_CM) ?
                                         MAX_VALID_DISTANCE_CM : front_raw_cm;
                    front_valid <= 1;
                    front_age <= 0;
                end else front_valid <= 0;
            end
            if (left_done) begin
                left_update <= 1;
                left_status <= left_raw_status;
                left_measured_valid <= left_raw_status == 0 &&
                                        left_raw_cm <= MAX_VALID_DISTANCE_CM;
                if (left_raw_status == 0 || left_raw_status == 1) begin
                    left_distance_cm <= (left_raw_status == 1 || left_raw_cm > MAX_VALID_DISTANCE_CM) ?
                                         MAX_VALID_DISTANCE_CM : left_raw_cm;
                    left_valid <= 1;
                    left_age <= 0;
                end else left_valid <= 0;
            end
            if (rear_done) begin
                rear_update <= 1;
                rear_status <= rear_raw_status;
                rear_measured_valid <= rear_raw_status == 0 &&
                                        rear_raw_cm <= MAX_VALID_DISTANCE_CM;
                if (rear_raw_status == 0 || rear_raw_status == 1) begin
                    rear_distance_cm <= (rear_raw_status == 1 || rear_raw_cm > MAX_VALID_DISTANCE_CM) ?
                                         MAX_VALID_DISTANCE_CM : rear_raw_cm;
                    rear_valid <= 1;
                    rear_age <= 0;
                end else rear_valid <= 0;
            end
            if (right_done) begin
                right_update <= 1;
                right_status <= right_raw_status;
                right_measured_valid <= right_raw_status == 0 &&
                                        right_raw_cm <= MAX_VALID_DISTANCE_CM;
                if (right_raw_status == 0 || right_raw_status == 1) begin
                    right_distance_cm <= (right_raw_status == 1 || right_raw_cm > MAX_VALID_DISTANCE_CM) ?
                                         MAX_VALID_DISTANCE_CM : right_raw_cm;
                    right_valid <= 1;
                    right_age <= 0;
                end else right_valid <= 0;
            end
        end
    end
endmodule
