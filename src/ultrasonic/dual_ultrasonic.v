// Two fixed-direction CS100A sensors using the existing uart_echo measurement path.
// Alternate starts by 60 ms: a complete 40 ms timeout fits within either slot.
// User's open-space convention: no echo or a completed out-of-range/no-target
// pulse means clear space, represented as the 500 cm range limit. An ECHO
// stuck high, high-pulse timeout, or stale result remains an invalid sensor.
// Zero centimetres is valid and represents a dangerously close obstacle.
// Display validity is independent: only an in-range completed ECHO may be
// shown as a measured distance; clear-space fallbacks leave the display blank.
module dual_ultrasonic #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer MEASURE_PERIOD_MS = 120,
    parameter integer STALE_MS = 350,
    parameter integer MAX_VALID_DISTANCE_CM = 500
)(
    input wire clk,
    input wire reset_n,
    input wire front_echo,
    input wire rear_echo,
    output wire front_trig,
    output wire rear_trig,
    output reg [15:0] front_distance_cm,
    output reg [15:0] rear_distance_cm,
    output reg front_valid,
    output reg rear_valid,
    output reg front_measured_valid,
    output reg rear_measured_valid,
    output reg front_update,
    output reg rear_update,
    output reg [1:0] front_status,
    output reg [1:0] rear_status,
    output wire front_busy,
    output wire rear_busy
);
    localparam integer STALE_CLKS = (CLK_FREQ_HZ / 1000) * STALE_MS
                                 + ((CLK_FREQ_HZ % 1000) * STALE_MS) / 1000;
    reg [31:0] front_age, rear_age;
    wire [15:0] front_raw_cm, rear_raw_cm;
    wire [1:0] front_raw_status, rear_raw_status;
    wire front_done, rear_done;

    uart_echo #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .MEASURE_PERIOD_MS(MEASURE_PERIOD_MS),
        .START_OFFSET_MS(0)
    ) front_sensor (
        .clk(clk), .reset_n(reset_n), .echo(front_echo), .trig(front_trig),
        .uart_tx(), .distance_cm(front_raw_cm), .measure_status(front_raw_status),
        .measure_done(front_done), .measure_busy(front_busy)
    );
    uart_echo #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .MEASURE_PERIOD_MS(MEASURE_PERIOD_MS),
        .START_OFFSET_MS(MEASURE_PERIOD_MS / 2)
    ) rear_sensor (
        .clk(clk), .reset_n(reset_n), .echo(rear_echo), .trig(rear_trig),
        .uart_tx(), .distance_cm(rear_raw_cm), .measure_status(rear_raw_status),
        .measure_done(rear_done), .measure_busy(rear_busy)
    );

    // Latch each result before asserting update: downstream logic can sample
    // distance, status and validity together on the update clock.
    // A new TRIG never erases the last valid distance during measurement.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            front_distance_cm <= 0;
            rear_distance_cm <= 0;
            front_valid <= 0;
            rear_valid <= 0;
            front_measured_valid <= 0;
            rear_measured_valid <= 0;
            front_update <= 0;
            rear_update <= 0;
            front_status <= 1;
            rear_status <= 1;
            front_age <= STALE_CLKS;
            rear_age <= STALE_CLKS;
        end else begin
            front_update <= 0;
            rear_update <= 0;
            if (front_age < STALE_CLKS) front_age <= front_age + 1'b1;
            if (rear_age < STALE_CLKS) rear_age <= rear_age + 1'b1;
            if (front_age >= STALE_CLKS - 1) begin
                front_valid <= 0;
                front_measured_valid <= 0;
            end
            if (rear_age >= STALE_CLKS - 1) begin
                rear_valid <= 0;
                rear_measured_valid <= 0;
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
        end
    end
endmodule
