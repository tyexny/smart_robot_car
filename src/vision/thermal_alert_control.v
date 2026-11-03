`timescale 1ns/1ps
// Threshold belongs to the Pi web UI; no physical key is consumed.
module thermal_alert_control #(
    parameter integer CLK_HZ=50000000,
    parameter integer STALE_CYCLES=CLK_HZ*2
)(
    input wire clk,reset_n,sample_valid,
    input wire sample_sensor_valid,sample_task_active,
    input wire signed [15:0] sample_maximum_centi,
    input wire [6:0] sample_threshold_c,
    output reg task_present,
    output wire sensor_ok,alarm,
    output reg signed [15:0] maximum_centi,
    output reg [6:0] threshold_c
);
    reg [31:0] age;
    reg seen,sensor_valid;
    wire fresh=seen && age<STALE_CYCLES;
    wire signed [15:0] threshold_centi=threshold_c==60 ? 16'sd6000 :
        threshold_c==50 ? 16'sd5000 : 16'sd4000;
    assign sensor_ok=fresh && sensor_valid;
    assign alarm=task_present && sensor_ok && maximum_centi>=threshold_centi;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            age<=STALE_CYCLES;seen<=0;sensor_valid<=0;task_present<=0;
            maximum_centi<=0;threshold_c<=40;
        end else if(sample_valid) begin
            age<=0;seen<=1;sensor_valid<=sample_sensor_valid;
            task_present<=sample_task_active;maximum_centi<=sample_maximum_centi;
            threshold_c<=sample_threshold_c;
        end else if(age<STALE_CYCLES) age<=age+1'b1;
    end
endmodule
