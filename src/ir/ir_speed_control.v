`timescale 1ns/1ps

// One accepted NEC frame changes the requested speed by one bounded step.
// The command filter belongs to the caller; NEC repeat frames do not change it.
module ir_speed_control #(
    parameter [7:0] IR_SPEED_UP=8'h45,
    parameter [7:0] IR_SPEED_DOWN=8'h47,
    parameter integer SPEED_INITIAL=5,
    parameter integer SPEED_MIN=1,
    parameter integer SPEED_MAX=10,
    parameter integer SPEED_STEP=1
)(
    input wire clk,
    input wire reset_n,
    input wire dec_done,
    input wire [7:0] command,
    output reg [7:0] requested_speed
);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)
            requested_speed<=SPEED_INITIAL;
        else if(dec_done) begin
            if(command==IR_SPEED_UP) begin
                if(requested_speed+SPEED_STEP>=SPEED_MAX)
                    requested_speed<=SPEED_MAX;
                else
                    requested_speed<=requested_speed+SPEED_STEP;
            end else if(command==IR_SPEED_DOWN) begin
                if(requested_speed<=SPEED_MIN+SPEED_STEP)
                    requested_speed<=SPEED_MIN;
                else
                    requested_speed<=requested_speed-SPEED_STEP;
            end
        end
    end
endmodule
