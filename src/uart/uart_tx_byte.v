`timescale 1ns/1ps

module uart_tx_byte #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer BAUD_RATE=115_200
)(
    input wire clk, reset_n, start,
    input wire [7:0] data,
    output reg uart_tx, busy, done
);
    localparam integer CLKS_PER_BIT=CLK_FREQ_HZ/BAUD_RATE;
    localparam [1:0] IDLE=0, START=1, DATA=2, STOP=3;
    reg [1:0] state;
    reg [31:0] count;
    reg [2:0] bit_index;
    reg [7:0] latched;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE; count<=0; bit_index<=0; latched<=0;
            uart_tx<=1; busy<=0; done<=0;
        end else begin
            done<=0;
            case(state)
                IDLE: begin
                    uart_tx<=1; busy<=0; count<=0;
                    if(start) begin latched<=data; busy<=1; state<=START; end
                end
                START: begin
                    uart_tx<=0;
                    if(count>=CLKS_PER_BIT-1) begin count<=0; bit_index<=0; state<=DATA; end
                    else count<=count+1'b1;
                end
                DATA: begin
                    uart_tx<=latched[bit_index];
                    if(count>=CLKS_PER_BIT-1) begin
                        count<=0;
                        if(bit_index==7) state<=STOP;
                        else bit_index<=bit_index+1'b1;
                    end else count<=count+1'b1;
                end
                STOP: begin
                    uart_tx<=1;
                    if(count>=CLKS_PER_BIT-1) begin count<=0; busy<=0; done<=1; state<=IDLE; end
                    else count<=count+1'b1;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
