`timescale 1ns/1ps
// SW6 is asynchronous. Require a stable new level for 20 ms after two-flop
// synchronization. Reset selects ordinary remote control (avoidance disabled).
module avoidance_switch #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer DEBOUNCE_MS=20
)(
    input wire clk, reset_n, switch_async,
    output reg avoidance_enabled
);
    localparam integer RAW_CLKS=(CLK_FREQ_HZ/1000)*DEBOUNCE_MS
                              + ((CLK_FREQ_HZ%1000)*DEBOUNCE_MS)/1000;
    localparam integer DEBOUNCE_CLKS=(RAW_CLKS<1) ? 1 : RAW_CLKS;
    function integer counter_width;
        input integer cycles;
        integer remaining;
        begin
            counter_width=1;
            remaining=cycles-1;
            while(remaining>1) begin
                counter_width=counter_width+1;
                remaining=remaining>>1;
            end
        end
    endfunction
    localparam integer COUNT_WIDTH=counter_width(DEBOUNCE_CLKS);
    reg switch_meta, switch_sync;
    reg [COUNT_WIDTH-1:0] stable_count;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            switch_meta<=0; switch_sync<=0;
        end else begin
            switch_meta<=switch_async;
            switch_sync<=switch_meta;
        end
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            avoidance_enabled<=0;
            stable_count<=0;
        end else if(switch_sync==avoidance_enabled) begin
            stable_count<=0;
        end else if(stable_count>=DEBOUNCE_CLKS-1) begin
            avoidance_enabled<=switch_sync;
            stable_count<=0;
        end else stable_count<=stable_count+1'b1;
    end
endmodule
