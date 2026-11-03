// Both switches use the same 20ms filtering as motor_motion_top's SW6.
// Mapping never drives motors. SW6 always owns the effective mode.
module scan_mode_control #(
    parameter integer CLK_FREQ_HZ=50000000, DEBOUNCE_MS=20
)(
    input wire clk,reset_n,sw5_async,sw6_async,
    output wire mapping_active,
    output wire avoidance_active
);
    wire sw5_on;
    avoidance_switch #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(DEBOUNCE_MS)) sw5(
        .clk(clk),.reset_n(reset_n),.switch_async(sw5_async),.avoidance_enabled(sw5_on));
    avoidance_switch #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(DEBOUNCE_MS)) sw6(
        .clk(clk),.reset_n(reset_n),.switch_async(sw6_async),.avoidance_enabled(avoidance_active));
    assign mapping_active=sw5_on && !avoidance_active;
endmodule
