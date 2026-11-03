// User acquisition switch, independent of map/odometry enable. A pause cancels
// the unfinished frame through the existing allow/rescan gates, but never
// resets map RAM, the accepted transform, or the stationary-origin state.
module scan_run_control(
    input wire clk,reset_n,enable,toggle_event,clear_event,
    input wire engine_busy,map_busy,
    output reg running,
    output wire paused_ready
);
    reg [1:0] idle_cycles;
    // Two idle edges allow an already-issued ray pulse to reach the map RAM
    // and finish. Busy includes the matcher's accepted late-response drain.
    assign paused_ready=enable && !running && idle_cycles==2 && !engine_busy && !map_busy;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin running<=0;idle_cycles<=0;end
        else if(!enable || clear_event) begin running<=0;idle_cycles<=0;end
        else if(toggle_event) begin running<=!running;idle_cycles<=0;end
        else if(running || engine_busy || map_busy) idle_cycles<=0;
        else if(idle_cycles<2) idle_cycles<=idle_cycles+1'b1;
    end
endmodule
