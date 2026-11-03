// Only the forward +/-30 degree sector feeds the navigation safety gate.
// Missing/zero returns never become a fictitious clear 8m measurement.
// Publish the minimum from each complete validated revolution; age it out.
module lidar_front_sector #(parameter integer CLK_FREQ_HZ=50000000,STALE_MS=500)(
    input wire clk,reset_n,online,scan_start,point_valid,point_ready,range_valid,
    input wire [15:0] distance_mm,input wire [11:0] angle_tenths,
    output reg [15:0] front_distance_mm,output wire front_valid,output reg front_update
);
    localparam integer STALE_CLKS=CLK_FREQ_HZ/1000*STALE_MS;
    reg started,seen;
    reg [15:0] minimum;
    reg [31:0] age;
    assign front_valid=online && age<STALE_CLKS;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin started<=0;seen<=0;minimum<=16'hffff;age<=STALE_CLKS;front_distance_mm<=0;front_update<=0;end
        else begin
            front_update<=0;
            if(age<STALE_CLKS) age<=age+1'b1;
            if(!online) begin started<=0;seen<=0;minimum<=16'hffff;age<=STALE_CLKS;end
            else begin
                if(scan_start) begin
                    if(started && seen) begin front_distance_mm<=minimum;age<=0;front_update<=1;end
                    else age<=STALE_CLKS;
                    started<=1;seen<=0;minimum<=16'hffff;
                end
                if(point_valid && point_ready && range_valid &&
                    (angle_tenths<=300 || angle_tenths>=3300)) begin
                    seen<=1;
                    if(distance_mm<minimum) minimum<=distance_mm;
                end
            end
        end
    end
endmodule
