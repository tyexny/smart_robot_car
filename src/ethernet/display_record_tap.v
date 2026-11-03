// Passive mirror of the FINAL HDMI raster, after all pages and overlays.
// The existing packetizer downsamples by two in X/Y and owns both line RAMs.
// Register RGB/DE/VS together: no new clock, DDR access or HDMI backpressure.
module display_record_tap(
    input wire pixel_clk, reset_n,
    input wire [23:0] hdmi_rgb,
    input wire hdmi_de, hdmi_vs,
    output reg [15:0] pixel_rgb565,
    output reg pixel_valid, capture_vsync
);
    always @(posedge pixel_clk or negedge reset_n) begin
        if(!reset_n) begin
            pixel_rgb565<=0;
            pixel_valid<=0;
            // Do not manufacture a blank edge at reset release. Wait for the
            // first real active-low HDMI VS before assigning a frame ID.
            capture_vsync<=0;
        end else begin
            pixel_rgb565<={hdmi_rgb[23:19],hdmi_rgb[15:10],hdmi_rgb[7:3]};
            pixel_valid<=hdmi_de;
            // disp_driver uses active-LOW VS; the camera packetizer expects
            // active-HIGH vertical blank and counts only pixels outside it.
            capture_vsync<=!hdmi_vs;
        end
    end
endmodule
