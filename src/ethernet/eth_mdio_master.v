// Clause 22 MDIO master. 32-bit preamble, explicit read turnaround release.
module eth_mdio_master #(
    parameter CLK_HZ=50000000, parameter MDC_HZ=1000000
)(
    input clk, input reset_n, input start, input read_op,
    input [4:0] phy_addr, input [4:0] reg_addr, input [15:0] write_data,
    output reg busy, output reg done, output reg [15:0] read_data,
    output reg read_ack, output reg mdc, inout mdio
);
    localparam HALF_DIV=CLK_HZ/(2*MDC_HZ);
    reg [15:0] divider;
    reg [6:0] bit_index;
    reg [63:0] frame;
    reg reading, mdio_out, mdio_enable;
    assign mdio=mdio_enable?mdio_out:1'bz;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            busy<=0; done<=0; read_data<=0; read_ack<=0; mdc<=0;
            divider<=0; bit_index<=0; frame<=0; reading<=0;
            mdio_out<=1; mdio_enable<=0;
        end else begin
            done<=0;
            if(!busy) begin
                mdc<=0; divider<=0; mdio_enable<=0;
                if(start) begin
                    frame<={32'hffffffff,2'b01,(read_op?2'b10:2'b01),
                            phy_addr,reg_addr,2'b10,write_data};
                    bit_index<=0; busy<=1; reading<=read_op;
                    read_data<=0; read_ack<=0; mdio_out<=1; mdio_enable<=1;
                end
            end else if(divider==HALF_DIV-1) begin
                divider<=0;
                if(!mdc) begin
                    mdc<=1;
                end else begin
                    // RTL8211F t6 permits read data to become valid up to 300 ns
                    // AFTER MDC rises. Sample at the end of its 500 ns high phase.
                    if(reading&&bit_index==47) read_ack<=!mdio;
                    if(reading&&bit_index>=48&&bit_index<64) read_data<={read_data[14:0],mdio};
                    mdc<=0;
                    // Keep MDC running for eight idle cycles after the frame.
                    // RTL8211F requires >= 7 cycles to finish an MDIO operation.
                    if(bit_index==71) begin busy<=0; done<=1; mdio_enable<=0; end
                    else begin
                        bit_index<=bit_index+1'b1;
                        if(bit_index<63) mdio_out<=frame[62-bit_index];
                        else mdio_out<=1;
                        // Bits 46/47 are turnaround, 48..63 are PHY read data.
                        mdio_enable<=(bit_index<63)&&!(reading&&bit_index>=45);
                    end
                end
            end else divider<=divider+1'b1;
        end
    end
endmodule
