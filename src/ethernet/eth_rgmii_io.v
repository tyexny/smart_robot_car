// Gowin DDR IO primitives. RX_CTL rising edge = RX_DV, falling = RX_DV XOR RX_ER.
// The board/PHY must provide the RGMII clock-data skew; see Ethernet README.
module eth_rgmii_io #(
    // GW5A: about 12.5 ps per tap. Match all five RX data/control inputs.
    // This compensates the additional on-chip RX clock insertion, not the
    // PHY's skew. Keep PHY RX delay enabled. Verify setup AND hold after PNR.
    parameter RX_INPUT_DELAY_TAPS=80
)(
    input tx_clk, input tx_clk_shifted,
    input [7:0] gmii_txd, input gmii_txen,
    output eth_gtxc, output [3:0] eth_txd, output eth_txen,
    input eth_rxc, input [3:0] eth_rxd, input eth_rxdv,
    output rx_clk, output [7:0] gmii_rxd, output gmii_rxdv, output gmii_rxer
);
    wire rx_ctl_rise, rx_ctl_fall;
    wire [3:0] rx_data_delayed;
    wire rx_ctl_delayed;
    assign rx_clk=eth_rxc;
    assign gmii_rxdv=rx_ctl_rise;
    assign gmii_rxer=rx_ctl_rise^rx_ctl_fall;
    genvar i;
    generate for(i=0;i<4;i=i+1) begin:g_ddr
        IODELAY #(.C_STATIC_DLY(RX_INPUT_DELAY_TAPS),.DYN_DLY_EN("FALSE"),
                  .ADAPT_EN("FALSE")) rx_delay(
            .DO(rx_data_delayed[i]),.DF(),.DI(eth_rxd[i]),
            .SDTAP(1'b0),.VALUE(1'b0),.DLYSTEP(8'd0));
        ODDR tx_ddr(.Q0(eth_txd[i]),.Q1(),.D0(gmii_txd[i]),
                    .D1(gmii_txd[i+4]),.TX(1'b1),.CLK(tx_clk));
        IDDR rx_ddr(.Q0(gmii_rxd[i]),.Q1(gmii_rxd[i+4]),
                    .D(rx_data_delayed[i]),.CLK(eth_rxc));
    end endgenerate
    ODDR tx_ctl_ddr(.Q0(eth_txen),.Q1(),.D0(gmii_txen),.D1(gmii_txen),
                     .TX(1'b1),.CLK(tx_clk));
    ODDR tx_clk_ddr(.Q0(eth_gtxc),.Q1(),.D0(1'b1),.D1(1'b0),
                     .TX(1'b1),.CLK(tx_clk_shifted));
    IODELAY #(.C_STATIC_DLY(RX_INPUT_DELAY_TAPS),.DYN_DLY_EN("FALSE"),
              .ADAPT_EN("FALSE")) rx_ctl_delay(
        .DO(rx_ctl_delayed),.DF(),.DI(eth_rxdv),
        .SDTAP(1'b0),.VALUE(1'b0),.DLYSTEP(8'd0));
    IDDR rx_ctl_ddr(.Q0(rx_ctl_rise),.Q1(rx_ctl_fall),
                    .D(rx_ctl_delayed),.CLK(eth_rxc));
endmodule
