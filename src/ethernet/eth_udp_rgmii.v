// 1 Gbps RGMII endpoint. UDP video is TX-only; compact detection results are RX.
// Fixed IPv4 peers, ARP reply and peer MAC learning, no fragmentation or VLAN.
module eth_udp_rgmii #(
    parameter [47:0] LOCAL_MAC=48'h020000001002,
    parameter [31:0] LOCAL_IP=32'hc0a80a02,
    parameter [31:0] PEER_IP=32'hc0a80a01,
    parameter [15:0] SRC_PORT=5000, parameter [15:0] DST_PORT=5000,
    parameter [15:0] RX_PORT=5001, parameter [4:0] PHY_ADDR=5,
    parameter TX_CLOCK_DELAY=0, parameter MAX_RX_PAYLOAD=1472,
    parameter AUTO_PHY_ADDR=0, parameter PHY_MDC_HZ=1000000,
    parameter USE_CH59=1, parameter DYNAMIC_DST_PORT=0
)(
    input clk125m, input clk125m_90, input clk50m, input reset_n,
    input tx_start, input [15:0] tx_length, input [7:0] tx_data,
    input [15:0] tx_destination_port,
    output tx_busy, output reg tx_done, output tx_data_req,
    output [10:0] tx_payload_addr,
    output eth_gtxc, output [3:0] eth_txd, output eth_txen,
    output eth_rst_n, output eth_mdc, inout eth_mdio,
    input eth_rxc, input [3:0] eth_rxd, input eth_rxdv,
    output rx_clk,
    output [7:0] rx_payload_data, output rx_payload_valid,
    output rx_payload_start, output rx_payload_end,
    output [15:0] rx_payload_length,
    output rx_packet_good, output rx_packet_error,
    output phy_init_done, output phy_link_up, output [31:0] phy_id,
    output phy_any_read_ack, output phy_id_verified, output [4:0] phy_address,
    output phy_tx_ready
);
    wire [7:0] gmii_txd, gmii_rxd;
    wire gmii_txen, gmii_rxdv, gmii_rxer;
    wire mac_busy, mac_tx_done, arp_done;
    wire local_mac_busy,local_mac_done,local_data_req;
    wire [10:0] local_payload_addr;
    wire [7:0] local_txd,factory_txd;
    wire local_txen,factory_txen,factory_busy,factory_done;
    wire factory_data_req;
    wire [10:0] factory_payload_addr;
    reg selected_arp;
    reg user_start,arp_start;
    reg [47:0] tx_peer_mac;
    reg [15:0] user_length;
    reg [15:0] user_port;
    wire rx_arp_request, rx_peer_update;
    wire [47:0] rx_peer_mac;

    // Async assertion and local synchronous reset release in each clock domain.
    reg [1:0] tx_reset_sync, rx_reset_sync;
    always @(posedge clk125m or negedge reset_n)
        if(!reset_n) tx_reset_sync<=0;
        else tx_reset_sync<={tx_reset_sync[0],
            USE_CH59?1'b1:(phy_init_done&&phy_link_up)};
    always @(posedge rx_clk or negedge reset_n)
        if(!reset_n) rx_reset_sync<=0;
        else rx_reset_sync<={rx_reset_sync[0],1'b1};
    wire tx_reset_n=tx_reset_sync[1];
    wire rx_reset_n=rx_reset_sync[1];
    // Keep the shared MAC reset out of synchronous launch/busy logic. Reset
    // still asserts immediately; arbitration releases one local clock later.
    reg tx_control_ready;
    always @(posedge clk125m or negedge tx_reset_n)
        if(!tx_reset_n) tx_control_ready<=0;
        else tx_control_ready<=1;
    // Factory completion does not measure BMSR link, nor acknowledge an ID.
    // This ready bit reports only the local transmit reset release.
    assign phy_tx_ready=tx_reset_n;

    generate if(USE_CH59) begin:g_ch59
        wire [15:0] factory_bmcr_read;
        phy_reg_config factory_phy(
            .Clk(clk50m),.Rst_n(reset_n),.Phy_rst_n(eth_rst_n),
            .Rd_Data(factory_bmcr_read),.Phy_init_done(phy_init_done),
            .mdc(eth_mdc),.mdio(eth_mdio));
        assign phy_link_up=1'b0; // no BMSR read in the manufacturer initializer
        assign phy_id=32'd0;
        assign phy_any_read_ack=1'b0;
        assign phy_id_verified=1'b0;
        assign phy_address=5'd5;
        ch59_udp_tx_adapter #(.LOCAL_MAC(LOCAL_MAC),.LOCAL_IP(LOCAL_IP),
            .PEER_IP(PEER_IP),.SRC_PORT(SRC_PORT),.DST_PORT(DST_PORT),.DYNAMIC_DST_PORT(DYNAMIC_DST_PORT)) factory_tx(
            .clk(clk125m),.reset_n(tx_reset_n),.tx_start(user_start),
            .tx_length(user_length),.tx_data(tx_data),.peer_mac(tx_peer_mac),.destination_port(user_port),
            .tx_busy(factory_busy),.tx_done(factory_done),
            .tx_data_req(factory_data_req),.tx_payload_addr(factory_payload_addr),
            .gmii_txd(factory_txd),.gmii_txen(factory_txen));
        // One common output register keeps the arbitration mux out of DDR IO.
        // Hold the owner for the complete frame and its interframe gap.
        reg [7:0] selected_txd;
        reg selected_txen;
        always @(posedge clk125m or negedge reset_n) begin
            if(!reset_n) begin selected_txd<=0;selected_txen<=0;end
            else if(!tx_reset_n) begin selected_txd<=0;selected_txen<=0;end
            else begin
                selected_txd<=selected_arp?local_txd:factory_txd;
                selected_txen<=selected_arp?local_txen:factory_txen;
            end
        end
        assign gmii_txd=selected_txd;
        assign gmii_txen=selected_txen;
        gmii_to_rgmii factory_ddr(
            .reset_n(reset_n),.gmii_tx_clk(clk125m),
            .gmii_txd(gmii_txd),.gmii_txen(gmii_txen),.gmii_txer(1'b0),
            .rgmii_tx_clk(eth_gtxc),.rgmii_txd(eth_txd),.rgmii_txen(eth_txen));
    end else begin:g_legacy
        eth_phy_config #(.PHY_ADDR(PHY_ADDR),.FPGA_TX_DELAY(TX_CLOCK_DELAY),
            .AUTO_PHY_ADDR(AUTO_PHY_ADDR),.MDC_HZ(PHY_MDC_HZ)) phy_configuration(
            .clk(clk50m),.reset_n(reset_n),.phy_reset_n(eth_rst_n),
            .mdc(eth_mdc),.mdio(eth_mdio),.init_done(phy_init_done),
            .link_up(phy_link_up),.phy_id(phy_id),
            .any_read_ack(phy_any_read_ack),.id_verified(phy_id_verified),
            .phy_address(phy_address));
        assign factory_txd=0;assign factory_txen=0;
        assign factory_busy=0;assign factory_done=0;
        assign factory_data_req=0;assign factory_payload_addr=0;
        assign gmii_txd=local_txd;assign gmii_txen=local_txen;
    end endgenerate
    // Preserve the existing RX DDR/delay hierarchy and validated RX decoder.
    // The factory branch uses the manufacturer ODDR converter for all TX;
    // disconnected TX outputs below are removed by synthesis.
    eth_rgmii_io rgmii_io(
        .tx_clk(clk125m),.tx_clk_shifted(TX_CLOCK_DELAY?clk125m_90:clk125m),
        .gmii_txd(gmii_txd),.gmii_txen(gmii_txen),
        .eth_gtxc(),.eth_txd(),.eth_txen(),
        .eth_rxc(eth_rxc),.eth_rxd(eth_rxd),.eth_rxdv(eth_rxdv),
        .rx_clk(rx_clk),.gmii_rxd(gmii_rxd),.gmii_rxdv(gmii_rxdv),.gmii_rxer(gmii_rxer));
    generate if(!USE_CH59) begin:g_legacy_ddr
        eth_rgmii_io transmit_io(
            .tx_clk(clk125m),.tx_clk_shifted(TX_CLOCK_DELAY?clk125m_90:clk125m),
            .gmii_txd(gmii_txd),.gmii_txen(gmii_txen),
            .eth_gtxc(eth_gtxc),.eth_txd(eth_txd),.eth_txen(eth_txen),
            .eth_rxc(eth_rxc),.eth_rxd(eth_rxd),.eth_rxdv(eth_rxdv),
            .rx_clk(),.gmii_rxd(),.gmii_rxdv(),.gmii_rxer());
    end endgenerate
    eth_udp_rx #(.LOCAL_MAC(LOCAL_MAC),.LOCAL_IP(LOCAL_IP),.PEER_IP(PEER_IP),
        .PEER_PORT(DST_PORT),.LOCAL_PORT(RX_PORT),.MAX_RX_PAYLOAD(MAX_RX_PAYLOAD),.ALLOW_RPC_PORTS(DYNAMIC_DST_PORT)) rx_mac(
        .clk(rx_clk),.reset_n(rx_reset_n),.gmii_rxd(gmii_rxd),
        .gmii_rxdv(gmii_rxdv),.gmii_rxer(gmii_rxer),
        .rx_payload_data(rx_payload_data),.rx_payload_valid(rx_payload_valid),
        .rx_payload_start(rx_payload_start),.rx_payload_end(rx_payload_end),
        .rx_payload_length(rx_payload_length),.rx_packet_good(rx_packet_good),
        .rx_packet_error(rx_packet_error),.arp_request(rx_arp_request),
        .peer_update(rx_peer_update),.peer_mac(rx_peer_mac));

    // Two held-data toggle mailboxes: peer MAC learning and pending ARP reply.
    // A held bus is sampled only after its control toggle has crossed two FFs.
    reg peer_request_toggle, arp_request_toggle;
    reg [47:0] peer_mailbox, arp_mailbox;
    reg peer_ack_toggle, arp_ack_toggle;
    reg [1:0] peer_ack_rx, arp_ack_rx, peer_req_tx, arp_req_tx;
    reg user_pending, user_active;
    reg ingress_start;
    // One reservation spans ingress, pending and active ownership. Its
    // local fanout avoids re-decoding the distributed MAC state at ingress.
    reg user_reserved;
    reg mac_busy_visible;
    reg [15:0] ingress_length,ingress_port;
    // Register the complete ingress transaction before MTU validation and
    // queue ownership. A MAC busy -> producer grant -> metadata mux -> MTU
    // decode path must not reach every queued length/port clock enable.
    always @(posedge clk125m or negedge reset_n)begin
        if(!reset_n)begin ingress_start<=0;user_reserved<=0;ingress_length<=0;ingress_port<=DST_PORT;mac_busy_visible<=1;end
        else begin
            // Requests are reserved by ingress/pending/active immediately.
            // Registered MAC occupancy only delays idle visibility; actual
            // arbitration below still uses live mac_busy and queues ARP races.
            mac_busy_visible<=mac_busy;
            ingress_start<=tx_start&&!user_reserved;
            if(tx_start&&!user_reserved)user_reserved<=1;
            else if((mac_tx_done&&user_active)||
                    ((user_pending||user_active)&&!tx_reset_n)||
                    (ingress_start&&(ingress_length>1472||(USE_CH59&&ingress_length==0))))
                user_reserved<=0;
            ingress_length<=tx_length;
            ingress_port<=DYNAMIC_DST_PORT?tx_destination_port:DST_PORT;
        end
    end
    reg last_grant_was_arp;
    wire arp_pending=arp_req_tx[1]!=arp_ack_toggle;
    // Public producer backpressure is a local registered observation. The
    // arbiter below still uses live pending, and ingress already queues an
    // accepted producer request if an ARP reply wins on this same edge.
    reg arp_pending_visible;
    always @(posedge clk125m or negedge tx_reset_n)
        if(!tx_reset_n)arp_pending_visible<=0;
        else arp_pending_visible<=arp_pending;
    // Round-robin only when both classes are waiting. A continuous ARP stream
    // cannot hold an accepted row bank forever, and video cannot starve ARP.
    // Queue one exclusive grant before the MAC and owner/ack registers consume
    // it together. A queued grant occupies arbitration until the next edge;
    // otherwise the still-idle MAC could accept the same request twice.
    always @(posedge clk125m or negedge tx_reset_n) begin
        if(!tx_reset_n) begin arp_start<=0;user_start<=0;end
        else begin
            arp_start<=0;user_start<=0;
            if(tx_control_ready && !mac_busy && !arp_start && !user_start) begin
                if(arp_pending && (!user_pending || !last_grant_was_arp)) arp_start<=1;
                else if(user_pending && (!arp_pending || last_grant_was_arp)) user_start<=1;
            end
        end
    end
    assign tx_busy=mac_busy_visible||arp_pending_visible||ingress_start||user_pending||user_active||
        arp_start||user_start||!tx_control_ready;
    assign mac_busy=local_mac_busy||factory_busy;
    assign mac_tx_done=USE_CH59?factory_done:local_mac_done;
    assign tx_data_req=USE_CH59?factory_data_req:local_data_req;
    assign tx_payload_addr=USE_CH59?factory_payload_addr:local_payload_addr;
    always @(posedge clk125m or negedge reset_n) begin
        if(!reset_n) selected_arp<=0;
        else if(arp_start) selected_arp<=1;
        else if(user_start) selected_arp<=0;
    end

    // A producer can observe idle one edge before an ARP request crosses CDC.
    // Latch its request unconditionally, then serialize it after the ARP reply.
    // On link loss, done is an abort notification so its held line buffer releases.
    always @(posedge clk125m or negedge reset_n) begin
        if(!reset_n) begin
            user_pending<=0; user_active<=0; user_length<=0;user_port<=DST_PORT; tx_done<=0;
            last_grant_was_arp<=0;
        end else begin
            tx_done<=0;
            if(arp_start) last_grant_was_arp<=1;
            else if(user_start) last_grant_was_arp<=0;
            if(ingress_start&&!user_pending&&!user_active) begin
                if(ingress_length<=1472&&(!USE_CH59||ingress_length!=0)) begin
                    user_pending<=1; user_length<=ingress_length;user_port<=ingress_port;
                end
                else tx_done<=1; // reject an invalid MTU-sized request without deadlock
            end
            if(user_start) begin user_pending<=0; user_active<=1; end
            if(mac_tx_done) begin user_active<=0; tx_done<=1; end
            if((user_pending||user_active)&&!tx_reset_n) begin
                user_pending<=0; user_active<=0; tx_done<=1;
            end
        end
    end

    always @(posedge rx_clk or negedge reset_n) begin
        if(!reset_n) begin
            peer_request_toggle<=0; arp_request_toggle<=0;
            peer_mailbox<=48'hffffffffffff; arp_mailbox<=0;
            peer_ack_rx<=0; arp_ack_rx<=0;
        end else begin
            peer_ack_rx<={peer_ack_rx[0],peer_ack_toggle};
            arp_ack_rx<={arp_ack_rx[0],arp_ack_toggle};
            if(rx_peer_update&&peer_request_toggle==peer_ack_rx[1]) begin
                peer_mailbox<=rx_peer_mac; peer_request_toggle<=~peer_request_toggle;
            end
            if(rx_arp_request&&arp_request_toggle==arp_ack_rx[1]) begin
                arp_mailbox<=rx_peer_mac; arp_request_toggle<=~arp_request_toggle;
            end
        end
    end
    always @(posedge clk125m or negedge reset_n) begin
        if(!reset_n) begin
            peer_ack_toggle<=0; arp_ack_toggle<=0; peer_req_tx<=0; arp_req_tx<=0;
            tx_peer_mac<=48'hffffffffffff;
        end else begin
            peer_req_tx<={peer_req_tx[0],peer_request_toggle};
            arp_req_tx<={arp_req_tx[0],arp_request_toggle};
            if(peer_req_tx[1]!=peer_ack_toggle) begin
                tx_peer_mac<=peer_mailbox; peer_ack_toggle<=peer_req_tx[1];
            end
            if(arp_start) arp_ack_toggle<=arp_req_tx[1];
        end
    end
    eth_udp_tx #(.LOCAL_MAC(LOCAL_MAC),.LOCAL_IP(LOCAL_IP),.PEER_IP(PEER_IP),
        .SRC_PORT(SRC_PORT),.DST_PORT(DST_PORT),.DYNAMIC_DST_PORT(DYNAMIC_DST_PORT)) tx_mac(
        .clk(clk125m),.reset_n(tx_reset_n),
        // In CH59 mode this MAC sends ARP replies only. Isolate its unused
        // UDP input explicitly; unreachable UDP states must not route the
        // live video RAM output through its header/CRC byte-selection cone.
        .tx_start(USE_CH59?1'b0:user_start),.tx_length(USE_CH59?16'd0:user_length),
        .tx_data(USE_CH59?8'd0:tx_data),
        .peer_mac(tx_peer_mac),.destination_port(user_port),.arp_start(arp_start),.arp_mac(arp_mailbox),
        .tx_busy(local_mac_busy),.tx_done(local_mac_done),.tx_data_req(local_data_req),
        .tx_payload_addr(local_payload_addr),.arp_done(arp_done),
        .gmii_txd(local_txd),.gmii_txen(local_txen));
endmodule
