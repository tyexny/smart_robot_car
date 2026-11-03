// Ethernet II / IPv4 / UDP transmitter, one octet per 125 MHz clock.
// tx_data_req and tx_payload_addr request data ONE clock before consumption.
// A synchronous byte RAM can therefore drive tx_data without a combinational read.
module eth_udp_tx #(
    parameter [47:0] LOCAL_MAC = 48'h020000001002,
    parameter [31:0] LOCAL_IP = 32'hc0a80a02,
    parameter [31:0] PEER_IP = 32'hc0a80a01,
    parameter [15:0] SRC_PORT = 16'd5000,
    parameter [15:0] DST_PORT = 16'd5000,parameter DYNAMIC_DST_PORT=0
)(
    input clk, input reset_n,
    input tx_start, input [15:0] tx_length, input [7:0] tx_data,
    input [47:0] peer_mac,input [15:0] destination_port,
    input arp_start, input [47:0] arp_mac,
    output tx_busy, output reg tx_done,
    output tx_data_req, output [10:0] tx_payload_addr,
    output reg arp_done,
    output reg [7:0] gmii_txd, output reg gmii_txen
);
    localparam IDLE=0, PREAMBLE=1, BODY=2, FCS=3, IFG=4;
    reg [2:0] state;
    reg [10:0] index, body_length, payload_end, request_end;
    reg [3:0] small_count;
    reg is_arp, payload_selected;
    // Keep the upper five packet-index bits off the header byte mux. This
    // flag clears after header byte41 and stays clear through all payload
    // and padding bytes; the six-bit header index therefore never wraps.
    reg header_region /* synthesis syn_preserve = 1 */;
    reg [47:0] destination_mac;
    reg [15:0] packet_id, active_id, ip_checksum;
    reg [31:0] crc, fcs;
    reg [7:0] body_byte;
    reg [7:0] frame_byte;
    reg [15:0] ip_length, udp_length;
    // Sum constants at elaboration, then use one carry chain per preamble
    // cycle. The original complete checksum expression exceeded 8 ns in STA.
    localparam [19:0] IP_CHECKSUM_BASE=20'h04500+20'h04000+20'h04011+
        LOCAL_IP[31:16]+LOCAL_IP[15:0]+PEER_IP[31:16]+PEER_IP[15:0];
    reg [19:0] checksum_sum;
    reg [16:0] checksum_fold;

    function [31:0] crc_byte;
        input [31:0] c; input [7:0] d;
        reg [31:0] t; integer b;
        begin
            t=c;
            for(b=0;b<8;b=b+1)
                if(t[0]^d[b]) t=(t>>1)^32'hedb88320;
                else t=t>>1;
            crc_byte=t;
        end
    endfunction
    assign tx_busy=state!=IDLE;
    assign tx_data_req=(state==BODY)&&!is_arp&&(index>=11'd41)&&
                       (index<request_end);
    assign tx_payload_addr=index-11'd41;

    always @* begin
        body_byte=8'd0;
        // Fixed byte slices avoid a subtract/multiply/barrel-shift chain
        // driven by the frame index. One parallel case also avoids nested
        // Ethernet/ARP/IP header mux priority along the 125 MHz byte path.
        if(header_region) begin
        case(index[5:0])
             0:body_byte=destination_mac[47:40];
             1:body_byte=destination_mac[39:32];
             2:body_byte=destination_mac[31:24];
             3:body_byte=destination_mac[23:16];
             4:body_byte=destination_mac[15:8];
             5:body_byte=destination_mac[7:0];
             6:body_byte=LOCAL_MAC[47:40];
             7:body_byte=LOCAL_MAC[39:32];
             8:body_byte=LOCAL_MAC[31:24];
             9:body_byte=LOCAL_MAC[23:16];
            10:body_byte=LOCAL_MAC[15:8];
            11:body_byte=LOCAL_MAC[7:0];
            12:body_byte=8'h08;
            13:body_byte=is_arp?8'h06:8'h00;
            14:body_byte=is_arp?8'h00:8'h45;
            15:body_byte=is_arp?8'h01:8'h00;
            16:body_byte=is_arp?8'h08:ip_length[15:8];
            17:body_byte=is_arp?8'h00:ip_length[7:0];
            18:body_byte=is_arp?8'h06:active_id[15:8];
            19:body_byte=is_arp?8'h04:active_id[7:0];
            20:body_byte=is_arp?8'h00:8'h40; // UDP: don't fragment
            21:body_byte=is_arp?8'h02:8'h00; // ARP: reply
            22:body_byte=is_arp?LOCAL_MAC[47:40]:8'h40;
            23:body_byte=is_arp?LOCAL_MAC[39:32]:8'h11;
            24:body_byte=is_arp?LOCAL_MAC[31:24]:ip_checksum[15:8];
            25:body_byte=is_arp?LOCAL_MAC[23:16]:ip_checksum[7:0];
            26:body_byte=is_arp?LOCAL_MAC[15:8]:LOCAL_IP[31:24];
            27:body_byte=is_arp?LOCAL_MAC[7:0]:LOCAL_IP[23:16];
            28:body_byte=is_arp?LOCAL_IP[31:24]:LOCAL_IP[15:8];
            29:body_byte=is_arp?LOCAL_IP[23:16]:LOCAL_IP[7:0];
            30:body_byte=is_arp?LOCAL_IP[15:8]:PEER_IP[31:24];
            31:body_byte=is_arp?LOCAL_IP[7:0]:PEER_IP[23:16];
            32:body_byte=is_arp?destination_mac[47:40]:PEER_IP[15:8];
            33:body_byte=is_arp?destination_mac[39:32]:PEER_IP[7:0];
            34:body_byte=is_arp?destination_mac[31:24]:SRC_PORT[15:8];
            35:body_byte=is_arp?destination_mac[23:16]:SRC_PORT[7:0];
            36:body_byte=is_arp?destination_mac[15:8]:DYNAMIC_DST_PORT?destination_port[15:8]:DST_PORT[15:8];
            37:body_byte=is_arp?destination_mac[7:0]:DYNAMIC_DST_PORT?destination_port[7:0]:DST_PORT[7:0];
            38:body_byte=is_arp?PEER_IP[31:24]:udp_length[15:8];
            39:body_byte=is_arp?PEER_IP[23:16]:udp_length[7:0];
            40:body_byte=is_arp?PEER_IP[15:8]:8'd0;
            41:body_byte=is_arp?PEER_IP[7:0]:8'd0;
            default:body_byte=8'd0;
        endcase
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE; index<=0; small_count<=0; is_arp<=0; payload_selected<=0;
            header_region<=1;
            body_length<=0; payload_end<=0; request_end<=0; destination_mac<=0;
            ip_length<=0; udp_length<=0; checksum_sum<=0; checksum_fold<=0;
            packet_id<=0; active_id<=0; ip_checksum<=0;
            crc<=32'hffffffff; fcs<=0; frame_byte<=0;
            gmii_txd<=0; gmii_txen<=0; tx_done<=0; arp_done<=0;
        end else begin
            tx_done<=0; arp_done<=0;
            case(state)
                IDLE:begin
                    gmii_txen<=0; gmii_txd<=0;
                    if(arp_start || (tx_start&&tx_length<=1472)) begin
                        state<=PREAMBLE; small_count<=0; index<=0;
                        header_region<=1;
                        is_arp<=arp_start;
                        destination_mac<=arp_start?arp_mac:peer_mac;
                        ip_length<={5'd0,tx_length[10:0]}+16'd28;
                        udp_length<={5'd0,tx_length[10:0]}+16'd8;
                        payload_end<=tx_length[10:0]+11'd42;
                        request_end<=tx_length[10:0]+11'd41;
                        body_length<=arp_start?11'd60:
                            ((tx_length<18)?11'd60:tx_length[10:0]+11'd42);
                        active_id<=packet_id;
                        if(!arp_start) packet_id<=packet_id+1'b1;
                        crc<=32'hffffffff;
                    end
                end
                PREAMBLE:begin
                    gmii_txen<=1;
                    gmii_txd<=(small_count==7)?8'hd5:8'h55;
                    case(small_count)
                        0:checksum_sum<=IP_CHECKSUM_BASE+ip_length;
                        1:checksum_sum<=checksum_sum+active_id;
                        2:checksum_fold<={1'b0,checksum_sum[15:0]}+
                                         {13'd0,checksum_sum[19:16]};
                        3:ip_checksum<=~(checksum_fold[15:0]+{15'd0,checksum_fold[16]});
                        default:begin end
                    endcase
                    if(small_count==7) begin
                        // Prime the byte pipeline while emitting SFD. There
                        // is no bubble between SFD, body bytes and FCS.
                        frame_byte<=body_byte;
                        payload_selected<=0;
                        state<=BODY; index<=1;
                    end
                    else small_count<=small_count+1'b1;
                end
                BODY:begin
                    // The old value still selects byte41 on this edge.
                    // All later bytes use zero unless selected from RAM.
                    if(index[5:0]==6'd41) header_region<=0;
                    // Keep packet RAM/header selection out of the CRC path.
                    // index selects the NEXT byte; frame_byte is the current
                    // wire byte. RAM requests still precede byte capture by
                    // one clock, preserving the synchronous-read contract.
                    gmii_txen<=1; gmii_txd<=frame_byte;
                    // Payload data bypasses the large header case. Its
                    // selection was registered with the preceding RAM read
                    // request, so only a final 2:1 mux lies after tx_data.
                    frame_byte<=payload_selected?tx_data:body_byte;
                    payload_selected<=tx_data_req;
                    crc<=crc_byte(crc,frame_byte);
                    if(index==body_length) begin
                        fcs<=~crc_byte(crc,frame_byte); state<=FCS; small_count<=0;
                    end else index<=index+1'b1;
                end
                FCS:begin
                    gmii_txen<=1; gmii_txd<=fcs[7:0]; fcs<=fcs>>8;
                    if(small_count==3) begin
                        state<=IFG; small_count<=0;
                        if(is_arp) arp_done<=1; else tx_done<=1;
                    end else small_count<=small_count+1'b1;
                end
                IFG:begin
                    gmii_txen<=0; gmii_txd<=0;
                    if(small_count==11) state<=IDLE;
                    else small_count<=small_count+1'b1;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
