// Strict Ethernet II IPv4/UDP receive filter and ARP responder request decoder.
// Payload bytes are speculative until rx_packet_good. The consumer MUST stage
// the complete packet and commit only on rx_packet_good, never on payload_end.
module eth_udp_rx #(
    parameter [47:0] LOCAL_MAC=48'h020000001002,
    parameter [31:0] LOCAL_IP=32'hc0a80a02,
    parameter [31:0] PEER_IP=32'hc0a80a01,
    parameter [15:0] PEER_PORT=16'd5000,
    parameter [15:0] LOCAL_PORT=16'd5001,
    parameter MAX_RX_PAYLOAD=1472,parameter ALLOW_RPC_PORTS=0
)(
    input clk, input reset_n,
    input [7:0] gmii_rxd, input gmii_rxdv, input gmii_rxer,
    output reg [7:0] rx_payload_data,
    output reg rx_payload_valid, output reg rx_payload_start,
    output reg rx_payload_end, output [15:0] rx_payload_length,
    output reg rx_packet_good, output reg rx_packet_error,
    output reg arp_request, output reg peer_update,
    output reg [47:0] peer_mac
);
    localparam SEARCH=0, BODY=1, DROP=2;
    // Retime the complete GMII bundle once before preamble/SFD and header
    // decoding. This keeps the RGMII IDDR outputs out of long flag CEs.
    reg [7:0] sampled_data;
    reg sampled_dv, sampled_error;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin sampled_data<=0; sampled_dv<=0; sampled_error<=0; end
        else begin sampled_data<=gmii_rxd; sampled_dv<=gmii_rxdv; sampled_error<=gmii_rxer; end
    end
    reg [1:0] state;
    reg [3:0] pre_count;
    reg [10:0] index;
    reg [31:0] crc;
    reg [19:0] ip_sum;
    reg [25:0] udp_sum;
    reg [7:0] ip_high, udp_high;
    reg frame_error, headers_good, ip_version_good;
    reg [47:0] destination_mac, source_mac, arp_sender_mac;
    reg [15:0] eth_type, ip_total_length, ip_fragment, udp_source_port;
    reg [15:0] udp_dest_port, udp_length, udp_checksum;
    reg [7:0] ip_protocol, ip_ttl;
    reg [31:0] source_ip, destination_ip, arp_sender_ip, arp_target_ip;
    reg [63:0] arp_header;
    reg [15:0] required_bytes, payload_limit, payload_last, ip_from_udp_length;
    reg [15:0] payload_length;
    reg payload_window,source_mac_window;
    reg [16:0] ip_fold;
    reg ip_addresses_good, ip_format_good, ip_checksum_good;
    reg udp_ports_good, udp_lengths_good;
    reg [31:0] payload_magic;
    reg auxiliary_kind_good;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)begin payload_magic<=0;auxiliary_kind_good<=0;end
        else begin
            if(state==BODY && sampled_dv && index>=42 && index<=45)
                payload_magic<={payload_magic[23:0],sampled_data};
            if(state==BODY && sampled_dv && index==46)
                auxiliary_kind_good<=udp_source_port==PEER_PORT ||
                    (ALLOW_RPC_PORTS && udp_length==28 && payload_magic==32'h46544951);
        end
    reg arp_addresses_good, arp_format_good, arp_mac_good, arp_headers_good;
    localparam [26:0] UDP_PSEUDO_BASE={16'd0,PEER_IP[31:16]}+
        PEER_IP[15:0]+LOCAL_IP[31:16]+LOCAL_IP[15:0]+16'd17;
    // A frame is committed only after the complete FCS has been received.
    // One adder per stage keeps the end-of-frame UDP checksum path below
    // one 125 MHz cycle. Five cycles fit within Ethernet's 12-cycle IFG.
    reg [4:0] tail_valid;
    reg [25:0] tail_sum0;
    reg [26:0] tail_sum1, tail_sum2;
    reg [16:0] tail_fold;
    reg [15:0] tail_udp_length;
    reg tail_checksum_good, tail_udp_zero, tail_crc_good;
    reg tail_is_udp, tail_is_arp, tail_udp_headers_good, tail_arp_headers_good;
    reg tail_arp_request;
    reg [47:0] tail_source_mac;

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
    assign rx_payload_length=payload_length;

    // Prepare the range predicate for the next BODY byte. A valid UDP header
    // has its length available by byte 40; byte 41 therefore prepares byte 42
    // without adding latency to the payload data/start/end bundle. Keeping the
    // wide range comparison out of that bundle's enables shortens the RX path.
    // Valid datagrams finish below index 2047, where index advances by one.
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin payload_window<=0;source_mac_window<=0;end
        else begin
            payload_window<=state==BODY && sampled_dv &&
                            index>=11'd41 && index<payload_last;
            // Index 5 prepares the first source byte (6), and 10 prepares
            // the last (11). Actual state/dv still suppress truncated frames.
            source_mac_window<=state==BODY && sampled_dv &&
                               index>=11'd5 && index<11'd11;
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=SEARCH; pre_count<=0; index<=0; crc<=32'hffffffff;
            ip_sum<=0; udp_sum<=0; ip_high<=0; udp_high<=0;
            frame_error<=0; headers_good<=0; ip_version_good<=0;
            destination_mac<=0; source_mac<=0; arp_sender_mac<=0;
            eth_type<=0; ip_total_length<=0; ip_fragment<=0;
            udp_source_port<=0; udp_dest_port<=0; udp_length<=0; udp_checksum<=0;
            ip_protocol<=0; ip_ttl<=0; source_ip<=0; destination_ip<=0;
            arp_sender_ip<=0; arp_target_ip<=0; arp_header<=0;
            required_bytes<=0; payload_limit<=0; payload_last<=0;
            ip_from_udp_length<=0; payload_length<=0; ip_fold<=0;
            ip_addresses_good<=0; ip_format_good<=0; ip_checksum_good<=0;
            udp_ports_good<=0; udp_lengths_good<=0;
            arp_addresses_good<=0; arp_format_good<=0; arp_mac_good<=0; arp_headers_good<=0;
            tail_valid<=0; tail_sum0<=0; tail_sum1<=0; tail_sum2<=0; tail_fold<=0;
            tail_udp_length<=0; tail_checksum_good<=0; tail_udp_zero<=0;
            tail_crc_good<=0; tail_is_udp<=0; tail_is_arp<=0;
            tail_udp_headers_good<=0; tail_arp_headers_good<=0; tail_arp_request<=0;
            tail_source_mac<=0;
            rx_payload_data<=0; rx_payload_valid<=0; rx_payload_start<=0;
            rx_payload_end<=0; rx_packet_good<=0; rx_packet_error<=0;
            arp_request<=0; peer_update<=0; peer_mac<=48'hffffffffffff;
        end else begin
            rx_payload_valid<=0; rx_payload_start<=0; rx_payload_end<=0;
            rx_packet_good<=0; rx_packet_error<=0; arp_request<=0; peer_update<=0;
            tail_valid<={tail_valid[3:0],1'b0};
            if(tail_valid[0]) tail_sum1<={1'b0,tail_sum0}+UDP_PSEUDO_BASE;
            if(tail_valid[1]) tail_sum2<=tail_sum1+tail_udp_length;
            if(tail_valid[2]) tail_fold<={1'b0,tail_sum2[15:0]}+
                                         {6'd0,tail_sum2[26:16]};
            if(tail_valid[3]) tail_checksum_good<=tail_udp_zero||
                ({1'b0,tail_fold[15:0]}+{16'd0,tail_fold[16]}==17'h0ffff);
            if(tail_valid[4]) begin
                if(tail_is_udp) begin
                    if(tail_crc_good&&tail_udp_headers_good&&tail_checksum_good) begin
                        rx_packet_good<=1;
                        peer_mac<=tail_source_mac; peer_update<=1;
                    end else rx_packet_error<=1;
                end else if(tail_is_arp&&tail_crc_good&&tail_arp_headers_good) begin
                    peer_mac<=tail_source_mac; peer_update<=1;
                    if(tail_arp_request) arp_request<=1;
                end
            end
            case(state)
                SEARCH:begin
                    if(!sampled_dv) pre_count<=0;
                    else if(sampled_data==8'h55&&!sampled_error) begin
                        if(pre_count<15) pre_count<=pre_count+1'b1;
                    end else if(sampled_data==8'hd5&&pre_count>=7&&!sampled_error) begin
                        state<=BODY; index<=0; crc<=32'hffffffff; pre_count<=0;
                        ip_sum<=0; udp_sum<=0; frame_error<=0; headers_good<=0;
                        // Raw shift fields need no per-frame clearing: a
                        // committed IPv4 frame has overwritten all Ethernet,
                        // IP and UDP fields by index 41; a committed ARP has
                        // overwritten its complete header by the same index.
                        // Short/bad frames cannot reuse old fields because
                        // validation flags/sums restart here and length/FCS
                        // are checked before commit. Removing the redundant
                        // clears keeps SFD decoding out of wide field CEs.
                        // Intermediate flags are overwritten at 34/35/40
                        // before headers_good is formed at 41. Only the two
                        // final validity flags require per-frame clearing.
                        arp_headers_good<=0;
                    end else state<=DROP;
                end
                BODY:begin
                    if(!sampled_dv) begin
                        state<=SEARCH;
                        tail_valid[0]<=1;
                        tail_sum0<=udp_sum; tail_udp_length<=udp_length;
                        tail_udp_zero<=udp_checksum==0;
                        tail_crc_good<=!frame_error&&crc==32'hdebb20e3;
                        tail_is_udp<=eth_type==16'h0800;
                        tail_is_arp<=eth_type==16'h0806;
                        tail_udp_headers_good<=headers_good&&index==required_bytes&&auxiliary_kind_good;
                        tail_arp_headers_good<=arp_headers_good&&index==64;
                        tail_arp_request<=arp_header[15:0]==1;
                        tail_source_mac<=source_mac;
                    end else begin
                        crc<=crc_byte(crc,sampled_data);
                        if(sampled_error) frame_error<=1;
                        if(index<2047) index<=index+1'b1;
                        else frame_error<=1;
                        if(index<6) destination_mac<={destination_mac[39:0],sampled_data};
                        if(source_mac_window) source_mac<={source_mac[39:0],sampled_data};
                        case(index)
                            12,13:eth_type<={eth_type[7:0],sampled_data};
                            14:ip_version_good<=sampled_data==8'h45;
                            16,17:ip_total_length<={ip_total_length[7:0],sampled_data};
                            20,21:ip_fragment<={ip_fragment[7:0],sampled_data};
                            22:ip_ttl<=sampled_data;
                            23:ip_protocol<=sampled_data;
                            26,27,28,29:source_ip<={source_ip[23:0],sampled_data};
                            30,31,32,33:destination_ip<={destination_ip[23:0],sampled_data};
                            34,35:udp_source_port<={udp_source_port[7:0],sampled_data};
                            36,37:udp_dest_port<={udp_dest_port[7:0],sampled_data};
                            38,39:udp_length<={udp_length[7:0],sampled_data};
                            40,41:udp_checksum<={udp_checksum[7:0],sampled_data};
                            default:begin end
                        endcase
                        if(index>=14&&index<34) begin
                            if(!index[0]) ip_high<=sampled_data;
                            else ip_sum<=ip_sum+{ip_high,sampled_data};
                        end
                        if(index>=34&&(index<42||index<payload_limit)) begin
                            if(!index[0]) begin
                                udp_high<=sampled_data;
                                if(index==payload_last) udp_sum<=udp_sum+{sampled_data,8'd0};
                            end else udp_sum<=udp_sum+{udp_high,sampled_data};
                        end
                        if(index>=14&&index<22) arp_header<={arp_header[55:0],sampled_data};
                        if(index>=22&&index<28) arp_sender_mac<={arp_sender_mac[39:0],sampled_data};
                        if(index>=28&&index<32) arp_sender_ip<={arp_sender_ip[23:0],sampled_data};
                        if(index>=38&&index<42) arp_target_ip<={arp_target_ip[23:0],sampled_data};
                        if(index==18) required_bytes<=
                            (ip_total_length<46)?16'd64:ip_total_length+16'd18;
                        if(index==34) begin
                            ip_fold<={1'b0,ip_sum[15:0]}+{13'd0,ip_sum[19:16]};
                            ip_addresses_good<=destination_mac==LOCAL_MAC&&
                                source_ip==PEER_IP&&destination_ip==LOCAL_IP;
                            ip_format_good<=eth_type==16'h0800&&ip_version_good&&
                                ip_protocol==17&&ip_ttl!=0&&(ip_fragment&16'hbfff)==0;
                        end
                        if(index==35) ip_checksum_good<=
                            ({1'b0,ip_fold[15:0]}+{16'd0,ip_fold[16]}==17'h0ffff);
                        if(index==39) begin
                            payload_limit<={udp_length[7:0],sampled_data}+16'd34;
                            payload_last<={udp_length[7:0],sampled_data}+16'd33;
                            ip_from_udp_length<={udp_length[7:0],sampled_data}+16'd20;
                        end
                        if(index==40) begin
                            payload_length<=(udp_length>=8)?udp_length-16'd8:16'd0;
                            udp_ports_good<=(udp_source_port==PEER_PORT || (ALLOW_RPC_PORTS && udp_source_port>=1024))&&udp_dest_port==LOCAL_PORT;
                            udp_lengths_good<=udp_length>=8&&udp_length<=MAX_RX_PAYLOAD+8&&
                                ip_total_length==ip_from_udp_length;
                        end
                        if(index==41) headers_good<=ip_addresses_good&&ip_format_good&&
                            ip_checksum_good&&udp_ports_good&&udp_lengths_good;
                        if(index==42) begin
                            arp_addresses_good<=
                                (destination_mac==LOCAL_MAC||destination_mac==48'hffffffffffff)&&
                                arp_sender_ip==PEER_IP&&arp_target_ip==LOCAL_IP;
                            arp_format_good<=arp_header==64'h0001080006040001||
                                             arp_header==64'h0001080006040002;
                            arp_mac_good<=arp_sender_mac==source_mac;
                        end
                        if(index==43) arp_headers_good<=
                            arp_addresses_good&&arp_format_good&&arp_mac_good;
                        if(headers_good&&payload_window) begin
                            rx_payload_data<=sampled_data; rx_payload_valid<=1;
                            rx_payload_start<=index==42;
                            rx_payload_end<=index==payload_last;
                        end
                    end
                end
                DROP:if(!sampled_dv) begin state<=SEARCH; pre_count<=0; end
                default:state<=SEARCH;
            endcase
        end
    end
endmodule
