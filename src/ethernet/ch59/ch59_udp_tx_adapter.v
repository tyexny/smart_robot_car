// Adapt the ch59 FWFT sender (passive next-state tap) to synchronous RAM.
// Prime byte zero at start. Each factory payload request consumes the current
// byte and requests the NEXT byte, except the last request which must not read
// beyond tx_length. The factory's early tx_done is deliberately not propagated:
// hold busy through the actual GMII envelope and twelve complete idle cycles.
module ch59_udp_tx_adapter #(
    parameter [47:0] LOCAL_MAC=48'h020000001002,
    parameter [31:0] LOCAL_IP=32'hc0a80a02,
    parameter [31:0] PEER_IP=32'hc0a80a01,
    parameter [15:0] SRC_PORT=5000, parameter [15:0] DST_PORT=5000,parameter DYNAMIC_DST_PORT=0
)(
    input clk, input reset_n,
    input tx_start, input [15:0] tx_length, input [7:0] tx_data,
    input [47:0] peer_mac,input [15:0] destination_port,
    output tx_busy, output reg tx_done,
    output tx_data_req, output [10:0] tx_payload_addr,
    output [7:0] gmii_txd, output gmii_txen
);
    localparam IDLE=0,SEND=1,GAP=2,PRIME=3;
    reg [1:0] state;
    reg [3:0] gap_count;
    reg [10:0] payload_index,length_reg;
    reg [10:0] read_ahead_address;
    reg previous_en;
    reg prefetch_remaining;
    reg [10:0] final_prefetch_index;
    wire factory_payload_req;
    wire factory_payload_req_next;
    wire factory_early_done;
    wire start_packet=tx_start&&state==IDLE;
    assign tx_busy=state!=IDLE;
    // Register the accepted launch before priming the producer. ARP/user
    // arbitration must not reach every producer RAM/header CE in one cycle.
    // The factory sees its start on the same edge that byte zero is primed.
    wire factory_start=state==PRIME;
    // Predict this exact same-cycle request at the preceding clock edge.
    // Keep the factory state -> owner decode -> distributed RAM OCE chain
    // off the producer critical path, without delaying any payload request.
    wire send_next=state==PRIME || (state==SEND && !(previous_en&&!gmii_txen));
    // The MAC requests each legal payload byte once, in increasing order.
    // Once the penultimate byte is consumed, no more producer prefetches are
    // allowed. Equality against a header-time constant avoids add/compare
    // arithmetic in this request lookahead. Length zero/one starts disabled.
    wire remaining_next=prefetch_remaining &&
        !(state==SEND&&factory_payload_req&&payload_index==final_prefetch_index);
    (* syn_preserve=1 *) reg producer_request_q;
    always @(posedge clk or negedge reset_n)
        if(!reset_n)producer_request_q<=0;
        else producer_request_q<=start_packet || (send_next&&factory_payload_req_next&&remaining_next);
    assign tx_data_req=producer_request_q;
    // Prime zero from IDLE, then keep the next requested byte in a local
    // register. This removes state mux -> increment -> RAM address arithmetic
    // from the synchronous row RAM read path. Gaps do not advance the address.
    assign tx_payload_addr=read_ahead_address;
    eth_udp_tx_gmii factory_udp(
        .clk125m(clk),.reset_p(!reset_n),.tx_en_pulse(factory_start),
        .tx_done(factory_early_done),
        .dst_mac(peer_mac),.src_mac(LOCAL_MAC),
        .dst_ip(PEER_IP),.src_ip(LOCAL_IP),.dst_port(DYNAMIC_DST_PORT?destination_port:DST_PORT),.src_port(SRC_PORT),
        .data_length(tx_length),.payload_req_o(factory_payload_req),.payload_req_next_o(factory_payload_req_next),
        .payload_dat_i(tx_data),.gmii_tx_clk(),
        .gmii_txen(gmii_txen),.gmii_txd(gmii_txd));
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;gap_count<=0;payload_index<=0;length_reg<=0;read_ahead_address<=0;
            previous_en<=0;tx_done<=0;
            prefetch_remaining<=0;final_prefetch_index<=0;
        end else begin
            tx_done<=0;previous_en<=gmii_txen;
            case(state)
                IDLE:if(start_packet) begin
                    state<=PRIME;payload_index<=0;length_reg<=tx_length[10:0];read_ahead_address<=0;
                    prefetch_remaining<=tx_length[10:0]>11'd1;
                end
                PRIME:begin state<=SEND;read_ahead_address<=1;final_prefetch_index<=length_reg-11'd2;end
                SEND:begin
                    if(factory_payload_req) begin
                        payload_index<=payload_index+1'b1;
                        read_ahead_address<=read_ahead_address+1'b1;
                        // Prepare the next cycle's final-byte qualifier in
                        // advance; the wide comparator no longer drives the
                        // producer header-selector clock enables directly.
                        prefetch_remaining<=remaining_next;
                    end
                    if(previous_en&&!gmii_txen) begin state<=GAP;gap_count<=0;end
                end
                GAP:if(gap_count==11) begin state<=IDLE;tx_done<=1;read_ahead_address<=0;end
                    else gap_count<=gap_count+1'b1;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
