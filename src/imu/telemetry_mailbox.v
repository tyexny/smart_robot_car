// Coherent multibit crossing: source holds payload until acknowledged.
// Periodic source requests ensure disconnect/freshness changes also cross.
module telemetry_mailbox #(parameter WIDTH=1,parameter RESET_SYNC_RELEASE=0)(
    input wire src_clk, dst_clk, reset_n, src_valid,
    input wire [WIDTH-1:0] src_data,
    output reg dst_valid,
    output reg [WIDTH-1:0] dst_data
);
    reg [WIDTH-1:0] held;
    reg request_toggle, ack_toggle;
    reg ack_meta, ack_sync, req_meta, req_sync;
    wire src_domain_reset_n,dst_domain_reset_n;
    generate if(RESET_SYNC_RELEASE) begin:g_reset_release
        (* ASYNC_REG="TRUE" *) reg src_reset_meta,src_reset_sync;
        (* ASYNC_REG="TRUE" *) reg dst_reset_meta,dst_reset_sync;
        always @(posedge src_clk or negedge reset_n)
            if(!reset_n) begin src_reset_meta<=1;src_reset_sync<=1;end
            else begin src_reset_meta<=0;src_reset_sync<=src_reset_meta;end
        always @(posedge dst_clk or negedge reset_n)
            if(!reset_n) begin dst_reset_meta<=1;dst_reset_sync<=1;end
            else begin dst_reset_meta<=0;dst_reset_sync<=dst_reset_meta;end
        assign src_domain_reset_n=~src_reset_sync;
        assign dst_domain_reset_n=~dst_reset_sync;
    end else begin:g_common_reset
        assign src_domain_reset_n=reset_n;
        assign dst_domain_reset_n=reset_n;
    end endgenerate
    always @(posedge src_clk or negedge src_domain_reset_n) begin
        if(!src_domain_reset_n) begin held<=0; request_toggle<=0; ack_meta<=0; ack_sync<=0; end
        else begin
            ack_meta<=ack_toggle; ack_sync<=ack_meta;
            if(src_valid && request_toggle==ack_sync) begin
                held<=src_data; request_toggle<=~request_toggle;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_domain_reset_n) begin
        if(!dst_domain_reset_n) begin req_meta<=0; req_sync<=0; ack_toggle<=0; dst_valid<=0; dst_data<=0; end
        else begin
            req_meta<=request_toggle; req_sync<=req_meta; dst_valid<=0;
            if(req_sync!=ack_toggle) begin
                dst_data<=held; dst_valid<=1; ack_toggle<=req_sync;
            end
        end
    end
endmodule
