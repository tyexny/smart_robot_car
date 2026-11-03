// Two clients share the map's one synchronous system-clock read port.
// The accepted request's owner is retained through the response, including an
// abort/mode change. A late matcher response can never become planner data.
module scan_map_read_mux #(parameter integer GRID_BITS=7)(
    input wire clk,reset_n,
    input wire planner_req,input wire [2*GRID_BITS-1:0] planner_addr,
    output wire planner_ready,output wire planner_rsp_valid,
    input wire matcher_req,input wire [2*GRID_BITS-1:0] matcher_addr,
    output wire matcher_ready,output wire matcher_rsp_valid,
    output wire map_req,output wire [2*GRID_BITS-1:0] map_addr,
    input wire map_ready,map_rsp_valid
);
    reg pending,owner_matcher,grant_valid,grant_matcher;
    reg [2*GRID_BITS-1:0] grant_addr;
    wire choose_matcher=!planner_req && matcher_req;
    wire granted_req=grant_matcher ? matcher_req : planner_req;
    assign map_req=!pending && grant_valid && granted_req;
    assign map_addr=grant_addr;
    assign planner_ready=!pending && grant_valid && !grant_matcher && map_ready;
    assign matcher_ready=!pending && grant_valid && grant_matcher && map_ready;
    assign planner_rsp_valid=pending && !owner_matcher && map_rsp_valid;
    assign matcher_rsp_valid=pending && owner_matcher && map_rsp_valid;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin pending<=0;owner_matcher<=0;grant_valid<=0;grant_matcher<=0;grant_addr<=0;end
        else if(pending) begin if(map_rsp_valid) pending<=0;end
        else if(grant_valid) begin
            if(!granted_req) grant_valid<=0;
            else if(map_ready) begin pending<=1;owner_matcher<=grant_matcher;grant_valid<=0;end
        end else if(planner_req || matcher_req) begin
            grant_valid<=1;grant_matcher<=choose_matcher;grant_addr<=choose_matcher ? matcher_addr : planner_addr;
        end
    end
endmodule
