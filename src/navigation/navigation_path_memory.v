// One complete planner result is published only after done/success.
module navigation_path_memory #(parameter integer GRID_BITS=7, GRID_X=(1<<GRID_BITS), GRID_Y=(1<<GRID_BITS), RD_ADDRESS_COMPACT=0, RETAIN_ENABLE=0)(
    input wire clk,reset_n,clear,write_valid,
    input wire retain_path,
    input wire [2*GRID_BITS-1:0] write_addr,
    output wire clear_busy,
    output reg write_bank,
    input wire rd_clk,
    input wire [2*GRID_BITS-1:0] rd_addr,
    input wire rd_bank,
    output reg rd_data
);
    localparam integer STORAGE_BITS=$clog2(GRID_X*GRID_Y);
    function [STORAGE_BITS-1:0] index;
        input [2*GRID_BITS-1:0] addr;
        begin index=addr[2*GRID_BITS-1:GRID_BITS]*GRID_X+addr[GRID_BITS-1:0];end
    endfunction
    wire [STORAGE_BITS-1:0] write_index=index(write_addr),read_index=RD_ADDRESS_COMPACT ? rd_addr[STORAGE_BITS-1:0] : index(rd_addr);
    (* ram_style="block", syn_ramstyle="block_ram" *) reg mask [0:(1<<(STORAGE_BITS+1))-1];
    reg clearing;
    reg [STORAGE_BITS-1:0] clear_addr;
    wire hold_route=RETAIN_ENABLE && retain_path;
    wire clear_request=clear && !hold_route;
    assign clear_busy=clearing;
    always @(posedge clk) begin
        if(!clear_request) begin
            if(clearing) mask[{write_bank,clear_addr}]<=0;
            else if(write_valid&&!hold_route) mask[{write_bank,write_index}]<=1;
        end
    end
    always @(posedge rd_clk) rd_data<=mask[{rd_bank,read_index}];
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin clearing<=1;clear_addr<=0;write_bank<=0;end
        else if(clear_request) begin clearing<=1;clear_addr<=0;write_bank<=!write_bank;end
        else if(clearing) begin
            if(clear_addr==((1<<STORAGE_BITS)-1)) clearing<=0;
            else clear_addr<=clear_addr+1'b1;
        end
    end
endmodule
