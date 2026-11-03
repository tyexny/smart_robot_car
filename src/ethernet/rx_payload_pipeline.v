// Keep the complete CRC-qualified RX bundle together. The parser consumes
// exactly the original sequence, one clock later, including payload bubbles
// and a packet-start coincident with the first byte.
module rx_payload_pipeline(
 input wire clk,reset_n,
 input wire packet_start,payload_valid,payload_end,packet_good,packet_error,
 input wire [7:0] payload_data,
 input wire [15:0] payload_length,
 output reg out_start,out_valid,out_end,out_good,out_error,
 output reg [7:0] out_data,
 output reg [15:0] out_length
);
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin
   out_start<=0;out_valid<=0;out_end<=0;out_good<=0;out_error<=0;
   out_data<=0;out_length<=0;
  end else begin
   out_start<=packet_start;out_valid<=payload_valid;out_end<=payload_end;
   out_good<=packet_good;out_error<=packet_error;
   out_data<=payload_data;out_length<=payload_length;
  end
endmodule
