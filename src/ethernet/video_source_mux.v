// Packet-level source selection. The owner is held through MAC completion,
// including when the page or SW4 changes in the middle of a packet.
// Disabled producers drain their pending row banks locally.
module video_source_mux #(parameter integer REGISTERED_MAC=0)(
 input wire clk,reset_n,enabled,screen_select,
 input wire mac_busy,mac_done,mac_req,
 input wire camera_start,screen_start,
 input wire [15:0] camera_length,screen_length,
 input wire [7:0] camera_data,screen_data,
 output wire camera_busy,camera_done,camera_req,
 output wire screen_busy,screen_done,screen_req,
 output wire tx_start,
 output wire [15:0] tx_length,
 output wire [7:0] tx_data
);
 reg screen_owner;
 wire allow_camera=enabled && !screen_select;
 wire allow_screen=enabled && screen_select;
 wire launch_camera=camera_start && allow_camera && !mac_busy;
 wire launch_screen=screen_start && allow_screen && !mac_busy;
 assign tx_start=launch_camera || launch_screen;
 assign tx_length=launch_screen ? screen_length : camera_length;
 assign tx_data=screen_owner ? screen_data : camera_data;
 assign camera_busy=allow_camera ? mac_busy : 1'b0;
 assign screen_busy=allow_screen ? mac_busy : 1'b0;
 assign camera_done=(mac_done && !screen_owner) ||
                    (camera_start && (!allow_camera || mac_busy));
 assign screen_done=(mac_done && screen_owner) ||
                    (screen_start && (!allow_screen || mac_busy));
 // A direct MAC may prime byte zero on the launch edge itself.
 // The queued Ethernet endpoint primes only after its registered ingress.
 // Its owner has already latched; do not route live ARP/busy arbitration
 // back through every producer CE during the body of an accepted packet.
 wire request_screen_owner=REGISTERED_MAC ? screen_owner : (tx_start ? launch_screen : screen_owner);
 assign camera_req=mac_req && !request_screen_owner;
 assign screen_req=mac_req && request_screen_owner;
 always @(posedge clk or negedge reset_n)
  if(!reset_n)screen_owner<=0;
  else if(tx_start)screen_owner<=launch_screen;
endmodule
