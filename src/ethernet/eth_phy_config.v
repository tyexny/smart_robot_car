// ACG720 RTL8211F-CG: address 5, verified against PHY ID 0x001cc916.
// RGMII delay register facts match the upstream Linux Realtek PHY driver.
// No vendor register is written unless the expected PHY ID was acknowledged.
module eth_phy_config #(
    parameter CLK_HZ=50000000, parameter [4:0] PHY_ADDR=5,
    parameter FPGA_TX_DELAY=0,
    parameter WAIT_MS=100, parameter RETRY_MS=1000,
    parameter AUTO_PHY_ADDR=0, parameter MDC_HZ=1000000
)(
    input clk, input reset_n,
    output phy_reset_n, output mdc, inout mdio,
    output reg init_done, output reg link_up,
    output reg [31:0] phy_id,
    output reg any_read_ack, output reg [4:0] phy_address,
    output reg id_verified
);
    localparam RESET_CYCLES=CLK_HZ/100; // 10 ms reset
    // Divide before multiplying to avoid 32-bit overflow at 50 MHz / 1 s.
    localparam WAIT_CYCLES=RESET_CYCLES+(CLK_HZ/1000)*WAIT_MS;
    localparam RETRY_CYCLES_RAW=(CLK_HZ/1000)*RETRY_MS;
    localparam RETRY_CYCLES=RETRY_CYCLES_RAW>0?RETRY_CYCLES_RAW:1;
    localparam POLL_CYCLES=CLK_HZ/10;
    reg [31:0] startup_count, poll_count, retry_count;
    reg [4:0] step;
    reg id_high_ack, scan_started;
    reg [5:0] probe_index;
    wire [5:0] first_probe=(PHY_ADDR==0)?6'd1:6'd0;
    wire [5:0] probe_increment=probe_index+6'd1;
    wire [5:0] next_probe=(probe_increment=={1'b0,PHY_ADDR})?
        probe_increment+6'd1:probe_increment;
    reg issuing, start;
    reg read_op;
    reg [4:0] address;
    reg [15:0] write_data;
    reg [15:0] tx_control, rx_control;
    localparam [15:0] TX_DELAY_BIT=FPGA_TX_DELAY?16'h0000:16'h0100;
    wire busy, done, read_ack;
    wire [15:0] read_data;
    assign phy_reset_n=reset_n&&(startup_count>=RESET_CYCLES);
    eth_mdio_master #(.CLK_HZ(CLK_HZ),.MDC_HZ(MDC_HZ)) mdio_master(
        .clk(clk),.reset_n(phy_reset_n),.start(start),.read_op(read_op),
        .phy_addr(phy_address),.reg_addr(address),.write_data(write_data),
        .busy(busy),.done(done),.read_data(read_data),.read_ack(read_ack),
        .mdc(mdc),.mdio(mdio));
    always @* begin
        read_op=0; address=0; write_data=16'h1200;
        case(step)
            0:begin read_op=1; address=2; end
            1:begin read_op=1; address=3; end
            2:begin address=31; write_data=16'h0d08; end // RGMII page
            3:begin read_op=1; address=17; end
            4:begin address=17; write_data=(tx_control&16'hfeff)|TX_DELAY_BIT; end
            5:begin read_op=1; address=21; end
            6:begin address=21; write_data=rx_control|16'h0008; end
            7:begin read_op=1; address=17; end // verify TX delay
            8:begin read_op=1; address=21; end // verify RX delay
            9:begin address=31; write_data=0; end // standard page
            10:begin address=4; write_data=16'h0001; end // CSMA, no 10/100
            11:begin address=9; write_data=16'h0200; end // 1000BASE-T full
            12:begin read_op=1; address=9; end // verify gigabit advertisement
            13:begin address=0; write_data=16'h1200; end // restart autoneg
            default:begin read_op=1; address=1; end // BMSR (read twice)
        endcase
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            startup_count<=0; poll_count<=0; retry_count<=0; step<=0; issuing<=0;
            start<=0; init_done<=0; link_up<=0; phy_id<=0;
            tx_control<=0; rx_control<=0; any_read_ack<=0; phy_address<=PHY_ADDR; id_verified<=0;
            id_high_ack<=0; scan_started<=0; probe_index<=0;
        end else begin
            start<=0;
            if(step==31) begin
                // A transient MDIO/readback failure must not disable the
                // endpoint forever. Retry only after a complete hardware reset;
                // every attempt begins with the acknowledged PHY-ID guard.
                init_done<=0; link_up<=0; issuing<=0;
                if(retry_count==RETRY_CYCLES-1) begin
                    startup_count<=0; poll_count<=0; retry_count<=0;
                    step<=0; phy_id<=0; tx_control<=0; rx_control<=0;
                    any_read_ack<=0; phy_address<=PHY_ADDR; id_verified<=0;
                    id_high_ack<=0; scan_started<=0; probe_index<=0;
                end else retry_count<=retry_count+1'b1;
            end else if(startup_count<WAIT_CYCLES) startup_count<=startup_count+1'b1;
            else if(!issuing) begin
                if(step!=31&&(step<16||poll_count==POLL_CYCLES-1)) begin
                    start<=1; issuing<=1; poll_count<=0;
                end else poll_count<=poll_count+1'b1;
            end else if(done) begin
                issuing<=0;
                if(read_op&&read_ack) any_read_ack<=1;
                case(step)
                    0:begin
                        phy_id[31:16]<=read_data; id_high_ack<=read_ack;
                        step<=(read_ack||AUTO_PHY_ADDR)?1:31;
                    end
                    1:begin
                        phy_id[15:0]<=read_data;
                        if(id_high_ack&&read_ack&&
                           {phy_id[31:16],read_data}==32'h001cc916) begin
                            id_verified<=1; step<=2;
                        end
                        else if(AUTO_PHY_ADDR) begin
                            // Preferred PHY_ADDR first, then remaining 0..31 once.
                            // ID reads stay read-only until both ACKs and full ID match.
                            if(!scan_started) begin
                                scan_started<=1; probe_index<=first_probe;
                                phy_address<=first_probe[4:0]; step<=0;
                            end else if(next_probe>=6'd32) step<=31;
                            else begin
                                probe_index<=next_probe;
                                phy_address<=next_probe[4:0]; step<=0;
                            end
                        end else step<=31;
                    end
                    2,4,6,9,10,11:step<=step+1'b1;
                    3:begin tx_control<=read_data; step<=read_ack?4:31; end
                    5:begin rx_control<=read_data; step<=read_ack?6:31; end
                    7:step<=(read_ack&&(read_data&16'h0100)==TX_DELAY_BIT)?8:31;
                    8:step<=(read_ack&&read_data[3])?9:31;
                    12:step<=(read_ack&&(read_data&16'h0300)==16'h0200)?13:31;
                    13:begin init_done<=1; step<=14; end
                    14:step<=15; // clear BMSR latched-low status
                    15:begin link_up<=read_ack&&read_data[2]&&read_data[5]; step<=16; end
                    default:step<=14;
                endcase
            end
        end
    end
endmodule
