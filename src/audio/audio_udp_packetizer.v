`timescale 1ns/1ps
// ES8388 stereo 16-bit samples -> 128-sample 16 kHz mono UDP datagrams.
// The codec's I2S FIFO crosses into the Ethernet TX clock domain externally.
module audio_udp_packetizer(
    input wire clk, reset_n,
    input wire fifo_empty,
    input wire [31:0] fifo_data,
    output reg fifo_read,
    input wire tx_busy, tx_done, tx_data_req,
    input wire [10:0] tx_payload_addr,
    output wire tx_request,
    output wire tx_active,
    output reg tx_start,
    output wire [15:0] tx_length,
    output reg [7:0] tx_data
);
    reg [15:0] sample_bank0[0:127];
    reg [15:0] sample_bank1[0:127];
    reg [1:0] ready;
    reg fill_bank,active_bank,sending;
    reg [6:0] fill_index;
    reg read_wait,read_pending;
    reg [31:0] packet_sequence,active_sequence;
    wire signed [16:0] mixed=
        $signed({fifo_data[31],fifo_data[31:16]})+
        $signed({fifo_data[15],fifo_data[15:0]});
    wire [15:0] mono=mixed[16:1];
    wire fill_free=!ready[fill_bank] && !(sending && active_bank==fill_bank);
    assign tx_request=(ready[0] || ready[1]) && !sending;
    assign tx_active=tx_request || sending || tx_start;
    assign tx_length=16'd272;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            ready<=0;fill_bank<=0;active_bank<=0;sending<=0;
            fill_index<=0;read_wait<=0;read_pending<=0;fifo_read<=0;tx_start<=0;
            packet_sequence<=0;active_sequence<=0;
        end else begin
            fifo_read<=0;tx_start<=0;
            if(read_wait) begin
                read_wait<=0;read_pending<=1;
            end else if(read_pending) begin
                if(fill_bank) sample_bank1[fill_index]<=mono;
                else sample_bank0[fill_index]<=mono;
                if(fill_index==7'd127) begin
                    ready[fill_bank]<=1;
                    fill_bank<=~fill_bank;
                    fill_index<=0;
                end else fill_index<=fill_index+1'b1;
                read_pending<=0;
            end else if(fill_free && !fifo_empty) begin
                fifo_read<=1;
                read_wait<=1;
            end
            if(tx_request && !tx_busy) begin
                active_bank<=ready[0] ? 1'b0 : 1'b1;
                active_sequence<=packet_sequence;
                packet_sequence<=packet_sequence+1'b1;
                sending<=1;tx_start<=1;
            end
            if(sending && tx_done) begin
                ready[active_bank]<=0;
                sending<=0;
            end
        end
    end
    wire [10:0] sample_byte_addr=tx_payload_addr-11'd16;
    reg [15:0] read_word0,read_word1;
    reg [3:0] read_header_addr;
    reg read_is_sample,read_low_byte;
    always @(posedge clk) begin
        if(tx_data_req) begin
            read_word0<=sample_bank0[sample_byte_addr[7:1]];
            read_word1<=sample_bank1[sample_byte_addr[7:1]];
            read_header_addr<=tx_payload_addr[3:0];
            read_is_sample<=tx_payload_addr>=16;
            read_low_byte<=sample_byte_addr[0];
        end
    end
    wire [15:0] read_word=active_bank ? read_word1 : read_word0;
    always @(*) begin
        if(read_is_sample) tx_data=read_low_byte ? read_word[7:0] : read_word[15:8];
        else case(read_header_addr)
            0:tx_data=8'h46;1:tx_data=8'h41;2:tx_data=8'h55;3:tx_data=8'h44;
            4:tx_data=8'd1;5:tx_data=8'd1;6:tx_data=0;7:tx_data=8'd16;
            8:tx_data=active_sequence[31:24];9:tx_data=active_sequence[23:16];
            10:tx_data=active_sequence[15:8];11:tx_data=active_sequence[7:0];
            12:tx_data=8'h3e;13:tx_data=8'h80; // 16,000 samples/s
            14:tx_data=0;15:tx_data=8'd128;
            default:tx_data=0;
        endcase
    end
endmodule
