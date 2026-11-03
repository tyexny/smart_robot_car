// Final HDMI RGB565 tap. A bounded asynchronous row FIFO absorbs the burst
// from the 75 MHz raster; no DDR port and no backpressure into HDMI.
// FREC v3 uses the existing 36-byte segmented RGB565 layout, with zero lock
// fields. Both half-row packets must complete before a row can be overwritten.
module screen_rgb565_packetizer #(
    parameter integer IMAGE_WIDTH=1280, IMAGE_HEIGHT=720,
    parameter integer FRAME_STRIDE=2, FIFO_ROWS=32,
    parameter integer BUFFER_ROWS=FIFO_ROWS>25 ? 25 : FIFO_ROWS
)(
    input wire camera_clk,tx_clk,reset_n,camera_vsync,capture_enable,
    input wire pixel_valid,
    input wire [15:0] pixel_rgb565,
    input wire tx_busy,tx_done,tx_data_req,
    input wire [10:0] tx_payload_addr,
    output reg tx_start,
    output wire [15:0] tx_length,
    output reg [7:0] tx_data,
    output reg [31:0] packets_sent,rows_dropped
);
    localparam integer P=$clog2(FIFO_ROWS), A=$clog2(BUFFER_ROWS*IMAGE_WIDTH);
    localparam integer HALF=IMAGE_WIDTH/2, WORDS=BUFFER_ROWS*IMAGE_WIDTH;
    localparam [15:0] W=IMAGE_WIDTH,H=IMAGE_HEIGHT,N=HALF;
    reg [P:0] write_pointer,write_gray,read_pointer,read_gray;
    (* syn_preserve=1 *) reg [P:0] read_gray_meta,read_gray_sync;
    (* syn_preserve=1 *) reg [P:0] write_gray_meta,write_gray_sync;
    reg [A-1:0] write_base,packet_base;
    reg [11:0] x,y;
    reg [31:0] frame_id;
    reg vsync_d,enable_meta,enable_sync,row_accept,frame_selected;
    reg [15:0] stride_count;
    // Logical descriptor pointers retain a power-of-two Gray ring. Pixel
    // storage wraps independently after BUFFER_ROWS rows. Reserving at most
    // 25 rows fits 32000 RGB565 words in two 16K RAM slices, rather than
    // allocating a mostly empty third slice for 32 rows.
    function [P:0] gray_to_binary;
        input [P:0] gray;
        integer bit_index;
        begin
            gray_to_binary[P]=gray[P];
            for(bit_index=P-1;bit_index>=0;bit_index=bit_index-1)
                gray_to_binary[bit_index]=gray_to_binary[bit_index+1]^gray[bit_index];
        end
    endfunction
    wire [P:0] occupied=write_pointer-gray_to_binary(read_gray_sync);
    wire full=occupied>=BUFFER_ROWS;
    wire empty=read_gray==write_gray_sync;
    wire selected=pixel_valid && !camera_vsync && enable_sync && frame_selected && y<IMAGE_HEIGHT;
    wire writing=selected && (x==0 ? !full : row_accept);
    wire commit=writing && x==IMAGE_WIDTH-1;
    wire [P:0] next_write=write_pointer+1'b1,next_read=read_pointer+1'b1;
    reg [15:0] pixels[0:WORDS-1];
    reg [47:0] row_metadata[0:FIFO_ROWS-1];
    wire [A-1:0] write_address=write_base+x;
    // Data/metadata are stable from publication through the second tx_done.
    always @(posedge camera_clk) begin
        if(writing) pixels[write_address]<=pixel_rgb565;
        if(commit) row_metadata[write_pointer[P-1:0]]<={frame_id,4'd0,y};
    end
    always @(posedge camera_clk or negedge reset_n) begin
        if(!reset_n)begin
            write_pointer<=0;write_gray<=0;write_base<=0;
            read_gray_meta<=0;read_gray_sync<=0;
            x<=0;y<=0;frame_id<=0;vsync_d<=0;
            enable_meta<=0;enable_sync<=0;row_accept<=0;
            frame_selected<=0;stride_count<=0;rows_dropped<=0;
        end else begin
            read_gray_meta<=read_gray;read_gray_sync<=read_gray_meta;
            enable_meta<=capture_enable;enable_sync<=enable_meta;
            vsync_d<=camera_vsync;
            if(camera_vsync && !vsync_d)begin
                x<=0;y<=0;row_accept<=0;frame_id<=frame_id+1'b1;
                frame_selected<=stride_count==0;
                if(stride_count==FRAME_STRIDE-1)stride_count<=0;
                else stride_count<=stride_count+1'b1;
            end else if(pixel_valid && !camera_vsync)begin
                if(x==IMAGE_WIDTH-1)begin x<=0;if(y<IMAGE_HEIGHT)y<=y+1'b1;end
                else x<=x+1'b1;
                if(selected && x==0)begin
                    row_accept<=!full;
                    if(full)rows_dropped<=rows_dropped+1'b1;
                end
                if(commit)begin
                    row_accept<=0;write_pointer<=next_write;
                    write_gray<=(next_write>>1)^next_write;
                    write_base<=write_base==WORDS-IMAGE_WIDTH ? 0 : write_base+IMAGE_WIDTH;
                end
            end
        end
    end
    reg [47:0] metadata_q;
    reg [31:0] active_frame;
    reg [15:0] active_row;
    reg active_half,sending,second_pending,metadata_ready;
    wire [10:0] byte_offset=tx_payload_addr-11'd36;
    // Byte34 primes the first RAM word. Each high-byte request latches the
    // current complete word and starts prefetching the next one. Holding the
    // complete word through its low byte also supports arbitrary MAC gaps.
    // The RAM output now ends at a local word register, before byte/header
    // selection and the Ethernet transmitter's own payload register.
    reg [A-1:0] read_address;
    always @(posedge tx_clk or negedge reset_n)begin
        if(!reset_n)read_address<=0;
        else if(tx_data_req)begin
            if(tx_payload_addr==34)read_address<=packet_base;
            else if(tx_payload_addr>=36&&!tx_payload_addr[0])
                read_address<=read_address==WORDS-1 ? 0 : read_address+1'b1;
        end
    end
    reg [15:0] pixel_ram_q,pixel_q;
    reg [7:0] header_value;
    always @(posedge tx_clk)begin
        metadata_q<=row_metadata[read_pointer[P-1:0]];
        pixel_ram_q<=pixels[read_address];
        if(tx_data_req&&tx_payload_addr>=36&&!tx_payload_addr[0])pixel_q<=pixel_ram_q;
        // Preserve the first-byte selector during the production MAC's
        // preamble/IP/UDP wait. RAM reads themselves remain unconditional.
        if(tx_data_req)begin
            // Same requested-byte response edge as the former word/selector
            // registers. Select before this byte register, not after BSRAM.
            tx_data<=tx_payload_addr>=36 ?
                (byte_offset[0]?pixel_q[7:0]:pixel_ram_q[15:8]) : header_value;
        end
    end
    assign tx_length=36+IMAGE_WIDTH;
    always @*begin
        header_value=0;
        case(tx_payload_addr[5:0])
            0:header_value="F";1:header_value="R";2:header_value="E";3:header_value="C";
            4:header_value=3;5:header_value=1;7:header_value=36;
            8:header_value=active_frame[31:24];9:header_value=active_frame[23:16];
            10:header_value=active_frame[15:8];11:header_value=active_frame[7:0];
            12:header_value=W[15:8];13:header_value=W[7:0];
            14:header_value=H[15:8];15:header_value=H[7:0];
            16:header_value=active_row[15:8];17:header_value=active_row[7:0];19:header_value=1;
            32:header_value=active_half ? N[15:8] : 0;
            33:header_value=active_half ? N[7:0] : 0;
            34:header_value=N[15:8];35:header_value=N[7:0];
            default:;
        endcase
    end
    always @(posedge tx_clk or negedge reset_n)begin
        if(!reset_n)begin
            write_gray_meta<=0;write_gray_sync<=0;read_pointer<=0;read_gray<=0;
            active_frame<=0;active_row<=0;active_half<=0;
            sending<=0;second_pending<=0;tx_start<=0;packets_sent<=0;packet_base<=0;metadata_ready<=0;
        end else begin
            write_gray_meta<=write_gray;write_gray_sync<=write_gray_meta;
            tx_start<=0;
            metadata_ready<=1;
            if(metadata_ready && !sending && !second_pending && !empty && !tx_busy)begin
                active_frame<=metadata_q[47:16];active_row<=metadata_q[15:0];
                active_half<=0;sending<=1;tx_start<=1;
            end
            if(second_pending && !tx_busy)begin
                second_pending<=0;sending<=1;tx_start<=1;
            end
            if(sending && tx_done)begin
                sending<=0;packets_sent<=packets_sent+1'b1;
                packet_base<=packet_base==WORDS-HALF ? 0 : packet_base+HALF;
                if(!active_half)begin active_half<=1;second_pending<=1;end
                else begin read_pointer<=next_read;read_gray<=(next_read>>1)^next_read;metadata_ready<=0;end
            end
        end
    end
endmodule
