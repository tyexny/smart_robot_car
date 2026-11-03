`timescale 1ns/1ps
// FTIQ v1 20-byte request; FTRP v1 response. Fixed sensor 0x33, max32 words.
// Request: magic/ver/op/count/zero/sequence32/register16/value16/reply_port16/zero16.
// Reply: magic/ver/status/count16/sequence32/register16/zero16, big-endian words.
module thermal_i2c_rpc(
    input wire rx_clk,tx_clk,reset_n,
    input wire packet_start,payload_valid,packet_good,packet_error,
    input wire [7:0] payload_data,input wire [15:0] payload_length,
    input wire [64:0] arrival_status,
    input wire tx_busy,tx_done,tx_data_req,input wire [10:0] tx_payload_addr,
    output wire tx_active,output reg tx_start,output wire [15:0] tx_length,tx_port,
    output wire [7:0] tx_data,inout wire scl,sda
);
    reg [159:0] packet;
    reg [5:0] received;
    reg damaged,request,eligible,good_pending;
    reg [159:0] request_data;
    wire command_valid;
    wire [159:0] incoming_command;
    always @(posedge rx_clk or negedge reset_n)begin
        if(!reset_n)begin packet<=0;received<=0;damaged<=0;request<=0;eligible<=0;good_pending<=0;request_data<=0;end
        else begin
            request<=0;
            good_pending<=packet_good&&!packet_error;
            eligible<=!damaged&&payload_length==20&&received==20&&
               packet[159:128]==32'h46544951&&packet[127:120]==1&&packet[103:96]==0&&packet[15:0]==0&&packet[31:16]>=1024;
            if(packet_start)begin packet<={152'd0,payload_data};received<=payload_valid?1:0;damaged<=0;eligible<=0;end
            else if(payload_valid)begin packet<={packet[151:0],payload_data};if(received!=63)received<=received+1'b1;end
            if(packet_error)damaged<=1;
            // CRC acceptance belongs to the completed packet. A subsequent
            // packet's start/error must not cancel that accepted snapshot;
            // packet/eligible still hold the old values on this clock edge.
            if(good_pending&&eligible)begin
                request_data<=packet;request<=1;
            end
        end
    end
    telemetry_mailbox #(.WIDTH(160),.RESET_SYNC_RELEASE(1)) u_request_cdc(
        .src_clk(rx_clk),.dst_clk(tx_clk),.reset_n(reset_n),.src_valid(request),.src_data(request_data),.dst_valid(command_valid),.dst_data(incoming_command));
    reg [1:0] state;
    reg [159:0] command;
    reg command_pending,queued_read_allowed;
    reg [31:0] transaction;
    reg [15:0] register_r,port_r;
    reg [5:0] count_r;
    reg [7:0] result;
    reg [15:0] words[0:31];
    reg i2c_start,i2c_write;
    reg [15:0] i2c_value;
    wire i2c_busy,i2c_done,i2c_error,i2c_data_valid;
    wire [4:0] i2c_index;wire [15:0] i2c_word;
    reg [31:0] last_write_id;reg [15:0] last_write_port;reg last_write_ok;
    mlx90640_i2c_master u_i2c(.clk(tx_clk),.reset_n(reset_n),.start(i2c_start),.write(i2c_write),
        .address(register_r),.write_value(i2c_value),.word_count(count_r),.busy(i2c_busy),.done(i2c_done),.error(i2c_error),
        .data_valid(i2c_data_valid),.data_index(i2c_index),.data_word(i2c_word),.scl(scl),.sda(sda));
    wire take_command=state==0&&command_pending&&!i2c_busy;
    // The Pi arrival poll and native sensor worker are independent clients.
    // Retain one waiting transaction while I2C/TX handles the other, rather
    // than silently losing every command_valid pulse received while busy.
    always @(posedge tx_clk or negedge reset_n)begin
        if(!reset_n)begin command<=0;command_pending<=0;queued_read_allowed<=0;end
        else begin
            if(take_command)command_pending<=0;
            if(command_valid&&(!command_pending||take_command))begin
                command<=incoming_command;command_pending<=1;
                // Decode the address window before the execution state, to
                // keep its 17-bit span arithmetic out of the I2C start path.
                queued_read_allowed<=incoming_command[111:104]>=1&&incoming_command[111:104]<=32&&
                    ((incoming_command[63:48]>=16'h2400&&{1'b0,incoming_command[63:48]}+{9'd0,incoming_command[111:104]}<=17'h02740)||
                     (incoming_command[63:48]>=16'h0400&&{1'b0,incoming_command[63:48]}+{9'd0,incoming_command[111:104]}<=17'h00740)||
                     (incoming_command[111:104]==1&&(incoming_command[63:48]==16'h8000||incoming_command[63:48]==16'h800d)));
            end
        end
    end
    assign tx_active=state==2||state==3;
    assign tx_length=16'd16+{9'd0,count_r,1'b0};
    assign tx_port=port_r;
    wire [127:0] header={32'h46545250,8'd1,result,10'd0,count_r,transaction,register_r,16'd0};
    reg [15:0] read_word;
    reg [7:0] read_header;
    reg read_payload,read_low;
    always @(posedge tx_clk)if(tx_data_req)begin
        read_word<=words[tx_payload_addr[5:1]-5'd8];
        read_header<=header[(15-tx_payload_addr[3:0])*8+:8];
        read_payload<=tx_payload_addr>=16;read_low<=tx_payload_addr[0];
    end
    assign tx_data=read_payload?(read_low?read_word[7:0]:read_word[15:8]):read_header;
    // Status shares the same coherent SYS50->TX mailbox as arrival identity.
    // Only explicit approach_near completion sets its flag, never lock loss.
    always @(posedge tx_clk or negedge reset_n)begin
        if(!reset_n)begin state<=0;transaction<=0;register_r<=0;port_r<=0;count_r<=0;result<=0;tx_start<=0;i2c_start<=0;i2c_write<=0;i2c_value<=0;last_write_ok<=0;last_write_id<=0;last_write_port<=0;end
        else begin
            tx_start<=0;i2c_start<=0;
            if(i2c_data_valid)words[i2c_index]<=i2c_word;
            case(state)
                0:if(take_command)begin
                    transaction<=command[95:64];register_r<=command[63:48];port_r<=command[31:16];result<=0;count_r<=0;
                    if(command[119:112]==3)begin
                        count_r<=5;words[0]<=arrival_status[63:48];words[1]<=arrival_status[47:32];
                        words[2]<=arrival_status[31:16];words[3]<=arrival_status[15:0];words[4]<={15'd0,arrival_status[64]};state<=2;
                    end else if(command[119:112]==1&&queued_read_allowed)begin
                        count_r<=command[109:104];i2c_write<=0;i2c_start<=1;state<=1;
                    end else if(command[119:112]==2&&command[111:104]==1&&(command[63:48]==16'h8000||command[63:48]==16'h800d))begin
                        if(last_write_ok&&last_write_id==command[95:64]&&last_write_port==command[31:16])state<=2;
                        else begin i2c_write<=1;i2c_value<=command[47:32];count_r<=1;i2c_start<=1;state<=1;end
                    end else begin result<=2;state<=2;end
                end
                1:if(i2c_done)begin
                    if(i2c_write)begin
                        count_r<=0;
                        if(!i2c_error)begin last_write_ok<=1;last_write_id<=transaction;last_write_port<=port_r;end
                    end
                    if(i2c_error)begin result<=1;count_r<=0;end
                    state<=2;
                end
                2:if(!tx_busy)begin tx_start<=1;state<=3;end
                3:if(tx_done)state<=0;
                default:state<=0;
            endcase
        end
    end
endmodule
