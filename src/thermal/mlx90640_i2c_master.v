`timescale 1ns/1ps
// Open-drain 400 kHz register transport. Temperature mathematics stays on Pi.
module mlx90640_i2c_master #(
    parameter integer CLK_HZ=125000000,I2C_HZ=400000,
    parameter integer QUARTER=CLK_HZ/(4*I2C_HZ),TIMEOUT=CLK_HZ/40
)(
    input wire clk,reset_n,start,write,
    input wire [15:0] address,write_value,input wire [5:0] word_count,
    output wire busy,output reg done,error,data_valid,
    output reg [4:0] data_index,output reg [15:0] data_word,
    inout wire scl,sda
);
    localparam IDLE=0,BUS=1,START=2,START_LOW=3,SET=4,RISE=5,HIGH=6,LOW=7,
        ACK_SET=8,ACK_RISE=9,ACK_HIGH=10,ACK_LOW=11,NEXT=12,
        RESTART_SET=13,RESTART_RISE=14,RESTART_HIGH=15,RESTART_LOW=16,
        READ_SET=17,READ_RISE=18,READ_HIGH=19,READ_LOW=20,
        MASTER_ACK_SET=21,MASTER_ACK_RISE=22,MASTER_ACK_HIGH=23,MASTER_ACK_LOW=24,
        STOP_LOW=25,STOP_RISE=26,STOP_HIGH=27,STOP_RELEASE=28;
    reg [4:0] state;
    reg scl_low,sda_low;
    assign scl=scl_low?1'b0:1'bz;
    assign sda=sda_low?1'b0:1'bz;
    (* ASYNC_REG="TRUE" *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    always @(posedge clk)begin scl_meta<=scl;scl_sync<=scl_meta;sda_meta<=sda;sda_sync<=sda_meta;end
    reg [$clog2(QUARTER+1)-1:0] divider;
    reg [$clog2(TIMEOUT+1)-1:0] age;
    reg [7:0] shift,high_byte;
    reg [2:0] bit_number,command_phase;
    reg [6:0] byte_index,byte_count;
    // Preserve the carry at 127+1 without a 32-bit unsized addition.
    wire [7:0] next_byte_index={1'b0,byte_index}+8'd1;
    wire last_byte=next_byte_index>={1'b0,byte_count};
    reg write_r;
    reg [15:0] register_r,value_r;
    assign busy=state!=IDLE;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=IDLE;scl_low<=0;sda_low<=0;divider<=0;age<=0;done<=0;error<=0;data_valid<=0;
            data_index<=0;data_word<=0;shift<=0;high_byte<=0;bit_number<=7;command_phase<=0;
            byte_index<=0;byte_count<=0;write_r<=0;register_r<=0;value_r<=0;
        end else begin
            done<=0;data_valid<=0;
            if(state==IDLE)begin
                age<=0;divider<=0;scl_low<=0;sda_low<=0;
                if(start)begin
                    write_r<=write;register_r<=address;value_r<=write_value;
                    byte_count<={word_count,1'b0};byte_index<=0;command_phase<=0;error<=0;
                    state<=BUS;
                end
            end else if(age>=TIMEOUT)begin
                state<=IDLE;scl_low<=0;sda_low<=0;error<=1;done<=1;
            end else begin
                age<=age+1'b1;
                if(divider==QUARTER-1)begin
                    divider<=0;
                    case(state)
                        BUS:if(scl_sync&&sda_sync)state<=START;
                        START:begin sda_low<=1;state<=START_LOW;end
                        START_LOW:begin scl_low<=1;shift<=8'h66;bit_number<=7;state<=SET;end
                        SET:begin sda_low<=!shift[bit_number];state<=RISE;end
                        RISE:begin scl_low<=0;state<=HIGH;end
                        HIGH:if(scl_sync)state<=LOW;
                        LOW:begin scl_low<=1;if(bit_number==0)state<=ACK_SET;else begin bit_number<=bit_number-1'b1;state<=SET;end end
                        ACK_SET:begin sda_low<=0;state<=ACK_RISE;end
                        ACK_RISE:begin scl_low<=0;state<=ACK_HIGH;end
                        ACK_HIGH:if(scl_sync)begin if(sda_sync)error<=1;state<=ACK_LOW;end
                        ACK_LOW:begin scl_low<=1;state<=NEXT;end
                        NEXT:if(error)state<=STOP_LOW;else begin
                            bit_number<=7;
                            case(command_phase)
                                0:begin shift<=register_r[15:8];command_phase<=1;state<=SET;end
                                1:begin shift<=register_r[7:0];command_phase<=2;state<=SET;end
                                2:if(write_r)begin shift<=value_r[15:8];command_phase<=3;state<=SET;end else state<=RESTART_SET;
                                3:begin shift<=value_r[7:0];command_phase<=4;state<=SET;end
                                4:state<=STOP_LOW;
                                5:begin shift<=0;state<=READ_SET;end
                                default:state<=STOP_LOW;
                            endcase
                        end
                        RESTART_SET:begin sda_low<=0;state<=RESTART_RISE;end
                        RESTART_RISE:begin scl_low<=0;state<=RESTART_HIGH;end
                        RESTART_HIGH:if(scl_sync)begin sda_low<=1;state<=RESTART_LOW;end
                        RESTART_LOW:begin scl_low<=1;shift<=8'h67;command_phase<=5;state<=SET;end
                        READ_SET:begin sda_low<=0;state<=READ_RISE;end
                        READ_RISE:begin scl_low<=0;state<=READ_HIGH;end
                        READ_HIGH:if(scl_sync)begin shift<={shift[6:0],sda_sync};state<=READ_LOW;end
                        READ_LOW:begin scl_low<=1;if(bit_number==0)state<=MASTER_ACK_SET;else begin bit_number<=bit_number-1'b1;state<=READ_SET;end end
                        MASTER_ACK_SET:begin
                            sda_low<=!last_byte;
                            if(!byte_index[0])high_byte<=shift;
                            else begin data_word<={high_byte,shift};data_index<=byte_index[5:1];data_valid<=1;end
                            state<=MASTER_ACK_RISE;
                        end
                        MASTER_ACK_RISE:begin scl_low<=0;state<=MASTER_ACK_HIGH;end
                        MASTER_ACK_HIGH:if(scl_sync)state<=MASTER_ACK_LOW;
                        MASTER_ACK_LOW:begin
                            scl_low<=1;
                            if(last_byte)state<=STOP_LOW;
                            else begin byte_index<=byte_index+1'b1;bit_number<=7;shift<=0;state<=READ_SET;end
                        end
                        STOP_LOW:begin scl_low<=1;sda_low<=1;state<=STOP_RISE;end
                        STOP_RISE:begin scl_low<=0;state<=STOP_HIGH;end
                        STOP_HIGH:if(scl_sync)begin sda_low<=0;state<=STOP_RELEASE;end
                        STOP_RELEASE:begin done<=1;state<=IDLE;end
                        default:state<=IDLE;
                    endcase
                end else divider<=divider+1'b1;
            end
        end
    end
endmodule
