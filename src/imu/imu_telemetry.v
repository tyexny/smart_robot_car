`timescale 1ns/1ps
// YbImu UART: 7E 23 total_length function payload sum8, 115200 8N1.
// Packed signed32 fields (LSB first): AX AY AZ (mg), GX GY GZ (0.1deg/s),
// MX MY MZ (0.1uT), Q0 Q1 Q2 Q3 (1e-4), R P Y (0.1deg),
// height(cm), temperature(0.1C), pressure/reference(Pa), relative yaw(0.1deg).
module imu_telemetry #(
    parameter integer CLK_FREQ_HZ=50000000,
    parameter integer BAUD_RATE=115200
)(
    input wire clk, reset_n, uart_rx,
    output wire [671:0] values,
    output wire [3:0] known,
    output wire ready,
    output reg update,
    output reg yaw_update // One cycle only when a valid Euler frame is committed.
);
    wire [7:0] byte_data;
    wire byte_valid, byte_error;
    uart_rx_byte #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) rx(
      .clk(clk),.reset_n(reset_n),.uart_rx(uart_rx),.data(byte_data),
      .valid(byte_valid),.framing_error(byte_error));
    reg [2:0] state;
    reg [7:0] length, func, sum;
    reg [5:0] index;
    reg [7:0] payload[0:17];
    reg [31:0] raw[0:19];
    reg [3:0] pending;
    reg [3:0] seen;
    reg [3:0] fresh;
    reg [31:0] age[0:3];
    reg [31:0] timeout_count;
    reg [4:0] convert_index, last_index;
    reg busy;
    assign ready=!busy && pending==0;
    reg signed [31:0] fixed_value[0:20];
    reg zero_valid;
    reg signed [31:0] zero_yaw;
    reg [31:0] operand;
    reg [31:0] scale;
    reg [2:0] phase;
    reg [63:0] magnitude, product, rounded, round_mask;
    reg [23:0] mantissa;
    reg sign_bit;
    reg signed [9:0] shift;
    reg signed [31:0] converted;
    wire signed [32:0] delta = $signed({converted[31],converted})-
                                             $signed({zero_yaw[31],zero_yaw});
    integer n;
    genvar g;
    generate for(g=0;g<21;g=g+1) begin: pack
        assign values[g*32+:32]=fixed_value[g];
    end endgenerate
    generate for(g=0;g<4;g=g+1) begin: freshness
        assign known[g]=fresh[g];
    end endgenerate

    // Shared seven-cycle converter: select, decode, multiply, round, shift,
    // apply sign, commit. It stays well below the UART inter-frame interval.

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=0; length<=0; func<=0; sum<=0; index<=0;
            pending<=0; seen<=0; fresh<=0; timeout_count<=0; busy<=0;
            convert_index<=0; last_index<=0; update<=0; yaw_update<=0;
            zero_valid<=0; zero_yaw<=0;
            phase<=0; operand<=0; scale<=1; magnitude<=0; product<=0;
            rounded<=0; round_mask<=0; mantissa<=0; sign_bit<=0; shift<=0; converted<=0;
            for(n=0;n<20;n=n+1) raw[n]<=0;
            for(n=0;n<21;n=n+1) fixed_value[n]<=0;
            for(n=0;n<4;n=n+1) age[n]<=CLK_FREQ_HZ;
        end else begin
            update<=0; yaw_update<=0;
            // Register freshness at its producer. The half-second age decode
            // previously fed every navigation decision in the same cycle.
            // Predict the next age, preserving the exact expiration edge;
            // a newly committed group below overrides this with fresh=1.
            for(n=0;n<4;n=n+1)begin
                if(age[n]<CLK_FREQ_HZ) age[n]<=age[n]+1'b1;
                fresh[n]<=seen[n] && age[n]<CLK_FREQ_HZ/2-1;
            end
            if(!busy) begin
                phase<=0;
                if(pending[0]) begin busy<=1; convert_index<=0; last_index<=8; pending[0]<=0; end
                else if(pending[1]) begin busy<=1; convert_index<=9; last_index<=12; pending[1]<=0; end
                else if(pending[2]) begin busy<=1; convert_index<=13; last_index<=15; pending[2]<=0; end
                else if(pending[3]) begin busy<=1; convert_index<=16; last_index<=19; pending[3]<=0; end
            end else if(phase==0) begin
                operand<=raw[convert_index]; phase<=1;
            end else if(phase==1) begin
                scale<=1;
                if(convert_index<9) begin
                    sign_bit<=operand[15]; shift<=30;
                    mantissa<=operand[15] ? {8'd0,(~operand[15:0]+16'd1)} : {8'd0,operand[15:0]};
                    if(convert_index<3) scale<=524304000;
                    else if(convert_index<6) scale<=655380000;
                    else scale<=262152000;
                end else begin
                    sign_bit<=operand[31];
                    mantissa<=(operand[30:23]==0 || operand[30:23]==255) ? 0 : {1'b1,operand[22:0]};
                    shift<=10'sd150-$signed({2'b00,operand[30:23]});
                    if(convert_index<13) scale<=10000;
                    else if(convert_index<16) begin
                        scale<=37549362;
                        shift<=10'sd166-$signed({2'b00,operand[30:23]});
                    end else if(convert_index==16) scale<=100;
                    else if(convert_index==17) scale<=10;
                end
                phase<=2;
            end else if(phase==2) begin
                product<=mantissa*scale;
                round_mask<=(shift>0 && shift<64) ? (64'd1<<(shift-1)) : 0;
                phase<=3;
            end else if(phase==3) begin
                rounded<=product+round_mask; phase<=4;
            end else if(phase==4) begin
                if(shift>0 && shift<64) magnitude<=rounded>>shift;
                else if(shift>=64) magnitude<=0;
                else if(shift>-24) magnitude<=rounded<<(-shift);
                else magnitude<=64'h7fffffff;
                phase<=5;
            end else if(phase==5) begin
                if(magnitude>64'd2147483647) converted<=sign_bit ? -32'sd2147483647 : 32'sd2147483647;
                else converted<=sign_bit ? -$signed(magnitude[31:0]) : $signed(magnitude[31:0]);
                phase<=6;
            end else begin
                phase<=0;
                fixed_value[convert_index]<=converted;
                if(convert_index==last_index) begin
                    busy<=0; update<=1;
                    case(last_index)
                        8:begin seen[0]<=1; age[0]<=0; fresh[0]<=1; end
                        12:begin seen[1]<=1; age[1]<=0; fresh[1]<=1; end
                        15:begin
                            seen[2]<=1; age[2]<=0; fresh[2]<=1; yaw_update<=1;
                            if(!zero_valid) begin
                                zero_yaw<=converted; zero_valid<=1; fixed_value[20]<=0;
                            end else if(delta>1800) fixed_value[20]<=delta-3600;
                            else if(delta< -1800) fixed_value[20]<=delta+3600;
                            else fixed_value[20]<=delta[31:0];
                        end
                        19:begin seen[3]<=1; age[3]<=0; fresh[3]<=1; end
                        default:;
                    endcase
                end else convert_index<=convert_index+1'b1;
            end
            // Recover after incomplete frames, wrong stop bits or length.
            if(byte_error || (state!=0 && timeout_count>=CLK_FREQ_HZ/500)) begin
                state<=0; timeout_count<=0;
            end else if(byte_valid) begin
                timeout_count<=0;
                case(state)
                    0:if(byte_data==8'h7e) begin state<=1; sum<=8'h7e; end
                    1:if(byte_data==8'h23) begin state<=2; sum<=sum+byte_data; end
                      else if(byte_data!=8'h7e) state<=0;
                    2:if(byte_data>=5 && byte_data<=23) begin
                        length<=byte_data; sum<=sum+byte_data; state<=3;
                      end else state<=0;
                    3:begin func<=byte_data; sum<=sum+byte_data; index<=0; state<=4; end
                    4:begin
                        if(index==length-5) begin
                            state<=0;
                            if(byte_data==sum) begin
                                if(func==8'h04 && length==23) begin
                                    for(n=0;n<9;n=n+1) raw[n]<={16'd0,payload[n*2+1],payload[n*2]};
                                    pending[0]<=1;
                                end else if(func==8'h16 && length==21) begin
                                    for(n=0;n<4;n=n+1) raw[n+9]<={payload[n*4+3],payload[n*4+2],payload[n*4+1],payload[n*4]};
                                    pending[1]<=1;
                                end else if(func==8'h26 && length==17) begin
                                    for(n=0;n<3;n=n+1) raw[n+13]<={payload[n*4+3],payload[n*4+2],payload[n*4+1],payload[n*4]};
                                    pending[2]<=1;
                                end else if(func==8'h32 && length==21) begin
                                    for(n=0;n<4;n=n+1) raw[n+16]<={payload[n*4+3],payload[n*4+2],payload[n*4+1],payload[n*4]};
                                    pending[3]<=1;
                                end
                            end
                        end else begin
                            if(index<18) payload[index]<=byte_data;
                            sum<=sum+byte_data; index<=index+1'b1;
                        end
                    end
                    default:state<=0;
                endcase
            end else if(state!=0) timeout_count<=timeout_count+1'b1;
        end
    end
endmodule
