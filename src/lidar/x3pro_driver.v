// Receive-only X3 Pro, 115200 8N1, AA 55 CT LSN FSA LSA CS samples.
// All words are little-endian; distance is sample[15:2] mm. No intensity byte.
// X3 low two sample bits are interference flags: reject mirror/light flags 2/3.
// No point or revolution event is published until the ENTIRE packet passes
// XOR checksum, angle check bits and length validation. Two packet banks
// decouple UART reception from map rasterization. Overflow drops a whole
// packet; parser recovers at the next header, never replays stale points.
module x3pro_driver #(
    parameter integer CLK_FREQ_HZ=50000000, BAUD_RATE=115200,
    parameter integer PWM_FREQ_HZ=10000, PWM_DUTY_PERCENT=50,
    parameter integer STALE_MS=500, BYTE_TIMEOUT_MS=5, SENSOR_LATENCY_US=0
)(
    input wire clk,reset_n,uart_rx,
    input wire [7:0] motor_duty_percent,
    output wire motor_pwm,
    input wire point_ready,
    output reg point_valid,
    output reg [15:0] distance_mm,
    output reg [11:0] angle_tenths,
    output reg range_valid,no_return,scan_start,
    output wire online,
    output reg [31:0] point_timestamp,output reg point_timestamp_valid,
    output reg [7:0] fault,
    output reg [15:0] good_packets,bad_packets,dropped_packets
);
    localparam integer STALE_CLKS=CLK_FREQ_HZ/1000*STALE_MS;
    localparam integer TIMEOUT_CLKS=CLK_FREQ_HZ/1000*BYTE_TIMEOUT_MS;
    localparam integer PWM_CLKS=CLK_FREQ_HZ/PWM_FREQ_HZ;
    reg [31:0] pwm_count,packet_age,byte_age;
    reg [31:0] pwm_high_clks;
    wire [7:0] pwm_limited_percent=motor_duty_percent>8'd100 ? 8'd100 : motor_duty_percent;
    // Split the constant scale so the production 5000-clock period uses a
    // multiply by 50, while other clock parameters retain integer rounding.
    wire [31:0] pwm_requested_high_clks=(PWM_CLKS/100)*pwm_limited_percent+
        ((PWM_CLKS%100)*pwm_limited_percent)/100;
    // Latch the requested duty only at period rollover: no partial pulses.
    // M_CTR at 0% requests minimum motor speed; it does not stop the motor.
    assign motor_pwm=reset_n && pwm_count<pwm_high_clks;
    assign online=packet_age<STALE_CLKS;
    wire [7:0] byte_data;
    wire byte_valid,framing_error;
    uart_rx_byte #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) u_rx(
        .clk(clk),.reset_n(reset_n),.uart_rx(uart_rx),.data(byte_data),
        .valid(byte_valid),.framing_error(framing_error));
    localparam [3:0] H0=0,H1=1,CT=2,N=3,F0=4,F1=5,L0=6,L1=7,C0=8,C1=9,D0=10,D1=11;
    reg [3:0] parser;
    reg write_bank,drop_packet;
    // Host-clock estimate of acquisition time. Header arrival anchors the last
    // acquired sample; packet spacing estimates the sampling interval. Sensor
    // processing latency remains an explicit calibratable board parameter.
    reg [31:0] acquisition_clock,header_time,previous_header,sample_period;
    reg timing_seen;
    reg period_start,period_measured;
    reg [31:0] period_interval;
    reg [7:0] period_samples;
    wire period_done;
    wire [31:0] measured_period;
    x3pro_period_divider period_divider(.clk(clk),.reset_n(reset_n),.start(period_start),
        .interval_clks(period_interval),.sample_count(period_samples),.done(period_done),.quotient(measured_period));
    reg [31:0] bank_time[0:1],bank_period[0:1];
    reg bank_timing_valid[0:1];
    reg [1:0] ready;
    reg [7:0] ct,n,lo,sample_index;
    reg [15:0] fsa,lsa,checksum,received_checksum;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [15:0] samples[0:511];
    reg [7:0] bank_n[0:1],bank_ct[0:1];
    reg [15:0] bank_fsa[0:1],bank_span[0:1];
    wire [15:0] sample_word={byte_data,lo};
    wire packet_good=(checksum^sample_word)==received_checksum &&
        fsa[0] && lsa[0] && fsa[15:1]<15'd23040 && lsa[15:1]<15'd23040 &&
        (!ct[0] || n==1);
    localparam [3:0] E_IDLE=0,E_DIV=1,E_READ=2,E_LATCH=3,E_CORRECT=4,E_PUBLISH=5,E_WAIT=6;
    reg [3:0] emitter;
    reg emit_bank,preferred_bank;
    reg [7:0] emit_index,emit_n,denominator;
    reg [15:0] quotient,angle_q6,step_q6,step_remainder;
    reg [8:0] remainder,fraction;
    reg [4:0] div_bit;
    reg [15:0] sample_read,emit_distance;
    reg emit_range_valid,emit_no_return;
    reg [11:0] raw_angle_tenths;
    reg signed [13:0] corrected_angle;
    wire [8:0] trial={remainder[7:0],quotient[15]};
    wire [16:0] next_angle={1'b0,angle_q6}+step_q6+
        ((fraction+step_remainder)>=denominator ? 17'd1 : 17'd0);
    wire signed [13:0] normalized_angle=corrected_angle<0 ? corrected_angle+14'sd3600 :
        corrected_angle>=3600 ? corrected_angle-14'sd3600 : corrected_angle;
    wire [16:0] wrapped_next_angle=next_angle>=23040 ? next_angle-17'd23040 : next_angle;
    wire [15:0] next_fraction=fraction+step_remainder>=denominator ?
        fraction+step_remainder-denominator : fraction+step_remainder;
    wire signed [7:0] correction;
    x3pro_angle_correction u_correction(.clk(clk),.distance_mm(emit_distance),
        .correction_tenths(correction));
    always @(posedge clk) begin
        if(byte_valid && parser==D1 && !drop_packet)
            samples[{write_bank,sample_index}]<=sample_word;
        sample_read<=samples[{emit_bank,emit_index}];
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            acquisition_clock<=0;header_time<=0;previous_header<=0;sample_period<=CLK_FREQ_HZ/3000;
            timing_seen<=0;point_timestamp<=0;point_timestamp_valid<=0;
            period_start<=0;period_measured<=0;period_interval<=0;period_samples<=0;
            pwm_count<=0;packet_age<=STALE_CLKS;byte_age<=0;parser<=H0;
            pwm_high_clks<=PWM_CLKS*PWM_DUTY_PERCENT/100;
            write_bank<=0;drop_packet<=0;ready<=0;ct<=0;n<=0;lo<=0;sample_index<=0;
            fsa<=0;lsa<=0;checksum<=0;received_checksum<=0;
            bank_n[0]<=0;bank_n[1]<=0;bank_ct[0]<=0;bank_ct[1]<=0;
            bank_fsa[0]<=0;bank_fsa[1]<=0;bank_span[0]<=0;bank_span[1]<=0;
            emitter<=E_IDLE;emit_bank<=0;preferred_bank<=0;emit_index<=0;emit_n<=0;
            denominator<=1;quotient<=0;angle_q6<=0;step_q6<=0;step_remainder<=0;
            remainder<=0;fraction<=0;div_bit<=0;emit_distance<=0;emit_range_valid<=0;raw_angle_tenths<=0;
            corrected_angle<=0;point_valid<=0;distance_mm<=0;angle_tenths<=0;
            range_valid<=0;no_return<=0;emit_no_return<=0;scan_start<=0;fault<=0;good_packets<=0;bad_packets<=0;dropped_packets<=0;
        end else begin
            acquisition_clock<=acquisition_clock+1'b1;
            period_start<=0;
            if(period_done&&measured_period>=CLK_FREQ_HZ/10000&&measured_period<=CLK_FREQ_HZ/500)begin
                sample_period<=period_measured?(sample_period*3+measured_period)/4:measured_period;
                period_measured<=1;
            end
            scan_start<=0;
            if(pwm_count>=PWM_CLKS-1) begin
                pwm_count<=0;
                pwm_high_clks<=pwm_requested_high_clks;
            end else pwm_count<=pwm_count+1'b1;
            if(packet_age<STALE_CLKS) packet_age<=packet_age+1'b1;
            if(byte_valid) byte_age<=0;else if(byte_age<TIMEOUT_CLKS) byte_age<=byte_age+1'b1;
            if(framing_error || (parser!=H0 && byte_age>=TIMEOUT_CLKS && !byte_valid)) begin
                parser<=H0;fault<=framing_error ? 8'd2 : 8'd3;
                if(bad_packets!=16'hffff) bad_packets<=bad_packets+1'b1;
            end else if(byte_valid) case(parser)
                H0:if(byte_data==8'haa)begin parser<=H1;header_time<=acquisition_clock;end
                H1:if(byte_data==8'h55) parser<=CT;
                    else begin
                        parser<=byte_data==8'haa ? H1 : H0;
                        if(byte_data==8'haa)header_time<=acquisition_clock;
                    end
                CT:begin
                    ct<=byte_data;drop_packet<=ready==2'b11;
                    if(ready[write_bank] && !ready[!write_bank]) write_bank<=!write_bank;
                    parser<=N;
                end
                N:begin
                    n<=byte_data;checksum<=16'h55aa^{byte_data,ct};sample_index<=0;
                    if(byte_data==0) begin parser<=H0;fault<=4;bad_packets<=bad_packets+1'b1;end
                    else parser<=F0;
                end
                F0:begin lo<=byte_data;parser<=F1;end
                F1:begin fsa<={byte_data,lo};checksum<=checksum^{byte_data,lo};parser<=L0;end
                L0:begin lo<=byte_data;parser<=L1;end
                L1:begin lsa<={byte_data,lo};checksum<=checksum^{byte_data,lo};parser<=C0;end
                C0:begin lo<=byte_data;parser<=C1;end
                C1:begin received_checksum<={byte_data,lo};parser<=D0;end
                D0:begin lo<=byte_data;parser<=D1;end
                D1:begin
                    checksum<=checksum^sample_word;
                    if(sample_index==n-1'b1) begin
                        parser<=H0;
                        if(packet_good) begin
                            packet_age<=0;good_packets<=good_packets+1'b1;
                            previous_header<=header_time;timing_seen<=1;
                            if(timing_seen&&n>1&&header_time-previous_header<CLK_FREQ_HZ/10)begin
                                period_interval<=header_time-previous_header;period_samples<=n;period_start<=1;
                            end
                            if(drop_packet) begin fault<=5;dropped_packets<=dropped_packets+1'b1;end
                            else begin
                                fault<=0;ready[write_bank]<=1;bank_n[write_bank]<=n;bank_ct[write_bank]<=ct;
                                bank_time[write_bank]<=header_time-CLK_FREQ_HZ/1000000*SENSOR_LATENCY_US;
                                bank_period[write_bank]<=sample_period;
                                bank_timing_valid[write_bank]<=period_measured;
                                bank_fsa[write_bank]<={1'b0,fsa[15:1]};
                                bank_span[write_bank]<=lsa[15:1]>=fsa[15:1] ?
                                    lsa[15:1]-fsa[15:1] : 16'd23040+lsa[15:1]-fsa[15:1];
                                write_bank<=!write_bank;
                            end
                        end else begin fault<=1;bad_packets<=bad_packets+1'b1;end
                    end else begin sample_index<=sample_index+1'b1;parser<=D0;end
                end
                default:parser<=H0;
            endcase
            case(emitter)
                E_IDLE:if(ready!=0) begin
                    emit_bank<=ready[preferred_bank] ? preferred_bank : !preferred_bank;
                    emitter<=E_READ; // metadata latched on the next edge
                    emit_index<=0;
                end
                E_READ:begin
                    emit_n<=bank_n[emit_bank];angle_q6<=bank_fsa[emit_bank];fraction<=0;
                    denominator<=bank_n[emit_bank]>1 ? bank_n[emit_bank]-1'b1 : 8'd1;
                    quotient<=bank_n[emit_bank]>1 ? bank_span[emit_bank] : 16'd0;
                    remainder<=0;div_bit<=15;emitter<=E_DIV;
                    scan_start<=bank_ct[emit_bank][0];
                end
                E_DIV:begin
                    if(trial>=denominator) begin remainder<=trial-denominator;quotient<={quotient[14:0],1'b1};end
                    else begin remainder<=trial;quotient<={quotient[14:0],1'b0};end
                    if(div_bit==0) begin
                        step_q6<=trial>=denominator ? {quotient[14:0],1'b1} : {quotient[14:0],1'b0};
                        step_remainder<=trial>=denominator ? trial-denominator : trial;
                        emitter<=E_LATCH;
                    end else div_bit<=div_bit-1'b1;
                end
                E_LATCH:begin
                    emit_distance<={2'b0,sample_read[15:2]};
                    emit_range_valid<=sample_read[15:2]>=14'd100 && sample_read[15:2]<=14'd8000 &&
                        !sample_read[1]; // Flag 2: mirror reflection; flag 3: ambient light.
                    // A clean zero in a CRC-checked packet is distinct from a
                    // missing/interfered sample. It remains invalid for mapping.
                    emit_no_return<=sample_read[15:2]==0 && !sample_read[1];
                    raw_angle_tenths<=(angle_q6*17'd5+17'd16)>>5;
                    emitter<=E_CORRECT;
                end
                E_CORRECT:emitter<=E_PUBLISH; // synchronous correction ROM
                E_PUBLISH:begin
                    corrected_angle<=$signed({2'b0,raw_angle_tenths})+$signed(correction);
                    emitter<=E_WAIT;
                end
                E_WAIT:begin
                    if(!point_valid) begin
                        angle_tenths<=normalized_angle[11:0];
                        distance_mm<=emit_distance;range_valid<=emit_range_valid;no_return<=emit_no_return;
                        point_valid<=1;
                        point_timestamp<=bank_time[emit_bank]-(emit_n-1-emit_index)*bank_period[emit_bank];
                        point_timestamp_valid<=bank_timing_valid[emit_bank];
                    end else if(point_ready) begin
                        point_valid<=0;
                        if(emit_index==emit_n-1'b1) begin
                            ready[emit_bank]<=0;preferred_bank<=!emit_bank;emitter<=E_IDLE;
                        end else begin
                            emit_index<=emit_index+1'b1;
                            angle_q6<=wrapped_next_angle[15:0];
                            fraction<=next_fraction[8:0];
                            emitter<=7; // allow synchronous RAM one full edge at the new index
                        end
                    end
                end
                7:emitter<=E_LATCH;
                default:emitter<=E_IDLE;
            endcase
        end
    end
endmodule

// Thirty-two short subtract/shift steps replace a long combinational divider
// in the packet receiver's 50 MHz timing path.
module x3pro_period_divider(
    input wire clk,reset_n,start,input wire [31:0] interval_clks,
    input wire [7:0] sample_count,output reg done,output reg [31:0] quotient
);
    reg active;
    reg [31:0] dividend,result;
    reg [7:0] divisor,remainder;
    reg [5:0] step;
    wire [8:0] trial={remainder,dividend[31]};
    wire fits=trial>={1'b0,divisor};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin active<=0;done<=0;quotient<=0;dividend<=0;result<=0;divisor<=0;remainder<=0;step<=0;end
        else begin
            done<=0;
            if(start&&!active&&sample_count!=0)begin
                dividend<=interval_clks;divisor<=sample_count;result<=0;remainder<=0;step<=0;active<=1;
            end else if(active)begin
                dividend<={dividend[30:0],1'b0};result<={result[30:0],fits};
                remainder<=fits?trial-divisor:trial;
                if(step==31)begin quotient<={result[30:0],fits};done<=1;active<=0;end
                else step<=step+1'b1;
            end
        end
    end
endmodule
