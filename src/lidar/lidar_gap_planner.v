// ND-inspired finite-direction local steering, independently implemented RTL.
// Gap width + goal progress + previous-direction hysteresis; not full MRPT ND,
// SLAM, global planning or guaranteed return-to-line. No fixed reverse/offset leg.
module lidar_gap_planner #(
    parameter integer CLK_FREQ_HZ=50000000,MOTOR_SPEED_SCALE=2,
    parameter integer PLAN_PERIOD_MS=10,ACCEL_STEP_MS=80,
    parameter integer CHOICE_CLEARANCE_MM=250,STOP_CLEARANCE_MM=150,
    parameter integer FORWARD_PREFERENCE_MM=600,MAX_AUTO_SPEED=5
)(
    input wire clk,reset_n,enable,healthy,
    input wire [3:0] requested_motion,
    input wire [7:0] requested_speed,
    input wire [143:0] clearances_mm,input wire [8:0] direction_valid,
    output reg [3:0] motion,output reg [7:0] speed,
    output wire auto_active,output wire [3:0] selected_index,
    output reg [5:0] state,output reg [3:0] fault
);
    localparam integer PERIOD=(CLK_FREQ_HZ/1000*PLAN_PERIOD_MS<1)?1:CLK_FREQ_HZ/1000*PLAN_PERIOD_MS;
    localparam integer RAMP=(CLK_FREQ_HZ/1000*ACCEL_STEP_MS<1)?1:CLK_FREQ_HZ/1000*ACCEL_STEP_MS;
    reg [31:0] timer,ramp_timer;
    reg [1:0] phase;
    reg [3:0] index,chosen,best_index,left_count,right_count;
    reg [3:0] left_run[0:8];
    reg [143:0] snapshot;
    reg [8:0] valid_snapshot,eligible;
    reg [7:0] evaluated_speed,target_speed,command_speed;
    reg have_plan,best_found;
    reg [11:0] best_score;
    assign auto_active=enable && requested_motion==1 && requested_speed!=0;
    assign selected_index=chosen;
    wire [15:0] live_clearance=clearances_mm[chosen*16+:16];
    wire live_ok=healthy && have_plan && direction_valid[chosen] && live_clearance>STOP_CLEARANCE_MM;
    wire [15:0] candidate_clearance=snapshot[index*16+:16];
    wire ratio_direction=index==1 || index==3 || index==5 || index==7;
    wire candidate_ok=valid_snapshot[index] && candidate_clearance>=CHOICE_CLEARANCE_MM &&
        (!ratio_direction || evaluated_speed*MOTOR_SPEED_SCALE>=3);
    wire [3:0] deviation=index>4?index-4:4-index;
    wire [15:0] score_distance=candidate_clearance>1000?16'd1000:candidate_clearance;
    wire [11:0] score=(score_distance>>2)+(4-deviation)*40+
        (left_run[index]+right_count)*8+(chosen==index?60:0)+
        (index==4 && candidate_clearance>=FORWARD_PREFERENCE_MM?500:0);
    wire [7:0] bounded_request=requested_speed>MAX_AUTO_SPEED?MAX_AUTO_SPEED:requested_speed;
    wire [7:0] live_speed=command_speed>bounded_request?bounded_request:command_speed;
    wire live_ratio=chosen==1 || chosen==3 || chosen==5 || chosen==7;
    wire [7:0] selected_limit=snapshot[best_index*16+:16]>=800?bounded_request:
        snapshot[best_index*16+:16]>=450?(bounded_request>3?8'd3:bounded_request):
        bounded_request>2?8'd2:bounded_request;
    integer k;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            phase<=0;timer<=0;index<=0;chosen<=4;best_index<=4;left_count<=0;right_count<=0;
            snapshot<=0;valid_snapshot<=0;eligible<=0;evaluated_speed<=0;target_speed<=0;
            have_plan<=0;best_found<=0;best_score<=0;
            for(k=0;k<9;k=k+1)left_run[k]<=0;
        end else if(!auto_active || !healthy)begin
            phase<=0;timer<=PERIOD;chosen<=4;have_plan<=0;target_speed<=0;
        end else begin
            if(timer<PERIOD)timer<=timer+1'b1;
            case(phase)
                0:if(timer>=PERIOD-1)begin
                    snapshot<=clearances_mm;valid_snapshot<=direction_valid;eligible<=0;
                    evaluated_speed<=command_speed!=0?command_speed:bounded_request>=2?8'd2:bounded_request;
                    index<=0;left_count<=0;timer<=0;phase<=1;
                end
                1:begin
                    eligible[index]<=candidate_ok;
                    left_run[index]<=candidate_ok?left_count+1'b1:4'd0;
                    left_count<=candidate_ok?left_count+1'b1:4'd0;
                    if(index==8)begin index<=8;right_count<=0;best_found<=0;best_score<=0;phase<=2;end
                    else index<=index+1'b1;
                end
                2:begin
                    right_count<=eligible[index]?right_count+1'b1:4'd0;
                    // Previous direction wins through hysteresis; otherwise
                    // an exact symmetric tie keeps the old right preference.
                    if(eligible[index] && (!best_found || score>best_score ||
                            (score==best_score && index<best_index)))begin
                        best_found<=1;best_score<=score;best_index<=index;
                    end
                    if(index==0)phase<=3;else index<=index-1'b1;
                end
                3:begin
                    have_plan<=best_found;
                    if(best_found)begin chosen<=best_index;target_speed<=selected_limit;end
                    else target_speed<=0;
                    phase<=0;
                end
            endcase
        end
    end
    // Acceleration is slewed; reductions and emergency STOP are never delayed
    // by the ramp. The final live-corridor guard remains after this planner.
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin command_speed<=0;ramp_timer<=0;end
        else if(!auto_active || !healthy || !have_plan)begin command_speed<=0;ramp_timer<=0;end
        else if(target_speed<command_speed)begin command_speed<=target_speed;ramp_timer<=0;end
        else if(command_speed==0)begin command_speed<=target_speed>2?8'd2:target_speed;ramp_timer<=0;end
        else if(command_speed<target_speed)begin
            if(ramp_timer>=RAMP-1)begin command_speed<=command_speed+1'b1;ramp_timer<=0;end
            else ramp_timer<=ramp_timer+1'b1;
        end else ramp_timer<=0;
    end
    always @*begin
        motion=requested_motion;speed=requested_motion==0?8'd0:requested_speed;state=0;fault=0;
        if(auto_active)begin
            motion=0;speed=0;
            if(live_ok && live_speed!=0 && (!live_ratio || live_speed*MOTOR_SPEED_SCALE>=3))begin
                case(chosen)
                    0:motion=4;1,2,3:motion=10;4:motion=1;5,6,7:motion=9;8:motion=3;
                    default:motion=0;
                endcase
                speed=live_speed;state=chosen==4?6'd0:6'd35;
            end else begin
                state=7;fault=!healthy || direction_valid==0?4'd14:4'd9;
            end
        end
    end
endmodule
