// Map-relative odometry. 90 mm wheels, 390 counts/revolution, four wheels:
// sum(abs(delta)) * 742 / 4096 mm. In-place turns never become translation.
// Lateral travel can use an independently measured mm/count scale.
// The last translation direction survives STOP so its final coast is counted.
module navigation_pose #(
    parameter integer CLK_FREQ_HZ=50000000, STALE_MS=500, MAX_WHEEL_STEP=2048,
    parameter integer CELL_SHIFT=4,
    parameter integer CELL_MM=(1<<CELL_SHIFT), GRID_SIZE=128, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y), MM_PER_SUM_COUNT_Q12=742,
    parameter integer ODOM_LIMIT_MM=GRID_SIZE*CELL_MM/2,
    parameter integer STRAFE_MM_PER_SUM_COUNT_Q12=MM_PER_SUM_COUNT_Q12,
    parameter integer ENABLE_ARC_MIDPOINT=0, ENABLE_TRANSLATION_MIDPOINT=0
)(
    input wire clk,reset_n,enable,rebase,
    input wire signed [31:0] heading_tenths,
    input wire heading_valid,yaw_update,
    input wire [63:0] counts,speeds,
    input wire motor_update,motor_online,
    input wire [3:0] motion,
    output wire valid,
    output reg fault,moved,
    output wire signed [15:0] x_mm,y_mm,
    output wire [GRID_BITS-1:0] cell_x,cell_y
);

    localparam integer MAP_HALF_X_MM=GRID_X*CELL_MM/2, MAP_HALF_Y_MM=GRID_Y*CELL_MM/2;
    localparam integer ORIGIN_BIAS_X_MM=(GRID_X%2)*(CELL_MM/2), ORIGIN_BIAS_Y_MM=(GRID_Y%2)*(CELL_MM/2);
    localparam integer GRID_RECIPROCAL_SHIFT=CELL_MM==40?21:20;
    localparam [15:0] GRID_RECIPROCAL_MULTIPLIER=CELL_MM==30?16'd34953:16'd52429;
    // Signed floor division: -1 mm belongs to the cell left of the origin.
    function signed [31:0] grid_floor;
        input signed [31:0] v;
        reg [31:0] magnitude,quotient;
        reg [63:0] product;
        begin
            if(CELL_MM==(1<<CELL_SHIFT))grid_floor=v>>>CELL_SHIFT;
            else if(CELL_MM==20||CELL_MM==30||CELL_MM==40)begin
                magnitude=v[31]?-v:v;
                product=magnitude*(CELL_MM==30?64'd2290649225:64'd3435973837);
                quotient=product>>(CELL_MM==40?37:36);
                grid_floor=v[31]?-(quotient+(magnitude!=quotient*CELL_MM)):(quotient);
            end else grid_floor=v>=0?v/CELL_MM:-((-v+CELL_MM-1)/CELL_MM);
        end
    endfunction
    localparam integer STALE_CLKS=CLK_FREQ_HZ/1000*STALE_MS;
    reg baseline,yaw_seen,previous_enable;
    reg [63:0] previous_counts;
    reg [31:0] motor_age,yaw_age;
    reg signed [31:0] x_q12,y_q12;
    reg [3:0] coast_motion,step_motion;
    reg signed [12:0] previous_feedback_heading,step_start_heading;
    reg previous_feedback_heading_valid;
    reg [3:0] previous_feedback_motion;
    reg signed [13:0] step_heading_delta;
    reg signed [12:0] step_midpoint_heading;
    reg [2:0] phase;
    reg [17:0] step_sum;
    reg signed [12:0] step_heading;
    reg trig_start;
    reg signed [15:0] step_cos,step_sin;
    reg signed [31:0] distance_q12;
    reg signed [47:0] product_x,product_y;
    wire signed [15:0] cos_q14,sin_q14;
    wire trig_done;
    heading_lut_pipeline u_trig(.clk(clk),.reset_n(reset_n),.start(trig_start),
        .cancel(!enable || !previous_enable || rebase),.angle_tenths(step_heading),
        .done(trig_done),.cos_q14(cos_q14),.sin_q14(sin_q14));
    wire signed [15:0] da=counts[63:48]-previous_counts[63:48];
    wire signed [15:0] db=counts[47:32]-previous_counts[47:32];
    wire signed [15:0] dc=counts[31:16]-previous_counts[31:16];
    wire signed [15:0] dd=counts[15:0]-previous_counts[15:0];
    function [15:0] magnitude;
        input signed [15:0] v;
        begin magnitude=v<0 ? -v : v;end
    endfunction
    wire [17:0] delta_sum={2'b0,magnitude(da)}+{2'b0,magnitude(db)}+
                          {2'b0,magnitude(dc)}+{2'b0,magnitude(dd)};
    wire jump=magnitude(da)>MAX_WHEEL_STEP || magnitude(db)>MAX_WHEEL_STEP ||
              magnitude(dc)>MAX_WHEEL_STEP || magnitude(dd)>MAX_WHEEL_STEP;
    wire translating=motion==1 || motion==2 || motion==3 || motion==4 || motion==7 || motion==8;
    wire [3:0] interval_motion=translating ? motion : motion==0 ? coast_motion : 4'd0;
    // These are IMU headings sampled at motor-feedback endpoints, without
    // timestamps or an executed-motion acknowledgement. Their wrapped midpoint
    // estimates the projection heading; it is not exact time interpolation.
    // Require matching arc requests at both endpoints: FWD/turn/other-arc ->
    // arc uses the legacy end heading on its first frame. An arc's STOP coast
    // retains its effective arc motion, including the final zero-speed frame.
    wire feedback_heading_fresh=heading_valid &&
        (yaw_update || (yaw_seen && yaw_age<STALE_CLKS)) && motor_online;
    wire use_arc_midpoint=ENABLE_ARC_MIDPOINT &&
        (interval_motion==7 || interval_motion==8 ||
         (ENABLE_TRANSLATION_MIDPOINT && interval_motion>=1 && interval_motion<=4)) &&
        previous_feedback_motion==interval_motion &&
        previous_feedback_heading_valid && feedback_heading_fresh;
    // Sample the heading subtraction whenever the local datapath is idle.
    // Accepted feedback still selects phase5 on the SAME edge; its delta is
    // therefore identical, without the long IR->motion->midpoint CE cone.
    // A phase5 wrap on a cancel edge is harmless: cancellation clears phase
    // and the next accepted idle sample overwrites this private operand.
    // Neither live IR/motion nor the late owner-enable cone gates this bank.
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)step_heading_delta<=0;
        else if(phase==0)
            step_heading_delta<=$signed({heading_tenths[12],heading_tenths[12:0]})-
                $signed({previous_feedback_heading[12],previous_feedback_heading});
        else if(phase==5)begin
            if(step_heading_delta>14'sd1800)step_heading_delta<=step_heading_delta-14'sd3600;
            else if(step_heading_delta< -14'sd1800)step_heading_delta<=step_heading_delta+14'sd3600;
        end
    end
    // Capture the idle feedback endpoint alongside its heading difference.
    // Acceptance reads it only after entering phase5; live motion/IR decoding
    // must not drive this 13-bit snapshot's clock enable.
    always @(posedge clk or negedge reset_n)
        if(!reset_n)step_start_heading<=0;
        else if(phase==0)step_start_heading<=previous_feedback_heading;
    assign x_mm=x_q12>>>12;
    assign y_mm=y_q12>>>12;
    wire signed [15:0] grid_x=grid_floor(x_mm+ORIGIN_BIAS_X_MM)+(GRID_X/2);
    wire signed [15:0] grid_y=grid_floor(y_mm+ORIGIN_BIAS_Y_MM)+(GRID_Y/2);
    assign cell_x=grid_x[GRID_BITS-1:0];
    assign cell_y=grid_y[GRID_BITS-1:0];
    assign valid=enable && baseline && yaw_seen && heading_valid && motor_online &&
        motor_age<STALE_CLKS && yaw_age<STALE_CLKS && !fault &&
        x_mm>=-ODOM_LIMIT_MM && x_mm<ODOM_LIMIT_MM && y_mm>=-ODOM_LIMIT_MM && y_mm<ODOM_LIMIT_MM;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            baseline<=0;yaw_seen<=0;previous_enable<=0;previous_counts<=0;
            motor_age<=STALE_CLKS;yaw_age<=STALE_CLKS;x_q12<=0;y_q12<=0;
            coast_motion<=0;step_motion<=0;phase<=0;step_sum<=0;step_heading<=0;trig_start<=0;
            previous_feedback_heading<=0;previous_feedback_heading_valid<=0;
            previous_feedback_motion<=0;
            step_midpoint_heading<=0;
            step_cos<=0;step_sin<=0;distance_q12<=0;product_x<=0;product_y<=0;
            fault<=0;moved<=0;
        end else begin
            previous_enable<=enable;
            trig_start<=0;
            if(!enable || !previous_enable || rebase) begin
                baseline<=0;yaw_seen<=0;motor_age<=STALE_CLKS;yaw_age<=STALE_CLKS;
                x_q12<=0;y_q12<=0;coast_motion<=0;phase<=0;fault<=0;moved<=0;
                previous_feedback_heading<=0;previous_feedback_heading_valid<=0;
                previous_feedback_motion<=0;
            end else begin
                if(motor_age<STALE_CLKS) motor_age<=motor_age+1'b1;
                if(yaw_age<STALE_CLKS) yaw_age<=yaw_age+1'b1;
                if(yaw_update && heading_valid) begin yaw_seen<=1;yaw_age<=0;end
                // Do not bridge a heading/motor-health loss with an old endpoint.
                if(!feedback_heading_fresh) previous_feedback_heading_valid<=0;
                if(motion!=0) coast_motion<=translating ? motion : 4'd0;
                if(motor_update) begin
                    previous_counts<=counts;motor_age<=0;
                    // Refresh on every feedback, including baseline, rotate and
                    // STOP. A zero-delta frame still establishes a new endpoint.
                    previous_feedback_heading<=heading_tenths[12:0];
                    previous_feedback_heading_valid<=feedback_heading_fresh;
                    previous_feedback_motion<=interval_motion;
                    if(!baseline) baseline<=1;
                    else if(jump || motor_age>=STALE_CLKS) fault<=1;
                    else if(interval_motion!=0 && delta_sum!=0) begin
                        if(phase!=0 || !heading_valid) fault<=1;
                        else begin
                            step_sum<=delta_sum;step_motion<=interval_motion;
                            step_heading<=heading_tenths[12:0];moved<=1;
                            if(use_arc_midpoint) begin
                                // Subtract on the sampling edge; wrap, midpoint
                                // addition and final wrap each have their own edge.
                                phase<=5;
                            end else phase<=1;
                        end
                    end
                    if(motion==0 && speeds==0) coast_motion<=0;
                end
                if(baseline && motor_age>=STALE_CLKS && !motor_update) fault<=1;
                case(phase)
                    5:begin
                        phase<=6;
                    end
                    6:begin
                        // Arithmetic half rounds an odd negative tenth toward
                        // -infinity; midpoint quantization is below 0.1 degree.
                        step_midpoint_heading<=
                            $signed({step_start_heading[12],step_start_heading})+
                            (step_heading_delta>>>1);
                        phase<=7;
                    end
                    7:begin
                        if(step_midpoint_heading>13'sd1800)
                            step_heading<=step_midpoint_heading-13'sd3600;
                        else if(step_midpoint_heading< -13'sd1800)
                            step_heading<=step_midpoint_heading+13'sd3600;
                        else step_heading<=step_midpoint_heading;
                        phase<=1;
                    end
                    1:begin
                        trig_start<=1;
                        // Use the accepted interval's motion, including STOP
                        // coast, rather than a later live direction command.
                        distance_q12<=(step_motion==3||step_motion==4) ?
                            step_sum*STRAFE_MM_PER_SUM_COUNT_Q12 : step_sum*MM_PER_SUM_COUNT_Q12;
                        phase<=2;
                    end
                    2:if(trig_done) begin step_cos<=cos_q14;step_sin<=sin_q14;phase<=3;end
                    3:begin
                        case(step_motion)
                            2:begin product_x<=-distance_q12*step_cos;product_y<=-distance_q12*step_sin;end
                            3:begin product_x<=-distance_q12*step_sin;product_y<=distance_q12*step_cos;end
                            4:begin product_x<=distance_q12*step_sin;product_y<=-distance_q12*step_cos;end
                            default:begin product_x<=distance_q12*step_cos;product_y<=distance_q12*step_sin;end
                        endcase
                        phase<=4;
                    end
                    4:begin x_q12<=x_q12+(product_x>>>14);y_q12<=y_q12+(product_y>>>14);phase<=0;end
                    default:begin end
                endcase
            end
        end
    end
endmodule
