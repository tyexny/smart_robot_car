// Last authority on physical motion, after manual/follow/navigation arbitration.
// Thresholds are provisional low-speed margins, NOT a calibrated brake model.
// Speed grades are not m/s. Recalibrate against measured speed/stopping distance.
module lidar_motion_guard #(
    parameter integer STOP_CM=15,SLOW_CM=35,SLOW_SPEED=3,
    parameter integer EXTRA_STOP_CM_PER_GRADE=3,
    parameter integer SPEED_GRADE_MAX=255
)(
    input wire [3:0] motion,input wire [7:0] speed,
    input wire rotation_request,
    input wire healthy,front_valid,rear_valid,left_valid,right_valid,rotation_valid,
    input wire [15:0] front_cm,rear_cm,left_cm,right_cm,rotation_cm,
    output reg blocked,slowdown,unknown,
    output wire [3:0] safe_motion,output wire [7:0] safe_speed
);
    reg required_valid;
    reg [15:0] nearest;
    function [15:0] stop_at_grade;
        input [7:0] grade;integer k;
        begin
            if(SPEED_GRADE_MAX<=10)begin
                stop_at_grade=16'hffff;
                for(k=0;k<=10;k=k+1)if(k<=SPEED_GRADE_MAX&&grade==k)
                    stop_at_grade=STOP_CM+(k>SLOW_SPEED?(k-SLOW_SPEED)*EXTRA_STOP_CM_PER_GRADE:0);
            end else stop_at_grade=STOP_CM+(grade>SLOW_SPEED?(grade-SLOW_SPEED)*EXTRA_STOP_CM_PER_GRADE:0);
        end
    endfunction
    wire [15:0] stop_limit=stop_at_grade(speed);
    wire [15:0] slow_limit=stop_limit+(SLOW_CM-STOP_CM);
    always @* begin
        nearest=16'hffff;required_valid=healthy;
        case(motion)
            1,7,8:begin nearest=front_cm;required_valid=required_valid && front_valid;end
            2:begin nearest=rear_cm;required_valid=required_valid && rear_valid;end
            3:begin nearest=left_cm;required_valid=required_valid && left_valid;end
            4:begin nearest=right_cm;required_valid=required_valid && right_valid;end
            5,6:begin nearest=rotation_cm;required_valid=required_valid && rotation_valid;end
            0:begin end
            default:required_valid=0;
        endcase
        // Reverse + yaw vector commands must check the sweep too, not just rear.
        if(rotation_request || motion==5 || motion==6 || motion==7 || motion==8) begin
            required_valid=required_valid && rotation_valid;
            if(rotation_cm<nearest) nearest=rotation_cm;
        end
        unknown=motion!=0 && !required_valid;
        blocked=motion!=0 && (!required_valid || nearest<=stop_limit);
        slowdown=motion!=0 && !blocked && nearest<=slow_limit;
    end
    assign safe_motion=blocked?4'd0:motion;
    assign safe_speed=blocked || motion==0?8'd0:slowdown && speed>SLOW_SPEED?SLOW_SPEED:speed;
endmodule
