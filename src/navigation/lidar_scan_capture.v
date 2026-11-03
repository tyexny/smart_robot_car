// Manual park-and-scan mapping. Only rescan opens a one-revolution capture
// window when MANUAL_SCAN is enabled. Motion invalidates an active projection; a fresh
// STOP acknowledgement, zero wheel speeds and stable yaw are required before
// accepting a NEW point. Pose and yaw are latched together for each ray.
// Manual travel never clears the map or rebases the IMU/encoder origin.
// X3 Pro angle is clockwise; map yaw is counterclockwise (+Y is left).
module lidar_scan_capture #(
    parameter integer CLK_FREQ_HZ=50000000, SETTLE_MS=200, STALE_MS=500,
    parameter integer SENSOR_FORWARD_MM=0,SENSOR_LEFT_MM=0,SENSOR_YAW_TENTHS=0,
    parameter integer MANUAL_SCAN=0
)(
    input wire clk,reset_n,enable,restart,rescan,sample_allow,
    input wire cancel_scan,
    input wire signed [31:0] yaw_tenths,
    input wire yaw_valid,yaw_update,
    input wire motor_online,motor_update,stop_confirmed,
    input wire [63:0] motor_speeds,
    input wire [3:0] motion,
    input wire pose_valid,pose_fault,
    input wire signed [15:0] pose_x_mm,pose_y_mm,
    input wire lidar_online,point_valid,range_valid,scan_start,
    input wire [15:0] distance_mm,
    input wire [11:0] angle_tenths,
    output wire point_ready,
    input wire map_busy,map_clear_done,
    output reg map_clear,ray_update,ray_valid,
    output reg signed [31:0] ray_start_x_mm,ray_start_y_mm,ray_end_x_mm,ray_end_y_mm,
    output wire signed [31:0] heading_tenths,
    output wire heading_valid,
    output reg [3:0] status,
    output reg [15:0] accepted_samples
);
    localparam integer SETTLE_CLKS=CLK_FREQ_HZ/1000*SETTLE_MS;
    localparam integer STALE_CLKS=CLK_FREQ_HZ/1000*STALE_MS;
    reg previous_enable,origin_valid,clear_pending,new_map_pending,lap_started,lap_done;
    reg scan_requested,scan_active,scan_finishing;
    reg signed [31:0] yaw_origin,stable_yaw;
    reg [31:0] yaw_age,motor_age,settle;
    reg [15:0] lap_samples;
    wire signed [31:0] delta=yaw_tenths-yaw_origin;
    assign heading_tenths=delta>1800 ? delta-3600 : delta < -1800 ? delta+3600 : delta;
    assign heading_valid=enable && origin_valid && yaw_valid && yaw_age<STALE_CLKS;
    wire signed [31:0] stable_delta=yaw_tenths-stable_yaw;
    wire signed [31:0] stable_error=stable_delta>1800 ? stable_delta-3600 :
        stable_delta < -1800 ? stable_delta+3600 : stable_delta;
    wire stopped=stop_confirmed && motor_online && motor_speeds==0 && motion==0 && motor_age<STALE_CLKS;
    wire stability_valid=new_map_pending ? (yaw_valid && yaw_age<STALE_CLKS) : heading_valid;
    wire yaw_unstable=yaw_update && (stable_error>5 || stable_error < -5);
    wire base_allowed=enable && !new_map_pending && !clear_pending && !map_clear && sample_allow && stopped &&
        heading_valid && pose_valid && !pose_fault && lidar_online && settle>=SETTLE_CLKS &&
        (MANUAL_SCAN==0 || !yaw_unstable);
    // Closing a lap stops NEW points while the last latched ray drains.
    wire allowed=base_allowed && (MANUAL_SCAN==0 || scan_requested);
    wire accepting=base_allowed && (MANUAL_SCAN==0 ||
        (scan_requested && !scan_finishing && (scan_active || scan_start) &&
        !(scan_active && scan_start)));
    wire manual_cancel=MANUAL_SCAN!=0 && cancel_scan;
    reg [2:0] phase;
    reg trig_start;
    reg signed [12:0] ray_heading,body_heading;
    reg signed [15:0] snapshot_x,snapshot_y;
    reg [15:0] snapshot_distance;
    reg signed [31:0] range_product_x,range_product_y,offset_product_x,offset_product_y;
    wire signed [15:0] ray_cos,ray_sin,body_cos,body_sin;
    wire ray_trig_done,body_trig_done;
    wire cancel=!allowed || restart || rescan || manual_cancel;
    heading_lut_fine_pipeline u_ray_trig(.clk(clk),.reset_n(reset_n),.start(trig_start),
        .cancel(cancel),.angle_tenths(ray_heading),.done(ray_trig_done),.cos_q14(ray_cos),.sin_q14(ray_sin));
    heading_lut_fine_pipeline u_body_trig(.clk(clk),.reset_n(reset_n),.start(trig_start),
        .cancel(cancel),.angle_tenths(body_heading),.done(body_trig_done),.cos_q14(body_cos),.sin_q14(body_sin));
    wire signed [31:0] new_ray_heading=heading_tenths+SENSOR_YAW_TENTHS-$signed({1'b0,angle_tenths});
    wire signed [31:0] wrapped_ray_heading=new_ray_heading>1800 ? new_ray_heading-3600 :
        new_ray_heading < -1800 ? new_ray_heading+3600 : new_ray_heading;
    // Drain checked points during motion, so stopping cannot replay a backlog.
    assign point_ready=phase==0 && (!accepting || !map_busy);
    always @* begin
        if(!enable) status=0;
        else if(clear_pending || map_clear) status=9;
        else if(pose_fault) status=8;
        else if(!lidar_online) status=1;
        else if(!heading_valid) status=2;
        else if(!motor_online || motor_age>=STALE_CLKS) status=3;
        else if(MANUAL_SCAN!=0 && !scan_requested) status=lap_done ? 12 : 7;
        else if(!base_allowed) status=4;
        else if(MANUAL_SCAN!=0 && !scan_active && !scan_finishing) status=7;
        else status=lap_done ? 12 : 5;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            previous_enable<=0;origin_valid<=0;clear_pending<=0;new_map_pending<=0;lap_started<=0;lap_done<=0;
            scan_requested<=0;scan_active<=0;scan_finishing<=0;
            yaw_origin<=0;stable_yaw<=0;yaw_age<=STALE_CLKS;motor_age<=STALE_CLKS;
            settle<=0;lap_samples<=0;accepted_samples<=0;phase<=0;trig_start<=0;
            ray_heading<=0;body_heading<=0;snapshot_x<=0;snapshot_y<=0;snapshot_distance<=0;
            range_product_x<=0;range_product_y<=0;offset_product_x<=0;offset_product_y<=0;
            map_clear<=0;ray_update<=0;ray_valid<=0;
            ray_start_x_mm<=0;ray_start_y_mm<=0;ray_end_x_mm<=0;ray_end_y_mm<=0;
        end else begin
            previous_enable<=enable;map_clear<=0;ray_update<=0;ray_valid<=0;trig_start<=0;
            if(yaw_age<STALE_CLKS) yaw_age<=yaw_age+1'b1;
            if(motor_age<STALE_CLKS) motor_age<=motor_age+1'b1;
            if(yaw_update && yaw_valid) begin
                yaw_age<=0;
            end
            if(motor_update) motor_age<=0;
            if(map_clear_done && !new_map_pending) clear_pending<=0;
            if(!enable || !previous_enable || restart) begin
                phase<=0;settle<=0;origin_valid<=0;lap_started<=0;lap_done<=0;lap_samples<=0;
                scan_requested<=MANUAL_SCAN!=0 && enable && rescan;
                scan_active<=0;scan_finishing<=0;
                yaw_age<=STALE_CLKS;motor_age<=STALE_CLKS;accepted_samples<=0;
                stable_yaw<=yaw_tenths;
                if(enable) begin new_map_pending<=1;clear_pending<=1;end
                else begin new_map_pending<=0;clear_pending<=0;end
            end else begin
                if(!stopped || !stability_valid || (!sample_allow && !new_map_pending) ||
                    (pose_fault && !new_map_pending) || rescan ||
                    (yaw_update && (stable_error>5 || stable_error < -5))) begin
                    settle<=0;stable_yaw<=yaw_tenths;lap_started<=0;lap_done<=0;lap_samples<=0;
                end else if(settle<SETTLE_CLKS) settle<=settle+1'b1;
                // Rebase only AFTER the physical STOP and yaw settling. This
                // excludes pre-stop coast from a newly requested origin.
                if(new_map_pending && stopped && stability_valid && !yaw_unstable && settle>=SETTLE_CLKS) begin
                    map_clear<=1;new_map_pending<=0;clear_pending<=1;
                    yaw_origin<=yaw_tenths;origin_valid<=1;accepted_samples<=0;
                end
                if(MANUAL_SCAN!=0) begin
                    if(manual_cancel) begin
                        scan_requested<=0;scan_active<=0;scan_finishing<=0;
                        lap_started<=0;lap_done<=0;lap_samples<=0;
                    end else if(rescan) begin
                        // Keep this request while the commanded STOP is still
                        // coasting, then wait for a fresh full-lap boundary.
                        scan_requested<=1;scan_active<=0;scan_finishing<=0;
                        lap_started<=0;lap_done<=0;lap_samples<=0;
                    end else if((scan_active || scan_finishing) && !base_allowed) begin
                        // An interrupted scan needs another explicit request.
                        scan_requested<=0;scan_active<=0;scan_finishing<=0;
                        lap_started<=0;lap_done<=0;lap_samples<=0;
                    end else if(scan_requested && base_allowed) begin
                        if(scan_finishing && phase==0 && !ray_update && !map_busy) begin
                            scan_requested<=0;scan_active<=0;scan_finishing<=0;
                            lap_done<=1;
                        end else if(scan_start && !scan_finishing) begin
                            if(scan_active) begin
                                scan_active<=0;scan_finishing<=1;
                            end else begin
                                scan_active<=1;lap_started<=1;lap_done<=0;lap_samples<=0;
                            end
                        end
                    end
                end else if(allowed && scan_start) begin
                    if(lap_started && lap_samples!=0) lap_done<=1;
                    lap_started<=1;lap_samples<=0;
                end
                if(cancel) phase<=0;
                else case(phase)
                    0:if(accepting && point_valid && point_ready && range_valid) begin
                        snapshot_x<=pose_x_mm;snapshot_y<=pose_y_mm;snapshot_distance<=distance_mm;
                        body_heading<=heading_tenths[12:0];
                        ray_heading<=wrapped_ray_heading[12:0];
                        phase<=1;
                    end
                    1:begin trig_start<=1;phase<=2;end
                    2:if(ray_trig_done && body_trig_done) begin
                        range_product_x<=$signed({1'b0,snapshot_distance})*ray_cos;
                        range_product_y<=$signed({1'b0,snapshot_distance})*ray_sin;
                        offset_product_x<=SENSOR_FORWARD_MM*body_cos-SENSOR_LEFT_MM*body_sin;
                        offset_product_y<=SENSOR_FORWARD_MM*body_sin+SENSOR_LEFT_MM*body_cos;phase<=3;
                    end
                    3:begin
                        ray_start_x_mm<=snapshot_x+(offset_product_x>>>14);
                        ray_start_y_mm<=snapshot_y+(offset_product_y>>>14);
                        ray_end_x_mm<=snapshot_x+(offset_product_x>>>14)+(range_product_x>>>14);
                        ray_end_y_mm<=snapshot_y+(offset_product_y>>>14)+(range_product_y>>>14);phase<=4;
                    end
                    4:if(!map_busy) begin
                        ray_update<=1;ray_valid<=1;phase<=5;
                        if(accepted_samples!=16'hffff) accepted_samples<=accepted_samples+1'b1;
                        if(lap_samples!=16'hffff) lap_samples<=lap_samples+1'b1;
                    end
                    5:phase<=0;
                    default:phase<=0;
                endcase
            end
        end
    end
endmodule
