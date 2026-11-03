// Continuous per-point projection. Origin establishment still requires a
// stationary, fresh-telemetry interval; later scans may be collected in motion.
// Rays use raw odometry interpolated at the driver's estimated acquisition
// time. Packet transport/IMU latency still requires physical calibration;
// the sensor does not provide a hardware acquisition timestamp.
module continuous_lidar_capture #(
    parameter integer CLK_FREQ_HZ=50000000,SETTLE_MS=200,STALE_MS=500,
    parameter integer SENSOR_FORWARD_MM=0,SENSOR_LEFT_MM=0,SENSOR_YAW_TENTHS=0,
    parameter integer QUIET_SPEED=2
)(
    input wire clk,reset_n,enable,restart,rescan,sample_allow,
    input wire signed [31:0] yaw_tenths,
    input wire yaw_valid,yaw_update,motor_online,motor_update,stop_confirmed,
    input wire [63:0] motor_speeds,input wire [3:0] motion,
    input wire pose_valid,pose_fault,
    input wire signed [15:0] pose_x_mm,pose_y_mm,
    input wire signed [31:0] projection_heading_tenths,
    input wire lidar_online,point_valid,range_valid,scan_start,
    input wire [15:0] distance_mm,input wire [11:0] angle_tenths,
    input wire [31:0] point_timestamp,input wire point_timestamp_valid,
    output wire point_ready,
    input wire map_clear_done,
    output reg map_clear,
    output wire signed [31:0] odom_heading_tenths,
    output wire odom_heading_valid,capture_healthy,
    output reg ray_update,ray_frame_start,
    output reg signed [31:0] ray_start_x_mm,ray_start_y_mm,ray_end_x_mm,ray_end_y_mm,
    output reg [6:0] ray_bin,
    output reg [3:0] ray_bin_error,
    output reg [31:0] ray_timestamp,
    output reg signed [31:0] ray_body_heading_tenths,
    output reg [3:0] status
);
    localparam integer STALE_CLKS=CLK_FREQ_HZ/1000*STALE_MS;
    localparam integer SETTLE_CLKS=CLK_FREQ_HZ/1000*SETTLE_MS;
    reg previous_enable,origin_valid,origin_pending,clear_pending,first_pending;
    reg signed [12:0] yaw_origin;
    reg signed [31:0] stable_yaw;
    reg [31:0] yaw_age,motor_age,settle,timestamp;
    wire signed [13:0] relative_yaw=$signed(yaw_tenths[12:0])-$signed(yaw_origin);
    wire signed [31:0] stable_delta=yaw_tenths-stable_yaw;
    wire signed [31:0] stable_error=stable_delta>1800 ? stable_delta-3600 :
        stable_delta< -1800 ? stable_delta+3600 : stable_delta;
    wire signed [13:0] wrapped_odom_heading=relative_yaw>14'sd1800 ? relative_yaw-14'sd3600 :
        relative_yaw< -14'sd1800 ? relative_yaw+14'sd3600 : relative_yaw;
    assign odom_heading_tenths=$signed(wrapped_odom_heading);
    assign odom_heading_valid=enable && origin_valid && yaw_valid && yaw_age<STALE_CLKS;
    function quiet;
        input signed [15:0] speed;
        begin quiet=speed>=-QUIET_SPEED && speed<=QUIET_SPEED;end
    endfunction
    wire stopped=motor_online && motor_age<STALE_CLKS && motion==0 && stop_confirmed &&
        quiet(motor_speeds[63:48]) && quiet(motor_speeds[47:32]) &&
        quiet(motor_speeds[31:16]) && quiet(motor_speeds[15:0]);
    wire yaw_stable=yaw_valid && yaw_age<STALE_CLKS && !(yaw_update && (stable_error>5 || stable_error< -5));
    assign capture_healthy=enable && !origin_pending && !clear_pending && odom_heading_valid &&
        motor_online && motor_age<STALE_CLKS && lidar_online;
    wire allowed=capture_healthy && pose_valid && !pose_fault && sample_allow;
    reg [2:0] phase;
    reg history_start;
    wire history_done,history_valid;
    wire signed [15:0] history_x,history_y;
    wire signed [12:0] history_yaw;
    wire signed [14:0] history_ray_heading=$signed(history_yaw)+SENSOR_YAW_TENTHS-$signed({1'b0,snapshot_angle});
    wire signed [14:0] wrapped_history_heading=history_ray_heading>15'sd1800 ? history_ray_heading-15'sd3600 :
        history_ray_heading< -15'sd1800 ? history_ray_heading+15'sd3600 : history_ray_heading;
    slam_pose_history #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) pose_history(
        .clk(clk),.reset_n(reset_n),.clear(!enable||restart||rescan||!sample_allow||!capture_healthy||!pose_valid||pose_fault),.pose_valid(pose_valid&&!pose_fault),
        .pose_x(pose_x_mm),.pose_y(pose_y_mm),.pose_yaw(projection_heading_tenths),
        .query_start(history_start),.query_time(point_timestamp),.done(history_done),.valid(history_valid),
        .x_mm(history_x),.y_mm(history_y),.yaw_tenths(history_yaw));
    reg trig_start,snapshot_first;
    reg signed [12:0] snapshot_body_heading,snapshot_ray_heading;
    reg signed [15:0] snapshot_x,snapshot_y;
    reg [15:0] snapshot_distance;
    reg [6:0] snapshot_bin;
    reg [11:0] snapshot_angle;
    reg [3:0] snapshot_bin_error;
    reg [31:0] snapshot_time;
    reg signed [31:0] product_x,product_y,offset_x,offset_y;
    wire trig_done,body_trig_done;
    wire signed [15:0] ray_cos,ray_sin,body_cos,body_sin;
    wire signed [14:0] ray_heading=$signed(snapshot_body_heading)+SENSOR_YAW_TENTHS-$signed({1'b0,snapshot_angle});
    wire signed [14:0] wrapped_ray=ray_heading>15'sd1800 ? ray_heading-15'sd3600 :
        ray_heading< -15'sd1800 ? ray_heading+15'sd3600 : ray_heading;
    wire cancel=!allowed || restart || rescan;
    // Select a stable representative near 1.5 + 3*k degrees. Compute this
    // after the angular bin has been registered, away from the input divide.
    wire [11:0] snapshot_bin_offset=snapshot_angle-snapshot_bin*12'd30;
    wire [11:0] snapshot_center_error=snapshot_bin_offset>=12'd15 ?
        snapshot_bin_offset-12'd15 : 12'd15-snapshot_bin_offset;
    // Always drain the UART decoder when inactive. Busy projection holds it
    // only for a few system clocks, not for matching or map replay.
    assign point_ready=phase==0;
    heading_lut_fine_pipeline ray_trig(.clk(clk),.reset_n(reset_n),.start(trig_start),.cancel(cancel),
        .angle_tenths(snapshot_ray_heading),.done(trig_done),.cos_q14(ray_cos),.sin_q14(ray_sin));
    heading_lut_fine_pipeline body_trig(.clk(clk),.reset_n(reset_n),.start(trig_start),.cancel(cancel),
        .angle_tenths(snapshot_body_heading),.done(body_trig_done),.cos_q14(body_cos),.sin_q14(body_sin));
    always @* begin
        if(!enable) status=0;
        else if(origin_pending || clear_pending || map_clear) status=9;
        else if(!lidar_online) status=1;
        else if(!odom_heading_valid) status=2;
        else if(!motor_online || motor_age>=STALE_CLKS) status=3;
        else if(pose_fault) status=8;
        else if(!pose_valid || !sample_allow) status=4;
        else status=5;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            previous_enable<=0;origin_valid<=0;origin_pending<=0;clear_pending<=0;first_pending<=0;
            yaw_origin<=0;stable_yaw<=0;yaw_age<=STALE_CLKS;motor_age<=STALE_CLKS;settle<=0;timestamp<=0;
            phase<=0;trig_start<=0;history_start<=0;snapshot_first<=0;snapshot_body_heading<=0;snapshot_ray_heading<=0;
            snapshot_x<=0;snapshot_y<=0;snapshot_distance<=0;snapshot_bin<=0;snapshot_time<=0;
            snapshot_angle<=0;snapshot_bin_error<=0;
            product_x<=0;product_y<=0;offset_x<=0;offset_y<=0;
            map_clear<=0;ray_update<=0;ray_frame_start<=0;ray_start_x_mm<=0;ray_start_y_mm<=0;
            ray_end_x_mm<=0;ray_end_y_mm<=0;ray_bin<=0;ray_timestamp<=0;
            ray_bin_error<=0;
            ray_body_heading_tenths<=0;
        end else begin
            previous_enable<=enable;map_clear<=0;ray_update<=0;ray_frame_start<=0;trig_start<=0;
            timestamp<=timestamp+1'b1;history_start<=0;
            if(yaw_age<STALE_CLKS) yaw_age<=yaw_age+1'b1;
            if(motor_age<STALE_CLKS) motor_age<=motor_age+1'b1;
            if(yaw_update && yaw_valid) yaw_age<=0;
            if(motor_update) motor_age<=0;
            if(!enable || !previous_enable || restart) begin
                origin_valid<=0;origin_pending<=enable;clear_pending<=enable;first_pending<=0;
                settle<=0;stable_yaw<=yaw_tenths;phase<=0;
            end else begin
                if(origin_pending) begin
                    if(!stopped || !yaw_stable) begin settle<=0;stable_yaw<=yaw_tenths;end
                    else if(settle<SETTLE_CLKS) settle<=settle+1'b1;
                    else begin
                        yaw_origin<=yaw_tenths;origin_valid<=1;origin_pending<=0;
                        clear_pending<=1;map_clear<=1;first_pending<=0;
                    end
                end else if(map_clear_done) clear_pending<=0;
                // The X3 decoder emits scan_start BEFORE its first point, while
                // preparing packet interpolation. It is an independent event,
                // not a tag qualified by point_valid/point_ready. Preserve it
                // through projection busy cycles and invalid first returns.
                // Cancellation below has priority so old boundaries cannot
                // leak into the next collection after rescan/health loss.
                if(scan_start) first_pending<=1;
                if(cancel) begin phase<=0;first_pending<=0;end
                else case(phase)
                    0:if(point_valid && point_ready) begin
                        // Far returns are kept: the driver already bounds the
                        // sensor at 8000 mm, and the map clips a long ray at
                        // its own edge cells instead of discarding the whole
                        // ray (which left the near field UNKNOWN).
                        if(range_valid && angle_tenths<3600 && distance_mm>=80 && distance_mm<=8000) begin
                            snapshot_x<=pose_x_mm;snapshot_y<=pose_y_mm;snapshot_distance<=distance_mm;
                            snapshot_body_heading<=projection_heading_tenths[12:0];
                            snapshot_bin<=angle_tenths/30;
                            snapshot_angle<=angle_tenths;
                            snapshot_first<=scan_start || first_pending;
                            if(point_timestamp_valid)snapshot_time<=point_timestamp;
                            else snapshot_time<=timestamp;
                            first_pending<=0;
                            if(point_timestamp_valid)begin history_start<=1;phase<=6;end else phase<=7;
                        end
                    end
                    6:if(history_done)begin
                        if(history_valid)begin
                            snapshot_x<=history_x;snapshot_y<=history_y;snapshot_body_heading<=history_yaw;
                            snapshot_ray_heading<=wrapped_history_heading[12:0];phase<=1;
                        end else begin first_pending<=snapshot_first;phase<=0;end
                    end
                    7:begin snapshot_ray_heading<=wrapped_ray[12:0];phase<=1;end
                    1:begin
                        // A second short wrap also covers a nonzero mounting
                        // yaw combined with an angle near the scan boundary.
                        if(snapshot_ray_heading>13'sd1800)snapshot_ray_heading<=snapshot_ray_heading-13'sd3600;
                        else if(snapshot_ray_heading< -13'sd1800)snapshot_ray_heading<=snapshot_ray_heading+13'sd3600;
                        snapshot_bin_error<=snapshot_center_error[3:0];
                        trig_start<=1;phase<=2;
                    end
                    2:if(trig_done && body_trig_done) begin
                        product_x<=$signed({1'b0,snapshot_distance})*ray_cos;
                        product_y<=$signed({1'b0,snapshot_distance})*ray_sin;
                        offset_x<=SENSOR_FORWARD_MM*body_cos-SENSOR_LEFT_MM*body_sin;
                        offset_y<=SENSOR_FORWARD_MM*body_sin+SENSOR_LEFT_MM*body_cos;phase<=3;
                    end
                    3:begin
                        ray_start_x_mm<=snapshot_x+(offset_x>>>14);ray_start_y_mm<=snapshot_y+(offset_y>>>14);
                        ray_end_x_mm<=snapshot_x+(offset_x>>>14)+(product_x>>>14);
                        ray_end_y_mm<=snapshot_y+(offset_y>>>14)+(product_y>>>14);phase<=4;
                    end
                    4:begin
                        ray_update<=1;ray_frame_start<=snapshot_first;ray_bin<=snapshot_bin;
                        ray_bin_error<=snapshot_bin_error;
                        ray_timestamp<=snapshot_time;phase<=5;
                        ray_body_heading_tenths<=$signed(snapshot_body_heading);
                    end
                    5:phase<=0;
                    default:phase<=0;
                endcase
            end
        end
    end
endmodule
