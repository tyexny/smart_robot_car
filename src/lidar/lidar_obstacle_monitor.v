// Live body-frame collision clearances, independent of mapping/IMU/odometry.
// X3 bearings are clockwise; body coordinates are forward X, left Y.
// Strict mode requires checked samples in every15-degree bin of a hemisphere.
// With NO_RETURN_IS_CLEAR, clean zero samples count as observed empty space,
// Missing/rejected samples never establish actual coverage. The explicit
// MISSING_SAMPLE_IS_CLEAR option instead assumes uncovered bins are clear on
// a live lap, without claiming those bins were measured or altering mapping.
// Near points lower clearance immediately after projection. Clearance can rise
// only on a complete fresh lap; keeping the previous lap avoids premature clear.
module lidar_obstacle_monitor #(
    parameter integer CLK_FREQ_HZ=50000000, STALE_MS=400, MIN_LAP_MS=40,
    parameter integer BODY_HALF_LENGTH_MM=200, BODY_HALF_WIDTH_MM=140,
    parameter integer BODY_FRONT_MM=BODY_HALF_LENGTH_MM, BODY_REAR_MM=BODY_HALF_LENGTH_MM,
    parameter integer SIDE_SAFETY_MARGIN_MM=0,
    parameter integer NO_RETURN_IS_CLEAR=0,
    // User-selected experimental policy: lack of accepted angular samples
    // does not veto an otherwise live lap. Retain accepted obstacles and the
    // independent UART/lap watchdog; this is NOT evidence of free space.
    parameter integer MISSING_SAMPLE_IS_CLEAR=0,
    parameter integer ENABLE_GAP_PROFILE=0,
    parameter integer CORRIDOR_MARGIN_MM=50, ROTATION_RADIUS_MM=250,
    parameter integer SENSOR_FORWARD_MM=0, SENSOR_LEFT_MM=0, SENSOR_YAW_TENTHS=0,
    parameter integer MAX_CLEARANCE_MM=3000
)(
    input wire clk,reset_n,online,scan_start,point_valid,range_valid,
    input wire no_return,
    input wire [15:0] distance_mm,input wire [11:0] angle_tenths,
    output wire point_ready,scan_healthy,
    output wire [15:0] front_mm,rear_mm,left_mm,right_mm,rotation_mm,
    output wire front_valid,rear_valid,left_valid,right_valid,rotation_valid,
    output reg front_update,rear_update,left_update,right_update,
    output wire [23:0] coverage,
    output wire [143:0] gap_clearance_mm,
    output wire [8:0] gap_direction_valid
);
    localparam integer STALE_CLKS=(CLK_FREQ_HZ/1000*STALE_MS<1)?1:CLK_FREQ_HZ/1000*STALE_MS;
    localparam integer MIN_LAP_CLKS=(CLK_FREQ_HZ/1000*MIN_LAP_MS<1)?1:CLK_FREQ_HZ/1000*MIN_LAP_MS;
    localparam integer SENSOR_RADIUS_PADDING=(SENSOR_FORWARD_MM<0?-SENSOR_FORWARD_MM:SENSOR_FORWARD_MM)+
        (SENSOR_LEFT_MM<0?-SENSOR_LEFT_MM:SENSOR_LEFT_MM);
    localparam integer GUARDED_HALF_WIDTH_MM=BODY_HALF_WIDTH_MM+SIDE_SAFETY_MARGIN_MM;
    reg [31:0] lap_age;
    reg started,lap_good,boundary_pending;
    reg [23:0] current_coverage,previous_coverage;
    reg [23:0] current_observed,previous_observed;
    reg [15:0] current_min[0:4],previous_min[0:4];
    reg [2:0] phase;
    reg trig_start;
    reg signed [12:0] heading;
    reg [15:0] sample_distance;
    reg sample_no_return;
    reg [4:0] sample_bin;
    reg signed [31:0] product_x,product_y;
    reg signed [15:0] x_mm,y_mm;
    wire signed [31:0] raw_heading=SENSOR_YAW_TENTHS-$signed({1'b0,angle_tenths});
    wire signed [31:0] wrapped_heading=raw_heading>1800?raw_heading-3600:
        raw_heading< -1800?raw_heading+3600:raw_heading;
    wire [12:0] positive_heading=wrapped_heading<0?wrapped_heading+3600:wrapped_heading;
    wire trig_done;
    wire signed [15:0] cosine,sine;
    heading_lut_fine_pipeline trig(.clk(clk),.reset_n(reset_n),.start(trig_start),
        .cancel(!online),.angle_tenths(heading),.done(trig_done),.cos_q14(cosine),.sin_q14(sine));
    assign point_ready=online && phase==0 && !boundary_pending && !scan_start;
    assign scan_healthy=online && lap_good && lap_age<STALE_CLKS;
    assign coverage=previous_coverage;
    wire [23:0] usable_coverage=MISSING_SAMPLE_IS_CLEAR?24'hffffff:
        NO_RETURN_IS_CLEAR?previous_observed:previous_coverage;
    assign front_valid=scan_healthy && (usable_coverage & 24'hFC003F)==24'hFC003F;
    assign rear_valid=scan_healthy && (usable_coverage & 24'h03FFC0)==24'h03FFC0;
    assign left_valid=scan_healthy && (usable_coverage & 24'h000FFF)==24'h000FFF;
    assign right_valid=scan_healthy && (usable_coverage & 24'hFFF000)==24'hFFF000;
    assign rotation_valid=scan_healthy && usable_coverage==24'hFFFFFF;
    assign front_mm=current_min[0]<previous_min[0]?current_min[0]:previous_min[0];
    assign rear_mm=current_min[1]<previous_min[1]?current_min[1]:previous_min[1];
    assign left_mm=current_min[2]<previous_min[2]?current_min[2]:previous_min[2];
    assign right_mm=current_min[3]<previous_min[3]?current_min[3]:previous_min[3];
    assign rotation_mm=current_min[4]<previous_min[4]?current_min[4]:previous_min[4];
    wire signed [16:0] abs_x=x_mm<0?-$signed({x_mm[15],x_mm}):$signed({x_mm[15],x_mm});
    wire signed [16:0] abs_y=y_mm<0?-$signed({y_mm[15],y_mm}):$signed({y_mm[15],y_mm});
    // Ignore only the real chassis, never the added safety band: an object in
    // that band is a zero-gap obstacle, not a chassis echo or free space.
    wire [15:0] x_extent=x_mm>=0?BODY_FRONT_MM:BODY_REAR_MM;
    wire self_return=abs_x<x_extent && abs_y<BODY_HALF_WIDTH_MM;
    wire [15:0] x_gap=abs_x<=x_extent?16'd0:abs_x-x_extent;
    wire [15:0] y_gap=abs_y<=GUARDED_HALF_WIDTH_MM?16'd0:abs_y-GUARDED_HALF_WIDTH_MM;
    wire [15:0] rotation_gap=sample_distance<=ROTATION_RADIUS_MM+SENSOR_RADIUS_PADDING?16'd0:
        sample_distance-ROTATION_RADIUS_MM-SENSOR_RADIUS_PADDING;
    reg profile_boundary,profile_boundary_good;
    wire profile_hit_ready,profile_ready;
    generate if(ENABLE_GAP_PROFILE)begin: gap_profile
        lidar_gap_profile #(.BODY_HALF_LENGTH_MM(BODY_HALF_LENGTH_MM),
            .GUARDED_HALF_WIDTH_MM(GUARDED_HALF_WIDTH_MM),.CORRIDOR_MARGIN_MM(CORRIDOR_MARGIN_MM),
            .MAX_CLEARANCE_MM(MAX_CLEARANCE_MM)) engine(
            .clk(clk),.reset_n(reset_n),.online(online),.boundary(profile_boundary),
            .boundary_good(profile_boundary_good),.hit_valid(phase==4 && !self_return),
            .hit_x_mm(x_mm),.hit_y_mm(y_mm),.hit_ready(profile_hit_ready),
            .profile_ready(profile_ready),.clearances_mm(gap_clearance_mm));
    end else begin: no_gap_profile
        assign profile_hit_ready=1;assign profile_ready=0;assign gap_clearance_mm=0;
    end endgenerate
    assign gap_direction_valid[0]=profile_ready && scan_healthy && (usable_coverage & 24'hfff000)==24'hfff000;
    assign gap_direction_valid[1]=profile_ready && scan_healthy && (usable_coverage & 24'hffe003)==24'hffe003;
    assign gap_direction_valid[2]=profile_ready && scan_healthy && (usable_coverage & 24'hff8007)==24'hff8007;
    assign gap_direction_valid[3]=profile_ready && scan_healthy && (usable_coverage & 24'hff001f)==24'hff001f;
    assign gap_direction_valid[4]=profile_ready && scan_healthy && (usable_coverage & 24'hfc003f)==24'hfc003f;
    assign gap_direction_valid[5]=profile_ready && scan_healthy && (usable_coverage & 24'hf800ff)==24'hf800ff;
    assign gap_direction_valid[6]=profile_ready && scan_healthy && (usable_coverage & 24'he001ff)==24'he001ff;
    assign gap_direction_valid[7]=profile_ready && scan_healthy && (usable_coverage & 24'hc007ff)==24'hc007ff;
    assign gap_direction_valid[8]=profile_ready && scan_healthy && (usable_coverage & 24'h000fff)==24'h000fff;
    integer i;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            lap_age<=STALE_CLKS;started<=0;lap_good<=0;boundary_pending<=0;
            current_coverage<=0;previous_coverage<=0;phase<=0;trig_start<=0;
            current_observed<=0;previous_observed<=0;sample_no_return<=0;
            profile_boundary<=0;profile_boundary_good<=0;
            heading<=0;sample_distance<=0;sample_bin<=0;product_x<=0;product_y<=0;x_mm<=0;y_mm<=0;
            front_update<=0;rear_update<=0;left_update<=0;right_update<=0;
            for(i=0;i<5;i=i+1) begin current_min[i]<=MAX_CLEARANCE_MM;previous_min[i]<=MAX_CLEARANCE_MM;end
        end else begin
            trig_start<=0;front_update<=0;rear_update<=0;left_update<=0;right_update<=0;profile_boundary<=0;
            if(lap_age<STALE_CLKS) lap_age<=lap_age+1'b1;
            if(!online) begin
                started<=0;lap_good<=0;boundary_pending<=0;phase<=0;lap_age<=STALE_CLKS;
                current_coverage<=0;previous_coverage<=0;
                current_observed<=0;previous_observed<=0;
                for(i=0;i<5;i=i+1) begin current_min[i]<=MAX_CLEARANCE_MM;previous_min[i]<=MAX_CLEARANCE_MM;end
            end else begin
                // scan_start precedes point_valid and is never handshake-qualified.
                if(scan_start) boundary_pending<=1;
                if(phase==0 && (boundary_pending || scan_start)) begin
                    boundary_pending<=0;started<=1;lap_age<=0;
                    profile_boundary<=1;profile_boundary_good<=started && lap_age>=MIN_LAP_CLKS && lap_age<STALE_CLKS;
                    lap_good<=started && lap_age>=MIN_LAP_CLKS && lap_age<STALE_CLKS;
                    if(started && lap_age>=MIN_LAP_CLKS && lap_age<STALE_CLKS) begin
                        previous_coverage<=current_coverage;
                        previous_observed<=current_observed;
                        for(i=0;i<5;i=i+1) previous_min[i]<=current_min[i];
                        front_update<=1;rear_update<=1;left_update<=1;right_update<=1;
                    end else begin
                        previous_coverage<=0;
                        previous_observed<=0;
                        for(i=0;i<5;i=i+1) previous_min[i]<=MAX_CLEARANCE_MM;
                    end
                    current_coverage<=0;
                    current_observed<=0;
                    for(i=0;i<5;i=i+1) current_min[i]<=MAX_CLEARANCE_MM;
                end else case(phase)
                    0:if(point_valid && point_ready && started && angle_tenths<3600 &&
                            ((range_valid && distance_mm>=100 && distance_mm<=8000) ||
                             (NO_RETURN_IS_CLEAR && no_return && !range_valid && distance_mm==0))) begin
                        heading<=wrapped_heading[12:0];sample_distance<=distance_mm;
                        sample_no_return<=NO_RETURN_IS_CLEAR && no_return && !range_valid && distance_mm==0;
                        sample_bin<=positive_heading/13'd150;phase<=1;
                    end
                    1:begin
                        if(sample_no_return) begin
                            current_observed[sample_bin]<=1;phase<=0;
                        end else begin trig_start<=1;phase<=2;end
                    end
                    2:if(trig_done) begin
                        product_x<=$signed({1'b0,sample_distance})*cosine;
                        product_y<=$signed({1'b0,sample_distance})*sine;phase<=3;
                    end
                    3:begin
                        x_mm<=(product_x>>>14)+SENSOR_FORWARD_MM;
                        y_mm<=(product_y>>>14)+SENSOR_LEFT_MM;phase<=4;
                    end
                    4:begin
                        // Chassis echoes do not constitute free-space evidence.
                        if(!self_return) begin
                            current_coverage[sample_bin]<=1;
                            current_observed[sample_bin]<=1;
                            if(abs_y<=GUARDED_HALF_WIDTH_MM+CORRIDOR_MARGIN_MM) begin
                                if(x_mm>=0 && x_gap<current_min[0]) current_min[0]<=x_gap;
                                if(x_mm<0 && x_gap<current_min[1]) current_min[1]<=x_gap;
                            end
                            if(x_mm>=-(BODY_REAR_MM+CORRIDOR_MARGIN_MM) &&
                               x_mm<=BODY_FRONT_MM+CORRIDOR_MARGIN_MM) begin
                                if(y_mm>=0 && y_gap<current_min[2]) current_min[2]<=y_gap;
                                if(y_mm<0 && y_gap<current_min[3]) current_min[3]<=y_gap;
                            end
                            if(rotation_gap<current_min[4]) current_min[4]<=rotation_gap;
                        end
                        if(!ENABLE_GAP_PROFILE || self_return || profile_hit_ready)phase<=0;
                    end
                    default:phase<=0;
                endcase
            end
        end
    end
endmodule
