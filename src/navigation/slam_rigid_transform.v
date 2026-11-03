// Transactional SE(2) point transform. All arithmetic shares the existing
// system clock; multiply, subtract and coordinate addition have separate edges.
// out = pivot + translation + R(angle) * (point - pivot).
module slam_rigid_transform(
    input wire clk,reset_n,start,cancel,
    input wire signed [15:0] point_x,point_y,pivot_x,pivot_y,translation_x,translation_y,
    input wire signed [12:0] angle_tenths,
    output wire busy,
    output reg done,
    output reg signed [31:0] result_x,result_y
);
    reg [3:0] phase;
    reg trig_start;
    reg signed [12:0] angle;
    reg signed [16:0] relative_x,relative_y,origin_x,origin_y;
    reg signed [32:0] product_xc,product_ys,product_xs,product_yc;
    reg signed [33:0] rotated_x,rotated_y;
    wire trig_done;
    wire signed [15:0] cosine,sine;
    assign busy=phase!=0;
    heading_lut_fine_pipeline trig(.clk(clk),.reset_n(reset_n),.start(trig_start),
        .cancel(cancel),.angle_tenths(angle),.done(trig_done),.cos_q14(cosine),.sin_q14(sine));
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            phase<=0;trig_start<=0;done<=0;result_x<=0;result_y<=0;angle<=0;
            relative_x<=0;relative_y<=0;origin_x<=0;origin_y<=0;
            product_xc<=0;product_ys<=0;product_xs<=0;product_yc<=0;
            rotated_x<=0;rotated_y<=0;
        end else begin
            done<=0;trig_start<=0;
            if(cancel) phase<=0;
            else case(phase)
                0:if(start) begin
                    relative_x<=$signed({point_x[15],point_x})-$signed({pivot_x[15],pivot_x});
                    relative_y<=$signed({point_y[15],point_y})-$signed({pivot_y[15],pivot_y});
                    origin_x<=$signed({pivot_x[15],pivot_x})+$signed({translation_x[15],translation_x});
                    origin_y<=$signed({pivot_y[15],pivot_y})+$signed({translation_y[15],translation_y});
                    angle<=angle_tenths;phase<=1;
                end
                1:begin trig_start<=1;phase<=2;end
                2:if(trig_done) begin
                    product_xc<=relative_x*cosine;product_ys<=relative_y*sine;
                    product_xs<=relative_x*sine;product_yc<=relative_y*cosine;phase<=3;
                end
                3:begin
                    rotated_x<=$signed({product_xc[32],product_xc})-$signed({product_ys[32],product_ys});
                    rotated_y<=$signed({product_xs[32],product_xs})+$signed({product_yc[32],product_yc});
                    phase<=4;
                end
                4:begin
                    result_x<=$signed(origin_x)+(rotated_x>>>14);
                    result_y<=$signed(origin_y)+(rotated_y>>>14);
                    done<=1;phase<=0;
                end
                default:phase<=0;
            endcase
        end
    end
endmodule

// Continuous map<-odometry transform. Odometry remains in its original frame:
// map corrections never feed back into encoder integration or the raw IMU.
module slam_pose_transform #(parameter integer CELL_SHIFT=6, CELL_MM=(1<<CELL_SHIFT), GRID_SIZE=128, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y))(
    input wire clk,reset_n,enable,clear,
    input wire odom_valid,odom_fault,
    input wire signed [15:0] odom_x_mm,odom_y_mm,
    input wire signed [31:0] odom_heading_tenths,
    input wire transform_update,
    input wire signed [15:0] transform_x_mm,transform_y_mm,
    input wire signed [12:0] transform_yaw_tenths,
    output wire valid,
    output reg fault,moved,
    output reg transform_pending,
    output reg signed [15:0] x_mm,y_mm,
    output reg signed [31:0] heading_tenths,
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
    reg have_pose,start;
    localparam POSE_OFFSET_BITS=$clog2((GRID_X>GRID_Y?GRID_X:GRID_Y)*CELL_MM+1);
    reg [1:0] publish_phase;
    reg [POSE_OFFSET_BITS-1:0] publish_offset_x,publish_offset_y;
    reg [POSE_OFFSET_BITS+15:0] publish_product_x,publish_product_y;
    reg signed [15:0] publish_x,publish_y;
    reg [GRID_BITS-1:0] published_cell_x,published_cell_y;
    reg signed [15:0] translation_x,translation_y,snapshot_x,snapshot_y;
    reg signed [12:0] rotation,latched_heading;
    wire busy,done;
    wire signed [31:0] result_x,result_y;
    wire signed [13:0] heading_sum=$signed(odom_heading_tenths[12:0])+$signed(rotation);
    wire signed [13:0] wrapped_heading=heading_sum>1800 ? heading_sum-3600 :
        heading_sum< -1800 ? heading_sum+3600 : heading_sum;
    wire cancel=!enable || clear || transform_update || !odom_valid || odom_fault;
    wire in_bounds=result_x>=-MAP_HALF_X_MM && result_x<MAP_HALF_X_MM &&
                        result_y>=-MAP_HALF_Y_MM && result_y<MAP_HALF_Y_MM;
    assign valid=enable && !clear && odom_valid && !odom_fault && have_pose && !fault;
    assign cell_x=(CELL_MM==20||CELL_MM==30||CELL_MM==40)&&POSE_OFFSET_BITS<=16?published_cell_x:grid_floor(x_mm+ORIGIN_BIAS_X_MM)+(GRID_X/2);
    assign cell_y=(CELL_MM==20||CELL_MM==30||CELL_MM==40)&&POSE_OFFSET_BITS<=16?published_cell_y:grid_floor(y_mm+ORIGIN_BIAS_Y_MM)+(GRID_Y/2);
    slam_rigid_transform transform(.clk(clk),.reset_n(reset_n),.start(start),.cancel(cancel),
        .point_x(snapshot_x),.point_y(snapshot_y),.pivot_x(16'sd0),.pivot_y(16'sd0),
        .translation_x(translation_x),.translation_y(translation_y),.angle_tenths(rotation),
        .busy(busy),.done(done),.result_x(result_x),.result_y(result_y));
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            have_pose<=0;start<=0;translation_x<=0;translation_y<=0;snapshot_x<=0;snapshot_y<=0;rotation<=0;
            latched_heading<=0;fault<=0;moved<=0;transform_pending<=0;x_mm<=0;y_mm<=0;heading_tenths<=0;
            publish_phase<=0;publish_x<=0;publish_y<=0;publish_offset_x<=0;publish_offset_y<=0;
            publish_product_x<=0;publish_product_y<=0;published_cell_x<=GRID_X/2;published_cell_y<=GRID_Y/2;
        end else begin
            start<=0;moved<=0;
            if(!enable || clear) begin
                have_pose<=0;translation_x<=0;translation_y<=0;rotation<=0;
                fault<=0;transform_pending<=0;x_mm<=0;y_mm<=0;heading_tenths<=0;
                publish_phase<=0;published_cell_x<=GRID_X/2;published_cell_y<=GRID_Y/2;
            end else if(transform_update) begin
                translation_x<=transform_x_mm;translation_y<=transform_y_mm;rotation<=transform_yaw_tenths;
                publish_phase<=0;transform_pending<=1;
                // Keep the last coherent snapshot until the replacement is
                // complete. The caller freezes navigation during map commit.
            end else if(!odom_valid || odom_fault) begin
                have_pose<=0;fault<=odom_fault;publish_phase<=0;transform_pending<=0;
            end else begin
                if(!busy && !start && !done && publish_phase==0) begin
                    start<=1;latched_heading<=wrapped_heading[12:0];snapshot_x<=odom_x_mm;snapshot_y<=odom_y_mm;
                end
                if(done) begin
                    fault<=!in_bounds;
                    if(!in_bounds)begin have_pose<=0;transform_pending<=0;end
                    if(in_bounds) begin
                        if((CELL_MM==20||CELL_MM==30||CELL_MM==40)&&POSE_OFFSET_BITS<=16)begin
                            publish_x<=result_x[15:0];publish_y<=result_y[15:0];
                            publish_offset_x<=result_x+MAP_HALF_X_MM;
                            publish_offset_y<=result_y+MAP_HALF_Y_MM;publish_phase<=1;
                        end else begin
                            have_pose<=1;transform_pending<=0;moved<=x_mm!=result_x[15:0] || y_mm!=result_y[15:0];
                            x_mm<=result_x[15:0];y_mm<=result_y[15:0];heading_tenths<=latched_heading;
                        end
                    end
                end
                if(publish_phase==1)begin
                    publish_product_x<=publish_offset_x*GRID_RECIPROCAL_MULTIPLIER;publish_product_y<=publish_offset_y*GRID_RECIPROCAL_MULTIPLIER;
                    publish_phase<=2;
                end else if(publish_phase==2)begin
                    have_pose<=1;transform_pending<=0;moved<=x_mm!=publish_x||y_mm!=publish_y;
                    x_mm<=publish_x;y_mm<=publish_y;heading_tenths<=latched_heading;
                    published_cell_x<=publish_product_x>>GRID_RECIPROCAL_SHIFT;published_cell_y<=publish_product_y>>GRID_RECIPROCAL_SHIFT;publish_phase<=0;
                end
            end
        end
    end
endmodule
