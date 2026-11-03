// Independent fixed-point swept-corridor evaluator. Inspired by the gap/risk
// separation in MRPT CHolonomicND, not a port of the C++/ROS library.
// Nine translation vectors: (0,-1),(1,-2),(1,-1),(2,-1),(1,0),
// (2,1),(1,1),(1,2),(0,1). The vehicle does NOT rotate with the vector.
module lidar_gap_profile #(
    parameter integer BODY_HALF_LENGTH_MM=135, GUARDED_HALF_WIDTH_MM=170,
    parameter integer CORRIDOR_MARGIN_MM=50,MAX_CLEARANCE_MM=3000
)(
    input wire clk,reset_n,online,boundary,boundary_good,hit_valid,
    input wire signed [15:0] hit_x_mm,hit_y_mm,
    output wire hit_ready,profile_ready,
    output wire [143:0] clearances_mm
);
    reg [2:0] phase;
    reg [3:0] index;
    reg pending,pending_good,have_lap;
    reg signed [15:0] x,y;
    reg signed [31:0] xc,ys,xs,yc;
    reg signed [32:0] dot,cross_sum;
    reg [15:0] evaluated_gap;
    reg evaluated_hit;
    reg [15:0] current_min[0:8],previous_min[0:8];
    function signed [15:0] cos_q14(input [3:0] d);
        case(d)
            0,8:cos_q14=0;1,7:cos_q14=7327;2,6:cos_q14=11585;
            3,5:cos_q14=14654;default:cos_q14=16384;
        endcase
    endfunction
    function signed [15:0] sin_q14(input [3:0] d);
        case(d)
            0:sin_q14=-16384;1:sin_q14=-14654;2:sin_q14=-11585;3:sin_q14=-7327;
            5:sin_q14=7327;6:sin_q14=11585;7:sin_q14=14654;8:sin_q14=16384;
            default:sin_q14=0;
        endcase
    endfunction
    function [15:0] leading_edge(input [3:0] d);
        integer c,s;
        begin c=cos_q14(d);s=sin_q14(d);if(s<0)s=-s;
            leading_edge=(BODY_HALF_LENGTH_MM*c+GUARDED_HALF_WIDTH_MM*s+16383)/16384;
        end
    endfunction
    function [15:0] cross_extent(input [3:0] d);
        integer c,s;
        begin c=cos_q14(d);s=sin_q14(d);if(s<0)s=-s;
            cross_extent=(BODY_HALF_LENGTH_MM*s+GUARDED_HALF_WIDTH_MM*c+16383)/16384+CORRIDOR_MARGIN_MM;
        end
    endfunction
    // Literal ROM arguments keep geometry divisions/multiplies elaboration-only.
    reg [15:0] lead,extent;
    always @* begin
        case(index)
            0:begin lead=leading_edge(0);extent=cross_extent(0);end
            1:begin lead=leading_edge(1);extent=cross_extent(1);end
            2:begin lead=leading_edge(2);extent=cross_extent(2);end
            3:begin lead=leading_edge(3);extent=cross_extent(3);end
            5:begin lead=leading_edge(5);extent=cross_extent(5);end
            6:begin lead=leading_edge(6);extent=cross_extent(6);end
            7:begin lead=leading_edge(7);extent=cross_extent(7);end
            8:begin lead=leading_edge(8);extent=cross_extent(8);end
            default:begin lead=leading_edge(4);extent=cross_extent(4);end
        endcase
    end
    wire signed [32:0] along_mm=dot>>>14;
    wire signed [32:0] cross_mm=cross_sum>>>14;
    wire [32:0] abs_cross=cross_mm<0?-cross_mm:cross_mm;
    assign hit_ready=online && phase==0 && !pending && !boundary;
    assign profile_ready=online && have_lap;
    genvar g;
    generate for(g=0;g<9;g=g+1)begin: outputs
        assign clearances_mm[g*16+:16]=current_min[g]<previous_min[g]?current_min[g]:previous_min[g];
    end endgenerate
    integer i;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            phase<=0;index<=0;pending<=0;pending_good<=0;have_lap<=0;x<=0;y<=0;
            xc<=0;ys<=0;xs<=0;yc<=0;dot<=0;cross_sum<=0;evaluated_gap<=0;evaluated_hit<=0;
            for(i=0;i<9;i=i+1)begin current_min[i]<=MAX_CLEARANCE_MM;previous_min[i]<=MAX_CLEARANCE_MM;end
        end else if(!online) begin
            phase<=0;pending<=0;have_lap<=0;
            for(i=0;i<9;i=i+1)begin current_min[i]<=MAX_CLEARANCE_MM;previous_min[i]<=MAX_CLEARANCE_MM;end
        end else begin
            if(boundary)begin pending<=1;pending_good<=boundary_good;end
            if(phase==0 && (pending || boundary))begin
                pending<=0;have_lap<=boundary?boundary_good:pending_good;
                for(i=0;i<9;i=i+1)begin
                    previous_min[i]<=(boundary?boundary_good:pending_good)?current_min[i]:MAX_CLEARANCE_MM;
                    current_min[i]<=MAX_CLEARANCE_MM;
                end
            end else case(phase)
                0:if(hit_valid && hit_ready)begin x<=hit_x_mm;y<=hit_y_mm;index<=0;phase<=1;end
                1:begin
                    xc<=x*cos_q14(index);ys<=y*sin_q14(index);
                    xs<=x*sin_q14(index);yc<=y*cos_q14(index);phase<=2;
                end
                2:begin
                    dot<=$signed({xc[31],xc})+$signed({ys[31],ys});
                    cross_sum<=$signed({yc[31],yc})-$signed({xs[31],xs});phase<=3;
                end
                3:begin
                    evaluated_hit<=along_mm>=0 && abs_cross<=extent;
                    evaluated_gap<=along_mm<=lead?16'd0:
                        along_mm-lead>MAX_CLEARANCE_MM?MAX_CLEARANCE_MM:along_mm-lead;
                    phase<=4;
                end
                4:begin
                    if(evaluated_hit && evaluated_gap<current_min[index])current_min[index]<=evaluated_gap;
                    if(index==8)phase<=0;else begin index<=index+1'b1;phase<=1;end
                end
                default:phase<=0;
            endcase
        end
    end
endmodule
