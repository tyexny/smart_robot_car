// Simple forward/strafe policy. No mandatory STOP or wheel-zero handshake
// between forward and lateral travel. The independent guard still has STOP
// authority. Only SW6 enables/disables this policy; IR never latches it off.
module lidar_flow_control #(
 parameter integer CLK_FREQ_HZ=50000000,
 parameter integer STOP_CM=8,TRIGGER_CM=24,SLOW_CM=32,CLEAR_CM=36,
 parameter integer SIDE_START_CM=18,SLOW_SPEED=3,RETURN_SPEED=2,FINE_RETURN_SPEED=1,
 parameter integer RETURN_ADVANCE_REDUCTION_MM=0,
 parameter integer ENABLE_RETURN_TO_LINE=0
)(
 input wire clk,reset_n,enable,command_update,
 input wire [3:0] requested_motion,input wire [7:0] requested_speed,
 input wire healthy,front_valid,rear_valid,left_valid,right_valid,
 input wire [15:0] front_cm,rear_cm,left_cm,right_cm,
 input wire front_update,left_update,right_update,
 // Coordinates in the fixed heading frame captured at automatic start.
 input wire route_valid,route_fault,
 input wire signed [15:0] route_x_mm,route_y_mm,
 output reg [3:0] motion,output reg [7:0] speed,
 output wire [5:0] state,output reg [3:0] fault
);
 generate if(ENABLE_RETURN_TO_LINE)begin: return_policy
  wire [3:0] m,f;wire [7:0] v;
  lidar_return_to_line #(.STOP_CM(STOP_CM),.TRIGGER_CM(TRIGGER_CM),
   .SLOW_CM(SLOW_CM),.CLEAR_CM(CLEAR_CM),.SIDE_START_CM(SIDE_START_CM),.SLOW_SPEED(SLOW_SPEED),
   .RETURN_SPEED(RETURN_SPEED),.FINE_RETURN_SPEED(FINE_RETURN_SPEED),
   .RETURN_ADVANCE_REDUCTION_MM(RETURN_ADVANCE_REDUCTION_MM)) policy(
   .clk(clk),.reset_n(reset_n),.enable(enable),.requested_motion(requested_motion),
   .requested_speed(requested_speed),.healthy(healthy),.front_valid(front_valid),
   .left_valid(left_valid),.right_valid(right_valid),.front_cm(front_cm),
   .left_cm(left_cm),.right_cm(right_cm),.front_update(front_update),
   .left_update(left_update),.right_update(right_update),
   .route_valid(route_valid),.route_fault(route_fault),.x_mm(route_x_mm),.y_mm(route_y_mm),
   .motion(m),.speed(v),.state(state),.fault(f));
  always @*begin motion=m;speed=v;fault=f;end
 end else begin: original_policy
 localparam [1:0] CRUISE=0,SHIFT=1,BLOCKED=2;
 reg [1:0] phase,next_phase;
 reg rightward,slow_latched;
 reg next_rightward;
 wire auto_forward=enable && requested_motion==1 && requested_speed!=0;
 wire left_open=left_valid && left_cm>=SIDE_START_CM;
 wire right_open=right_valid && right_cm>=SIDE_START_CM;
 wire choose_right=right_open && (!left_open || right_cm>=left_cm);
 wire side_safe=rightward?(right_valid && right_cm>STOP_CM):(left_valid && left_cm>STOP_CM);
 wire live_front=healthy && front_valid;
 wire front_clear=live_front && front_cm>=CLEAR_CM;
 wire [7:0] low_speed=requested_speed>SLOW_SPEED?SLOW_SPEED:requested_speed;
 assign state=!auto_forward?6'd0:phase==SHIFT?6'd35:phase==BLOCKED?6'd7:6'd0;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin phase<=CRUISE;rightward<=0;slow_latched<=0;end
  else if(!auto_forward)begin phase<=CRUISE;rightward<=0;slow_latched<=0;end
  else begin
   phase<=next_phase;rightward<=next_rightward;
   if(live_front)begin
    if(front_cm<=SLOW_CM)slow_latched<=1;
    else if(front_clear)slow_latched<=0;
   end
  end
 end
 // Shared decision for registered state and immediate output: changing a safe
 // direction must not create even a one-clock zero that latches a UART STOP.
 always @*begin
  next_phase=phase;next_rightward=rightward;
  if(auto_forward && live_front)begin
   // Front and both sides close: STOP even when the rear is clear.
   // This overrides a still-moving strafe before its lower STOP threshold.
   if(front_cm<=TRIGGER_CM && !left_open && !right_open)next_phase=BLOCKED;
   else if(phase==SHIFT && side_safe)begin
    if(front_update && front_clear)next_phase=CRUISE;
    else if((rightward?right_cm:left_cm)<SIDE_START_CM)begin
     if(front_cm>TRIGGER_CM)next_phase=CRUISE;
     else if(rightward && left_open)next_rightward=0;
     else if(!rightward && right_open)next_rightward=1;
     else next_phase=BLOCKED;
    end
   end else if(front_cm>TRIGGER_CM)next_phase=CRUISE;
   else if(left_open || right_open)begin next_phase=SHIFT;next_rightward=choose_right;end
   else next_phase=BLOCKED;
  end
 end
 always @*begin
  motion=requested_motion;speed=requested_motion==0?8'd0:requested_speed;fault=0;
  if(auto_forward)begin
   motion=0;speed=0;
   if(!live_front)fault=14;
   else case(next_phase)
    CRUISE:begin motion=1;speed=(slow_latched || front_cm<=SLOW_CM)?low_speed:requested_speed;end
    SHIFT:begin motion=next_rightward?4'd4:4'd3;speed=low_speed;end
    BLOCKED:fault=9;
   endcase
  end
  if(motion==0)speed=0;
 end
 end endgenerate
endmodule

// Continuous rectangular bypass: sideways, forward past the obstacle, then
// back to the ORIGINAL line. No timed STOP is inserted between these legs.
// Side clearance covers the whole front/rear chassis span in the radar monitor.
module lidar_return_to_line #(
 parameter integer STOP_CM=8,TRIGGER_CM=24,SLOW_CM=32,CLEAR_CM=36,
 parameter integer SIDE_START_CM=18,SLOW_SPEED=3,RETURN_SPEED=2,FINE_RETURN_SPEED=1,
 parameter integer BODY_LENGTH_MM=400,PASS_MARGIN_MM=60,RETURN_MARGIN_MM=60,
 parameter integer RETURN_ADVANCE_REDUCTION_MM=0,
 parameter integer RETURN_TOL_MM=15,LINE_DRIFT_MM=30,CLEAR_VOTES=2,
 parameter integer MAX_OFFSET_MM=1800,MAX_BYPASS_MM=6000
)(
 input wire clk,reset_n,enable,
 input wire [3:0] requested_motion,input wire [7:0] requested_speed,
 input wire healthy,front_valid,left_valid,right_valid,
 input wire [15:0] front_cm,left_cm,right_cm,
 input wire front_update,left_update,right_update,
 input wire route_valid,route_fault,
 input wire signed [15:0] x_mm,y_mm,
 output reg [3:0] motion,output reg [7:0] speed,
 output reg [5:0] state,output reg [3:0] fault
);
 localparam [1:0] CRUISE=0,SHIFT_OUT=1,PASS=2,RETURN=3;
 reg [1:0] phase,next_phase;
 reg outward_right,next_outward_right,return_from_left;
 reg slow_latched,seen_position,session_lost;
 reg signed [16:0] clear_after_x,encounter_x;
 reg [2:0] clear_votes;
 wire active=enable && requested_motion==1 && requested_speed!=0;
 wire live=healthy && front_valid;
 wire left_open=left_valid && left_cm>=SIDE_START_CM;
 wire right_open=right_valid && right_cm>=SIDE_START_CM;
 wire choose_right=right_open && (!left_open || right_cm>=left_cm);
 wire front_clear=live && front_cm>=CLEAR_CM;
 wire [16:0] offset_mm=y_mm[15]?-$signed({y_mm[15],y_mm}):$signed({y_mm[15],y_mm});
 wire return_right=y_mm>0;
 wire [15:0] inner_cm=return_right?right_cm:left_cm;
 wire inner_valid=return_right?right_valid:left_valid;
 wire inner_update=return_right?right_update:left_update;
 // Scale each side before the pose-dependent selection.
 // Both sums fit in 21 bits for every 16-bit centimetre input.
 wire [20:0] left_mm={2'b0,left_cm,3'b0}+{4'b0,left_cm,1'b0};
 wire [20:0] right_mm={2'b0,right_cm,3'b0}+{4'b0,right_cm,1'b0};
 wire [20:0] inner_mm=return_right?right_mm:left_mm;
 wire return_clear=inner_valid && inner_mm>offset_mm+RETURN_MARGIN_MM;
 wire [7:0] low_speed=requested_speed>SLOW_SPEED?SLOW_SPEED:requested_speed;
 wire [7:0] return_cap=offset_mm<40?FINE_RETURN_SPEED:RETURN_SPEED;
 wire [7:0] return_speed=requested_speed>return_cap?return_cap:requested_speed;
 wire position_ready=route_valid && !route_fault && !session_lost;
 wire passed=$signed({x_mm[15],x_mm})>=clear_after_x;
 wire bound_lost=offset_mm>MAX_OFFSET_MM ||
   (phase!=CRUISE && $signed({x_mm[15],x_mm})-encounter_x>MAX_BYPASS_MM);
 wire votes_ready=clear_votes>=CLEAR_VOTES;

 always @*begin
  next_phase=phase;next_outward_right=outward_right;
  if(active && live && position_ready && !bound_lost)case(phase)
   CRUISE:begin
    if(front_cm<=TRIGGER_CM && (left_open||right_open))begin next_phase=SHIFT_OUT;next_outward_right=choose_right;end
    else if(front_clear && offset_mm>=LINE_DRIFT_MM && return_clear)next_phase=RETURN;
   end
   SHIFT_OUT:begin
    if(front_update && front_clear)next_phase=PASS;
    else if(outward_right?!right_open:!left_open)begin
     if(front_cm>TRIGGER_CM)next_phase=PASS;
     else if(left_open||right_open)next_outward_right=choose_right;
    end
   end
   PASS:begin
    if(front_cm<=TRIGGER_CM && (left_open||right_open))begin next_phase=SHIFT_OUT;next_outward_right=choose_right;end
    else if(offset_mm<=RETURN_TOL_MM && front_cm>TRIGGER_CM)next_phase=CRUISE;
    else if(front_clear && passed && return_clear && votes_ready)next_phase=RETURN;
   end
   RETURN:begin
    if((return_from_left && y_mm<=RETURN_TOL_MM)||(!return_from_left && y_mm>=-RETURN_TOL_MM))next_phase=CRUISE;
    else if(front_cm<=TRIGGER_CM && (left_open||right_open))begin next_phase=SHIFT_OUT;next_outward_right=choose_right;end
    else if(!return_clear)next_phase=PASS; // continue safely forward; never force a blocked return
   end
  endcase
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   phase<=CRUISE;outward_right<=0;return_from_left<=0;slow_latched<=0;
   clear_after_x<=0;encounter_x<=0;clear_votes<=0;seen_position<=0;session_lost<=0;
  end else if(!active)begin
   phase<=CRUISE;outward_right<=0;return_from_left<=0;slow_latched<=0;
   clear_after_x<=0;encounter_x<=0;clear_votes<=0;seen_position<=0;session_lost<=0;
  end else begin
   if(route_valid)seen_position<=1;
   if(seen_position && (!route_valid||route_fault))session_lost<=1;
   phase<=next_phase;outward_right<=next_outward_right;
   if(next_phase==RETURN && phase!=RETURN)return_from_left<=return_right;
   if(next_phase==SHIFT_OUT && phase!=SHIFT_OUT)begin
    encounter_x<=$signed({x_mm[15],x_mm});
    clear_after_x<=$signed({x_mm[15],x_mm})+$signed({1'b0,front_cm})*10+BODY_LENGTH_MM+PASS_MARGIN_MM-RETURN_ADVANCE_REDUCTION_MM;
   end
   if(phase!=PASS || !front_clear || !return_clear || !passed)clear_votes<=0;
   else if(inner_update && clear_votes<CLEAR_VOTES)clear_votes<=clear_votes+1'b1;
   if(live)begin
    if(front_cm<=SLOW_CM)slow_latched<=1;
    else if(front_clear)slow_latched<=0;
   end
  end
 end
 always @*begin
  motion=requested_motion;speed=requested_motion==0?0:requested_speed;state=0;fault=0;
  if(active)begin
   motion=0;speed=0;
   if(!live)begin state=7;fault=14;end
   else if(!position_ready || bound_lost)begin state=7;fault=10;end
   else if(front_cm<=TRIGGER_CM && !left_open && !right_open)begin state=7;fault=9;end
   else case(next_phase)
    CRUISE:begin motion=1;speed=(slow_latched || front_cm<=SLOW_CM)?low_speed:requested_speed;end
    SHIFT_OUT:begin
     state=35;
     if(next_outward_right?right_open:left_open)begin motion=next_outward_right?4:3;speed=low_speed;end
     else begin state=7;fault=9;end
    end
    PASS:begin state=38;motion=1;speed=low_speed;end
    RETURN:begin
     state=39;
     if(return_clear)begin motion=return_right?4:3;speed=return_speed;end
     else if(front_cm>TRIGGER_CM)begin motion=1;speed=low_speed;end
     else begin state=7;fault=9;end
    end
   endcase
  end
 end
endmodule

// Signed mecanum odometry accounts for diagonal blends and yaw corrections,
// rather than assigning sum(abs(wheels)) to the last requested direction.
// F=(-A+B+C+D), L=(A+B+C-D); pure yaw cancels in both sums.
module avoidance_route_odometry #(
 parameter integer CLK_FREQ_HZ=50000000,STALE_MS=500,MAX_STEP=2048,
 parameter integer FORWARD_Q12=853,STRAFE_Q12=755,LIMIT_MM=16000
)(
 input wire clk,reset_n,enable,motor_online,heading_valid,
 input wire [63:0] counts,input wire counts_update,
 input wire signed [15:0] relative_heading,
 output wire valid,output reg fault,
 output wire signed [15:0] x_mm,y_mm
);
 localparam integer STALE_CLKS=(CLK_FREQ_HZ/1000*STALE_MS<1)?1:CLK_FREQ_HZ/1000*STALE_MS;
 reg baseline;reg [63:0] previous_counts;reg [31:0] age;
 reg [2:0] phase;reg trig_start;reg signed [12:0] angle;
 reg signed [31:0] f_q12,l_q12,x_q12,y_q12;
 // Wheel sums are captured first: the four-way add and the Q12
 // multiply used to share one 20 ns clock.
 reg signed [18:0] forward_q,left_q;
 reg signed [15:0] cosine,sine;
 reg signed [48:0] dx,dy;
 wire signed [15:0] a=counts[63:48]-previous_counts[63:48];
 wire signed [15:0] b=counts[47:32]-previous_counts[47:32];
 wire signed [15:0] c=counts[31:16]-previous_counts[31:16];
 wire signed [15:0] d=counts[15:0]-previous_counts[15:0];
 wire signed [18:0] aa={{3{a[15]}},a},bb={{3{b[15]}},b},cc={{3{c[15]}},c},dd={{3{d[15]}},d};
 wire signed [18:0] forward_sum=-aa+bb+cc+dd,left_sum=aa+bb+cc-dd;
 wire jump=a>MAX_STEP||a< -MAX_STEP||b>MAX_STEP||b< -MAX_STEP||c>MAX_STEP||c< -MAX_STEP||d>MAX_STEP||d< -MAX_STEP;
 wire trig_done;wire signed [15:0] trig_cos,trig_sin;
 heading_lut_fine_pipeline trig(.clk(clk),.reset_n(reset_n),.start(trig_start),.cancel(!enable),
  .angle_tenths(angle),.done(trig_done),.cos_q14(trig_cos),.sin_q14(trig_sin));
 assign x_mm=x_q12>>>12;assign y_mm=y_q12>>>12;
 assign valid=enable && baseline && motor_online && heading_valid && age<STALE_CLKS && !fault &&
  x_mm> -LIMIT_MM && x_mm<LIMIT_MM && y_mm> -LIMIT_MM && y_mm<LIMIT_MM;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin baseline<=0;previous_counts<=0;age<=STALE_CLKS;phase<=0;trig_start<=0;angle<=0;
   f_q12<=0;l_q12<=0;x_q12<=0;y_q12<=0;cosine<=0;sine<=0;dx<=0;dy<=0;fault<=0;
   forward_q<=0;left_q<=0;end
  else begin
   trig_start<=0;
   if(!enable)begin baseline<=0;age<=STALE_CLKS;phase<=0;x_q12<=0;y_q12<=0;fault<=0;end
   else begin
    if(age<STALE_CLKS)age<=age+1'b1;
    if(baseline && (!motor_online || !heading_valid || (age>=STALE_CLKS&&!counts_update)))fault<=1;
    if(counts_update)begin
     previous_counts<=counts;age<=0;
     if(!baseline)baseline<=1;
     else if(jump || phase!=0 || age>=STALE_CLKS)fault<=1;
     else if(heading_valid && motor_online && !fault)begin
      forward_q<=forward_sum;left_q<=left_sum;
      angle<=relative_heading[12:0];phase<=5;
     end
    end
    case(phase)
     5:begin f_q12<=forward_q*FORWARD_Q12;l_q12<=left_q*STRAFE_Q12;phase<=1;end
     1:begin trig_start<=1;phase<=2;end
     2:if(trig_done)begin cosine<=trig_cos;sine<=trig_sin;phase<=3;end
     3:begin dx<=$signed(f_q12)*cosine-$signed(l_q12)*sine;
       dy<=$signed(f_q12)*sine+$signed(l_q12)*cosine;phase<=4;end
     4:begin x_q12<=x_q12+(dx>>>14);y_q12<=y_q12+(dy>>>14);phase<=0;end
    endcase
   end
  end
 end
endmodule
