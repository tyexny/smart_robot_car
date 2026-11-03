// Bounded heading hold, yaw in tenths of degrees. Positive turn is right.
// Maintain one reference through forward/backward/strafe transitions; explicit
// STOP, rotation, ownership or SW6 changes release the session in the caller.
// Optional frame-quantized P/I/D has bounded integration. Translation leaves
// wheel budget for yaw; return mode can blend lateral travel into forward.
module imu_heading_hold #(
 parameter integer CLK_FREQ_HZ=50000000,MOTOR_SPEED_SCALE=2,
 parameter integer RIGHT_YAW_POSITIVE=0,STALE_MS=300,MAX_STEP_TENTHS=450,
 parameter integer DEADBAND_TENTHS=8,MEDIUM_TENTHS=60,LARGE_TENTHS=120,
 parameter integer MAX_TURN_UNITS=3,
 parameter integer FRACTIONAL_CONTROL=0,CONTROL_MS=20,SLEW_PER_FRAME=2,RETURN_LINE_BLEND=0,
 parameter integer KP_Q8=16,KD_Q8=128,KI_Q8=1,INTEGRAL_LIMIT_Q8=128
)(
 input wire clk,reset_n,enable,turn_allowed,
 input wire [3:0] motion,input wire [7:0] speed,
 input wire signed [31:0] yaw,input wire yaw_valid,yaw_update,
 input wire frame_committed,
 input wire returning,input wire signed [15:0] lateral_offset_mm,
 output reg active,turning,
 output reg [3:0] out_motion,output reg [7:0] out_speed,
 output reg signed [15:0] wheel0,wheel1,wheel2,wheel3,
 output reg signed [15:0] out_turn,
 output wire ready,
 output wire signed [15:0] error_tenths,
 output reg signed [15:0] reference_yaw
);
 localparam integer STALE_CLKS=(CLK_FREQ_HZ/1000*STALE_MS<1)?1:CLK_FREQ_HZ/1000*STALE_MS;
 reg seen,reference_valid;
 reg [31:0] age;
 reg signed [15:0] current_yaw;
 function signed [15:0] wrap(input signed [31:0] x);
 begin wrap=x>=1800?x-3600:x< -1800?x+3600:x;end
 endfunction
 function signed [15:0] difference(input signed [15:0] a,b);
 reg signed [16:0] d;
 begin d=$signed({a[15],a})-$signed({b[15],b});difference=d>1800?d-3600:d< -1800?d+3600:d;end
 endfunction
 // Normalize a coherent sample before the jump/health checks. Otherwise the
 // wrap add/subtract, wrapped jump comparison and live UART fault arbitration
 // become one combinational path. This stage adds one 50 MHz clock (20 ns).
 reg in_range,sample_valid,sample_update;
 reg signed [15:0] normalized;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin in_range<=0;sample_valid<=0;sample_update<=0;normalized<=0;end
  else begin
   in_range<=yaw>=-3600 && yaw<3600;
   normalized<=wrap(yaw);
   sample_valid<=yaw_valid;
   sample_update<=enable && yaw_update;
  end
 end
 wire signed [15:0] jump=difference(normalized,current_yaw);
 wire acceptable=in_range && (!seen || (jump<=MAX_STEP_TENTHS && jump>=-MAX_STEP_TENTHS));
 assign ready=enable && seen && reference_valid && yaw_valid && age<STALE_CLKS &&
     !(sample_update && !acceptable);
 assign error_tenths=difference(reference_yaw,current_yaw);
 reg signed [15:0] control_error;
 reg signed [15:0] sampled_delta,sampled_control_yaw;
 // P/I error, D delta and the derivative reference sample belong to the same
 // edge. This separates both wrapped subtractions from PID arithmetic. The
 // public diagnostic and final safety veto stay live. At 50MHz the control
 // input is delayed 20ns; its fixed-period sampling/quantization is unchanged.
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin control_error<=0;sampled_delta<=0;sampled_control_yaw<=0;end
  else begin
   sampled_control_yaw<=current_yaw;
   if(!ready)begin control_error<=0;sampled_delta<=0;end
   else begin
    control_error<=error_tenths;
    sampled_delta<=difference(current_yaw,previous_control_yaw);
   end
  end
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin seen<=0;reference_valid<=0;age<=STALE_CLKS;current_yaw<=0;reference_yaw<=0;end
  else if(!enable)begin seen<=0;reference_valid<=0;age<=STALE_CLKS;end
  else begin
   if(age<STALE_CLKS)age<=age+1'b1;
   if(!yaw_valid)age<=STALE_CLKS;
   if(sample_update && sample_valid && yaw_valid)begin
    if(acceptable)begin
     current_yaw<=normalized;seen<=1;age<=0;
     if(!reference_valid)begin reference_yaw<=normalized;reference_valid<=1;end
    end else age<=STALE_CLKS;
   end
  end
 end
 reg signed [15:0] f,l,t,drive,right_error;
 reg [15:0] budget,turn_units,turn_limit,error_abs;
 reg [15:0] lateral_magnitude;
 wire [15:0] offset_absolute=lateral_offset_mm<0?-lateral_offset_mm:lateral_offset_mm;
 reg [15:0] return_units;
 reg return_positive;
 // Keep the position conversion out of the clamp/mix/slew chain. Magnitude
 // and direction are captured together; control latency is one 50MHz edge.
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin return_units<=0;return_positive<=0;end
  else begin return_units<=({1'b0,offset_absolute}+17'd9)/17'd10;return_positive<=lateral_offset_mm>0;end
 end
 // P/I/D operates at a fixed period, independent of IMU packet spacing.
 // Quantization is advanced by ACTUAL UART frame commits, avoiding aliasing
 // between a fast PWM/dither clock and the slower 50ms motor update period.
 localparam integer CONTROL_CLKS=(CLK_FREQ_HZ/1000*CONTROL_MS<1)?1:CLK_FREQ_HZ/1000*CONTROL_MS;
 reg [31:0] control_age;
 reg signed [15:0] previous_control_yaw,pd_turn;
 reg signed [31:0] integral_q8,demand_q8;
 reg [7:0] fraction;reg previous_sign;reg [2:0] integral_div;
 // Preserve the original one-clock-period parameter corner: a delayed D
 // input cannot feed back its previous sample in a single cycle. Real board
 // control periods are 1,000,000 clocks and use the registered path.
 wire signed [15:0] pid_error=CONTROL_CLKS>1?control_error:error_tenths;
 wire signed [15:0] pid_yaw=CONTROL_CLKS>1?sampled_control_yaw:current_yaw;
 wire signed [15:0] pd_error=RIGHT_YAW_POSITIVE?pid_error:-pid_error;
 wire signed [15:0] sample_delta=CONTROL_CLKS>1?sampled_delta:difference(current_yaw,previous_control_yaw);
 wire signed [15:0] right_delta=RIGHT_YAW_POSITIVE?sample_delta:-sample_delta;
 reg signed [31:0] next_integral,raw_demand,bounded_demand;
 reg [31:0] demand_abs;
 reg [8:0] fractional_sum;
 integer pd_limit;
 always @*begin
  next_integral=integral_q8;
  if(pd_error<=DEADBAND_TENTHS && pd_error>=-DEADBAND_TENTHS)begin
   if(integral_q8>0)next_integral=integral_q8-1;
   else if(integral_q8<0)next_integral=integral_q8+1;
  end else if(integral_div==4 && pd_error<100 && pd_error> -100)begin
   // Conditional integration prevents windup while the P/D demand saturates.
   if(demand_q8<MAX_TURN_UNITS*256 && demand_q8> -MAX_TURN_UNITS*256)
    next_integral=integral_q8+pd_error*KI_Q8;
  end
  if(next_integral>INTEGRAL_LIMIT_Q8)next_integral=INTEGRAL_LIMIT_Q8;
  if(next_integral< -INTEGRAL_LIMIT_Q8)next_integral=-INTEGRAL_LIMIT_Q8;
  raw_demand=(pd_error<=DEADBAND_TENTHS && pd_error>=-DEADBAND_TENTHS?0:pd_error*KP_Q8)-right_delta*KD_Q8+next_integral;
  bounded_demand=raw_demand>MAX_TURN_UNITS*256?MAX_TURN_UNITS*256:
   raw_demand< -MAX_TURN_UNITS*256?-MAX_TURN_UNITS*256:raw_demand;
  demand_abs=demand_q8<0?-demand_q8:demand_q8;
  fractional_sum={1'b0,demand_abs[7:0]}+((previous_sign==demand_q8[31])?{1'b0,fraction}:9'd0);
  pd_limit=(speed*MOTOR_SPEED_SCALE)>>1;
  if(pd_limit>MAX_TURN_UNITS)pd_limit=MAX_TURN_UNITS;
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin control_age<=0;previous_control_yaw<=0;integral_q8<=0;demand_q8<=0;
   pd_turn<=0;fraction<=0;previous_sign<=0;integral_div<=0;end
  else if(!FRACTIONAL_CONTROL || !ready || motion<1 || motion>4 || speed==0 || !turn_allowed)begin
   control_age<=0;previous_control_yaw<=pid_yaw;integral_q8<=0;demand_q8<=0;
   pd_turn<=0;fraction<=0;integral_div<=0;
  end else begin
   if(control_age==CONTROL_CLKS-1)begin
    control_age<=0;previous_control_yaw<=pid_yaw;
    integral_div<=integral_div==4?0:integral_div+1'b1;
    integral_q8<=next_integral;demand_q8<=bounded_demand;
   end else control_age<=control_age+1'b1;
   if(frame_committed)begin
    fraction<=fractional_sum[7:0];previous_sign<=demand_q8[31];
    if((demand_abs>>8)+fractional_sum[8]>pd_limit)pd_turn<=demand_q8[31]?-pd_limit:pd_limit;
    else pd_turn<=demand_q8[31]?-$signed((demand_abs>>8)+fractional_sum[8]):$signed((demand_abs>>8)+fractional_sum[8]);
   end
  end
 end
 always @*begin
  budget=speed*MOTOR_SPEED_SCALE;turn_limit=budget>>1;
  right_error=RIGHT_YAW_POSITIVE?control_error:-control_error;
  error_abs=right_error<0?-right_error:right_error;
  turn_units=error_abs<=DEADBAND_TENTHS?0:error_abs<LARGE_TENTHS?
      (error_abs<MEDIUM_TENTHS?1:2):3;
  if(FRACTIONAL_CONTROL)turn_units=pd_turn<0?-pd_turn:pd_turn;
  if(turn_units>MAX_TURN_UNITS)turn_units=MAX_TURN_UNITS;
  if(turn_units>turn_limit)turn_units=turn_limit;
  if(!turn_allowed)turn_units=0;
  t=right_error<0?-$signed(turn_units):$signed(turn_units);
  if(FRACTIONAL_CONTROL)t=pd_turn<0?-$signed(turn_units):$signed(turn_units);
  drive=$signed(budget-turn_units);f=0;l=0;
  lateral_magnitude=lateral_offset_mm<0?-lateral_offset_mm:lateral_offset_mm;
  case(motion)
   1:f=drive;2:f=-drive;3:l=drive;4:l=-drive;default:begin end
  endcase
  // Keep the requested grade budget while smoothly steering the return
  // translation into the original forward direction. This is position-based
  // vector blending, not a final one-grade crawl or a timed STOP.
  if(RETURN_LINE_BLEND && returning && (motion==3||motion==4))begin
   lateral_magnitude=return_units;
   if(lateral_magnitude>drive)lateral_magnitude=drive;
   l=return_positive?-$signed(lateral_magnitude):$signed(lateral_magnitude);
   f=drive-$signed(lateral_magnitude);
  end
 end
 function signed [15:0] slew(input signed [15:0] old_value,target);
  reg signed [16:0] upper,lower,extended_target;
  localparam signed [16:0] STEP=SLEW_PER_FRAME;
  begin
   upper=$signed({old_value[15],old_value})+STEP;
   lower=$signed({old_value[15],old_value})-STEP;
   extended_target={target[15],target};
   slew=extended_target>upper?upper:extended_target<lower?lower:target;
  end
 endfunction
 // Components and their complete policy metadata cross together. The
 // next stage only remixes wheels and applies the existing frame slew.
 reg signed [15:0] mix_f,mix_l,mix_t;
 reg [15:0] mix_budget,mix_turn_units;
 reg [3:0] mix_motion;
 reg [7:0] mix_speed;
 reg mix_turn_allowed,mix_returning,mix_ready;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   mix_f<=0;mix_l<=0;mix_t<=0;mix_budget<=0;mix_turn_units<=0;
   mix_motion<=0;mix_speed<=0;mix_turn_allowed<=0;mix_returning<=0;mix_ready<=0;
  end else begin
   mix_f<=f;mix_l<=l;mix_t<=t;mix_budget<=budget;mix_turn_units<=turn_units;
   mix_motion<=motion;mix_speed<=speed;mix_turn_allowed<=turn_allowed;
   mix_returning<=returning;mix_ready<=ready;
  end
 end
 reg signed [15:0] target0,target1,target2,target3;
 always @*begin
  if(!mix_turn_allowed && !(RETURN_LINE_BLEND && mix_returning && (mix_motion==3||mix_motion==4)))case(mix_motion)
   1:begin target0=-$signed(mix_budget);target1=mix_budget;target2=mix_budget;target3=mix_budget;end
   2:begin target0=mix_budget;target1=-$signed(mix_budget);target2=-$signed(mix_budget);target3=-$signed(mix_budget);end
   3:begin target0=mix_budget;target1=mix_budget;target2=mix_budget;target3=-$signed(mix_budget);end
   4:begin target0=-$signed(mix_budget);target1=-$signed(mix_budget);target2=-$signed(mix_budget);target3=mix_budget;end
   default:begin target0=0;target1=0;target2=0;target3=0;end
  endcase
  else begin target0=-mix_f+mix_l-mix_t;target1=mix_f+mix_l-mix_t;target2=mix_f+mix_l+mix_t;target3=mix_f-mix_l-mix_t;end
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin active<=0;turning<=0;out_motion<=0;out_speed<=0;wheel0<=0;wheel1<=0;wheel2<=0;wheel3<=0;out_turn<=0;end
  else begin
   active<=ready && motion>=1 && motion<=4 && speed!=0 && mix_ready && mix_motion>=1 && mix_motion<=4 && mix_speed!=0;
   turning<=ready && motion>=1 && motion<=4 && speed!=0 && mix_ready && mix_motion>=1 && mix_motion<=4 && mix_speed!=0 && mix_turn_allowed && mix_turn_units!=0;
   out_motion<=motion==0?4'd0:mix_motion;out_speed<=motion>=1 && motion<=4 && mix_motion>=1 && mix_motion<=4?mix_speed:8'd0;
   out_turn<=ready && motion>=1 && motion<=4 && speed!=0 && turn_allowed && mix_ready && mix_turn_allowed?mix_t:16'sd0;
   // Put live readiness/rotation vetoes at the final mux, instead of before
   // subtraction and four-wheel mixing. Blocked yaw still permits the same
   // plain translation as the reference policy, with the full speed budget.
   if(!ready || motion<1 || motion>4 || speed==0 || !mix_ready || mix_motion<1 || mix_motion>4 || mix_speed==0)begin wheel0<=0;wheel1<=0;wheel2<=0;wheel3<=0;end
   else if(FRACTIONAL_CONTROL)begin
    if(frame_committed)begin
     wheel0<=slew(wheel0,target0);wheel1<=slew(wheel1,target1);
     wheel2<=slew(wheel2,target2);wheel3<=slew(wheel3,target3);
    end
   end else begin wheel0<=target0;wheel1<=target1;wheel2<=target2;wheel3<=target3;end
  end
 end
endmodule
