// Nearest-degree trig, signed Q14. Maximum angular rounding error 0.5deg.
module heading_lut(
    input wire signed [31:0] angle_tenths,
    output reg signed [15:0] cos_q14,sin_q14
);
    function signed [15:0] quarter_sin;
        input integer degree;
        begin
            case(degree)
                0: quarter_sin=16'sd0;
                1: quarter_sin=16'sd286;
                2: quarter_sin=16'sd572;
                3: quarter_sin=16'sd857;
                4: quarter_sin=16'sd1143;
                5: quarter_sin=16'sd1428;
                6: quarter_sin=16'sd1713;
                7: quarter_sin=16'sd1997;
                8: quarter_sin=16'sd2280;
                9: quarter_sin=16'sd2563;
                10: quarter_sin=16'sd2845;
                11: quarter_sin=16'sd3126;
                12: quarter_sin=16'sd3406;
                13: quarter_sin=16'sd3686;
                14: quarter_sin=16'sd3964;
                15: quarter_sin=16'sd4240;
                16: quarter_sin=16'sd4516;
                17: quarter_sin=16'sd4790;
                18: quarter_sin=16'sd5063;
                19: quarter_sin=16'sd5334;
                20: quarter_sin=16'sd5604;
                21: quarter_sin=16'sd5872;
                22: quarter_sin=16'sd6138;
                23: quarter_sin=16'sd6402;
                24: quarter_sin=16'sd6664;
                25: quarter_sin=16'sd6924;
                26: quarter_sin=16'sd7182;
                27: quarter_sin=16'sd7438;
                28: quarter_sin=16'sd7692;
                29: quarter_sin=16'sd7943;
                30: quarter_sin=16'sd8192;
                31: quarter_sin=16'sd8438;
                32: quarter_sin=16'sd8682;
                33: quarter_sin=16'sd8923;
                34: quarter_sin=16'sd9162;
                35: quarter_sin=16'sd9397;
                36: quarter_sin=16'sd9630;
                37: quarter_sin=16'sd9860;
                38: quarter_sin=16'sd10087;
                39: quarter_sin=16'sd10311;
                40: quarter_sin=16'sd10531;
                41: quarter_sin=16'sd10749;
                42: quarter_sin=16'sd10963;
                43: quarter_sin=16'sd11174;
                44: quarter_sin=16'sd11381;
                45: quarter_sin=16'sd11585;
                46: quarter_sin=16'sd11786;
                47: quarter_sin=16'sd11982;
                48: quarter_sin=16'sd12176;
                49: quarter_sin=16'sd12365;
                50: quarter_sin=16'sd12551;
                51: quarter_sin=16'sd12733;
                52: quarter_sin=16'sd12911;
                53: quarter_sin=16'sd13085;
                54: quarter_sin=16'sd13255;
                55: quarter_sin=16'sd13421;
                56: quarter_sin=16'sd13583;
                57: quarter_sin=16'sd13741;
                58: quarter_sin=16'sd13894;
                59: quarter_sin=16'sd14044;
                60: quarter_sin=16'sd14189;
                61: quarter_sin=16'sd14330;
                62: quarter_sin=16'sd14466;
                63: quarter_sin=16'sd14598;
                64: quarter_sin=16'sd14726;
                65: quarter_sin=16'sd14849;
                66: quarter_sin=16'sd14968;
                67: quarter_sin=16'sd15082;
                68: quarter_sin=16'sd15191;
                69: quarter_sin=16'sd15296;
                70: quarter_sin=16'sd15396;
                71: quarter_sin=16'sd15491;
                72: quarter_sin=16'sd15582;
                73: quarter_sin=16'sd15668;
                74: quarter_sin=16'sd15749;
                75: quarter_sin=16'sd15826;
                76: quarter_sin=16'sd15897;
                77: quarter_sin=16'sd15964;
                78: quarter_sin=16'sd16026;
                79: quarter_sin=16'sd16083;
                80: quarter_sin=16'sd16135;
                81: quarter_sin=16'sd16182;
                82: quarter_sin=16'sd16225;
                83: quarter_sin=16'sd16262;
                84: quarter_sin=16'sd16294;
                85: quarter_sin=16'sd16322;
                86: quarter_sin=16'sd16344;
                87: quarter_sin=16'sd16362;
                88: quarter_sin=16'sd16374;
                89: quarter_sin=16'sd16382;
                90: quarter_sin=16'sd16384;
                default:quarter_sin=0;
            endcase
        end
    endfunction
    // The caller wraps to +/-1800. Narrow arithmetic avoids a 32-bit divider.
    reg [11:0] magnitude;
    reg [8:0] degree;
    reg [7:0] acute;
    always @* begin
        magnitude=angle_tenths[12] ? -$signed(angle_tenths[12:0]) : angle_tenths[12:0];
        degree=(magnitude+12'd5)/12'd10;
        if(degree>180) degree=180;
        acute=degree>90 ? 180-degree : degree;
        sin_q14=quarter_sin(acute);
        if(angle_tenths<0) sin_q14=-sin_q14;
        cos_q14=quarter_sin(90-acute);
        if(degree>90) cos_q14=-cos_q14;
    end
endmodule

// Transactional trig pipeline for odometry. The input contract is a wrapped
// +/-1800 tenths-degree signed13 value. start is accepted only in IDLE;
// pulses while a transaction is running are ignored. The acceptance edge
// latches the absolute angle, then four edges perform rounding, quadrant
// folding, independent quarter-sine lookups, and sign application. done is
// a one-clock pulse with both Q14 outputs valid and held until the next done.
// reset_n or synchronous cancel cancels a transaction, including its final
// sign stage; no cancelled result can assert done. cancel has priority over
// start and leaves the previous output data held with done deasserted.
// The original combinational heading_lut above remains unchanged.
module heading_lut_pipeline(
    input wire clk, reset_n, start, cancel,
    input wire signed [12:0] angle_tenths,
    output reg done,
    output reg signed [15:0] cos_q14, sin_q14
);
    localparam [2:0] IDLE=0, ROUND=1, FOLD=2, LOOKUP=3, SIGN=4;
    reg [2:0] state;
    reg [11:0] magnitude;
    reg [8:0] degree;
    reg sine_negative, cosine_negative;
    reg [6:0] sine_degree, cosine_degree;
    reg signed [15:0] sine_magnitude, cosine_magnitude;
    wire [11:0] rounded_magnitude=magnitude+12'd5;
    wire [11:0] rounded_degree=rounded_magnitude/12'd10;
    function signed [15:0] quarter_sin;
        input [6:0] degree;
        begin
            case(degree)
                0: quarter_sin=16'sd0;
                1: quarter_sin=16'sd286;
                2: quarter_sin=16'sd572;
                3: quarter_sin=16'sd857;
                4: quarter_sin=16'sd1143;
                5: quarter_sin=16'sd1428;
                6: quarter_sin=16'sd1713;
                7: quarter_sin=16'sd1997;
                8: quarter_sin=16'sd2280;
                9: quarter_sin=16'sd2563;
                10: quarter_sin=16'sd2845;
                11: quarter_sin=16'sd3126;
                12: quarter_sin=16'sd3406;
                13: quarter_sin=16'sd3686;
                14: quarter_sin=16'sd3964;
                15: quarter_sin=16'sd4240;
                16: quarter_sin=16'sd4516;
                17: quarter_sin=16'sd4790;
                18: quarter_sin=16'sd5063;
                19: quarter_sin=16'sd5334;
                20: quarter_sin=16'sd5604;
                21: quarter_sin=16'sd5872;
                22: quarter_sin=16'sd6138;
                23: quarter_sin=16'sd6402;
                24: quarter_sin=16'sd6664;
                25: quarter_sin=16'sd6924;
                26: quarter_sin=16'sd7182;
                27: quarter_sin=16'sd7438;
                28: quarter_sin=16'sd7692;
                29: quarter_sin=16'sd7943;
                30: quarter_sin=16'sd8192;
                31: quarter_sin=16'sd8438;
                32: quarter_sin=16'sd8682;
                33: quarter_sin=16'sd8923;
                34: quarter_sin=16'sd9162;
                35: quarter_sin=16'sd9397;
                36: quarter_sin=16'sd9630;
                37: quarter_sin=16'sd9860;
                38: quarter_sin=16'sd10087;
                39: quarter_sin=16'sd10311;
                40: quarter_sin=16'sd10531;
                41: quarter_sin=16'sd10749;
                42: quarter_sin=16'sd10963;
                43: quarter_sin=16'sd11174;
                44: quarter_sin=16'sd11381;
                45: quarter_sin=16'sd11585;
                46: quarter_sin=16'sd11786;
                47: quarter_sin=16'sd11982;
                48: quarter_sin=16'sd12176;
                49: quarter_sin=16'sd12365;
                50: quarter_sin=16'sd12551;
                51: quarter_sin=16'sd12733;
                52: quarter_sin=16'sd12911;
                53: quarter_sin=16'sd13085;
                54: quarter_sin=16'sd13255;
                55: quarter_sin=16'sd13421;
                56: quarter_sin=16'sd13583;
                57: quarter_sin=16'sd13741;
                58: quarter_sin=16'sd13894;
                59: quarter_sin=16'sd14044;
                60: quarter_sin=16'sd14189;
                61: quarter_sin=16'sd14330;
                62: quarter_sin=16'sd14466;
                63: quarter_sin=16'sd14598;
                64: quarter_sin=16'sd14726;
                65: quarter_sin=16'sd14849;
                66: quarter_sin=16'sd14968;
                67: quarter_sin=16'sd15082;
                68: quarter_sin=16'sd15191;
                69: quarter_sin=16'sd15296;
                70: quarter_sin=16'sd15396;
                71: quarter_sin=16'sd15491;
                72: quarter_sin=16'sd15582;
                73: quarter_sin=16'sd15668;
                74: quarter_sin=16'sd15749;
                75: quarter_sin=16'sd15826;
                76: quarter_sin=16'sd15897;
                77: quarter_sin=16'sd15964;
                78: quarter_sin=16'sd16026;
                79: quarter_sin=16'sd16083;
                80: quarter_sin=16'sd16135;
                81: quarter_sin=16'sd16182;
                82: quarter_sin=16'sd16225;
                83: quarter_sin=16'sd16262;
                84: quarter_sin=16'sd16294;
                85: quarter_sin=16'sd16322;
                86: quarter_sin=16'sd16344;
                87: quarter_sin=16'sd16362;
                88: quarter_sin=16'sd16374;
                89: quarter_sin=16'sd16382;
                90: quarter_sin=16'sd16384;
                default:quarter_sin=0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE; done<=0; cos_q14<=0; sin_q14<=0;
            magnitude<=0; degree<=0; sine_negative<=0; cosine_negative<=0;
            sine_degree<=0; cosine_degree<=0;
            sine_magnitude<=0; cosine_magnitude<=0;
        end else begin
            done<=0;
            if(cancel) state<=IDLE;
            else case(state)
                IDLE:if(start) begin
                    magnitude<=angle_tenths[12] ? -angle_tenths : angle_tenths;
                    sine_negative<=angle_tenths[12];
                    state<=ROUND;
                end
                ROUND:begin
                    degree<=rounded_degree>12'd180 ? 9'd180 : rounded_degree[8:0];
                    state<=FOLD;
                end
                FOLD:begin
                    sine_degree<=degree>9'd90 ? 9'd180-degree : degree;
                    // Compute the complementary angle directly from degree,
                    // avoiding an acute-angle mux followed by a subtractor.
                    cosine_degree<=degree>9'd90 ? degree-9'd90 : 9'd90-degree;
                    cosine_negative<=degree>9'd90;
                    state<=LOOKUP;
                end
                LOOKUP:begin
                    sine_magnitude<=quarter_sin(sine_degree);
                    cosine_magnitude<=quarter_sin(cosine_degree);
                    state<=SIGN;
                end
                SIGN:begin
                    sin_q14<=sine_negative ? -sine_magnitude : sine_magnitude;
                    cos_q14<=cosine_negative ? -cosine_magnitude : cosine_magnitude;
                    done<=1;
                    state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule


// Fine transactional trig for scan rays. Quarter-degree coefficients are
// linearly interpolated at the exact tenth-degree input; legacy modules
// above retain their nearest-degree behavior. Seven edges after acceptance
// produce done, with cancel taking priority through the final sign stage.
module heading_lut_fine_pipeline(
    input wire clk,reset_n,start,cancel,
    input wire signed [12:0] angle_tenths,
    output reg done,
    output reg signed [15:0] cos_q14,sin_q14
);
    localparam [3:0] IDLE=0,FOLD=1,SPLIT=2,LOOKUP=3,DELTA=4,WEIGHT=5,INTERPOLATE=6,SIGN=7;
    reg [3:0] state;
    reg [11:0] magnitude,acute,complement;
    reg sine_negative,cosine_negative;
    reg [6:0] sine_degree,cosine_degree;
    reg [3:0] sine_fraction,cosine_fraction;
    reg signed [15:0] sine_low,sine_high,cosine_low,cosine_high;
    reg signed [16:0] sine_delta,cosine_delta;
    reg signed [21:0] sine_weight,cosine_weight;
    reg signed [15:0] sine_magnitude,cosine_magnitude;
    // For the wrapped +/-1800 input contract, quarter-sine is monotonic:
    // delta<=286 and fraction<=9, hence weight<=2574 and weight+5<=2579.
    // Bound the unsigned divider to 12 bits before the 16-bit magnitude add.
    wire [11:0] sine_rounded_weight=sine_weight[11:0]+12'd5;
    wire [11:0] cosine_rounded_weight=cosine_weight[11:0]+12'd5;
    wire [8:0] sine_correction=sine_rounded_weight/12'd10;
    wire [8:0] cosine_correction=cosine_rounded_weight/12'd10;
    function signed [15:0] quarter_sin;
        input [6:0] degree;
        begin
            case(degree)
                0: quarter_sin=16'sd0;
                1: quarter_sin=16'sd286;
                2: quarter_sin=16'sd572;
                3: quarter_sin=16'sd857;
                4: quarter_sin=16'sd1143;
                5: quarter_sin=16'sd1428;
                6: quarter_sin=16'sd1713;
                7: quarter_sin=16'sd1997;
                8: quarter_sin=16'sd2280;
                9: quarter_sin=16'sd2563;
                10: quarter_sin=16'sd2845;
                11: quarter_sin=16'sd3126;
                12: quarter_sin=16'sd3406;
                13: quarter_sin=16'sd3686;
                14: quarter_sin=16'sd3964;
                15: quarter_sin=16'sd4240;
                16: quarter_sin=16'sd4516;
                17: quarter_sin=16'sd4790;
                18: quarter_sin=16'sd5063;
                19: quarter_sin=16'sd5334;
                20: quarter_sin=16'sd5604;
                21: quarter_sin=16'sd5872;
                22: quarter_sin=16'sd6138;
                23: quarter_sin=16'sd6402;
                24: quarter_sin=16'sd6664;
                25: quarter_sin=16'sd6924;
                26: quarter_sin=16'sd7182;
                27: quarter_sin=16'sd7438;
                28: quarter_sin=16'sd7692;
                29: quarter_sin=16'sd7943;
                30: quarter_sin=16'sd8192;
                31: quarter_sin=16'sd8438;
                32: quarter_sin=16'sd8682;
                33: quarter_sin=16'sd8923;
                34: quarter_sin=16'sd9162;
                35: quarter_sin=16'sd9397;
                36: quarter_sin=16'sd9630;
                37: quarter_sin=16'sd9860;
                38: quarter_sin=16'sd10087;
                39: quarter_sin=16'sd10311;
                40: quarter_sin=16'sd10531;
                41: quarter_sin=16'sd10749;
                42: quarter_sin=16'sd10963;
                43: quarter_sin=16'sd11174;
                44: quarter_sin=16'sd11381;
                45: quarter_sin=16'sd11585;
                46: quarter_sin=16'sd11786;
                47: quarter_sin=16'sd11982;
                48: quarter_sin=16'sd12176;
                49: quarter_sin=16'sd12365;
                50: quarter_sin=16'sd12551;
                51: quarter_sin=16'sd12733;
                52: quarter_sin=16'sd12911;
                53: quarter_sin=16'sd13085;
                54: quarter_sin=16'sd13255;
                55: quarter_sin=16'sd13421;
                56: quarter_sin=16'sd13583;
                57: quarter_sin=16'sd13741;
                58: quarter_sin=16'sd13894;
                59: quarter_sin=16'sd14044;
                60: quarter_sin=16'sd14189;
                61: quarter_sin=16'sd14330;
                62: quarter_sin=16'sd14466;
                63: quarter_sin=16'sd14598;
                64: quarter_sin=16'sd14726;
                65: quarter_sin=16'sd14849;
                66: quarter_sin=16'sd14968;
                67: quarter_sin=16'sd15082;
                68: quarter_sin=16'sd15191;
                69: quarter_sin=16'sd15296;
                70: quarter_sin=16'sd15396;
                71: quarter_sin=16'sd15491;
                72: quarter_sin=16'sd15582;
                73: quarter_sin=16'sd15668;
                74: quarter_sin=16'sd15749;
                75: quarter_sin=16'sd15826;
                76: quarter_sin=16'sd15897;
                77: quarter_sin=16'sd15964;
                78: quarter_sin=16'sd16026;
                79: quarter_sin=16'sd16083;
                80: quarter_sin=16'sd16135;
                81: quarter_sin=16'sd16182;
                82: quarter_sin=16'sd16225;
                83: quarter_sin=16'sd16262;
                84: quarter_sin=16'sd16294;
                85: quarter_sin=16'sd16322;
                86: quarter_sin=16'sd16344;
                87: quarter_sin=16'sd16362;
                88: quarter_sin=16'sd16374;
                89: quarter_sin=16'sd16382;
                90: quarter_sin=16'sd16384;
                default:quarter_sin=0;
            endcase
        end
    endfunction
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;done<=0;cos_q14<=0;sin_q14<=0;
            magnitude<=0;acute<=0;complement<=0;sine_negative<=0;cosine_negative<=0;
            sine_degree<=0;cosine_degree<=0;sine_fraction<=0;cosine_fraction<=0;
            sine_low<=0;sine_high<=0;cosine_low<=0;cosine_high<=0;
            sine_delta<=0;cosine_delta<=0;sine_weight<=0;cosine_weight<=0;
            sine_magnitude<=0;cosine_magnitude<=0;
        end else begin
            done<=0;
            if(cancel) state<=IDLE;
            else case(state)
                IDLE:if(start) begin
                    magnitude<=angle_tenths[12] ? -angle_tenths : angle_tenths;
                    sine_negative<=angle_tenths[12];state<=FOLD;
                end
                FOLD:begin
                    acute<=magnitude>12'd900 ? 12'd1800-magnitude : magnitude;
                    complement<=magnitude>12'd900 ? magnitude-12'd900 : 12'd900-magnitude;
                    cosine_negative<=magnitude>12'd900;state<=SPLIT;
                end
                SPLIT:begin
                    sine_degree<=acute/12'd10;sine_fraction<=acute%12'd10;
                    cosine_degree<=complement/12'd10;cosine_fraction<=complement%12'd10;
                    state<=LOOKUP;
                end
                LOOKUP:begin
                    sine_low<=quarter_sin(sine_degree);
                    sine_high<=quarter_sin(sine_degree>=7'd90 ? 7'd90 : sine_degree+7'd1);
                    cosine_low<=quarter_sin(cosine_degree);
                    cosine_high<=quarter_sin(cosine_degree>=7'd90 ? 7'd90 : cosine_degree+7'd1);
                    state<=DELTA;
                end
                DELTA:begin
                    sine_delta<=$signed({sine_high[15],sine_high})-$signed({sine_low[15],sine_low});
                    cosine_delta<=$signed({cosine_high[15],cosine_high})-$signed({cosine_low[15],cosine_low});
                    state<=WEIGHT;
                end
                WEIGHT:begin
                    sine_weight<=sine_delta*$signed({1'b0,sine_fraction});
                    cosine_weight<=cosine_delta*$signed({1'b0,cosine_fraction});state<=INTERPOLATE;
                end
                INTERPOLATE:begin
                    sine_magnitude<=sine_low+$signed({7'd0,sine_correction});
                    cosine_magnitude<=cosine_low+$signed({7'd0,cosine_correction});state<=SIGN;
                end
                SIGN:begin
                    sin_q14<=sine_negative ? -sine_magnitude : sine_magnitude;
                    cos_q14<=cosine_negative ? -cosine_magnitude : cosine_magnitude;
                    done<=1;state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
