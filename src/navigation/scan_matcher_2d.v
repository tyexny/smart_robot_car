// Bounded correlative 2-D scan matcher. One clock, one outstanding map read.
// Input endpoints are already in the predicted map frame; input pivot is the
// predicted sensor/car rotation centre. Positive yaw rotates +X toward +Y.
// Only OCCUPIED=2 scores. FREE/UNKNOWN never attract the scan.
//
// Coarse search: x/y +/-2 cells, yaw +/-3 degrees, all 175 combinations.
// A point scores 6 for an occupied centre or 2 for an occupied axial neighbour
// (maximum, never a sum). Out-of-map points receive zero. Pose correction is
// based on CENTRE hits, not the permissive neighbour score. A spatially separated
// competing peak, insufficient support, or unconvincing improvement rejects
// correction. A well-supported unambiguous zero correction can be matched.
// An explicitly trusted stationary caller may also accept ZERO correction
// with broad near support, a minimum centre fraction, a near-best zero score
// and no separated competing zero peak. This never weakens nonzero gates.
//
// A caller must freeze the map for this read-only transaction. load_valid is
// accepted only while idle. busy covers draining a read after cancel; do not
// reset the shared read server separately while leaving an outstanding read.
// done is one pulse; outputs remain until the next start/cancel. No map write.
module scan_matcher_2d #(
    parameter integer MAX_POINTS=128,
    parameter integer CELL_SHIFT=6,
    parameter integer MIN_POINTS=24,
    parameter integer TRANSLATION_RADIUS_CELLS=2,
    parameter integer MIN_HIT_PERCENT=50,
    parameter integer MIN_SCORE_IMPROVEMENT=12,
    parameter integer MIN_IMPROVEMENT_PERCENT=10,
    parameter integer MIN_CORRECTION_HIT_PERCENT=75,
    parameter integer MIN_ZERO_HIT_PERCENT=75,
    parameter integer MIN_ZERO_NEAR_SUPPORT_PERCENT=90,
    parameter integer MIN_ZERO_NEAR_CENTER_PERCENT=30,
    parameter integer MIN_ZERO_NEAR_SCORE_PERCENT=90,
    parameter integer AMBIGUITY_MARGIN=8
)(
    input wire clk,reset_n,cancel,
    input wire load_valid,
    input wire [6:0] load_index,
    input wire signed [15:0] load_x_mm,load_y_mm,
    input wire start,
    input wire allow_zero_near,
    input wire [7:0] point_count,
    input wire signed [15:0] pivot_x_mm,pivot_y_mm,
    output reg busy,done,matched,
    output reg signed [15:0] delta_x_mm,delta_y_mm,
    output reg signed [12:0] delta_yaw_tenths,
    output reg [15:0] best_score,baseline_score,
    output reg [7:0] matched_points,
    // Union: a point with an occupied centre OR an occupied axial neighbour.
    // matched_points remains centre-only, including stationary near-zero.
    output reg [7:0] supported_points,
    // Read-only diagnostics of the evaluated zero-transform hypothesis.
    // A completed search exposes these independently of the selected peak.
    output wire [7:0] baseline_matched_points,baseline_supported_points,
    // Acceptance flags must not be reconstructed from the display reason:
    // weak support can take priority over ambiguity in that classification.
    output wire identity_ambiguous,
    // Full-density verification sweep. The caller streams the very endpoints it
    // loaded for the coarse search, one point per accepted request, and the
    // datapath scores ONE hypothesis against every point: the zero transform
    // (verify_zero=1) or the selected best candidate (verify_zero=0). Nothing is
    // stored, so the 128-entry point array is untouched. verify_ready is high
    // whenever a point may be presented; verify_done pulses once per sweep.
    input wire verify_start,
    input wire verify_zero,
    input wire [11:0] verify_total,
    input wire verify_valid,
    input wire signed [15:0] verify_x_mm,verify_y_mm,
    output reg verify_ready,verify_done,
    output reg [15:0] verify_score,
    output reg [11:0] verify_hits,verify_support,
    output reg [11:0] verify_eligible,
    output reg signed [3:0] selected_x,selected_y,selected_yaw,
    output reg [3:0] reject_reason,
    input wire map_req_ready,map_rsp_valid,
    input wire [1:0] map_rsp_data,
    output reg map_req,
    output reg [13:0] map_addr
);
    localparam integer SIDE=2*TRANSLATION_RADIUS_CELLS+1;
    localparam integer CANDIDATES=SIDE*SIDE*7;
    localparam [4:0] IDLE=0, POINT_LOAD=1, MULTIPLY=2, ROTATE_SUM=3,
        WORLD=4, CELL=5, QUERY=6, WAIT_MAP=7, POINT_END=8,
        CANDIDATE_END=9, COMPARE_INIT=10, COMPARE=11, FINISH=12,
        DRAIN=13,
        V_IDLE=14, V_LOAD=15, V_MULTIPLY=16, V_ROTATE=17, V_WORLD=18,
        V_CELL=19, V_QUERY=20, V_WAIT_MAP=21, V_END=22;
    reg [4:0] state;
    reg [11:0] v_total,v_index;
    reg signed [3:0] v_dx,v_dy,v_yaw;
    reg signed [15:0] v_x,v_y;
    reg [15:0] v_score_acc;
    reg [11:0] v_hits_acc,v_support_acc;
    reg [11:0] v_eligible_acc;
    reg v_eligible;
    reg signed [15:0] points_x[0:MAX_POINTS-1];
    reg signed [15:0] points_y[0:MAX_POINTS-1];
    reg [15:0] scores[0:CANDIDATES-1];
    reg [7:0] hits[0:CANDIDATES-1];
    reg [7:0] count,point_index;
    reg signed [15:0] pivot_x,pivot_y;
    reg signed [3:0] candidate_x,candidate_y,candidate_yaw;
    reg signed [3:0] best_x,best_y,best_yaw;
    reg [7:0] candidate_index,compare_index;
    reg signed [3:0] compare_x,compare_y,compare_yaw;
    reg signed [16:0] relative_x,relative_y;
    reg signed [15:0] cos_q14,sin_q14;
    reg signed [32:0] product_x_cos,product_y_sin,product_x_sin,product_y_cos;
    reg signed [33:0] rotated_x_q14,rotated_y_q14;
    reg signed [31:0] world_x,world_y;
    reg signed [31:0] cell_x,cell_y;
    reg signed [31:0] query_x,query_y;
    reg [2:0] neighbour;
    reg [3:0] point_score;
    reg centre_hit;
    reg [15:0] score_acc;
    reg [7:0] hits_acc,best_hits,baseline_hits,support_acc,best_support,baseline_support;
    reg zero_near_allowed;
    reg [7:0] best_penalty;
    reg [15:0] rival_score;
    reg rival_present;
    reg [15:0] zero_rival_score;
    reg zero_rival_present;
    assign baseline_matched_points=baseline_hits;
    assign baseline_supported_points=baseline_support;

    function [3:0] magnitude;
        input signed [3:0] value;
        begin magnitude=value<0 ? -value : value; end
    endfunction
    wire [7:0] candidate_penalty=(magnitude(candidate_x)+magnitude(candidate_y))*2+
        magnitude(candidate_yaw);
    wire candidate_is_better=score_acc>best_score ||
        (score_acc==best_score && candidate_penalty<best_penalty);
    wire separated=(magnitude(compare_x-best_x)>1) ||
        (magnitude(compare_y-best_y)>1) || (magnitude(compare_yaw-best_yaw)>1);
    wire zero_separated=(magnitude(compare_x)>1) ||
        (magnitude(compare_y)>1) || (magnitude(compare_yaw)>1);
    wire enough_hits=best_hits*100>=count*MIN_HIT_PERCENT;
    // A weak occupied-centre peak is often a grid-alias maximum: many points
    // can be one correct cell away, yet a false yaw happens to land some of
    // them on occupied centres. Require broad centre support before moving
    // the pose. Weak scans may be skipped; they must never invent a rotation.
    wire enough_correction_hits=best_hits*100>=count*MIN_CORRECTION_HIT_PERCENT;
    wire zero_best=best_x==0 && best_y==0 && best_yaw==0;
    wire enough_improvement=best_score>=baseline_score+MIN_SCORE_IMPROVEMENT &&
        (best_score-baseline_score)*100>=baseline_score*MIN_IMPROVEMENT_PERCENT;
    wire ambiguous=rival_present && rival_score+AMBIGUITY_MARGIN>=best_score;
    assign identity_ambiguous=ambiguous;
    wire accept_correction=enough_hits && enough_correction_hits && !zero_best &&
        best_score!=0 && !ambiguous && enough_improvement;
    // Prefer the existing pose when its supported zero hypothesis is within
    // the improvement deadband. This is a real evaluated candidate, not a
    // default pose used to hide a failed/ambiguous association.
    wire accept_zero=baseline_hits*100>=count*MIN_ZERO_HIT_PERCENT && baseline_score!=0 && !ambiguous &&
        (zero_best || !enough_improvement);
    // The permissive path can retain an already trusted stationary pose only.
    // Compare rival scores relative to ZERO as well as the original best-peak
    // ambiguity test: a weak displaced alias must not license map updates.
    wire zero_ambiguous=zero_rival_present && zero_rival_score+AMBIGUITY_MARGIN>=baseline_score;
    wire accept_zero_near=zero_near_allowed && !accept_correction && !ambiguous && !zero_ambiguous &&
        baseline_score!=0 && baseline_support*100>=count*MIN_ZERO_NEAR_SUPPORT_PERCENT &&
        baseline_hits*100>=count*MIN_ZERO_NEAR_CENTER_PERCENT &&
        baseline_score*100>=best_score*MIN_ZERO_NEAR_SCORE_PERCENT;

    // Query address selection contains no multiplication or map lookup.
    always @* begin
        query_x=cell_x;query_y=cell_y;
        case(neighbour)
            1: query_x=cell_x-1;
            2: query_x=cell_x+1;
            3: query_y=cell_y-1;
            4: query_y=cell_y+1;
            default: begin end
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=IDLE;busy<=0;done<=0;matched<=0;
            map_req<=0;map_addr<=0;delta_x_mm<=0;delta_y_mm<=0;delta_yaw_tenths<=0;
            best_score<=0;baseline_score<=0;matched_points<=0;supported_points<=0;reject_reason<=0;
            count<=0;point_index<=0;pivot_x<=0;pivot_y<=0;
            candidate_x<=0;candidate_y<=0;candidate_yaw<=0;candidate_index<=0;
            best_x<=0;best_y<=0;best_yaw<=0;best_hits<=0;baseline_hits<=0;best_penalty<=255;
            support_acc<=0;best_support<=0;baseline_support<=0;zero_near_allowed<=0;
            compare_x<=0;compare_y<=0;compare_yaw<=0;compare_index<=0;
            relative_x<=0;relative_y<=0;cos_q14<=16384;sin_q14<=0;
            product_x_cos<=0;product_y_sin<=0;product_x_sin<=0;product_y_cos<=0;
            rotated_x_q14<=0;rotated_y_q14<=0;world_x<=0;world_y<=0;cell_x<=0;cell_y<=0;
            neighbour<=0;point_score<=0;centre_hit<=0;score_acc<=0;hits_acc<=0;
            rival_score<=0;rival_present<=0;zero_rival_score<=0;zero_rival_present<=0;
            verify_ready<=0;verify_done<=0;verify_score<=0;verify_hits<=0;verify_support<=0;
            v_total<=0;v_index<=0;v_dx<=0;v_dy<=0;v_yaw<=0;
            verify_eligible<=0;v_eligible_acc<=0;v_eligible<=0;
            v_x<=0;v_y<=0;v_score_acc<=0;v_hits_acc<=0;v_support_acc<=0;
            selected_x<=0;selected_y<=0;selected_yaw<=0;
        end else begin
            done<=0;verify_done<=0;
            if(cancel) begin
                matched<=0;delta_x_mm<=0;delta_y_mm<=0;delta_yaw_tenths<=0;
                best_score<=0;baseline_score<=0;matched_points<=0;supported_points<=0;reject_reason<=0;map_req<=0;
                zero_near_allowed<=0;verify_ready<=0;
                // A request can be accepted on the cancellation edge itself.
                // Drain its response before permitting another transaction.
                if((state==WAIT_MAP || state==V_WAIT_MAP || (map_req && map_req_ready) || state==DRAIN) && !map_rsp_valid)
                    begin state<=DRAIN;busy<=1;end
                else begin state<=IDLE;busy<=0;end
            end else case(state)
                IDLE: begin
                    busy<=0;map_req<=0;
                    if(load_valid && load_index<MAX_POINTS) begin
                        points_x[load_index]<=load_x_mm;points_y[load_index]<=load_y_mm;
                    end
                    if(start) begin
                        matched<=0;delta_x_mm<=0;delta_y_mm<=0;delta_yaw_tenths<=0;
                        best_score<=0;baseline_score<=0;matched_points<=0;supported_points<=0;reject_reason<=0;
                        best_hits<=0;baseline_hits<=0;best_penalty<=255;rival_score<=0;rival_present<=0;
                        best_support<=0;baseline_support<=0;zero_rival_score<=0;zero_rival_present<=0;
                        // Procedural if intentionally makes an unconnected/X
                        // optional input disabled in legacy simulations.
                        if(allow_zero_near) zero_near_allowed<=1;
                        else zero_near_allowed<=0;
                        best_x<=0;best_y<=0;best_yaw<=0;
                        if(point_count<MIN_POINTS || point_count>MAX_POINTS) begin done<=1;reject_reason<=2;end
                        else begin
                            busy<=1;count<=point_count;pivot_x<=pivot_x_mm;pivot_y<=pivot_y_mm;
                            candidate_x<=-TRANSLATION_RADIUS_CELLS;
                            candidate_y<=-TRANSLATION_RADIUS_CELLS;candidate_yaw<=-3;
                            candidate_index<=0;point_index<=0;score_acc<=0;hits_acc<=0;support_acc<=0;
                            // sin(-3deg), cos(-3deg), rounded Q14.
                            cos_q14<=16362;sin_q14<=-857;state<=POINT_LOAD;
                        end
                    end else if(verify_start) begin
                        v_total<=verify_total;v_index<=0;
                        v_score_acc<=0;v_hits_acc<=0;v_support_acc<=0;
                        v_eligible_acc<=0;verify_eligible<=0;
                        verify_score<=0;verify_hits<=0;verify_support<=0;
                        v_dx<=verify_zero ? 4'sd0 : selected_x;
                        v_dy<=verify_zero ? 4'sd0 : selected_y;
                        v_yaw<=verify_zero ? 4'sd0 : selected_yaw;
                        verify_ready<=1;busy<=1;state<=V_IDLE;
                    end
                end
                POINT_LOAD: begin
                    relative_x<=$signed({points_x[point_index][15],points_x[point_index]})-
                        $signed({pivot_x[15],pivot_x});
                    relative_y<=$signed({points_y[point_index][15],points_y[point_index]})-
                        $signed({pivot_y[15],pivot_y});
                    state<=MULTIPLY;
                end
                MULTIPLY: begin
                    product_x_cos<=relative_x*cos_q14;product_y_sin<=relative_y*sin_q14;
                    product_x_sin<=relative_x*sin_q14;product_y_cos<=relative_y*cos_q14;
                    state<=ROTATE_SUM;
                end
                ROTATE_SUM: begin
                    rotated_x_q14<=$signed({product_x_cos[32],product_x_cos})-
                        $signed({product_y_sin[32],product_y_sin});
                    rotated_y_q14<=$signed({product_x_sin[32],product_x_sin})+
                        $signed({product_y_cos[32],product_y_cos});
                    state<=WORLD;
                end
                WORLD: begin
                    world_x<=(rotated_x_q14>>>14)+$signed(pivot_x)+
                        ($signed(candidate_x)<<<CELL_SHIFT);
                    world_y<=(rotated_y_q14>>>14)+$signed(pivot_y)+
                        ($signed(candidate_y)<<<CELL_SHIFT);
                    state<=CELL;
                end
                CELL: begin
                    // Arithmetic shift is floor, including negative positions.
                    cell_x<=(world_x>>>CELL_SHIFT)+64;cell_y<=(world_y>>>CELL_SHIFT)+64;
                    neighbour<=0;point_score<=0;centre_hit<=0;state<=QUERY;
                end
                QUERY: begin
                    // Do not let an out-of-map endpoint borrow an in-map
                    // neighbour. The complete point must score zero.
                    if(cell_x<0 || cell_x>127 || cell_y<0 || cell_y>127) begin
                        map_req<=0;state<=POINT_END;
                    end else if(query_x<0 || query_x>127 || query_y<0 || query_y>127) begin
                        map_req<=0;
                        if(neighbour==4) state<=POINT_END;
                        else neighbour<=neighbour+1'b1;
                    end else if(!map_req) begin
                        map_addr<={query_y[6:0],query_x[6:0]};map_req<=1;
                    end else if(map_req_ready) begin map_req<=0;state<=WAIT_MAP;end
                end
                WAIT_MAP: if(map_rsp_valid) begin
                    if(map_rsp_data==2) begin
                        if(neighbour==0) begin point_score<=6;centre_hit<=1;end
                        else if(point_score<2) point_score<=2;
                    end
                    if(neighbour==4) state<=POINT_END;
                    else begin neighbour<=neighbour+1'b1;state<=QUERY;end
                end
                POINT_END: begin
                    score_acc<=score_acc+point_score;hits_acc<=hits_acc+centre_hit;
                    support_acc<=support_acc+(point_score!=0);
                    if(point_index+1'b1==count) state<=CANDIDATE_END;
                    else begin point_index<=point_index+1'b1;state<=POINT_LOAD;end
                end
                CANDIDATE_END: begin
                    scores[candidate_index]<=score_acc;hits[candidate_index]<=hits_acc;
                    if(candidate_x==0 && candidate_y==0 && candidate_yaw==0) begin
                        baseline_score<=score_acc;baseline_hits<=hits_acc;baseline_support<=support_acc;
                    end
                    if(candidate_is_better) begin
                        best_score<=score_acc;best_hits<=hits_acc;best_penalty<=candidate_penalty;
                        best_support<=support_acc;
                        best_x<=candidate_x;best_y<=candidate_y;best_yaw<=candidate_yaw;
                    end
                    if(candidate_index==CANDIDATES-1) state<=COMPARE_INIT;
                    else begin
                        candidate_index<=candidate_index+1'b1;point_index<=0;score_acc<=0;hits_acc<=0;support_acc<=0;
                        if(candidate_x<TRANSLATION_RADIUS_CELLS) candidate_x<=candidate_x+1'b1;
                        else begin
                            candidate_x<=-TRANSLATION_RADIUS_CELLS;
                            if(candidate_y<TRANSLATION_RADIUS_CELLS) candidate_y<=candidate_y+1'b1;
                            else begin
                                candidate_y<=-TRANSLATION_RADIUS_CELLS;candidate_yaw<=candidate_yaw+1'b1;
                                case(candidate_yaw)
                                    -3: begin cos_q14<=16374;sin_q14<=-572;end
                                    -2: begin cos_q14<=16382;sin_q14<=-286;end
                                    -1: begin cos_q14<=16384;sin_q14<=0;end
                                     0: begin cos_q14<=16382;sin_q14<=286;end
                                     1: begin cos_q14<=16374;sin_q14<=572;end
                                     2: begin cos_q14<=16362;sin_q14<=857;end
                                    default: begin cos_q14<=16384;sin_q14<=0;end
                                endcase
                            end
                        end
                        state<=POINT_LOAD;
                    end
                end
                COMPARE_INIT: begin
                    compare_index<=0;compare_x<=-TRANSLATION_RADIUS_CELLS;
                    compare_y<=-TRANSLATION_RADIUS_CELLS;compare_yaw<=-3;
                    rival_score<=0;rival_present<=0;zero_rival_score<=0;zero_rival_present<=0;state<=COMPARE;
                end
                COMPARE: begin
                    if(zero_separated && (!zero_rival_present || scores[compare_index]>zero_rival_score)) begin
                        zero_rival_score<=scores[compare_index];zero_rival_present<=1;
                    end
                    if(separated && hits[compare_index]*100>=count*MIN_HIT_PERCENT &&
                        (!rival_present || scores[compare_index]>rival_score)) begin
                        rival_score<=scores[compare_index];rival_present<=1;
                    end
                    if(compare_index==CANDIDATES-1) state<=FINISH;
                    else begin
                        compare_index<=compare_index+1'b1;
                        if(compare_x<TRANSLATION_RADIUS_CELLS) compare_x<=compare_x+1'b1;
                        else begin
                            compare_x<=-TRANSLATION_RADIUS_CELLS;
                            if(compare_y<TRANSLATION_RADIUS_CELLS) compare_y<=compare_y+1'b1;
                            else begin compare_y<=-TRANSLATION_RADIUS_CELLS;compare_yaw<=compare_yaw+1'b1;end
                        end
                    end
                end
                FINISH: begin
                    // The selected coarse peak stays available to the caller for
                    // full-density verification, independent of the accept gate.
                    selected_x<=best_x;selected_y<=best_y;selected_yaw<=best_yaw;
                    matched<=accept_zero || accept_correction || accept_zero_near;
                    matched_points<=(accept_zero || accept_zero_near) ? baseline_hits : best_hits;
                    supported_points<=(accept_zero || accept_zero_near) ? baseline_support : best_support;
                    // Diagnostic classification only: none of these labels
                    // changes the acceptance predicates above. Weak support
                    // takes priority over a competing weak peak; gain is the
                    // remaining failure once support and uniqueness pass.
                    if(accept_zero || accept_correction || accept_zero_near) reject_reason<=0;
                    else if(!enough_hits || !enough_correction_hits || best_score==0) reject_reason<=7;
                    else if(ambiguous) reject_reason<=8;
                    else reject_reason<=9;
                    if(accept_correction) begin
                        delta_x_mm<=$signed(best_x)<<<CELL_SHIFT;
                        delta_y_mm<=$signed(best_y)<<<CELL_SHIFT;
                        delta_yaw_tenths<=$signed(best_yaw)*10;
                    end
                    done<=1;busy<=0;state<=IDLE;
                end
                DRAIN: if(map_rsp_valid) begin busy<=0;state<=IDLE;end
                V_IDLE: begin
                    if(verify_start) begin
                        v_total<=verify_total;v_index<=0;
                        v_score_acc<=0;v_hits_acc<=0;v_support_acc<=0;
                        v_eligible_acc<=0;verify_eligible<=0;
                        verify_score<=0;verify_hits<=0;verify_support<=0;
                        v_dx<=verify_zero ? 4'sd0 : selected_x;
                        v_dy<=verify_zero ? 4'sd0 : selected_y;
                        v_yaw<=verify_zero ? 4'sd0 : selected_yaw;
                        verify_ready<=1;busy<=1;
                    end else if(verify_valid && verify_ready) begin
                        verify_ready<=0;v_x<=verify_x_mm;v_y<=verify_y_mm;state<=V_LOAD;
                        // Both sweeps use the SAME original in-window cohort.
                        // A candidate cannot improve by dropping difficult points
                        // over an edge or pulling previously invisible points in.
                        v_eligible<=verify_x_mm>=-(64<<CELL_SHIFT) && verify_x_mm<(64<<CELL_SHIFT) &&
                            verify_y_mm>=-(64<<CELL_SHIFT) && verify_y_mm<(64<<CELL_SHIFT);
                    end
                end
                V_LOAD: begin
                    relative_x<=$signed({v_x[15],v_x})-$signed({pivot_x[15],pivot_x});
                    relative_y<=$signed({v_y[15],v_y})-$signed({pivot_y[15],pivot_y});
                    // Q14 rotation of the verified hypothesis, loaded per point.
                    // Indexed by the actual selected yaw (unlike the search
                    // loop, which preloads the NEXT candidate's constants).
                    case(v_yaw)
                        -3: begin cos_q14<=16362;sin_q14<=-857;end
                        -2: begin cos_q14<=16374;sin_q14<=-572;end
                        -1: begin cos_q14<=16382;sin_q14<=-286;end
                         0: begin cos_q14<=16384;sin_q14<=0;end
                         1: begin cos_q14<=16382;sin_q14<=286;end
                         2: begin cos_q14<=16374;sin_q14<=572;end
                        default: begin cos_q14<=16362;sin_q14<=857;end
                    endcase
                    state<=V_MULTIPLY;
                end
                V_MULTIPLY: begin
                    product_x_cos<=relative_x*cos_q14;product_y_sin<=relative_y*sin_q14;
                    product_x_sin<=relative_x*sin_q14;product_y_cos<=relative_y*cos_q14;
                    state<=V_ROTATE;
                end
                V_ROTATE: begin
                    rotated_x_q14<=$signed({product_x_cos[32],product_x_cos})-
                        $signed({product_y_sin[32],product_y_sin});
                    rotated_y_q14<=$signed({product_x_sin[32],product_x_sin})+
                        $signed({product_y_cos[32],product_y_cos});
                    state<=V_WORLD;
                end
                V_WORLD: begin
                    world_x<=(rotated_x_q14>>>14)+$signed(pivot_x)+($signed(v_dx)<<<CELL_SHIFT);
                    world_y<=(rotated_y_q14>>>14)+$signed(pivot_y)+($signed(v_dy)<<<CELL_SHIFT);
                    state<=V_CELL;
                end
                V_CELL: begin
                    cell_x<=(world_x>>>CELL_SHIFT)+64;cell_y<=(world_y>>>CELL_SHIFT)+64;
                    neighbour<=0;point_score<=0;centre_hit<=0;state<=V_QUERY;
                end
                V_QUERY: begin
                    if(!v_eligible || cell_x<0 || cell_x>127 || cell_y<0 || cell_y>127) begin
                        map_req<=0;state<=V_END;
                    end else if(query_x<0 || query_x>127 || query_y<0 || query_y>127) begin
                        map_req<=0;
                        if(neighbour==4) state<=V_END;
                        else neighbour<=neighbour+1'b1;
                    end else if(!map_req) begin
                        map_addr<={query_y[6:0],query_x[6:0]};map_req<=1;
                    end else if(map_req_ready) begin map_req<=0;state<=V_WAIT_MAP;end
                end
                V_WAIT_MAP: if(map_rsp_valid) begin
                    if(map_rsp_data==2) begin
                        if(neighbour==0) begin point_score<=6;centre_hit<=1;end
                        else if(point_score<2) point_score<=2;
                    end
                    if(neighbour==4) state<=V_END;
                    else begin neighbour<=neighbour+1'b1;state<=V_QUERY;end
                end
                V_END: begin
                    v_score_acc<=v_score_acc+point_score;
                    v_hits_acc<=v_hits_acc+centre_hit;
                    v_support_acc<=v_support_acc+(point_score!=0);
                    v_eligible_acc<=v_eligible_acc+v_eligible;
                    if(v_index+1'b1>=v_total) begin
                        verify_score<=v_score_acc+point_score;
                        verify_hits<=v_hits_acc+centre_hit;
                        verify_support<=v_support_acc+(point_score!=0);
                        verify_eligible<=v_eligible_acc+v_eligible;
                        verify_done<=1;verify_ready<=0;busy<=0;
                        state<=IDLE;
                    end else begin
                        v_index<=v_index+1'b1;verify_ready<=1;
                        state<=V_IDLE;
                    end
                end
                default: begin state<=IDLE;busy<=0;map_req<=0;end
            endcase
        end
    end
endmodule
