// Scan map: 128 x 128 cells. CELL_SHIFT selects the millimetres per cell.
// Address = {cell_y, cell_x}; (0 mm, 0 mm) is cell (64, 64).
// Cells are UNKNOWN=0, FREE=1, OCCUPIED=2. No angular-sector filling,
// endpoint interpolation, or morphological filtering is performed.
//
// A ray is accepted only while !busy, with ray_update && ray_valid, and
// a valid endpoint within +/-8192 mm on both axes. Pulses during busy are lost.
// The visible map covers [-1024, 1024) mm on both axes. A measured endpoint
// inside this map becomes OCCUPIED; preceding Bresenham cells become FREE,
// except that an existing OCCUPIED cell is preserved until clear/reset.
// A zero-length ray therefore writes only an OCCUPIED center cell.
// An endpoint beyond the visible map stays at its true, virtual-grid position:
// its Bresenham path is written FREE up to the last in-map cell, then stops.
// The border is never made OCCUPIED merely because a ray leaves the map.
// The +/-8192 mm guard exceeds the sensor's valid 4000 mm range while bounding
// the rasterizer arithmetic; larger or malformed coordinates are rejected.
//
// reset_n starts a sequential RAM clear; clear restarts that clear and
// interrupts any active ray. busy covers initialization, clearing and rays.
// clear_done pulses for one write clock when all 16384 cells are cleared;
// map_update pulses on clear completion or completion of an accepted ray.
// Holding clear high restarts at address zero; release it to finish clearing.
//
// RAM has no asynchronous reset: port A reads/writes on clk (50 MHz),
// port B reads synchronously on rd_clk (75 MHz). rd_data has one rd_clk
// edge of latency and is undefined before initial clear completes. A display
// may see a partially updated map; simultaneous read/write of the same cell
// has device-specific old/new-data behavior and must not be relied upon.
module scan_map_memory #(
    parameter integer CELL_SHIFT=4, ENDPOINT_LIMIT_MM=8192,
    parameter integer CELL_MM=(1<<CELL_SHIFT), GRID_SIZE=128, GRID_X=GRID_SIZE, GRID_Y=GRID_SIZE, GRID_BITS=$clog2(GRID_X>GRID_Y?GRID_X:GRID_Y),
    parameter integer ENABLE_RAY_START=0, ENABLE_PLANNER=0,
    parameter integer ENABLE_VEHICLE_FOOTPRINT=0,
    parameter integer BODY_HALF_LENGTH_MM=140, BODY_HALF_WIDTH_MM=105,
    parameter integer BODY_FRONT_MM=BODY_HALF_LENGTH_MM, BODY_REAR_MM=BODY_HALF_LENGTH_MM,
    parameter integer ENABLE_ROTATION_SWEEP=0, BODY_SWEEP_RADIUS_MM=140,
    // The default branch preserves the historical sticky-occupied map.
    // Confidence mode stores signed evidence in [-7,+7]. With scan epochs,
    // a complete scan contributes at most one miss OR one hit per cell;
    // a hit supersedes an earlier miss in the same scan. Metadata is stored
    // in the same dual-port RAM word, rather than a third memory read port.
    parameter integer ENABLE_CONFIDENCE=0, ENABLE_SCAN_EPOCH=0,
    // Explicit banks keep evidence feedback local to its physical RAMs.
    // REGISTER_RD_ADDRESS moves the existing display address register into
    // each bank; its input must then be the preceding, raw pixel address.
    parameter integer CONFIDENCE_BANK_BITS=0, REGISTER_RD_ADDRESS=0, DISPLAY_ADDRESS_COMPACT=0,
    parameter integer HIT_EVIDENCE=3,
    parameter integer OCCUPIED_EVIDENCE=2,
    // Optional display-only blank band. The board keeps real edge obstacles:
    // UNKNOWN is traversable under its existing navigation policy.
    parameter integer MAP_EDGE_CELLS=0
)(
    input wire clk,
    input wire reset_n,
    input wire clear,
    // One pulse after a confirmed complete in-place rotation. This records
    // only the inner disk physically swept by the car, preserving obstacles.
    // The board chooses 140mm, conservatively inside the 175mm corner radius.
    input wire rotation_sweep,
    input wire ray_update,
    input wire signed [31:0] ray_end_x_mm,
    input wire signed [31:0] ray_end_y_mm,
    input wire signed [31:0] ray_start_x_mm,ray_start_y_mm,
    input wire ray_valid,
    // Stable before the first ray and until every ray in that scan finishes.
    // Increment only for a newly accepted complete scan. A backwards change
    // (including 255->0) scrubs metadata while preserving map evidence. busy
    // blocks new rays/planner requests throughout that 32768-clock scrub.
    // Ignored, including an unconnected/X value, unless both enables are 1.
    input wire [7:0] scan_epoch,
    input wire planner_req,
    input wire [2*GRID_BITS-1:0] planner_addr,
    output wire planner_req_ready,
    output reg planner_rsp_valid,
    output wire [1:0] planner_rsp_data,
    input wire rd_clk,
    input wire [2*GRID_BITS-1:0] rd_addr,
    output wire [1:0] rd_data,
    output wire busy,
    output reg map_update,
    output reg clear_done
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
    localparam [1:0] UNKNOWN = 2'd0;
    localparam [1:0] FREE = 2'd1;
    localparam [1:0] OCCUPIED = 2'd2;
    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] CLEAR_MAP = 3'd1;
    localparam [2:0] RAY_READ = 3'd2;
    localparam [2:0] RAY_WRITE = 3'd3;
    localparam [2:0] SWEEP_READ = 3'd4, SWEEP_WRITE = 3'd5;
    localparam [2:0] TAG_READ = 3'd6, TAG_WRITE = 3'd7;
    localparam [3:0] RAY_SETUP=8,RAY_ERROR=9,RAY_DIVIDE=10,RAY_COORDINATES=11;
    localparam FAST_20MM=(CELL_MM==20||CELL_MM==30||CELL_MM==40)&&ENDPOINT_LIMIT_MM+ORIGIN_BIAS_X_MM<=32767;
    localparam [GRID_BITS-1:0] EDGE_LO = MAP_EDGE_CELLS;
    localparam [GRID_BITS-1:0] EDGE_HI_X=(GRID_X-1)-MAP_EDGE_CELLS, EDGE_HI_Y=(GRID_Y-1)-MAP_EDGE_CELLS;
    localparam integer COMPACT=(GRID_X!=GRID_Y), STORAGE_BITS=COMPACT?$clog2(GRID_X*GRID_Y):2*GRID_BITS;
    function [STORAGE_BITS-1:0] storage_address;
        input [2*GRID_BITS-1:0] packed_cell;
        begin storage_address=COMPACT ? packed_cell[2*GRID_BITS-1:GRID_BITS]*GRID_X+packed_cell[GRID_BITS-1:0] : packed_cell;end
    endfunction

    reg [3:0] state;
    reg sweep_pending;
    reg [2*GRID_BITS-1:0] clear_addr;
    localparam [GRID_BITS-1:0] LAST_X=GRID_X-1,LAST_Y=GRID_Y-1;
    wire clear_last=COMPACT ? clear_addr=={LAST_Y,LAST_X} : (&clear_addr);
    wire [2*GRID_BITS-1:0] next_clear_addr=COMPACT && clear_addr[GRID_BITS-1:0]==LAST_X ?
        {clear_addr[2*GRID_BITS-1:GRID_BITS]+1'b1,{GRID_BITS{1'b0}}} : clear_addr+1'b1;
    reg [GRID_BITS-1:0] cell_x, cell_y;
    reg signed [15:0] end_x, end_y;
    reg step_x_positive, step_y_positive;
    reg signed [15:0] delta_x_abs, delta_y_neg, error;
    reg [7:0] observed_epoch, ray_epoch;
    reg [15:0] coordinate_magnitude[0:3];
    reg [31:0] coordinate_product[0:3];
    reg [3:0] coordinate_negative;
    integer coordinate_index;
    function [15:0] magnitude_for_floor;
        input signed [15:0] value;
        begin magnitude_for_floor=value<0?-$signed(value)+CELL_MM-1:value;end
    endfunction

    // Arithmetic division by 16 rounds negative millimetres down, including
    // -1 mm -> cell 63. The full-width guard runs before the narrowed endpoint
    // is latched, so signed-32-bit extremes cannot alias into a valid ray.
    wire signed [31:0] biased_ray_end_x_mm=ray_end_x_mm+ORIGIN_BIAS_X_MM;
    wire signed [31:0] biased_ray_end_y_mm=ray_end_y_mm+ORIGIN_BIAS_Y_MM;
    wire signed [31:0] biased_ray_start_x_mm=ray_start_x_mm+ORIGIN_BIAS_X_MM;
    wire signed [31:0] biased_ray_start_y_mm=ray_start_y_mm+ORIGIN_BIAS_Y_MM;
    wire endpoint_usable =
        (ray_end_x_mm >= -ENDPOINT_LIMIT_MM) && (ray_end_x_mm <= ENDPOINT_LIMIT_MM) &&
        (ray_end_y_mm >= -ENDPOINT_LIMIT_MM) && (ray_end_y_mm <= ENDPOINT_LIMIT_MM);
    wire signed [15:0] input_cell_x = $signed(grid_floor(ray_end_x_mm+ORIGIN_BIAS_X_MM)) + (GRID_X/2);
    wire signed [15:0] input_cell_y = $signed(grid_floor(ray_end_y_mm+ORIGIN_BIAS_Y_MM)) + (GRID_Y/2);
    wire signed [15:0] input_start_x = ENABLE_RAY_START ?
        $signed(grid_floor(ray_start_x_mm+ORIGIN_BIAS_X_MM))+(GRID_X/2) : (GRID_X/2);
    wire signed [15:0] input_start_y = ENABLE_RAY_START ?
        $signed(grid_floor(ray_start_y_mm+ORIGIN_BIAS_Y_MM))+(GRID_Y/2) : (GRID_Y/2);
    wire start_usable = ENABLE_RAY_START==0 ||
        (ray_start_x_mm>=-MAP_HALF_X_MM && ray_start_x_mm<MAP_HALF_X_MM &&
         ray_start_y_mm>=-MAP_HALF_Y_MM && ray_start_y_mm<MAP_HALF_Y_MM);
    wire signed [15:0] input_dx_signed = (input_cell_x >= input_start_x) ?
        input_cell_x - input_start_x : input_start_x - input_cell_x;
    wire signed [15:0] input_dy_negative = (input_cell_y >= input_start_y) ?
        input_start_y - input_cell_y : input_cell_y - input_start_y;

    wire ray_last = ($signed({1'b0, cell_x}) == end_x) &&
                    ($signed({1'b0, cell_y}) == end_y);
    wire signed [16:0] twice_error = $signed({error[15], error}) <<< 1;
    wire advance_x = (twice_error >= delta_y_neg);
    wire advance_y = (twice_error <= delta_x_abs);
    // Stop after writing the last visible cell. The virtual endpoint and
    // slope stay unchanged; clipping the endpoint would invent an obstacle
    // and can distort shallow or steep rays at the border. The outermost
    // MAP_EDGE_CELLS cells are never written at all, so they remain UNKNOWN.
    wire ray_cell_writable=cell_x>=EDGE_LO && cell_x<=EDGE_HI_X &&
        cell_y>=EDGE_LO && cell_y<=EDGE_HI_Y;
    wire exits_map =
        (advance_x && ((step_x_positive && cell_x == (GRID_X-1)) ||
                       (!step_x_positive && cell_x == 0))) ||
        (advance_y && ((step_y_positive && cell_y == (GRID_Y-1)) ||
                       (!step_y_positive && cell_y == 0)));
    wire sweep_request=ENABLE_ROTATION_SWEEP && rotation_sweep;
    wire epoch_wrap_request=ENABLE_CONFIDENCE && ENABLE_SCAN_EPOCH &&
        scan_epoch<observed_epoch;
    assign busy = (state != IDLE) || sweep_pending || sweep_request || epoch_wrap_request;

    // A synchronous read on port A precedes each ray write. Keeping its
    // read and write address identical allows true dual-port block-RAM
    // inference, rather than adding an asynchronous third memory port.
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    reg [1:0] map_ram [0:(1<<STORAGE_BITS)-1];
    wire [1:0] port_a_data;
    // A ray pulse reserves this cycle even when its validity changes late.
    // This keeps immediate IR cancel out of the planner handshake path.
    assign planner_req_ready = ENABLE_PLANNER && reset_n && !busy &&
        !clear && !ray_update;
    wire planner_read = planner_req_ready && planner_req;
    // Idle reads may harmlessly prefetch planner_addr without a request.
    // Only planner_read publishes the result; ray reads always re-read their
    // own cell in RAY_READ before writing. Abort cannot lengthen ADA routing.
    wire [2*GRID_BITS-1:0] port_a_addr = (state == CLEAR_MAP || state==SWEEP_READ || state==SWEEP_WRITE ||
        state==TAG_READ || state==TAG_WRITE) ?
        clear_addr : (ENABLE_PLANNER && state==IDLE) ? planner_addr : {cell_y, cell_x};
    wire [STORAGE_BITS-1:0] storage_a=storage_address(port_a_addr);
    wire [STORAGE_BITS-1:0] storage_b=DISPLAY_ADDRESS_COMPACT ? rd_addr[STORAGE_BITS-1:0] : storage_address(rd_addr);
    wire swept_footprint;
    wire legacy_port_a_write = reset_n &&
        ((state == CLEAR_MAP) ||
          (!clear && (state == RAY_WRITE) && ray_cell_writable &&
          (ray_last || (port_a_data != OCCUPIED))) ||
         (!clear && state==SWEEP_WRITE && swept_footprint && port_a_data!=OCCUPIED));
    // Only cells entirely inside the car's actual initial rectangular
    // footprint are known free without a laser observation.
    wire initial_footprint = ENABLE_VEHICLE_FOOTPRINT &&
        clear_addr[GRID_BITS-1:0]>=(MAP_HALF_X_MM-BODY_REAR_MM+CELL_MM-1)/CELL_MM &&
        clear_addr[GRID_BITS-1:0]<(MAP_HALF_X_MM+BODY_FRONT_MM)/CELL_MM &&
        clear_addr[2*GRID_BITS-1:GRID_BITS]>=(MAP_HALF_Y_MM-BODY_HALF_WIDTH_MM+CELL_MM-1)/CELL_MM &&
        clear_addr[2*GRID_BITS-1:GRID_BITS]<(MAP_HALF_Y_MM+BODY_HALF_WIDTH_MM)/CELL_MM;
    // A cell spans [16*i,16*(i+1)]. Use its furthest corner, so the whole
    // cell must lie inside the physical sweep disk. The row table contains
    // elaboration constants only; no run-time squares or new DSPs are needed.
    wire [GRID_BITS-1:0] sweep_outer_x=clear_addr[GRID_BITS-1:0]>=GRID_X/2 ? clear_addr[GRID_BITS-1:0]-(GRID_X/2-1) : (GRID_X/2)-clear_addr[GRID_BITS-1:0];
    wire [GRID_BITS-1:0] sweep_outer_y=clear_addr[2*GRID_BITS-1:GRID_BITS]>=GRID_Y/2 ? clear_addr[2*GRID_BITS-1:GRID_BITS]-(GRID_Y/2-1) : (GRID_Y/2)-clear_addr[2*GRID_BITS-1:GRID_BITS];
    function [GRID_BITS-1:0] sweep_row_extent;
        input [GRID_BITS-1:0] outer_y;
        integer row_i,col_i;
        begin
            sweep_row_extent=0;
            for(row_i=1;row_i<=BODY_SWEEP_RADIUS_MM/CELL_MM;row_i=row_i+1)
                if(outer_y==row_i)
                    for(col_i=1;col_i<=BODY_SWEEP_RADIUS_MM/CELL_MM;col_i=col_i+1)
                        if(CELL_MM*CELL_MM*(col_i*col_i+row_i*row_i)<=BODY_SWEEP_RADIUS_MM*BODY_SWEEP_RADIUS_MM)
                            sweep_row_extent=col_i;
        end
    endfunction
    assign swept_footprint=ENABLE_ROTATION_SWEEP && sweep_outer_x<=sweep_row_extent(sweep_outer_y);
    wire [1:0] port_a_write_data = (state == CLEAR_MAP) ? (initial_footprint ? FREE : UNKNOWN) :
        state==SWEEP_WRITE ? FREE :
        (ray_last ? OCCUPIED : FREE);
    assign planner_rsp_data=port_a_data;
    always @(posedge clk or negedge reset_n)
        if(!reset_n) planner_rsp_valid<=0;
        else planner_rsp_valid<=planner_read;

    function [63:0] score_table;
        input integer increment;
        integer i,value;
        begin
            score_table=0;
            for(i=0;i<16;i=i+1)begin
                value=(i<8?i:i-16)+increment;
                if(value>7)value=7;
                if(value< -7)value=-7;
                score_table[i*4+:4]=value[3:0];
            end
        end
    endfunction
    generate if(ENABLE_CONFIDENCE==0) begin: g_legacy
        reg [1:0] legacy_port_a_data, legacy_rd_data;
        assign port_a_data=legacy_port_a_data;
        assign rd_data=legacy_rd_data;
        always @(posedge clk) begin
            if (legacy_port_a_write)
                map_ram[storage_a] <= port_a_write_data;
            else
                // NO_CHANGE on writes, preserving Gowin DPB inference.
                legacy_port_a_data <= map_ram[storage_a];
        end
        always @(posedge rd_clk) legacy_rd_data <= map_ram[storage_b];
    end else if(CONFIDENCE_BANK_BITS>0) begin: g_confidence_banked
        localparam integer BANKS=1<<CONFIDENCE_BANK_BITS;
        localparam integer BANK_ADDR_BITS=STORAGE_BITS-CONFIDENCE_BANK_BITS;
        localparam [2:0] OBS_NONE=0, OBS_FREE=1, OBS_FREE_SAT=2, OBS_HIT=3;
        // Each four-bit evidence result is a constant lookup, avoiding a
        // full-width signed arithmetic chain between read and write RAMs.
        localparam [63:0] CLAMP_SCORES=score_table(0), MISS_SCORES=score_table(-1);
        localparam [63:0] HIT_SCORES=score_table(HIT_EVIDENCE);
        localparam [63:0] UNDO_MISS_HIT_SCORES=score_table(HIT_EVIDENCE+1);
        function [1:0] classify;
            input signed [3:0] value;
            begin
                if(value>=OCCUPIED_EVIDENCE)classify=OCCUPIED;
                else if(value<0)classify=FREE;
                else classify=UNKNOWN;
            end
        endfunction
        wire [1:0] bank_a_data[0:BANKS-1],bank_b_data[0:BANKS-1];
        reg [CONFIDENCE_BANK_BITS-1:0] read_a_bank,read_b_bank;
        always @(posedge clk)read_a_bank<=storage_a[STORAGE_BITS-1:BANK_ADDR_BITS];
        assign port_a_data=bank_a_data[read_a_bank];
        assign rd_data=bank_b_data[read_b_bank];
        if(REGISTER_RD_ADDRESS)begin:g_registered_bank_select
            reg [CONFIDENCE_BANK_BITS-1:0] select_r;
            always @(posedge rd_clk or negedge reset_n)
                if(!reset_n)select_r<=0;
                else select_r<=storage_b[STORAGE_BITS-1:BANK_ADDR_BITS];
            always @(posedge rd_clk)read_b_bank<=select_r;
        end else begin:g_direct_bank_select
            always @(posedge rd_clk)read_b_bank<=storage_b[STORAGE_BITS-1:BANK_ADDR_BITS];
        end
        for(genvar bank=0;bank<BANKS;bank=bank+1)begin:g_bank
            (* ram_style="block",syn_ramstyle="block_ram" *)
            reg [14:0] evidence_ram[0:(1<<BANK_ADDR_BITS)-1];
            reg [14:0] evidence_a,evidence_b;
            wire signed [3:0] score=$signed(evidence_a[3:0]);
            wire same_epoch=ENABLE_SCAN_EPOCH && evidence_a[14:12]!=OBS_NONE &&
                evidence_a[11:4]==ray_epoch;
            wire selected=storage_a[STORAGE_BITS-1:BANK_ADDR_BITS]==bank;
            wire write_enable=reset_n && selected && ((state==CLEAR_MAP) ||
                (!clear && state==RAY_WRITE && ray_cell_writable) ||
                (!clear && state==TAG_WRITE) ||
                (!clear && state==SWEEP_WRITE && swept_footprint && classify(score)!=OCCUPIED));
            reg [3:0] next_score;
            reg [2:0] next_observation;
            reg [14:0] write_data;
            always @* begin
                next_score=CLAMP_SCORES[evidence_a[3:0]*4+:4];
                next_observation=OBS_NONE;
                if(state==CLEAR_MAP || state==SWEEP_WRITE)
                    next_score=(state==SWEEP_WRITE || initial_footprint)?4'd13:4'd0;
                else if(state==RAY_WRITE)begin
                    if(ray_last)begin
                        if(!same_epoch || evidence_a[14:12]!=OBS_HIT)
                            next_score=(same_epoch && evidence_a[14:12]==OBS_FREE)?
                                UNDO_MISS_HIT_SCORES[evidence_a[3:0]*4+:4]:
                                HIT_SCORES[evidence_a[3:0]*4+:4];
                        next_observation=OBS_HIT;
                    end else if(!same_epoch)begin
                        next_score=MISS_SCORES[evidence_a[3:0]*4+:4];
                        next_observation=score<= -7?OBS_FREE_SAT:OBS_FREE;
                    end else next_observation=evidence_a[14:12];
                end
                write_data={11'd0,next_score};
                if(ENABLE_SCAN_EPOCH && state==RAY_WRITE)
                    write_data={next_observation,ray_epoch,next_score};
                if(state==TAG_WRITE)write_data={11'd0,evidence_a[3:0]};
            end
            always @(posedge clk)begin
                if(write_enable)evidence_ram[storage_a[BANK_ADDR_BITS-1:0]]<=write_data;
                else evidence_a<=evidence_ram[storage_a[BANK_ADDR_BITS-1:0]];
            end
            wire [BANK_ADDR_BITS-1:0] display_address;
            if(REGISTER_RD_ADDRESS)begin:g_local_display_address
                reg [BANK_ADDR_BITS-1:0] address_r /* synthesis syn_preserve = 1 */;
                always @(posedge rd_clk or negedge reset_n)
                    if(!reset_n)address_r<=0;
                    else address_r<=storage_b[BANK_ADDR_BITS-1:0];
                assign display_address=address_r;
            end else begin:g_direct_display_address
                assign display_address=storage_b[BANK_ADDR_BITS-1:0];
            end
            always @(posedge rd_clk)evidence_b<=evidence_ram[display_address];
            assign bank_a_data[bank]=classify(score);
            assign bank_b_data[bank]=classify($signed(evidence_b[3:0]));
        end
    end else begin: g_confidence
        localparam integer MISS_EVIDENCE=1;
        // [3:0] signed evidence; [11:4] scan epoch; [14:12] observation.
        // FREE records whether the decrement saturated. This lets a later
        // same-scan hit exactly undo the miss, even at the -7 lower bound.
        localparam [2:0] OBS_NONE=0, OBS_FREE=1, OBS_FREE_SAT=2, OBS_HIT=3;
        (* ram_style = "block", syn_ramstyle = "block_ram" *)
        reg [14:0] evidence_ram [0:(1<<STORAGE_BITS)-1];
        reg [14:0] evidence_port_a, evidence_port_b;
        wire signed [3:0] score=$signed(evidence_port_a[3:0]);
        wire same_epoch=ENABLE_SCAN_EPOCH && evidence_port_a[14:12]!=OBS_NONE &&
            evidence_port_a[11:4]==ray_epoch;
        reg [14:0] evidence_write_data;
        reg signed [31:0] proposed_score;
        reg [2:0] proposed_observation;
        function [1:0] classify;
            input signed [3:0] value;
            begin
                if(value>=OCCUPIED_EVIDENCE) classify=OCCUPIED;
                else if(value<0) classify=FREE;
                else classify=UNKNOWN;
            end
        endfunction
        assign port_a_data=classify(score);
        assign rd_data=classify($signed(evidence_port_b[3:0]));
        wire evidence_write=reset_n && ((state==CLEAR_MAP) ||
            (!clear && state==RAY_WRITE && ray_cell_writable) || (!clear && state==TAG_WRITE) ||
            (!clear && state==SWEEP_WRITE && swept_footprint && port_a_data!=OCCUPIED));
        always @* begin
            proposed_score=$signed(score);
            proposed_observation=OBS_NONE;
            if(state==CLEAR_MAP || state==SWEEP_WRITE) begin
                proposed_score=(state==SWEEP_WRITE || initial_footprint) ? -3 : 0;
            end else if(state==RAY_WRITE) begin
                if(ray_last) begin
                    if(!same_epoch || evidence_port_a[14:12]!=OBS_HIT) begin
                        // Reverse only a unit miss that was actually applied;
                        // a miss at the lower bound did not change evidence.
                        if(same_epoch && evidence_port_a[14:12]==OBS_FREE)
                            proposed_score=proposed_score+MISS_EVIDENCE;
                        proposed_score=proposed_score+HIT_EVIDENCE;
                    end
                    proposed_observation=OBS_HIT;
                end else if(!same_epoch) begin
                    proposed_score=proposed_score-MISS_EVIDENCE;
                    proposed_observation=proposed_score < -7 ? OBS_FREE_SAT : OBS_FREE;
                end else proposed_observation=evidence_port_a[14:12];
            end
            if(proposed_score>7) proposed_score=7;
            if(proposed_score< -7) proposed_score=-7;
            evidence_write_data={11'd0,proposed_score[3:0]};
            if(ENABLE_SCAN_EPOCH && state==RAY_WRITE)
                evidence_write_data={proposed_observation,ray_epoch,proposed_score[3:0]};
            // Epoch wrap invalidates only metadata, never map occupancy.
            if(state==TAG_WRITE) evidence_write_data={11'd0,evidence_port_a[3:0]};
        end
        always @(posedge clk) begin
            if(evidence_write) evidence_ram[storage_a]<=evidence_write_data;
            else evidence_port_a<=evidence_ram[storage_a];
        end
        always @(posedge rd_clk) evidence_port_b<=evidence_ram[storage_b];
    end endgenerate

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= CLEAR_MAP;
            sweep_pending<=0;
            clear_addr <= 0;
            cell_x <= (GRID_X/2);
            cell_y <= (GRID_Y/2);
            end_x <= (GRID_X/2);
            end_y <= (GRID_Y/2);
            step_x_positive <= 1'b1;
            step_y_positive <= 1'b1;
            delta_x_abs <= 11'sd0;
            delta_y_neg <= 11'sd0;
            error <= 11'sd0;
            observed_epoch<=0;
            ray_epoch<=0;
            coordinate_negative<=0;
            for(coordinate_index=0;coordinate_index<4;coordinate_index=coordinate_index+1)begin
                coordinate_magnitude[coordinate_index]<=0;coordinate_product[coordinate_index]<=0;
            end
            map_update <= 1'b0;
            clear_done <= 1'b0;
        end else begin
            map_update <= 1'b0;
            clear_done <= 1'b0;
            if(sweep_request) sweep_pending<=1;
            if (clear) begin
                state <= CLEAR_MAP;
                clear_addr <= 0;
                sweep_pending<=0;
            end else begin
                case (state)
                    CLEAR_MAP: begin
                        if(ENABLE_CONFIDENCE && ENABLE_SCAN_EPOCH) observed_epoch<=scan_epoch;
                        if (clear_last) begin
                            state <= IDLE;
                            clear_done <= 1'b1;
                            map_update <= 1'b1;
                        end else begin
                            clear_addr <= next_clear_addr;
                        end
                    end
                    IDLE: begin
                        if(ENABLE_CONFIDENCE && ENABLE_SCAN_EPOCH) observed_epoch<=scan_epoch;
                        if(epoch_wrap_request) begin
                            state<=TAG_READ;clear_addr<=0;
                        end else if(sweep_pending) begin
                            state<=SWEEP_READ;clear_addr<=0;sweep_pending<=0;
                        end else if (!sweep_request && ray_update && ray_valid && endpoint_usable && start_usable) begin
                            if(FAST_20MM)begin
                                coordinate_magnitude[0]<=ENABLE_RAY_START?magnitude_for_floor(biased_ray_start_x_mm[15:0]):0;
                                coordinate_magnitude[1]<=ENABLE_RAY_START?magnitude_for_floor(biased_ray_start_y_mm[15:0]):0;
                                coordinate_magnitude[2]<=magnitude_for_floor(biased_ray_end_x_mm[15:0]);
                                coordinate_magnitude[3]<=magnitude_for_floor(biased_ray_end_y_mm[15:0]);
                                coordinate_negative<={biased_ray_end_y_mm[31],biased_ray_end_x_mm[31],
                                    ENABLE_RAY_START&&biased_ray_start_y_mm[31],ENABLE_RAY_START&&biased_ray_start_x_mm[31]};
                                state<=RAY_DIVIDE;
                            end else begin
                                cell_x <= input_start_x[GRID_BITS-1:0];cell_y <= input_start_y[GRID_BITS-1:0];
                                end_x <= input_cell_x;end_y <= input_cell_y;state<=RAY_SETUP;
                            end
                            ray_epoch<=scan_epoch;
                        end
                    end
                    RAY_DIVIDE:begin
                        for(coordinate_index=0;coordinate_index<4;coordinate_index=coordinate_index+1)
                            coordinate_product[coordinate_index]<=coordinate_magnitude[coordinate_index]*GRID_RECIPROCAL_MULTIPLIER;
                        state<=RAY_COORDINATES;
                    end
                    RAY_COORDINATES:begin
                        cell_x<=coordinate_negative[0]?GRID_X/2-(coordinate_product[0]>>GRID_RECIPROCAL_SHIFT):GRID_X/2+(coordinate_product[0]>>GRID_RECIPROCAL_SHIFT);
                        cell_y<=coordinate_negative[1]?GRID_Y/2-(coordinate_product[1]>>GRID_RECIPROCAL_SHIFT):GRID_Y/2+(coordinate_product[1]>>GRID_RECIPROCAL_SHIFT);
                        end_x<=coordinate_negative[2]?GRID_X/2-(coordinate_product[2]>>GRID_RECIPROCAL_SHIFT):GRID_X/2+(coordinate_product[2]>>GRID_RECIPROCAL_SHIFT);
                        end_y<=coordinate_negative[3]?GRID_Y/2-(coordinate_product[3]>>GRID_RECIPROCAL_SHIFT):GRID_Y/2+(coordinate_product[3]>>GRID_RECIPROCAL_SHIFT);
                        state<=RAY_SETUP;
                    end
                    RAY_SETUP:begin
                        step_x_positive<=end_x>=$signed({1'b0,cell_x});
                        step_y_positive<=end_y>=$signed({1'b0,cell_y});
                        delta_x_abs<=end_x>=$signed({1'b0,cell_x})?end_x-$signed({1'b0,cell_x}):$signed({1'b0,cell_x})-end_x;
                        delta_y_neg<=end_y>=$signed({1'b0,cell_y})?$signed({1'b0,cell_y})-end_y:end_y-$signed({1'b0,cell_y});
                        state<=RAY_ERROR;
                    end
                    RAY_ERROR:begin error<=delta_x_abs+delta_y_neg;state<=RAY_READ;end
                    RAY_READ: state <= RAY_WRITE;
                    SWEEP_READ:state<=SWEEP_WRITE;
                    TAG_READ:state<=TAG_WRITE;
                    TAG_WRITE:begin
                        if(clear_last) state<=IDLE;
                        else begin clear_addr<=next_clear_addr;state<=TAG_READ;end
                    end
                    SWEEP_WRITE:begin
                        if(clear_last) begin
                            state<=IDLE;map_update<=1;
                        end else begin
                            clear_addr<=next_clear_addr;state<=SWEEP_READ;
                        end
                    end
                    RAY_WRITE: begin
                        if (ray_last || exits_map) begin
                            state <= IDLE;
                            map_update <= 1'b1;
                        end else begin
                            if (advance_x)
                                cell_x <= step_x_positive ?
                                    cell_x + 1 : cell_x - 1;
                            if (advance_y)
                                cell_y <= step_y_positive ?
                                    cell_y + 1 : cell_y - 1;
                            if (advance_x && advance_y)
                                error <= error + delta_y_neg + delta_x_abs;
                            else if (advance_x)
                                error <= error + delta_y_neg;
                            else if (advance_y)
                                error <= error + delta_x_abs;
                            state <= RAY_READ;
                        end
                    end
                    default: begin
                        state <= CLEAR_MAP;
                        clear_addr <= 0;
                    end
                endcase
            end
        end
    end
endmodule
