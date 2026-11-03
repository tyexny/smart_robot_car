// Video network branch is independent of the original DDR3/HDMI data path.
// Pi returns compact COCO detections; this module delivers a coherent
// result in the display clock domain.
module ethernet_vision_bridge #(
    parameter IMAGE_WIDTH=1280, parameter IMAGE_HEIGHT=720,
    parameter FRAME_STRIDE=1,
    parameter [4:0] PHY_ADDR=5,
    parameter TX_CLOCK_DELAY=0,
    parameter AUTO_PHY_ADDR=0, parameter PHY_MDC_HZ=1000000,
    parameter USE_CH59=1,
    parameter FOLLOW_TRACK_ENABLE=0,
    parameter SCREEN_RECORD=0,
    parameter SCREEN_FULL_RES=0,
    parameter CAMERA_FULL_RES=0,
    parameter DYNAMIC_SCREEN=0,parameter SCREEN_FRAME_STRIDE=2
)(
    input wire clk50m, reset_n, camera_clk, camera_vsync,
    input wire capture_enable, pixel_valid,
    input wire screen_record,screen_vsync,screen_capture_enable,screen_pixel_valid,
    input wire [15:0] screen_pixel_rgb565,
    input wire recognition_enable, // debounced maintained SW4 level, clk50m
    input wire [15:0] pixel_rgb565,
    input wire audio_bclk,audio_lrclk,audio_adc_data,
    output wire audio_mclk,audio_i2c_scl,
    inout wire audio_i2c_sda,
    input wire display_clk,
    inout wire thermal_scl,thermal_sda,input wire [64:0] arrival_status,
    output wire boxes_valid,
    output wire [31:0] boxes_frame_id,
    output wire [3:0] boxes_count,
    output wire [511:0] boxes_coords,
    output wire [55:0] boxes_classes,
    output wire [119:0] boxes_label_text,
    output wire [3:0] boxes_label_length,
    output wire [31:0] boxes_tracker_session,
    output wire [255:0] boxes_track_ids,
    output wire [79:0] boxes_confidences,
    output wire [7:0] boxes_observed,
    output wire task_command_valid,
    output wire [7:0] task_command,
    output wire [5:0] task_command_actions,
    output wire thermal_sample_valid,thermal_sensor_valid,thermal_task_active,
    output wire signed [15:0] thermal_maximum_centi,
    output wire [6:0] thermal_threshold_c,
    input wire [31:0] follow_lock_session, follow_lock_id,
    input wire follow_lock_valid,
    output wire boxes_reset_n,
    input wire eth_rxc, input wire [3:0] eth_rxd, input wire eth_rxdv,
    output wire eth_gtxc, output wire [3:0] eth_txd, output wire eth_txen,
    output wire eth_rst_n, output wire eth_mdc, inout wire eth_mdio,
    output wire phy_link_up,
    output wire eth_pll_locked, phy_init_done, phy_id_match,
    output wire phy_any_read_ack, output wire [4:0] phy_address,
    output wire phy_tx_ready
);
    wire clk125m, clk125m_90, eth_pll_lock;
    eth_pll u_eth_pll(.clkin(clk50m),.reset(~reset_n),
        .clk125m(clk125m),.clk125m_90(clk125m_90),.lock(eth_pll_lock));
    assign eth_pll_locked=eth_pll_lock;
    wire [31:0] phy_id;
    // Assert only after both ID reads were acknowledged and matched.
    wire phy_id_verified;
    assign phy_id_match=phy_id_verified;
    wire network_reset_n=reset_n && eth_pll_lock;
    wire rx_clk;
    (* ASYNC_REG="TRUE" *) reg recognition_meta_camera, recognition_sync_camera;
    (* ASYNC_REG="TRUE" *) reg recognition_meta_tx, recognition_sync_tx;
    (* ASYNC_REG="TRUE" *) reg recognition_meta_rx, recognition_sync_rx;
    (* ASYNC_REG="TRUE" *) reg recognition_meta_video, recognition_sync_video;
    always @(posedge camera_clk or negedge network_reset_n)
        if(!network_reset_n) begin recognition_meta_camera<=0; recognition_sync_camera<=0; end
        else begin recognition_meta_camera<=recognition_enable; recognition_sync_camera<=recognition_meta_camera; end
    always @(posedge clk125m or negedge network_reset_n)
        if(!network_reset_n) begin recognition_meta_tx<=0; recognition_sync_tx<=0; end
        else begin recognition_meta_tx<=recognition_enable; recognition_sync_tx<=recognition_meta_tx; end
    always @(posedge rx_clk or negedge network_reset_n)
        if(!network_reset_n) begin recognition_meta_rx<=0; recognition_sync_rx<=0; end
        else begin recognition_meta_rx<=recognition_enable; recognition_sync_rx<=recognition_meta_rx; end
    always @(posedge display_clk or negedge network_reset_n)
        if(!network_reset_n) begin recognition_meta_video<=0; recognition_sync_video<=0; end
        else begin recognition_meta_video<=recognition_enable; recognition_sync_video<=recognition_meta_video; end
    (* ASYNC_REG="TRUE" *) reg screen_meta_camera,screen_sync_camera;
    (* ASYNC_REG="TRUE" *) reg screen_meta_tx,screen_sync_tx;
    // Register the final-page OR in its source domain. First CDC captures
    // then see one stable flop, rather than the long overlay priority cone.
    // The page changes in blanking; one video clock does not remove pixels.
    reg screen_mode_video /* synthesis syn_preserve = 1 */;
    always @(posedge display_clk or negedge network_reset_n)
        if(!network_reset_n)screen_mode_video<=0;
        else screen_mode_video<=screen_record;
    always @(posedge camera_clk or negedge network_reset_n)
        if(!network_reset_n)begin screen_meta_camera<=0;screen_sync_camera<=0;end
        else begin screen_meta_camera<=screen_mode_video;screen_sync_camera<=screen_meta_camera;end
    always @(posedge clk125m or negedge network_reset_n)
        if(!network_reset_n)begin screen_meta_tx<=0;screen_sync_tx<=0;end
        else begin screen_meta_tx<=screen_mode_video;screen_sync_tx<=screen_meta_tx;end
    // Only these reset-synchronizer primitives receive asynchronous SW4.
    // Their active-high outputs release ordinary logic on its own clock edge,
    // avoiding a long SYS50 route into wide video/RX reset banks.
    // Screen-only build holds all detection/session consumers in reset.
    // SW4 still controls the shared video sender; audio/task RX stays separate.
    wire session_reset_n=network_reset_n && recognition_enable && !SCREEN_RECORD;
    (* ASYNC_REG="TRUE" *) reg session_reset_meta_rx,session_reset_sync_rx;
    (* ASYNC_REG="TRUE" *) reg session_reset_meta_tx,session_reset_sync_tx;
    (* ASYNC_REG="TRUE" *) reg session_reset_meta_video,session_reset_sync_video;
    always @(posedge rx_clk or negedge session_reset_n)
        if(!session_reset_n) begin session_reset_meta_rx<=1;session_reset_sync_rx<=1;end
        else begin session_reset_meta_rx<=0;session_reset_sync_rx<=session_reset_meta_rx;end
    always @(posedge clk125m or negedge session_reset_n)
        if(!session_reset_n) begin session_reset_meta_tx<=1;session_reset_sync_tx<=1;end
        else begin session_reset_meta_tx<=0;session_reset_sync_tx<=session_reset_meta_tx;end
    always @(posedge display_clk or negedge session_reset_n)
        if(!session_reset_n) begin session_reset_meta_video<=1;session_reset_sync_video<=1;end
        else begin session_reset_meta_video<=0;session_reset_sync_video<=session_reset_meta_video;end
    wire rx_session_reset_n=~session_reset_sync_rx;
    wire tx_session_reset_n=~session_reset_sync_tx;
    assign boxes_reset_n=~session_reset_sync_video;
    reg camera_vsync_d, capture_session_active;
    always @(posedge camera_clk or negedge network_reset_n) begin
        if(!network_reset_n) begin camera_vsync_d<=0; capture_session_active<=0; end
        else begin
            camera_vsync_d<=camera_vsync;
            if(!recognition_sync_camera) capture_session_active<=0;
            else if(camera_vsync && !camera_vsync_d) capture_session_active<=1;
        end
    end
    wire tx_start, tx_busy, tx_done, tx_data_req;
    wire rpc_active,rpc_start,rpc_req,audio_tx_start;
    wire [15:0] rpc_length,rpc_port;
    wire [7:0] rpc_data;
    reg rpc_owner;
    wire [64:0] arrival_tx;
    telemetry_mailbox #(.WIDTH(65),.RESET_SYNC_RELEASE(1)) u_arrival_cdc(
        .src_clk(clk50m),.dst_clk(clk125m),.reset_n(network_reset_n),.src_valid(1'b1),.src_data(arrival_status),.dst_valid(),.dst_data(arrival_tx));
    always @(posedge clk125m or negedge network_reset_n)
        if(!network_reset_n)rpc_owner<=0;
        else if(rpc_start)rpc_owner<=1;
        else if(audio_tx_start||tx_start)rpc_owner<=0;

    wire [15:0] tx_length;
    wire [10:0] tx_payload_addr;
    wire [7:0] tx_data;
    wire audio_pll_lock,audio_init_done,audio_fifo_empty;
    wire [31:0] audio_fifo_data;
    wire audio_fifo_read,audio_tx_request,audio_tx_active;
    wire [15:0] audio_tx_length;
    wire [7:0] audio_tx_data;
    reg audio_owner;
    (* ASYNC_REG="TRUE" *) reg audio_ready_meta_bclk,audio_ready_sync_bclk;
    always @(posedge audio_bclk or negedge network_reset_n)
        if(!network_reset_n) begin audio_ready_meta_bclk<=0;audio_ready_sync_bclk<=0;end
        else begin
            audio_ready_meta_bclk<=audio_init_done && audio_pll_lock;
            audio_ready_sync_bclk<=audio_ready_meta_bclk;
        end
    Audio_PLL u_audio_pll(.clkin(clk50m),.reset(~reset_n),
        .lock(audio_pll_lock),.clkout0(audio_mclk));
    ES8388_Init u_audio_codec(.Clk(clk50m),.Rst_n(reset_n && audio_pll_lock),
        .I2C_Init_Done(audio_init_done),.i2c_sclk(audio_i2c_scl),
        .i2c_sdat(audio_i2c_sda));
    i2s_rx #(.DATA_WIDTH(32)) u_audio_rx(
        .reset_n(network_reset_n),
        .bclk(audio_bclk),.sample_enable(audio_ready_sync_bclk),
        .adclrc(audio_lrclk),.adcdat(audio_adc_data),
        .adcfifo_rdclk(clk125m),.adcfifo_read(audio_fifo_read),
        .adcfifo_empty(audio_fifo_empty),.adcfifo_readdata(audio_fifo_data));
    audio_udp_packetizer u_audio_packets(.clk(clk125m),
        .reset_n(network_reset_n),
        .fifo_empty(audio_fifo_empty),.fifo_data(audio_fifo_data),.fifo_read(audio_fifo_read),
        .tx_busy(tx_busy || tx_start || rpc_active),.tx_done(tx_done && audio_owner && !rpc_owner),
        .tx_data_req(tx_data_req && audio_owner && !rpc_owner),.tx_payload_addr(tx_payload_addr),
        .tx_request(audio_tx_request),.tx_active(audio_tx_active),
        .tx_start(audio_tx_start),.tx_length(audio_tx_length),.tx_data(audio_tx_data));
    always @(posedge clk125m or negedge network_reset_n)
        if(!network_reset_n) audio_owner<=0;
        else if(audio_tx_start) audio_owner<=1;
        else if(tx_start && recognition_sync_tx) audio_owner<=0;
    wire [31:0] latest_frame_id;
    wire latest_frame_valid;
    // SYS50 owns the selected identity. The complete valid/session/ID tuple
    // crosses into TX through a held-payload acknowledged mailbox, never
    // separate bit synchronizers. Network reset asserts asynchronously and
    // releases independently in both clocks. SW4 gates only the local valid
    // flag; an already accepted Ethernet packet is allowed to finish.
    wire [64:0] follow_control_tx;
    telemetry_mailbox #(.WIDTH(65),.RESET_SYNC_RELEASE(1)) u_follow_control_cdc(
        .src_clk(clk50m),.dst_clk(clk125m),.reset_n(network_reset_n),
        .src_valid(1'b1),
        .src_data(FOLLOW_TRACK_ENABLE ? {follow_lock_valid,follow_lock_session,follow_lock_id} : 65'd0),
        .dst_valid(),.dst_data(follow_control_tx));
    wire camera_start,camera_busy,camera_done,camera_req;
    wire screen_start,screen_busy,screen_done,screen_req;
    wire [15:0] camera_length,screen_length;
    wire [7:0] camera_data,screen_data;
    video_source_mux #(.REGISTERED_MAC(1)) u_video_source_mux(.clk(clk125m),.reset_n(network_reset_n),
        .enabled(recognition_sync_tx),.screen_select(DYNAMIC_SCREEN && screen_sync_tx),
        .mac_busy(tx_busy || audio_tx_active || rpc_active),.mac_done(tx_done && !audio_owner && !rpc_owner),
        .mac_req(tx_data_req && !audio_owner && !rpc_owner),
        .camera_start(camera_start),.camera_length(camera_length),.camera_data(camera_data),
        .screen_start(screen_start),.screen_length(screen_length),.screen_data(screen_data),
        .camera_busy(camera_busy),.camera_done(camera_done),.camera_req(camera_req),
        .screen_busy(screen_busy),.screen_done(screen_done),.screen_req(screen_req),
        .tx_start(tx_start),.tx_length(tx_length),.tx_data(tx_data));
    video_udp_packetizer #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .FRAME_STRIDE(FRAME_STRIDE),.TRACK_CONTROL_ENABLE(FOLLOW_TRACK_ENABLE),
        .SCREEN_RECORD(SCREEN_RECORD),.FULL_RES(SCREEN_RECORD && SCREEN_FULL_RES),
        .CAMERA_FULL_RES(CAMERA_FULL_RES),.REGISTER_BYTE_OUTPUT(USE_CH59)) u_video_packets(
        .camera_clk(camera_clk),.tx_clk(clk125m),.reset_n(network_reset_n),
        .camera_vsync(camera_vsync),
        .capture_enable(capture_enable && capture_session_active && !(DYNAMIC_SCREEN && screen_sync_camera)),
        .pixel_valid(pixel_valid),.pixel_rgb565(pixel_rgb565),
        .tx_busy(camera_busy),.tx_done(camera_done),.tx_data_req(camera_req),
        .tx_payload_addr(tx_payload_addr),.tx_start(camera_start),.tx_length(camera_length),
        .lock_session(follow_control_tx[63:32]),.lock_id(follow_control_tx[31:0]),
        .lock_valid(follow_control_tx[64] && recognition_sync_tx),
        .tx_data(camera_data),.latest_frame_id(latest_frame_id),
        .latest_frame_valid(latest_frame_valid),.packets_sent(),.rows_dropped());
    // RGB565 final HDMI, segmented within the existing MTU. The bounded row
    // FIFO absorbs active-raster bursts and drains during the skipped frame.
    screen_rgb565_packetizer #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .FRAME_STRIDE(SCREEN_FRAME_STRIDE)) u_screen_packets(
        .camera_clk(display_clk),.tx_clk(clk125m),.reset_n(network_reset_n),
        .camera_vsync(screen_vsync),
        .capture_enable(DYNAMIC_SCREEN && screen_capture_enable && recognition_sync_video && screen_record),
        .pixel_valid(screen_pixel_valid),.pixel_rgb565(screen_pixel_rgb565),
        .tx_busy(screen_busy),.tx_done(screen_done),.tx_data_req(screen_req),
        .tx_payload_addr(tx_payload_addr),.tx_start(screen_start),.tx_length(screen_length),
        .tx_data(screen_data),.packets_sent(),.rows_dropped());
    wire [7:0] rx_data;
    wire rx_valid, rx_start, rx_end, rx_good, rx_error;
    wire [15:0] rx_length;
    wire task_command_rx_valid;
    wire [7:0] task_command_rx;
    eth_udp_rgmii #(.PHY_ADDR(PHY_ADDR),.TX_CLOCK_DELAY(TX_CLOCK_DELAY),
        .AUTO_PHY_ADDR(AUTO_PHY_ADDR),.PHY_MDC_HZ(PHY_MDC_HZ),.USE_CH59(USE_CH59),.DYNAMIC_DST_PORT(1)) u_eth(
        .clk125m(clk125m),.clk125m_90(clk125m_90),.clk50m(clk50m),
        .reset_n(network_reset_n),
        .tx_start(rpc_start || audio_tx_start || (tx_start && recognition_sync_tx)),
        .tx_length(rpc_start ? rpc_length : audio_tx_start ? audio_tx_length : tx_length),
        .tx_destination_port(rpc_start ? rpc_port : 16'd5000),
        .tx_data(rpc_owner ? rpc_data : audio_owner ? audio_tx_data : tx_data),.tx_busy(tx_busy),.tx_done(tx_done),
        .tx_data_req(tx_data_req),.tx_payload_addr(tx_payload_addr),
        .eth_rxc(eth_rxc),.eth_rxd(eth_rxd),.eth_rxdv(eth_rxdv),
        .eth_gtxc(eth_gtxc),.eth_txd(eth_txd),.eth_txen(eth_txen),
        .eth_rst_n(eth_rst_n),.eth_mdc(eth_mdc),.eth_mdio(eth_mdio),
        .rx_clk(rx_clk),.rx_payload_data(rx_data),.rx_payload_valid(rx_valid),
        .rx_payload_start(rx_start),.rx_payload_end(rx_end),.rx_payload_length(rx_length),
        .rx_packet_good(rx_good),.rx_packet_error(rx_error),
        .phy_init_done(phy_init_done),.phy_link_up(phy_link_up),.phy_id(phy_id),
        .phy_any_read_ack(phy_any_read_ack),.phy_id_verified(phy_id_verified),
        .phy_address(phy_address),.phy_tx_ready(phy_tx_ready));

    thermal_i2c_rpc u_thermal_rpc(
        .rx_clk(rx_clk),.tx_clk(clk125m),.reset_n(network_reset_n),
        .packet_start(rx_start),.payload_valid(rx_valid),.payload_data(rx_data),.packet_good(rx_good),.packet_error(rx_error),.payload_length(rx_length),
        .arrival_status(arrival_tx),.tx_busy(tx_busy || audio_tx_start || tx_start),.tx_done(tx_done && rpc_owner),
        .tx_data_req(tx_data_req && rpc_owner),.tx_payload_addr(tx_payload_addr),
        .tx_active(rpc_active),.tx_start(rpc_start),.tx_length(rpc_length),.tx_port(rpc_port),.tx_data(rpc_data),.scl(thermal_scl),.sda(thermal_sda));
    task_command_parser u_task_parser(
        .clk(rx_clk),.reset_n(network_reset_n),
        .packet_start(rx_start),.payload_valid(rx_valid),.payload_data(rx_data),
        .packet_good(rx_good),.packet_error(rx_error),.payload_length(rx_length),
        .command_valid(task_command_rx_valid),.command(task_command_rx));
    // Bundle semantic bits with their command byte through the SAME mailbox.
    // Decoding before the held-data capture removes the byte-decode cone
    // from the SYS50 live motion/map veto, with no new command clock.
    function [5:0] task_action_decode;
        input [7:0] value;
        begin
            task_action_decode[0]=value==8'h15;
            task_action_decode[1]=value==8'h40||value==8'h19||value==8'h07||value==8'h09||
                                  value==8'h44||value==8'h43||value==8'h16||value==8'h0d;
            task_action_decode[2]=value==8'h46;
            task_action_decode[3]=value==8'h4a;
            task_action_decode[4]=value==8'h42;
            task_action_decode[5]=value==8'h1c;
        end
    endfunction
    telemetry_mailbox #(.WIDTH(14),.RESET_SYNC_RELEASE(1)) u_task_command_cdc(
        .src_clk(rx_clk),.dst_clk(clk50m),.reset_n(network_reset_n),
        .src_valid(task_command_rx_valid),.src_data({task_action_decode(task_command_rx),task_command_rx}),
        .dst_valid(task_command_valid),.dst_data({task_command_actions,task_command}));
    wire thermal_rx_valid,thermal_rx_sensor,thermal_rx_task;
    wire signed [15:0] thermal_rx_maximum;
    wire [6:0] thermal_rx_threshold;
    // Temperature remains available while SW4 video is disabled. Do not
    // put this parser/mailbox under the detection-session reset.
    thermal_packet_parser u_thermal_parser(
        .clk(rx_clk),.reset_n(network_reset_n),
        .packet_start(rx_start),.payload_valid(rx_valid),.payload_data(rx_data),
        .packet_good(rx_good),.packet_error(rx_error),.payload_length(rx_length),
        .sample_valid(thermal_rx_valid),.sensor_valid(thermal_rx_sensor),
        .task_active(thermal_rx_task),.maximum_centi(thermal_rx_maximum),
        .threshold_c(thermal_rx_threshold));
    telemetry_mailbox #(.WIDTH(25),.RESET_SYNC_RELEASE(1)) u_thermal_receive_cdc(
        .src_clk(rx_clk),.dst_clk(clk50m),.reset_n(network_reset_n),
        .src_valid(thermal_rx_valid),
        .src_data({thermal_rx_task,thermal_rx_sensor,thermal_rx_threshold,thermal_rx_maximum}),
        .dst_valid(thermal_sample_valid),
        .dst_data({thermal_task_active,thermal_sensor_valid,thermal_threshold_c,thermal_maximum_centi}));

    reg session_started;
    reg [31:0] session_first_frame_id;
    always @(posedge clk125m or negedge tx_session_reset_n) begin
        if(!tx_session_reset_n) begin session_started<=0; session_first_frame_id<=0; end
        else if(!recognition_sync_tx) session_started<=0;
        else if(camera_start && recognition_sync_tx && !(DYNAMIC_SCREEN && screen_sync_tx) && !session_started) begin
            session_started<=1;
            session_first_frame_id<=latest_frame_id;
        end
    end
    // Repeat the complete latest/minimum-id snapshot until mailbox ack. The
    // first actual new-session TX gates RX, and its lower bound rejects a Pi
    // reply that was still being inferred when SW4 was turned off.
    wire [64:0] latest_rx_snapshot;
    telemetry_mailbox #(.WIDTH(65),.RESET_SYNC_RELEASE(1)) u_video_id_cdc(
        .src_clk(clk125m),.dst_clk(rx_clk),.reset_n(session_reset_n),
        .src_valid(session_started && recognition_sync_tx),
        .src_data({session_started,session_first_frame_id,latest_frame_id}),
        .dst_valid(),.dst_data(latest_rx_snapshot));
    wire parsed_valid, parsed_ready;
    wire [31:0] parsed_frame_id;
    wire [3:0] parsed_count;
    wire [511:0] parsed_coords;
    wire [55:0] parsed_classes;
    wire [119:0] parsed_label_text;
    wire [3:0] parsed_label_length;
    wire [31:0] parsed_tracker_session;
    wire [255:0] parsed_track_ids;
    wire [79:0] parsed_confidences;
    wire [7:0] parsed_observed;
    wire box_rx_start,box_rx_valid,box_rx_end,box_rx_good,box_rx_error;
    wire [7:0] box_rx_data;
    wire [15:0] box_rx_length;
    rx_payload_pipeline u_box_rx_stage(
        .clk(rx_clk),.reset_n(rx_session_reset_n),
        .packet_start(rx_start),.payload_valid(rx_valid),.payload_end(rx_end),
        .packet_good(rx_good),.packet_error(rx_error),
        .payload_data(rx_data),.payload_length(rx_length),
        .out_start(box_rx_start),.out_valid(box_rx_valid),.out_end(box_rx_end),
        .out_good(box_rx_good),.out_error(box_rx_error),
        .out_data(box_rx_data),.out_length(box_rx_length));
    detection_box_parser #(.REQUIRE_VIDEO_FRAME(1),.TRACKING_ENABLE(FOLLOW_TRACK_ENABLE)) u_box_parser(
        .clk(rx_clk),.reset_n(rx_session_reset_n),
        .packet_start(box_rx_start),.payload_valid(box_rx_valid),.payload_data(box_rx_data),
        .payload_end(box_rx_end),.packet_done(box_rx_good),
        .packet_error(box_rx_error),.payload_length(box_rx_length),
        .boxes_ready(parsed_ready),.latest_video_valid(latest_rx_snapshot[64]),
        .latest_video_frame_id(latest_rx_snapshot[31:0]),
        .minimum_frame_id(latest_rx_snapshot[63:32]),
        .minimum_frame_valid(latest_rx_snapshot[64]),
        .boxes_valid(parsed_valid),.frame_id(parsed_frame_id),
        .box_count(parsed_count),.box_coords(parsed_coords),
        .box_label_text(parsed_label_text),.box_label_length(parsed_label_length),
        .box_classes(parsed_classes),.tracker_session(parsed_tracker_session),
        .box_track_ids(parsed_track_ids),.box_confidences(parsed_confidences),
        .box_observed(parsed_observed),.packet_reject());
    detection_box_cdc #(.RESET_SYNC_RELEASE(1),.TRACK_METADATA_ENABLE(FOLLOW_TRACK_ENABLE)) u_boxes_cdc(
        // Keep mailbox reset common to both clocks; do not release its RX
        // side from the video-domain enable synchronizer. No source transfer
        // is possible before RX's own enable, new VS, first TX and Pi reply.
        .src_clk(rx_clk),.dst_clk(display_clk),.reset_n(session_reset_n),
        .src_valid(parsed_valid),.src_ready(parsed_ready),
        .src_frame_id(parsed_frame_id),.src_box_count(parsed_count),
        .src_box_coords(parsed_coords),.src_box_classes(parsed_classes),
        .src_label_text(parsed_label_text),.src_label_length(parsed_label_length),
        .src_tracker_session(parsed_tracker_session),.src_box_track_ids(parsed_track_ids),
        .src_box_confidences(parsed_confidences),.src_box_observed(parsed_observed),
        .dst_valid(boxes_valid),
        .dst_frame_id(boxes_frame_id),.dst_box_count(boxes_count),
        .dst_box_coords(boxes_coords),.dst_box_classes(boxes_classes),
        .dst_label_text(boxes_label_text),.dst_label_length(boxes_label_length),
        .dst_tracker_session(boxes_tracker_session),.dst_box_track_ids(boxes_track_ids),
        .dst_box_confidences(boxes_confidences),.dst_box_observed(boxes_observed));
endmodule
