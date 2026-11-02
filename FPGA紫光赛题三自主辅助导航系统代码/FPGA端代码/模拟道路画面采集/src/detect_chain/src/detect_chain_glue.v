`timescale 1ns / 1ps
module detect_chain_glue #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter integer USE_SCENE = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_rgb,
    input  wire [1:0]  i_lane_sel,
    output wire        o_de,
    output wire        o_hs,
    output wire        o_vs,
    output wire [23:0] o_rgb,
    output wire [2:0]  o_lane_led,
    output wire        o_alarm,
    output wire [1:0]  o_arrow,
    output wire [1:0]  o_tl_state,
    output wire [1:0]  o_lane_type_l,
    output wire [1:0]  o_lane_type_r,
    output wire        o_zebra,
    output wire        o_box_valid,
    output wire [11:0] o_box_x0, o_box_x1, o_box_y0, o_box_y1,
    output wire [3:0]  o_led_dbg
);
    wire        src_de, src_hs, src_vs;
    wire [23:0] src_rgb;
    wire [15:0] src_rgb565;
    wire [11:0] src_x, src_y;
    generate
    if (USE_SCENE == 1) begin : g_scene
        wire        vs, hs, de;
        wire [11:0] x_act, y_act;
        sync_vg #(
            .X_BITS(12), .Y_BITS(12),
            .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_FP(H_FP), .H_ACT(H_ACT),
            .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_FP(V_FP), .V_ACT(V_ACT),
            .HV_OFFSET(12'd0)
        ) u_sync_vg (
            .clk (clk), .rstn (rst_n),
            .vs_out (vs), .hs_out (hs), .de_out (de), .de_re (),
            .x_act (x_act), .y_act (y_act)
        );
        reg [1:0]  tl_color;
        reg [23:0] tl_cnt;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                tl_color <= 2'd0; tl_cnt <= 24'd0;
            end else if (tl_cnt == 24'hFF_FFFF) begin
                tl_cnt <= 24'd0;
                tl_color <= (tl_color == 2'd2) ? 2'd0 : tl_color + 2'd1;
            end else begin
                tl_cnt <= tl_cnt + 24'd1;
            end
        end
        scene_gen u_scene (
            .clk (clk), .rst_n (rst_n),
            .i_de (de), .i_hs (hs), .i_vs (vs), .x (x_act), .y (y_act),
            .tl_color (tl_color),
            .o_de (src_de), .o_hs (src_hs), .o_vs (src_vs),
            .o_rgb888 (src_rgb), .o_rgb565 (src_rgb565),
            .o_x (src_x), .o_y (src_y)
        );
    end else begin : g_ext
        assign src_de     = i_de;
        assign src_hs     = i_hs;
        assign src_vs     = i_vs;
        assign src_rgb    = i_rgb;
        assign src_rgb565 = {i_rgb[23:19], i_rgb[15:10], i_rgb[7:3]};
        assign src_x      = 12'd0;
        assign src_y      = 12'd0;
    end
    endgenerate
    wire [1:0]  tl_state;
    wire        tl_vld;
    wire [17:0] tl_area, tl_box;
    traffic_light u_tl (
        .clk (clk), .rst_n (rst_n),
        .i_de (src_de), .i_hs (src_hs), .i_vs (src_vs),
        .i_rgb (src_rgb565),
        .i_xc (src_x), .i_yc (src_y),
        .o_state (tl_state), .o_valid (tl_vld), .o_area (tl_area), .o_box (tl_box)
    );
    wire        g_de, g_hs, g_vs;
    wire [7:0]  g_gray;
    rgb2gray #(.IN_FMT(1)) u_gray (
        .clk (clk), .rst_n (rst_n),
        .i_de (src_de), .i_hs (src_hs), .i_vs (src_vs), .i_data (src_rgb),
        .o_de (g_de), .o_hs (g_hs), .o_vs (g_vs), .o_gray (g_gray)
    );
    wire        gs_de, gs_hs, gs_vs;
    wire [7:0]  gs_gray;
    wire [11:0] gs_xc, gs_yc;
    gauss3x3 #(.H_ACT(H_ACT), .V_ACT(V_ACT), .AW(11)) u_gauss (
        .clk (clk), .rst_n (rst_n),
        .i_de (g_de), .i_hs (g_hs), .i_vs (g_vs), .i_gray (g_gray),
        .o_de (gs_de), .o_hs (gs_hs), .o_vs (gs_vs),
        .o_gray (gs_gray), .o_xc (gs_xc), .o_yc (gs_yc)
    );
    wire        b_de, b_hs, b_vs, b_bin;
    wire [11:0] b_xc, b_yc;
    wire [7:0]  b_level;
    binarize #(.H_ACT(H_ACT), .ROI_Y0(12'd0), .ROI_Y1(12'd1079)) u_binarize (
        .clk (clk), .rst_n (rst_n),
        .i_de (gs_de), .i_hs (gs_hs), .i_vs (gs_vs),
        .i_gray (gs_gray), .i_xc (gs_xc), .i_yc (gs_yc),
        .i_floor (8'd0), .i_floor_we (1'b0),
        .o_de (b_de), .o_hs (b_hs), .o_vs (b_vs),
        .o_bin (b_bin), .o_xc (b_xc), .o_yc (b_yc), .o_level (b_level)
    );
    wire        lane_vld;
    wire [11:0] lane_posl, lane_posr;
    lane_detect u_lane (
        .clk (clk), .rst_n (rst_n),
        .i_de (b_de), .i_hs (b_hs), .i_vs (b_vs),
        .i_bin (b_bin), .i_xc (b_xc), .i_yc (b_yc),
        .o_type_l (o_lane_type_l), .o_type_r (o_lane_type_r),
        .o_valid (lane_vld), .o_pos_l (lane_posl), .o_pos_r (lane_posr)
    );
    wire       zb_vld;
    wire [9:0] zb_runmax;
    wire [5:0] zb_ntrmax;
    zebra_detect u_zebra (
        .clk (clk), .rst_n (rst_n),
        .i_de (b_de), .i_hs (b_hs), .i_vs (b_vs),
        .i_bin (b_bin), .i_xc (b_xc), .i_yc (b_yc),
        .o_zebra (o_zebra), .o_valid (zb_vld),
        .o_runmax (zb_runmax), .o_ntrmax (zb_ntrmax)
    );
    wire [15:0] fd_cnt;
    frame_diff_ccl u_fd (
        .clk (clk), .rst_n (rst_n),
        .i_de (b_de), .i_hs (b_hs), .i_vs (b_vs),
        .i_bin (b_bin), .i_xc (b_xc), .i_yc (b_yc),
        .o_box_valid (o_box_valid),
        .o_x0 (o_box_x0), .o_x1 (o_box_x1), .o_y0 (o_box_y0), .o_y1 (o_box_y1),
        .o_cnt (fd_cnt)
    );
    reg  vs_d;
    wire vs_rise = src_vs & ~vs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vs_d <= 1'b0;
        else        vs_d <= src_vs;
    end
    wire [1:0] fsm_lane, fsm_tl, fsm_line;
    wire       fsm_motion;
    wire obstacle = o_box_valid | (|fd_cnt[15:8]);
    ctrl_fsm u_fsm (
        .clk (clk), .rst_n (rst_n),
        .i_fv (vs_rise),
        .i_lane_sel (i_lane_sel),
        .i_line_type (o_lane_type_r),
        .i_tl (tl_state),
        .i_zebra (o_zebra),
        .i_obstacle (obstacle),
        .o_lane (fsm_lane),
        .o_arrow (o_arrow),
        .o_led_lane (o_lane_led),
        .o_motion (fsm_motion),
        .o_tl (fsm_tl),
        .o_line_type (fsm_line),
        .o_alarm (o_alarm)
    );
    assign o_tl_state = tl_state;
    wire [8:0]  obs_act;
    wire [35:0] obs_st;
    assign obs_act[0] = g_de  & (|g_gray[7:1]);
    assign obs_act[1] = gs_de & (|gs_gray[7:1]);
    assign obs_act[2] = b_de  & b_bin;
    assign obs_act[3] = lane_vld;
    assign obs_act[4] = zb_vld;
    assign obs_act[5] = o_box_valid;
    assign obs_act[6] = tl_vld;
    assign obs_act[7] = (o_alarm | fsm_motion);
    assign obs_act[8] = src_de;
    assign obs_st[3:0]   = g_gray[7:4];
    assign obs_st[7:4]   = gs_gray[7:4];
    assign obs_st[11:8]  = b_level[7:4];
    assign obs_st[15:12] = {o_lane_type_l, o_lane_type_r};
    assign obs_st[19:16] = {o_zebra, zb_ntrmax[2:0]};
    assign obs_st[23:20] = fd_cnt[3:0];
    assign obs_st[27:24] = {tl_state, tl_area[17:16]};
    assign obs_st[31:28] = {o_alarm, o_lane_led};
    assign obs_st[35:32] = src_rgb[23:20];
    activity_stat u_stat (
        .clk (clk), .rst_n (rst_n),
        .act (obs_act), .st (obs_st),
        .led (o_led_dbg)
    );
    wire [12:0] box0_w = {1'b0, o_box_x1} - {1'b0, o_box_x0};
    wire [12:0] box0_h = {1'b0, o_box_y1} - {1'b0, o_box_y0};
    wire        box0_size_ok = (box0_w >= 13'd8) && (box0_h >= 13'd8);
    wire        box0_on      = o_box_valid & box0_size_ok;
    osd_overlay u_osd (
        .clk (clk), .rst_n (rst_n),
        .i_de (src_de), .i_hs (src_hs), .i_vs (src_vs), .i_rgb (src_rgb),
        .i_box0_en (box0_on),
        .i_box0_x0 (o_box_x0), .i_box0_y0 (o_box_y0),
        .i_box0_x1 (o_box_x1), .i_box0_y1 (o_box_y1),
        .i_box0_color (24'hFF0000), .i_box0_alpha (5'd16), .i_box0_fill (1'b0),
        .i_arrow_en (|o_lane_led),
        .i_arrow_sel (o_arrow),
        .i_arrow_x (12'd150), .i_arrow_y (12'd800),
        .i_arrow_color (24'hFFFFFF), .i_arrow_alpha (5'd16),
        .o_de (o_de), .o_hs (o_hs), .o_vs (o_vs), .o_rgb (o_rgb)
    );
endmodule
