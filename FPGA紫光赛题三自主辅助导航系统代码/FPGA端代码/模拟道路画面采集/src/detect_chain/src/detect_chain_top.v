`timescale 1ns / 1ps
module detect_chain_top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080
)(
    input  wire        sys_clk,
    input  wire        rst_n,
    output wire [3:0]  led,
    output wire        pixclk_out,
    output wire        vs_out,
    output wire        hs_out,
    output wire        de_out,
    output wire [7:0]  r_out,
    output wire [7:0]  g_out,
    output wire [7:0]  b_out,
    output wire        iic_scl,
    inout  wire        iic_sda,
    output wire        iic_tx_scl,
    inout  wire        iic_tx_sda
);
    wire pix_clk, cfg_clk, pll_lock;
    pll u_pll (
        .clkin1   (sys_clk  ),
        .clkout0  (pix_clk  ),
        .clkout1  (cfg_clk  ),
        .pll_lock (pll_lock )
    );
    reg rstn_p1, rstn_p2;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin rstn_p1 <= 1'b0; rstn_p2 <= 1'b0; end
        else        begin rstn_p1 <= 1'b1; rstn_p2 <= rstn_p1; end
    end
    wire rstn_pix = rstn_p2;
    wire        vs, hs, de;
    wire [11:0] x_act, y_act;
    sync_vg #(
        .X_BITS(12), .Y_BITS(12),
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_FP(H_FP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_FP(V_FP), .V_ACT(V_ACT),
        .HV_OFFSET(12'd0)
    ) u_sync_vg (
        .clk    (pix_clk  ),
        .rstn   (rstn_pix ),
        .vs_out (vs       ),
        .hs_out (hs       ),
        .de_out (de       ),
        .de_re  (         ),
        .x_act  (x_act    ),
        .y_act  (y_act    )
    );
    reg [1:0]  tl_color;
    reg [23:0] tl_cnt;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin
            tl_color <= 2'd0;
            tl_cnt   <= 24'd0;
        end else if (tl_cnt == 24'hFF_FFFF) begin
            tl_cnt   <= 24'd0;
            tl_color <= (tl_color == 2'd2) ? 2'd0 : tl_color + 2'd1;
        end else begin
            tl_cnt <= tl_cnt + 24'd1;
        end
    end
    wire        sc_de, sc_hs, sc_vs;
    wire [23:0] sc_rgb;
    wire [15:0] sc_rgb565;
    wire [11:0] sc_x, sc_y;
    scene_gen u_scene (
        .clk      (pix_clk  ),
        .rst_n    (rstn_pix ),
        .i_de     (de       ),
        .i_hs     (hs       ),
        .i_vs     (vs       ),
        .x        (x_act    ),
        .y        (y_act    ),
        .tl_color (tl_color ),
        .o_de     (sc_de    ),
        .o_hs     (sc_hs    ),
        .o_vs     (sc_vs    ),
        .o_rgb888 (sc_rgb   ),
        .o_rgb565 (sc_rgb565),
        .o_x      (sc_x     ),
        .o_y      (sc_y     )
    );
    wire        g_de, g_hs, g_vs;
    wire [7:0]  g_gray;
    rgb2gray #(
        .IN_FMT (1)
    ) u_gray (
        .clk    (pix_clk  ),
        .rst_n  (rstn_pix ),
        .i_de   (sc_de    ),
        .i_hs   (sc_hs    ),
        .i_vs   (sc_vs    ),
        .i_data (sc_rgb   ),
        .o_de   (g_de     ),
        .o_hs   (g_hs     ),
        .o_vs   (g_vs     ),
        .o_gray (g_gray   )
    );
    wire        gs_de, gs_hs, gs_vs;
    wire [7:0]  gs_gray;
    wire [11:0] gs_xc, gs_yc;
    gauss3x3 #(
        .H_ACT (H_ACT),
        .V_ACT (V_ACT),
        .AW    (11)
    ) u_gauss (
        .clk    (pix_clk  ),
        .rst_n  (rstn_pix ),
        .i_de   (g_de     ),
        .i_hs   (g_hs     ),
        .i_vs   (g_vs     ),
        .i_gray (g_gray   ),
        .o_de   (gs_de    ),
        .o_hs   (gs_hs    ),
        .o_vs   (gs_vs    ),
        .o_gray (gs_gray  ),
        .o_xc   (gs_xc    ),
        .o_yc   (gs_yc    )
    );
    wire        b_de, b_hs, b_vs, b_bin;
    wire [11:0] b_xc, b_yc;
    wire [7:0]  b_level;
    binarize #(
        .H_ACT  (H_ACT),
        .ROI_Y0 (12'd0   ),
        .ROI_Y1 (12'd1079)
    ) u_binarize (
        .clk        (pix_clk  ),
        .rst_n      (rstn_pix ),
        .i_de       (gs_de    ),
        .i_hs       (gs_hs    ),
        .i_vs       (gs_vs    ),
        .i_gray     (gs_gray  ),
        .i_xc       (gs_xc    ),
        .i_yc       (gs_yc    ),
        .i_floor    (8'd0     ),
        .i_floor_we (1'b0     ),
        .o_de       (b_de     ),
        .o_hs       (b_hs     ),
        .o_vs       (b_vs     ),
        .o_bin      (b_bin    ),
        .o_xc       (b_xc     ),
        .o_yc       (b_yc     ),
        .o_level    (b_level  )
    );
    wire [1:0]  lane_tl, lane_tr;
    wire        lane_vld;
    wire [11:0] lane_posl, lane_posr;
    lane_detect u_lane (
        .clk      (pix_clk  ),
        .rst_n    (rstn_pix ),
        .i_de     (b_de     ),
        .i_hs     (b_hs     ),
        .i_vs     (b_vs     ),
        .i_bin    (b_bin    ),
        .i_xc     (b_xc     ),
        .i_yc     (b_yc     ),
        .o_type_l (lane_tl  ),
        .o_type_r (lane_tr  ),
        .o_valid  (lane_vld ),
        .o_pos_l  (lane_posl),
        .o_pos_r  (lane_posr)
    );
    wire        zb_zebra, zb_vld;
    wire [9:0]  zb_runmax;
    wire [5:0]  zb_ntrmax;
    zebra_detect u_zebra (
        .clk      (pix_clk  ),
        .rst_n    (rstn_pix ),
        .i_de     (b_de     ),
        .i_hs     (b_hs     ),
        .i_vs     (b_vs     ),
        .i_bin    (b_bin    ),
        .i_xc     (b_xc     ),
        .i_yc     (b_yc     ),
        .o_zebra  (zb_zebra ),
        .o_valid  (zb_vld   ),
        .o_runmax (zb_runmax),
        .o_ntrmax (zb_ntrmax)
    );
    wire        fd_vld;
    wire [11:0] fd_x0, fd_x1, fd_y0, fd_y1;
    wire [15:0] fd_cnt;
    frame_diff_ccl u_fd (
        .clk        (pix_clk  ),
        .rst_n      (rstn_pix ),
        .i_de       (b_de     ),
        .i_hs       (b_hs     ),
        .i_vs       (b_vs     ),
        .i_bin      (b_bin    ),
        .i_xc       (b_xc     ),
        .i_yc       (b_yc     ),
        .o_box_valid(fd_vld   ),
        .o_x0       (fd_x0    ),
        .o_x1       (fd_x1    ),
        .o_y0       (fd_y0    ),
        .o_y1       (fd_y1    ),
        .o_cnt      (fd_cnt   )
    );
    wire [1:0]  tl_state;
    wire        tl_vld;
    wire [17:0] tl_area, tl_box;
    traffic_light u_tl (
        .clk     (pix_clk  ),
        .rst_n   (rstn_pix ),
        .i_de    (sc_de    ),
        .i_hs    (sc_hs    ),
        .i_vs    (sc_vs    ),
        .i_rgb   (sc_rgb565),
        .i_xc    (sc_x     ),
        .i_yc    (sc_y     ),
        .o_state (tl_state ),
        .o_valid (tl_vld   ),
        .o_area  (tl_area  ),
        .o_box   (tl_box   )
    );
    reg  vs_d;
    wire vs_rise = sc_vs & ~vs_d;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) vs_d <= 1'b0;
        else           vs_d <= sc_vs;
    end
    wire [1:0] fsm_lane, fsm_arrow, fsm_tl, fsm_line;
    wire [2:0] fsm_led;
    wire       fsm_motion, fsm_alarm;
    wire obstacle = fd_vld | (|fd_cnt[15:8]);
    ctrl_fsm u_fsm (
        .clk         (pix_clk  ),
        .rst_n       (rstn_pix ),
        .i_fv        (vs_rise  ),
        .i_lane_sel  (2'd1     ),
        .i_line_type (lane_tr  ),
        .i_tl        (tl_state ),
        .i_zebra     (zb_zebra ),
        .i_obstacle  (obstacle ),
        .o_lane      (fsm_lane ),
        .o_arrow     (fsm_arrow),
        .o_led_lane  (fsm_led  ),
        .o_motion    (fsm_motion),
        .o_tl        (fsm_tl   ),
        .o_line_type (fsm_line ),
        .o_alarm     (fsm_alarm)
    );
    wire [8:0]  obs_act;
    wire [35:0] obs_st;
    assign obs_act[0] = g_de  & (|g_gray[7:1]);
    assign obs_act[1] = gs_de & (|gs_gray[7:1]);
    assign obs_act[2] = b_de  & b_bin;
    assign obs_act[3] = lane_vld;
    assign obs_act[4] = zb_vld;
    assign obs_act[5] = fd_vld;
    assign obs_act[6] = tl_vld;
    assign obs_act[7] = (fsm_alarm | fsm_motion);
    assign obs_act[8] = sc_de;
    assign obs_st[3:0]   = g_gray[7:4];
    assign obs_st[7:4]   = gs_gray[7:4];
    assign obs_st[11:8]  = b_level[7:4];
    assign obs_st[15:12] = {lane_tl, lane_tr};
    assign obs_st[19:16] = {zb_zebra, zb_ntrmax[2:0]};
    assign obs_st[23:20] = fd_cnt[3:0];
    assign obs_st[27:24] = {tl_state, tl_area[17:16]};
    assign obs_st[31:28] = {fsm_alarm, fsm_led};
    assign obs_st[35:32] = sc_rgb[23:20];
    activity_stat u_stat (
        .clk   (pix_clk  ),
        .rst_n (rstn_pix ),
        .act   (obs_act  ),
        .st    (obs_st   ),
        .led   (led      )
    );
    reg [15:0] cfg_cnt;
    reg        rstn_cfg;
    always @(posedge cfg_clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_cnt  <= 16'd0;
            rstn_cfg <= 1'b0;
        end else if (cfg_cnt == 16'd10000) begin
            rstn_cfg <= 1'b1;
        end else begin
            cfg_cnt <= cfg_cnt + 16'd1;
        end
    end
    wire        osd_de, osd_hs, osd_vs;
    wire [23:0] osd_rgb;
    wire        init_over;
    osd_overlay u_osd (
        .clk            (pix_clk    ),
        .rst_n          (rstn_pix   ),
        .i_de           (sc_de      ),
        .i_hs           (sc_hs      ),
        .i_vs           (sc_vs      ),
        .i_rgb          (sc_rgb     ),
        .i_box0_en      (fd_vld     ),
        .i_box0_x0      (fd_x0      ),
        .i_box0_y0      (fd_y0      ),
        .i_box0_x1      (fd_x1      ),
        .i_box0_y1      (fd_y1      ),
        .i_box0_color   (24'hFF0000 ),
        .i_box0_alpha   (5'd16      ),
        .i_box0_fill    (1'b0       ),
        .i_arrow_en     (|fsm_led   ),
        .i_arrow_sel    (fsm_arrow  ),
        .i_arrow_x      (12'd150    ),
        .i_arrow_y      (12'd800    ),
        .i_arrow_color  (24'hFFFFFF ),
        .i_arrow_alpha  (5'd16      ),
        .o_de           (osd_de     ),
        .o_hs           (osd_hs     ),
        .o_vs           (osd_vs     ),
        .o_rgb          (osd_rgb    )
    );
    hdmi_tx u_tx (
        .cfg_clk     (cfg_clk    ),
        .cfg_rst_n   (rstn_cfg   ),
        .pix_clk     (pix_clk    ),
        .pix_rst_n   (rstn_pix   ),
        .i_de        (osd_de     ),
        .i_hs        (osd_hs     ),
        .i_vs        (osd_vs     ),
        .i_rgb       (osd_rgb    ),
        .o_pixclk    (pixclk_out ),
        .o_de        (de_out     ),
        .o_hs        (hs_out     ),
        .o_vs        (vs_out     ),
        .o_r        (r_out      ),
        .o_g        (g_out      ),
        .o_b        (b_out      ),
        .iic_scl     (iic_scl    ),
        .iic_sda     (iic_sda    ),
        .iic_tx_scl  (iic_tx_scl ),
        .iic_tx_sda  (iic_tx_sda ),
        .o_init_over (init_over  )
    );
endmodule
