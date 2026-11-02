`timescale 1ns / 1ps
module sys_top #(
    parameter MEM_ROW_ADDR_WIDTH = 15,
    parameter MEM_COL_ADDR_WIDTH = 10,
    parameter MEM_BADDR_WIDTH    = 3,
    parameter MEM_DQ_WIDTH       = 32,
    parameter H_NUM = 12'd1920,
    parameter V_NUM = 12'd1080,
    parameter SRC_HDMI = 1'b1,
    parameter [2:0] SRC_NUM = 3'd1,
    parameter        PCIE_EN = 1'b1
)(
    input  wire        sys_clk,
    output wire        hdmi_rst,
    input  wire        rst_n,
    input  wire        key1,
    output wire [3:0]  led,
    input  wire        pixclk_in,
    input  wire        vs_in,
    input  wire        hs_in,
    input  wire        de_in,
    input  wire [7:0]  r_in,
    input  wire [7:0]  g_in,
    input  wire [7:0]  b_in,
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
    inout  wire        iic_tx_sda,
    output wire                          mem_rst_n,
    output wire                          mem_ck,
    output wire                          mem_ck_n,
    output wire                          mem_cke,
    output wire                          mem_cs_n,
    output wire                          mem_ras_n,
    output wire                          mem_cas_n,
    output wire                          mem_we_n,
    output wire                          mem_odt,
    output wire [MEM_ROW_ADDR_WIDTH-1:0] mem_a,
    output wire [MEM_BADDR_WIDTH-1:0]    mem_ba,
    inout  wire [MEM_DQ_WIDTH/8-1:0]     mem_dqs,
    inout  wire [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n,
    inout  wire [MEM_DQ_WIDTH-1:0]       mem_dq,
    output wire [MEM_DQ_WIDTH/8-1:0]     mem_dm,
    input  wire        perst_n,
    input  wire        ref_clk_p,
    input  wire        ref_clk_n,
    input  wire [1:0]  rxp,
    input  wire [1:0]  rxn,
    output wire [1:0]  txp,
    output wire [1:0]  txn,
    output wire        uart_txd,
    input  wire        uart_rxd
);
    reg rstn_s1, rstn_s2;
    always @(posedge sys_clk) begin
        rstn_s1 <= rst_n;
        rstn_s2 <= rstn_s1;
    end
    reg [15:0] hdmi_rst_cnt;
    always @(posedge sys_clk) begin
        if (hdmi_rst_cnt != 16'h2710) begin
            hdmi_rst_cnt <= hdmi_rst_cnt + 16'd1;
        end
    end
    assign hdmi_rst = (hdmi_rst_cnt == 16'h2710);
    wire rstn = rstn_s2;
    wire pix_clk, cfg_clk, pll_lock;
    wire vs_disp, hs_disp, de_disp;
    wire [11:0] x_disp, y_disp;
    timing_gen u_tg (
        .sys_clk   (sys_clk   ),
        .rst_n     (rstn      ),
        .pix_clk   (pix_clk   ),
        .cfg_clk   (cfg_clk   ),
        .pll_lock  (pll_lock  ),
        .vs_out    (vs_disp   ),
        .hs_out    (hs_disp   ),
        .de_out    (de_disp   ),
        .x_act     (x_disp    ),
        .y_act     (y_disp    )
    );
    wire pix_rst_n = pll_lock;
    wire core_clk;
    wire ddr_init_done;
    wire ddr_pll_lock;
    wire [27:0]  axi_awaddr, axi_araddr;
    wire [3:0]   axi_awuser_id, axi_aruser_id;
    wire [3:0]   axi_awlen, axi_arlen;
    wire [255:0] axi_wdata, axi_rdata;
    wire [31:0]  axi_wstrb;
    wire         axi_awready, axi_awvalid, axi_wready, axi_wusero_last;
    wire         axi_arready, axi_arvalid, axi_rvalid, axi_rlast;
    wire [3:0]   axi_rid;
    localparam [27:0] TL_TICK = 28'd100_000_000;
    reg [27:0] tl_cnt;
    reg [1:0]  tl_color;
    always @(posedge sys_clk or negedge rstn) begin
        if (!rstn) begin
            tl_cnt   <= 28'd0;
            tl_color <= 2'd2;
        end else if (tl_cnt == TL_TICK) begin
            tl_cnt   <= 28'd0;
            tl_color <= (tl_color == 2'd2) ? 2'd0 :
                        (tl_color == 2'd0) ? 2'd1 : 2'd2;
        end else begin
            tl_cnt <= tl_cnt + 28'd1;
        end
    end
    wire        sc_de, sc_hs, sc_vs;
    wire [23:0] sc_rgb888;
    wire [15:0] sc_rgb565;
    wire [11:0] sc_x, sc_y;
    wire        fb_init_done;
    scene_gen u_wr_scene (
        .clk      (pix_clk      ),
        .rst_n    (pix_rst_n    ),
        .i_de     (de_disp      ),
        .i_hs     (hs_disp      ),
        .i_vs     (vs_disp      ),
        .x        (x_disp       ),
        .y        (y_disp       ),
        .tl_color (tl_color     ),
        .o_de     (sc_de        ),
        .o_hs     (sc_hs        ),
        .o_vs     (sc_vs        ),
        .o_rgb888 (sc_rgb888    ),
        .o_rgb565 (sc_rgb565    ),
        .o_x      (sc_x         ),
        .o_y      (sc_y         )
    );
    wire        vout_de;
    wire [15:0] vout_data;
    wire        hd_vs, hd_de;
    wire [15:0] hd_rgb565;
    hdmi_wr u_hdmi_wr (
        .clk      (pixclk_in),
        .rst_n    (rst_n    ),
        .de       (de_in    ),
        .r        (r_in     ),
        .g        (g_in     ),
        .b        (b_in     ),
        .o_vs     (hd_vs    ),
        .o_de     (hd_de    ),
        .o_rgb565 (hd_rgb565)
    );
    wire        wr_vin_clk = (SRC_HDMI != 1'b0) ? pixclk_in : pix_clk;
    wire        wr_fsync_f = (SRC_HDMI != 1'b0) ? hd_vs     : sc_vs;
    wire        wr_en_f    = (SRC_HDMI != 1'b0) ? hd_de     : sc_de;
    wire [15:0] wr_data_f  = (SRC_HDMI != 1'b0) ? hd_rgb565 : sc_rgb565;
    fram_buf #(
        .H_NUM     (H_NUM     ),
        .V_NUM     (V_NUM     ),
        .PIX_WIDTH (16        )
    ) u_fram_buf (
        .ddr_clk        (core_clk       ),
        .ddr_rstn       (ddr_init_done  ),
        .vin_clk        (wr_vin_clk    ),
        .wr_fsync       (wr_fsync_f    ),
        .wr_en          (wr_en_f       ),
        .wr_data        (wr_data_f     ),
        .vout_clk       (pix_clk        ),
        .rd_fsync       (vs_disp        ),
        .rd_en          (de_disp        ),
        .vout_de        (vout_de        ),
        .vout_data      (vout_data      ),
        .init_done      (fb_init_done   ),
        .axi_awaddr     (axi_awaddr     ),
        .axi_awid       (axi_awuser_id  ),
        .axi_awlen      (axi_awlen      ),
        .axi_awsize     (               ),
        .axi_awburst    (               ),
        .axi_awready    (axi_awready    ),
        .axi_awvalid    (axi_awvalid    ),
        .axi_wdata      (axi_wdata      ),
        .axi_wstrb      (axi_wstrb      ),
        .axi_wlast      (axi_wusero_last),
        .axi_wvalid     (               ),
        .axi_wready     (axi_wready     ),
        .axi_bid        (4'd0           ),
        .axi_araddr     (axi_araddr     ),
        .axi_arid       (axi_aruser_id  ),
        .axi_arlen      (axi_arlen      ),
        .axi_arsize     (               ),
        .axi_arburst    (               ),
        .axi_arvalid    (axi_arvalid    ),
        .axi_arready    (axi_arready    ),
        .axi_rready     (               ),
        .axi_rdata      (axi_rdata      ),
        .axi_rvalid     (axi_rvalid     ),
        .axi_rlast      (axi_rlast      ),
        .axi_rid        (axi_rid        )
    );
    wire [23:0] vout_rgb888 = {vout_data[15:11], vout_data[10:8],
                               vout_data[10:5],  vout_data[4:3],
                               vout_data[4:0],   vout_data[2:0]};
    wire        det_de, det_hs, det_vs;
    wire [23:0] det_rgb;
    wire [2:0]  det_lane_led;
    wire        det_alarm;
    wire [1:0]  det_arrow;
    wire [1:0]  det_tl_state;
    wire [1:0]  det_lane_type_l, det_lane_type_r;
    wire        det_zebra;
    wire        det_box_valid;
    wire [11:0] det_box_x0, det_box_x1, det_box_y0, det_box_y1;
    wire [3:0]  det_led_dbg;
    reg [2:0]  k1_sync;
    reg [19:0] k1_cnt;
    reg        k1_stable;
    reg        k1_stable_d;
    reg [1:0]  ui_lane;
    always @(posedge sys_clk or negedge rstn) begin
        if (!rstn) begin
            k1_sync     <= 3'b111;
            k1_cnt      <= 20'd0;
            k1_stable   <= 1'b1;
            k1_stable_d <= 1'b1;
            ui_lane     <= 2'd1;
        end else begin
            k1_sync <= {k1_sync[1:0], key1};
            if (k1_sync[2] == k1_stable) begin
                k1_cnt <= 20'd0;
            end else if (k1_cnt == 20'd999_999) begin
                k1_stable <= k1_sync[2];
                k1_cnt    <= 20'd0;
            end else begin
                k1_cnt <= k1_cnt + 20'd1;
            end
            k1_stable_d <= k1_stable;
            if (k1_stable_d && !k1_stable) begin
                ui_lane <= (ui_lane == 2'd2) ? 2'd0 : ui_lane + 2'd1;
            end
        end
    end
    wire       ui_alarm = det_alarm;
    detect_chain_glue #(
        .USE_SCENE (0)
    ) u_det (
        .clk            (pix_clk        ),
        .rst_n          (pix_rst_n      ),
        .i_de           (vout_de        ),
        .i_hs           (hs_disp        ),
        .i_vs           (vs_disp        ),
        .i_rgb          (vout_rgb888    ),
        .i_lane_sel     (ui_lane        ),
        .o_de           (det_de         ),
        .o_hs           (det_hs         ),
        .o_vs           (det_vs         ),
        .o_rgb          (det_rgb        ),
        .o_lane_led     (det_lane_led   ),
        .o_alarm        (det_alarm      ),
        .o_arrow        (det_arrow      ),
        .o_tl_state     (det_tl_state   ),
        .o_lane_type_l  (det_lane_type_l),
        .o_lane_type_r  (det_lane_type_r),
        .o_zebra        (det_zebra      ),
        .o_box_valid    (det_box_valid  ),
        .o_box_x0       (det_box_x0     ),
        .o_box_x1       (det_box_x1     ),
        .o_box_y0       (det_box_y0     ),
        .o_box_y1       (det_box_y1     ),
        .o_led_dbg      (det_led_dbg    )
    );
    localparam [11:0] DISP_H = 12'd1920;
    localparam [11:0] DISP_V = 12'd1080;
    localparam integer TAP_GRAY_DLY = 3;
    localparam integer TAP_BIN_DLY  = 4;
    wire        g_de, g_hs, g_vs;
    wire [7:0]  g_gray;
    rgb2gray #(.IN_FMT(1)) u_g_disp (
        .clk     (pix_clk    ),
        .rst_n   (pix_rst_n  ),
        .i_de    (det_de     ),
        .i_hs    (det_hs     ),
        .i_vs    (det_vs     ),
        .i_data  (det_rgb    ),
        .o_de    (g_de       ),
        .o_hs    (g_hs       ),
        .o_vs    (g_vs       ),
        .o_gray  (g_gray     )
    );
    wire [11:0] gx, gy;
    xy_cnt #(.XMAX(DISP_H), .YMAX(DISP_V)) u_cnt_gray (
        .clk (pix_clk), .rst_n (pix_rst_n), .i_de (g_de), .i_vs (g_vs),
        .qx (gx), .qy (gy)
    );
    wire        b_de, b_hs, b_vs, b_bin;
    wire [11:0] b_xc, b_yc;
    wire [7:0]  b_level;
    binarize #(.H_ACT(DISP_H), .ROI_Y0(12'd0), .ROI_Y1(12'd1079)) u_b_disp (
        .clk        (pix_clk    ),
        .rst_n      (pix_rst_n  ),
        .i_de       (g_de       ),
        .i_hs       (g_hs       ),
        .i_vs       (g_vs       ),
        .i_gray     (g_gray     ),
        .i_xc       (gx         ),
        .i_yc       (gy         ),
        .i_floor    (8'd0       ),
        .i_floor_we (1'b0       ),
        .o_de       (b_de       ),
        .o_hs       (b_hs       ),
        .o_vs       (b_vs       ),
        .o_bin      (b_bin      ),
        .o_xc       (b_xc       ),
        .o_yc       (b_yc       ),
        .o_level    (b_level    )
    );
    wire        r4_de, r4_hs, r4_vs;
    wire [23:0] r4_rgb;
    dly_pix #(.W(24), .N(TAP_BIN_DLY)) u_dly_rgb (
        .clk (pix_clk), .rst_n (pix_rst_n),
        .i_de (det_de), .i_hs (det_hs), .i_vs (det_vs), .i_data (det_rgb),
        .o_de (r4_de), .o_hs (r4_hs), .o_vs (r4_vs), .o_data (r4_rgb)
    );
    wire       g1_de, g1_hs, g1_vs;
    wire [7:0] g1_gray;
    dly_pix #(.W(8), .N(TAP_BIN_DLY - TAP_GRAY_DLY)) u_dly_gray (
        .clk (pix_clk), .rst_n (pix_rst_n),
        .i_de (g_de), .i_hs (g_hs), .i_vs (g_vs), .i_data (g_gray),
        .o_de (g1_de), .o_hs (g1_hs), .o_vs (g1_vs), .o_data (g1_gray)
    );
    wire [23:0] gray_rgb = {g1_gray, g1_gray, g1_gray};
    wire [23:0] bin_rgb  = b_bin ? 24'hFFFFFF : 24'h000000;
    wire [11:0] qx, qy;
    xy_cnt #(.XMAX(DISP_H), .YMAX(DISP_V)) u_cnt_pat (
        .clk (pix_clk), .rst_n (pix_rst_n), .i_de (b_de), .i_vs (b_vs),
        .qx (qx), .qy (qy)
    );
    wire [23:0] pat3_rgb = ~r4_rgb;
    wire        q_de, q_hs, q_vs;
    wire [23:0] q_rgb;
    wire [1:0]  q_sel;
    quad_mux #(.H_ACT(DISP_H), .V_ACT(DISP_V)) u_quad (
        .i_de    (b_de      ),
        .i_hs    (b_hs      ),
        .i_vs    (b_vs      ),
        .i_x     (qx        ),
        .i_y     (qy        ),
        .s0_rgb  (r4_rgb    ),
        .s1_rgb  (gray_rgb  ),
        .s2_rgb  (bin_rgb   ),
        .s3_rgb  (pat3_rgb  ),
        .o_de    (q_de      ),
        .o_hs    (q_hs      ),
        .o_vs    (q_vs      ),
        .o_rgb   (q_rgb     ),
        .o_quad  (q_sel     )
    );
    wire        out_de  = (SRC_NUM == 3'd4) ? q_de  : r4_de ;
    wire        out_hs  = (SRC_NUM == 3'd4) ? q_hs  : r4_hs ;
    wire        out_vs  = (SRC_NUM == 3'd4) ? q_vs  : r4_vs ;
    wire [23:0] out_rgb = (SRC_NUM == 3'd4) ? q_rgb : r4_rgb;
    reg  [1:0]  img_mode;
    wire [23:0] proc_rgb;
    wire        proc_de;
    reg         proc_hs, proc_vs;
    img_proc u_img (
        .clk   (pix_clk   ),
        .rst_n (pix_rst_n ),
        .i_de  (out_de    ),
        .i_rgb (out_rgb   ),
        .mode  (img_mode  ),
        .o_rgb (proc_rgb  ),
        .o_de  (proc_de   )
    );
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            proc_hs <= 1'b0;
            proc_vs <= 1'b0;
        end else begin
            proc_hs <= out_hs;
            proc_vs <= out_vs;
        end
    end
    reg [26:0] k1_hold;
    reg        k1_hold_hit;
    always @(posedge sys_clk or negedge rstn) begin
        if (!rstn) begin
            k1_hold     <= 27'd0;
            k1_hold_hit <= 1'b0;
            img_mode    <= 2'd0;
        end else if (k1_stable) begin
            k1_hold     <= 27'd0;
            k1_hold_hit <= 1'b0;
        end else if (!k1_hold_hit) begin
            if (k1_hold >= 27'd74_999_999) begin
                img_mode    <= img_mode + 2'd1;
                k1_hold_hit <= 1'b1;
            end else begin
                k1_hold <= k1_hold + 27'd1;
            end
        end
    end
    wire hdmi_init_over;
    hdmi_tx u_tx (
        .cfg_clk      (cfg_clk        ),
        .cfg_rst_n    (rstn           ),
        .pix_clk      (pix_clk        ),
        .pix_rst_n    (pix_rst_n      ),
        .i_de         (proc_de       ),
        .i_hs         (proc_hs       ),
        .i_vs         (proc_vs       ),
        .i_rgb        (proc_rgb      ),
        .o_pixclk     (pixclk_out     ),
        .o_de         (de_out         ),
        .o_hs         (hs_out         ),
        .o_vs         (vs_out         ),
        .o_r          (r_out          ),
        .o_g          (g_out          ),
        .o_b          (b_out          ),
        .iic_scl      (iic_scl        ),
        .iic_sda      (iic_sda        ),
        .iic_tx_scl   (iic_tx_scl     ),
        .iic_tx_sda   (iic_tx_sda     ),
        .o_init_over  (hdmi_init_over )
    );
    reg [31:0] det_status ;
    reg [11:0] quad_dbg   ;
    reg [3:0]  det_act    ;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            det_status <= 32'd0;
            quad_dbg   <= 12'd0;
            det_act    <= 4'd0;
        end else begin
            det_status <= {det_box_x0[11:5], det_box_y0[11:5],
                           det_box_x1[11:5], det_box_y1[11:5],
                           det_box_valid, det_zebra, det_tl_state};
            quad_dbg   <= {q_sel, qx[9:0]};
            det_act    <= det_led_dbg;
        end
    end
    reg [31:0] wr_beats ;
    reg [31:0] rd_beats ;
    always @(posedge core_clk) begin
        if (!ddr_init_done) begin
            wr_beats <= 32'd0;
            rd_beats <= 32'd0;
        end else begin
            if (axi_awvalid & axi_awready) wr_beats <= wr_beats + 32'd1;
            if (axi_arvalid & axi_arready) rd_beats <= rd_beats + 32'd1;
        end
    end
    wire smlh_link_up, rdlh_link_up;
    wire [15:0] pcie_rgb565 = {out_rgb[23:19], out_rgb[15:10], out_rgb[7:3]};
    generate
    if (PCIE_EN) begin : g_pcie_on
        wire [15:0] pcie_rgb565 = {out_rgb[23:19], out_rgb[15:10], out_rgb[7:3]};
        pcie_axis_top u_pcie (
            .button_rst_n   (rstn            ),
            .perst_n        (perst_n         ),
            .free_clk       (sys_clk         ),
            .txd            (uart_txd        ),
            .rxd            (uart_rxd        ),
            .ref_clk_n      (ref_clk_n       ),
            .ref_clk_p      (ref_clk_p       ),
            .rxn            (rxn             ),
            .rxp            (rxp             ),
            .txn            (txn             ),
            .txp            (txp             ),
            .ref_led        (                ),
            .pclk_led       (                ),
            .pclk_div2_led  (                ),
            .smlh_link_up   (smlh_link_up    ),
            .rdlh_link_up   (rdlh_link_up    ),
            .vin_clk        (pix_clk         ),
            .vin_de         (out_de          ),
            .vin_hs         (1'b0            ),
            .vin_vs         (out_vs          ),
            .vin_rgb565     (pcie_rgb565     ),
            .dbg_rd_addr    (12'd0           ),
            .dbg_rd_base    (12'd0           ),
            .dbg_rd_hs_cnt  (16'd0           ),
            .dbg_line_had_cnt(16'd0          ),
            .dbg_lock_fire_cnt(16'd0         ),
            .dbg_wr_step_bad(16'd0          ),
            .dbg_ddr_rdone_cnt(16'd0         ),
            .dbg_wr_line_snap(16'd0          ),
            .dbg_wr_addr_snap(9'd0           ),
            .dbg_px_bad_snap(12'd0          ),
            .dbg_hd_line_snap(12'd0         ),
            .dbg_x_glitch_snap(16'd0        ),
            .dbg_diag       (                )
        ,
        .lane_l        (det_lane_type_l ),
        .lane_r        (det_lane_type_r )
        );
    end else begin : g_pcie_off
        assign smlh_link_up = 1'b0;
        assign rdlh_link_up = 1'b0;
        assign txp = 2'b00;
        assign txn = 2'b00;
        assign uart_txd = 1'b1;
        wire _unused_pcie_in = &{1'b0, perst_n, ref_clk_p, ref_clk_n,
                                rxp[0], rxp[1], rxn[0], rxn[1]};
    end
    endgenerate
    assign led[0] = ddr_init_done;
    assign led[1] = pll_lock;
    assign led[2] = fb_init_done;
    assign led[3] = hdmi_init_over;
    DDR3_50H u_DDR3_50H (
        .ref_clk                   (sys_clk            ),
        .resetn                    (rstn               ),
        .ddr_init_done             (ddr_init_done      ),
        .ddrphy_clkin              (core_clk           ),
        .pll_lock                  (ddr_pll_lock      ),
        .axi_awaddr                (axi_awaddr         ),
        .axi_awuser_ap             (1'b0               ),
        .axi_awuser_id             (axi_awuser_id      ),
        .axi_awlen                 (axi_awlen          ),
        .axi_awready               (axi_awready        ),
        .axi_awvalid               (axi_awvalid        ),
        .axi_wdata                 (axi_wdata          ),
        .axi_wstrb                 (axi_wstrb          ),
        .axi_wready                (axi_wready         ),
        .axi_wusero_id             (                   ),
        .axi_wusero_last           (axi_wusero_last    ),
        .axi_araddr                (axi_araddr         ),
        .axi_aruser_ap             (1'b0               ),
        .axi_aruser_id             (axi_aruser_id      ),
        .axi_arlen                 (axi_arlen          ),
        .axi_arready               (axi_arready        ),
        .axi_arvalid               (axi_arvalid        ),
        .axi_rdata                 (axi_rdata          ),
        .axi_rid                   (axi_rid            ),
        .axi_rlast                 (axi_rlast          ),
        .axi_rvalid                (axi_rvalid         ),
        .apb_clk                   (1'b0               ),
        .apb_rst_n                 (1'b1               ),
        .apb_sel                   (1'b0               ),
        .apb_enable                (1'b0               ),
        .apb_addr                  (8'b0               ),
        .apb_write                 (1'b0               ),
        .apb_ready                 (                   ),
        .apb_wdata                 (16'b0              ),
        .apb_rdata                 (                   ),
        .apb_int                   (                   ),
        .mem_rst_n                 (mem_rst_n          ),
        .mem_ck                    (mem_ck             ),
        .mem_ck_n                  (mem_ck_n           ),
        .mem_cke                   (mem_cke            ),
        .mem_cs_n                  (mem_cs_n           ),
        .mem_ras_n                 (mem_ras_n          ),
        .mem_cas_n                 (mem_cas_n          ),
        .mem_we_n                  (mem_we_n           ),
        .mem_odt                   (mem_odt            ),
        .mem_a                     (mem_a              ),
        .mem_ba                    (mem_ba             ),
        .mem_dqs                   (mem_dqs            ),
        .mem_dqs_n                 (mem_dqs_n          ),
        .mem_dq                    (mem_dq             ),
        .mem_dm                    (mem_dm             ),
        .debug_data                (                   ),
        .debug_slice_state         (                   ),
        .debug_calib_ctrl          (                   ),
        .ck_dly_set_bin            (                   ),
        .force_ck_dly_en           (1'b0               ),
        .force_ck_dly_set_bin      (8'h05              ),
        .dll_step                  (                   ),
        .dll_lock                  (                   ),
        .init_read_clk_ctrl        (2'b0               ),
        .init_slip_step            (4'b0               ),
        .force_read_clk_ctrl       (1'b0               ),
        .ddrphy_gate_update_en     (1'b0               ),
        .update_com_val_err_flag   (                   ),
        .rd_fake_stop              (1'b0               )
    );
endmodule
module road_rom #(
    parameter integer ROM_WORDS = 14400,
    parameter integer ADDR_W    = 14
)(
    input  wire                 clk,
    input  wire [ADDR_W-1:0]    addr,
    output reg  [15:0]          dout
);
    (* ram_style = "block" *) reg [15:0] mem [0:ROM_WORDS-1];
    initial begin
        $readmemh("road_rom.mem", mem);
    end
    always @(posedge clk) begin
        dout <= mem[addr];
    end
endmodule
module hdmi_wr (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de,
    input  wire [7:0]  r,
    input  wire [7:0]  g,
    input  wire [7:0]  b,
    output wire        o_vs,
    output wire        o_de,
    output wire [15:0] o_rgb565
);
    localparam [19:0] GAP_TH = 20'd4400;
    reg  [19:0] gap_cnt;
    reg         de_d;
    reg  [ 5:0] vs_pulse;
    wire        de_rise   = de & ~de_d;
    wire        in_vblank = (gap_cnt > GAP_TH);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d     <= 1'b0;
            gap_cnt  <= 20'd0;
            vs_pulse <= 6'd0;
        end else begin
            de_d <= de;
            if (de)
                gap_cnt <= 20'd0;
            else if (gap_cnt != 20'hF_FFFF)
                gap_cnt <= gap_cnt + 20'd1;
            if (vs_pulse != 6'd0)
                vs_pulse <= vs_pulse - 6'd1;
            if (de_rise && in_vblank)
                vs_pulse <= 6'd63;
        end
    end
    assign o_vs     = (vs_pulse != 6'd0);
    assign o_de     = de;
    assign o_rgb565 = {r[7:3], g[7:2], b[7:3]};
endmodule
module img_proc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire [23:0] i_rgb,
    input  wire [1:0]  mode,
    output reg  [23:0] o_rgb,
    output reg         o_de
);
    wire [8:0] r = {1'b0, i_rgb[23:16]};
    wire [8:0] g = {1'b0, i_rgb[15:8]};
    wire [8:0] b = {1'b0, i_rgb[7:0]};
    wire [16:0] ysum = r*77 + g*150 + b*29;
    wire [8:0]  y    = ysum[16:8];
    wire signed [10:0] dr = $signed({2'b0, r}) - $signed({2'b0, y});
    wire signed [10:0] dg = $signed({2'b0, g}) - $signed({2'b0, y});
    wire signed [10:0] db = $signed({2'b0, b}) - $signed({2'b0, y});
    wire signed [10:0] gr = (mode == 2'd2) ? (dr + (dr >>> 1)) :
                            (mode == 2'd3) ? (dr >>> 1) : 11'sd0;
    wire signed [10:0] gg = (mode == 2'd2) ? (dg + (dg >>> 1)) :
                            (mode == 2'd3) ? (dg >>> 1) : 11'sd0;
    wire signed [10:0] gb = (mode == 2'd2) ? (db + (db >>> 1)) :
                            (mode == 2'd3) ? (db >>> 1) : 11'sd0;
    function [7:0] clp(input signed [10:0] v);
        begin
            if (v < 0)        clp = 8'd0;
            else if (v > 255) clp = 8'd255;
            else              clp = v[7:0];
        end
    endfunction
    wire [7:0] ro = (mode == 2'd1) ? y[7:0] : clp($signed({2'b0, y}) + gr);
    wire [7:0] go = (mode == 2'd1) ? y[7:0] : clp($signed({2'b0, y}) + gg);
    wire [7:0] bo = (mode == 2'd1) ? y[7:0] : clp($signed({2'b0, y}) + gb);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_rgb <= 24'd0;
            o_de  <= 1'b0;
        end else begin
            o_de  <= i_de;
            o_rgb <= (mode == 2'd0) ? i_rgb : {ro, go, bo};
        end
    end
endmodule
