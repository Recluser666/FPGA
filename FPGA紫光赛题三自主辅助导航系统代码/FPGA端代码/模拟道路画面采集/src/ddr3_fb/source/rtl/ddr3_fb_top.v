`timescale 1ns / 1ps
module ddr3_fb_top #(
    parameter MEM_ROW_ADDR_WIDTH = 15,
    parameter MEM_COL_ADDR_WIDTH = 10,
    parameter MEM_BADDR_WIDTH    = 3,
    parameter MEM_DQ_WIDTH       = 32,
    parameter MEM_DQS_WIDTH      = 32/8,
    parameter H_NUM              = 12'd1920,
    parameter V_NUM              = 12'd1080,
    parameter PIX_WIDTH          = 16,
    parameter PROC_MODE          = 1'b0,
    parameter PATTERN_SEL        = 1'b1,
    parameter SRC_HDMI            = 1'b1,
    parameter SRC_CAM             = 1'b1,
parameter MOSAIC_EN          = 1'b0,
    parameter TH_1S              = 27'd67_000_000,
    parameter [15:0] RSTN_1MS_MAX = 16'h2710
)(
    input  wire                          sys_clk,
    output wire                          hdmi_rst,
    //     PDS 报 `ConstraintEditor-0003: VCCIO value in the same BANK should be the same`。
    input  wire                          key,
    input  wire                          rst_n,
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
    inout  wire [MEM_DQS_WIDTH-1:0]      mem_dqs,
    inout  wire [MEM_DQS_WIDTH-1:0]      mem_dqs_n,
    inout  wire [MEM_DQ_WIDTH-1:0]       mem_dq,
    output wire [MEM_DQS_WIDTH-1:0]      mem_dm,
    output wire                          iic_scl,
    inout  wire                          iic_sda,
    output wire                          iic_tx_scl,
    inout  wire                          iic_tx_sda,
    output wire                          pixclk_out,
    output wire                          vs_out,
    output wire                          hs_out,
    output wire                          de_out,
    output wire [7:0]                    r_out,
    output wire [7:0]                    g_out,
    output wire [7:0]                    b_out,
    output wire                          uart_txd,
    input  wire                          pixclk_in,
    input  wire                          vs_in,
    input  wire                          hs_in,
    input  wire                          de_in,
    input  wire [7:0]                    r_in,
    input  wire [7:0]                    g_in,
    input  wire [7:0]                    b_in,
    inout  wire                          cmos1_scl,
    inout  wire                          cmos1_sda,
    input  wire                          cmos1_vsync,
    input  wire                          cmos1_href,
    input  wire                          cmos1_pclk,
    input  wire [7:0]                    cmos1_data,
    output wire                          cmos1_reset,
    inout  wire                          cmos2_scl,
    inout  wire                          cmos2_sda,
    input  wire                          cmos2_vsync,
    input  wire                          cmos2_href,
    input  wire                          cmos2_pclk,
    input  wire [7:0]                    cmos2_data,
    output wire                          cmos2_reset,
    input  wire                          pixclk_in3,
    input  wire                          vs_in3,
    input  wire                          hs_in3,
    input  wire                          de_in3,
    output wire [3:0]                    led
);
    wire pix_clk;
    wire cfg_clk;
    wire locked;
    wire core_clk;
    wire ddr_init_done;
    pll u_pll (
        .clkin1   (sys_clk ),
        .clkout0  (pix_clk ),
        .clkout1  (cfg_clk ),
        .pll_lock (locked  )
    );
    reg [15:0] rstn_1ms;
    reg [15:0] hdmi_rst_cnt;
    always @(posedge sys_clk) begin
        if (hdmi_rst_cnt != 16'h2710) begin
            hdmi_rst_cnt <= hdmi_rst_cnt + 16'd1;
        end
    end
    assign hdmi_rst = (hdmi_rst_cnt == 16'h2710);
    always @(posedge cfg_clk) begin
        if (!locked)
            rstn_1ms <= 16'd0;
        else if (rstn_1ms == RSTN_1MS_MAX)
            rstn_1ms <= rstn_1ms;
        else
            rstn_1ms <= rstn_1ms + 1'b1;
    end
    wire rstn_out = (rstn_1ms == RSTN_1MS_MAX);
    wire init_over_tx;
    wire init_over_rx;
    ms72xx_ctl u_ms72xx_ctl (
        .clk           (cfg_clk   ),
        .rst_n         (rstn_out  ),
        .init_over_tx  (init_over_tx),
        .init_over_rx  (init_over_rx),
        .iic_tx_scl    (iic_tx_scl),
        .iic_tx_sda    (iic_tx_sda),
        .iic_scl       (iic_scl   ),
        .iic_sda       (iic_sda   )
    );
    wire [11:0] wr_x, wr_y;
    sync_vg #(
        .X_BITS   (12),
        .Y_BITS   (12),
        .V_TOTAL  (12'd1125),
        .V_FP     (12'd4),
        .V_BP     (12'd36),
        .V_SYNC   (12'd5),
        .V_ACT    (12'd1080),
        .H_TOTAL  (12'd2200),
        .H_FP     (12'd88),
        .H_BP     (12'd148),
        .H_SYNC   (12'd44),
        .H_ACT    (12'd1920),
        .HV_OFFSET(12'd0)
    ) u_wr_sync (
        .clk    (pix_clk ),
        .rstn   (rstn_out),
        .vs_out (wr_vs   ),
        .hs_out (wr_hs   ),
        .de_out (wr_de   ),
        .de_re  (wr_de_re),
        .x_act  (wr_x    ),
        .y_act  (wr_y    )
    );
    wire [15:0] wr_pixel_stripe = {wr_y[3:0], wr_x[7:0], wr_y[7:4]};
    wire        sc_de, sc_hs, sc_vs;
    wire [23:0] sc_rgb888;
    wire [15:0] sc_rgb565;
    localparam [27:0] TL_TICK = 28'd148_437_500;
    reg [27:0] tl_cnt;
    reg [1:0]  tl_sec;
    reg [1:0]  tl_color;
    always @(posedge pix_clk or negedge fb_init_done) begin
        if (!fb_init_done) begin
            tl_cnt   <= 28'd0;
            tl_sec   <= 2'd0;
            tl_color <= 2'd2;
        end else if (tl_cnt == TL_TICK) begin
            tl_cnt <= 28'd0;
            if (tl_sec == 2'd1) begin
                tl_sec   <= 2'd0;
                tl_color <= (tl_color == 2'd2) ? 2'd0 : tl_color + 2'd1;
            end else begin
                tl_sec <= tl_sec + 2'd1;
            end
        end else begin
            tl_cnt <= tl_cnt + 28'd1;
        end
    end
    scene_gen u_scene (
        .clk      (pix_clk ),
        .rst_n    (rstn_out),
        .i_de     (wr_de   ),
        .i_hs     (wr_hs   ),
        .i_vs     (wr_vs   ),
        .x        (wr_x    ),
        .y        (wr_y    ),
        .tl_color (tl_color),
        .o_de     (sc_de   ),
        .o_hs     (sc_hs   ),
        .o_vs     (sc_vs   ),
        .o_rgb888 (sc_rgb888),
        .o_rgb565 (sc_rgb565),
        .o_x      (         ),
        .o_y      (         )
    );
    reg  [2:0] disp_mode;
    wire use_road = disp_mode[2];
    wire [11:0] rom_xs = wr_x;
    wire [11:0] rom_ys = wr_y;
    wire [8:0]  rom_x  = rom_xs[11:3];
    wire [7:0]  rom_y  = rom_ys[10:3];
    wire [16:0] rom_y17   = {9'd0, rom_y};
    wire [16:0] rom_row   = (rom_y17 << 8) - (rom_y17 << 4);
    wire [16:0] rom_sum   = rom_row + {8'd0, rom_x};
    wire [14:0] rom_addr  = rom_sum[14:0];
    wire [15:0] rom_dout;
    road_rom u_road_rom (
        .clk  (pix_clk   ),
        .addr (rom_addr  ),
        .dout (rom_dout  )
    );
    reg wr_de_d1;
    always @(posedge pix_clk or negedge rstn_out) begin
        if (!rstn_out) wr_de_d1 <= 1'b0;
        else           wr_de_d1 <= wr_de;
    end
    wire        hd_vs, hd_de;
    wire [15:0] hd_rgb565;
    hdmi_wr u_hdmi_wr (
        .clk      (pixclk_in),
        .rst_n    (rstn_out  ),
        .de       (de_in     ),
        .r        (r_in      ),
        .g        (g_in      ),
        .b        (b_in      ),
        .o_vs     (hd_vs     ),
        .o_de     (hd_de     ),
        .o_rgb565 (hd_rgb565 )
    );
    reg clk_25m_r;
    always @(posedge sys_clk) clk_25m_r <= ~clk_25m_r;
    wire clk_25m = clk_25m_r;
    wire cam_initial_en;
    wire cam_init_done;
    wire cam_clock_20k;
    wire [8:0] cam_reg_index;
    wire [15:0] cam_nack_cnt;
    power_on_delay u_cam_pwr (
        .clk_50M      (sys_clk        ),
        .reset_n      (1'b1           ),
        .camera1_rstn (cmos1_reset    ),
        .camera2_rstn (cmos2_reset    ),
        .camera_pwnd  (               ),
        .initial_en   (cam_initial_en )
    );
    reg_config u_cam_cfg (
        .clk_25M      (clk_25m        ),
        .camera_rstn  (cmos1_reset    ),
        .initial_en   (cam_initial_en ),
        .reg_conf_done(cam_init_done  ),
        .i2c_sclk     (cmos1_scl      ),
        .i2c_sdat     (cmos1_sda      ),
        .clock_20k    (cam_clock_20k  ),
        .reg_index    (cam_reg_index  ),
        .o_nack_cnt   (cam_nack_cnt   )
    );
    wire        cam2_init_done;
    wire [15:0] cam2_nack_cnt;
    reg_config u_cam2_cfg (
        .clk_25M      (clk_25m        ),
        .camera_rstn  (cmos2_reset    ),
        .initial_en   (cam_initial_en ),
        .reg_conf_done(cam2_init_done ),
        .i2c_sclk     (cmos2_scl      ),
        .i2c_sdat     (cmos2_sda      ),
        .clock_20k    (               ),
        .reg_index    (               ),
        .o_nack_cnt   (cam2_nack_cnt  )
    );
    reg [7:0] cam_d_d0;
    reg       cam_href_d0, cam_vsync_d0;
    always @(posedge cmos1_pclk) begin
        cam_d_d0     <= cmos1_data;
        cam_href_d0  <= cmos1_href;
        cam_vsync_d0 <= cmos1_vsync;
    end
    wire [15:0] cam_d_16bit;
    wire        cam_de_16bit;
    wire        cam_pclk_16bit;
    cmos_8_16bit u_cam_8_16 (
        .pclk      (cmos1_pclk       ),
        .rst_n     (cam_init_done    ),
        .pdata_i   (cam_d_d0         ),
        .de_i      (cam_href_d0      ),
        .vs_i      (cam_vsync_d0     ),
        .pixel_clk (cam_pclk_16bit   ),
        .pdata_o   (cam_d_16bit      ),
        .de_o      (cam_de_16bit     )
    );
    wire [15:0] cam_rgb565 = {cam_d_16bit[4:0], cam_d_16bit[10:5], cam_d_16bit[15:11]};
    wire        cam_vs     = cam_vsync_d0;
    reg [7:0] cam2_d_d0;
    reg       cam2_href_d0, cam2_vsync_d0;
    always @(posedge cmos2_pclk) begin
        cam2_d_d0     <= cmos2_data;
        cam2_href_d0  <= cmos2_href;
        cam2_vsync_d0 <= cmos2_vsync;
    end
    wire [15:0] cam2_d_16bit;
    wire        cam2_de_16bit;
    wire        cam2_pclk_16bit;
    cmos_8_16bit cmos2_8_16bit (
        .pclk      (cmos2_pclk       ),
        .rst_n     (cam2_init_done   ),
        .pdata_i   (cam2_d_d0        ),
        .de_i      (cam2_href_d0     ),
        .vs_i      (cam2_vsync_d0    ),
        .pixel_clk (cam2_pclk_16bit  ),
        .pdata_o   (cam2_d_16bit     ),
        .de_o      (cam2_de_16bit    )
    );
    wire [15:0] cam2_rgb565 = {cam2_d_16bit[4:0], cam2_d_16bit[10:5], cam2_d_16bit[15:11]};
    wire        cam2_vs     = cam2_vsync_d0;
    reg [1:0] c1vs_s, c2vs_s;
    always @(posedge sys_clk) begin
        c1vs_s <= {c1vs_s[0], cam_vs };
        c2vs_s <= {c2vs_s[0], cam2_vs};
    end
    wire c1vs_rise = (c1vs_s == 2'b01);
    wire c2vs_rise = (c2vs_s == 2'b01);
    reg [25:0] cam_swap_cnt;
    always @(posedge sys_clk) cam_swap_cnt <= cam_swap_cnt + 26'd1;
    reg cam_sel;
    reg cam_sel_req;
    always @(posedge sys_clk) begin
        if (!fb_init_done) begin
            cam_sel     <= 1'b0;
            cam_sel_req <= 1'b0;
        end
        else if (cam_sel_req) begin
            if ((cam_sel == 1'b0 && c1vs_rise) || (cam_sel == 1'b1 && c2vs_rise)) begin
                cam_sel     <= ~cam_sel;
                cam_sel_req <= 1'b0;
            end
        end
        else if (cam_swap_cnt == 26'd0) begin
            if (cam_sel == 1'b0 && cam2_init_done) cam_sel_req <= 1'b1;
            else if (cam_sel == 1'b1)              cam_sel_req <= 1'b1;
        end
    end
    wire        cam_ready   = cam_init_done;
    wire        cam1_active = SRC_CAM && cam_init_done  && !cam_sel;
    wire        cam2_active = SRC_CAM && cam2_init_done &&  cam_sel;
    wire        wr_use_cam  = cam1_active || cam2_active;
    wire        mosaic_fsync;
    wire        mosaic_de;
    wire [15:0] mosaic_pix;
    wire [15:0] q_srcw1, q_srcw2;
    wire [15:0] q_botline, q_camlines;
    compose_wr #(.H_LINE(1920), .V_TOP(540), .V_BOT(540), .H_CELL(960)) u_compose (
        .clk       (pixclk_in       ),
        .rst_n     (rstn_out        ),
        .hd_de     (hd_de           ),
        .hd_pix    (hd_rgb565       ),
        .hd_vs     (hd_vs           ),
        .c1_clk    (cam_pclk_16bit  ),
        .c1_vs     (cam_vs          ),
        .c1_de     (cam_de_16bit    ),
        .c1_pix    (cam_rgb565      ),
        .c2_clk    (cam2_pclk_16bit ),
        .c2_de     (cam2_de_16bit   ),
        .c2_pix    (cam2_rgb565     ),
        .o_de      (mosaic_de       ),
        .o_pix     (mosaic_pix      ),
        .o_fsync   (mosaic_fsync    ),
        .dbg_srcw1 (q_srcw1         ),
        .dbg_srcw2 (q_srcw2         ),
        .dbg_botline (q_botline     ),
        .dbg_camlines(q_camlines    )
    );
    wire        wr_vin_clk = pixclk_in;
    reg [2:0] rdvs_s;
    always @(posedge pixclk_in or negedge rstn_out) begin
        if (!rstn_out) rdvs_s <= 3'd0;
        else           rdvs_s <= {rdvs_s[1:0], rd_vs};
    end
    wire rdvs_rise = (rdvs_s[2:1] == 2'b01);
    reg  [1:0] wr_div2;
    always @(posedge pixclk_in or negedge rstn_out) begin
        if (!rstn_out)      wr_div2 <= 2'd0;
        else if (rdvs_rise) wr_div2 <= wr_div2 + 2'd1;
    end
    wire wr_frame_sync = rdvs_rise & (wr_div2 == 2'd3);
    localparam [11:0] CAM_SEG_H = 12'd540;
    localparam [11:0] CAM_SRC_W = 12'd640;
    localparam [11:0] CAM_SRC_H = 12'd1080;
    wire        cam_wr_fsync, cam_wr_en;
    wire [15:0] cam_wr_px;
    wire [11:0] cam_lines_dbg;
    cam_dual_wr #(
        .H_NUM   (CAM_SRC_W      ),
        .SEG_V   (CAM_SEG_H      )
    ) u_cam_frm (
        .clk      (cam_pclk_16bit ),
        .rst_n    (rstn_out       ),
        .vs1      (cam_vs         ),
        .de1      (cam_de_16bit   ),
        .pix1     (cam_rgb565     ),
        .clk2     (cam2_pclk_16bit),
        .vs2      (cam2_vs        ),
        .de2      (cam2_de_16bit  ),
        .pix2     (cam2_rgb565    ),
        .o_fsync  (cam_wr_fsync   ),
        .o_en     (cam_wr_en      ),
        .o_px     (cam_wr_px      ),
        .o_rows   (               ),
        .o_lines  (cam_lines_dbg  )
    );
    wire        wr_fsync_f = hd_vs;
    wire        wr_de_f    = hd_de;
    wire [15:0] wr_pixel_f = hd_rgb565;
    wire        rd_vs, rd_hs, rd_de, rd_de_re;
    wire [11:0] rd_x, rd_y;
    wire [15:0] vout_data;
    wire        vout_de;
    sync_vg #(
        .X_BITS   (12),
        .Y_BITS   (12),
        .V_TOTAL  (12'd1125),
        .V_FP     (12'd4   ),
        .V_BP     (12'd36  ),
        .V_SYNC   (12'd5   ),
        .V_ACT    (12'd1080),
        .H_TOTAL  (12'd2200),
        .H_FP     (12'd88  ),
        .H_BP     (12'd148 ),
        .H_SYNC   (12'd44  ),
        .H_ACT    (12'd1920),
        .H_ACT    (12'd1280),
        .HV_OFFSET(12'd0)
    ) u_rd_sync (
        .clk    (pix_clk ),
        .rstn   (fb_init_done),
        .vs_out (rd_vs   ),
        .hs_out (rd_hs   ),
        .de_out (rd_de   ),
        .de_re  (rd_de_re),
        .x_act  (rd_x    ),
        .y_act  (rd_y    )
    );
    wire [MEM_ROW_ADDR_WIDTH+MEM_BADDR_WIDTH+MEM_COL_ADDR_WIDTH-1:0] axi_awaddr;
    wire [3:0]  axi_awuser_id;
    wire [3:0]  axi_awlen;
    wire        axi_awready;
    wire        axi_awvalid;
    wire [MEM_DQ_WIDTH*8-1:0] axi_wdata;
    wire [MEM_DQ_WIDTH-1:0]   axi_wstrb;
    wire        axi_wready;
    wire        axi_wusero_last;
    wire [MEM_ROW_ADDR_WIDTH+MEM_BADDR_WIDTH+MEM_COL_ADDR_WIDTH-1:0] axi_araddr;
    wire [3:0]  axi_aruser_id;
    wire [3:0]  axi_arlen;
    wire        axi_arready;
    wire        axi_arvalid;
    wire [MEM_DQ_WIDTH*8-1:0] axi_rdata;
    wire        axi_rvalid;
    wire [3:0]  axi_rid;
    wire        axi_rlast;
    wire        pll_lock_ddr;
    DDR3_50H u_DDR3_50H (
        .ref_clk                 (sys_clk        ),
        .resetn                  (rstn_out       ),
        .ddr_init_done           (ddr_init_done  ),
        .ddrphy_clkin            (core_clk       ),
        .pll_lock                (pll_lock_ddr   ),
        .axi_awaddr              (axi_awaddr     ),
        .axi_awuser_ap           (1'b0           ),
        .axi_awuser_id           (axi_awuser_id  ),
        .axi_awlen               (axi_awlen      ),
        .axi_awready             (axi_awready    ),
        .axi_awvalid             (axi_awvalid    ),
        .axi_wdata               (axi_wdata      ),
        .axi_wstrb               (axi_wstrb      ),
        .axi_wready              (axi_wready     ),
        .axi_wusero_id           (               ),
        .axi_wusero_last         (axi_wusero_last),
        .axi_araddr              (axi_araddr     ),
        .axi_aruser_ap           (1'b0           ),
        .axi_aruser_id           (axi_aruser_id  ),
        .axi_arlen               (axi_arlen      ),
        .axi_arready             (axi_arready    ),
        .axi_arvalid             (axi_arvalid    ),
        .axi_rdata               (axi_rdata      ),
        .axi_rid                 (axi_rid        ),
        .axi_rlast               (axi_rlast      ),
        .axi_rvalid              (axi_rvalid     ),
        .apb_clk                 (1'b0           ),
        .apb_rst_n               (1'b1           ),
        .apb_sel                 (1'b0           ),
        .apb_enable              (1'b0           ),
        .apb_addr                (8'b0           ),
        .apb_write               (1'b0           ),
        .apb_ready               (               ),
        .apb_wdata               (16'b0          ),
        .apb_rdata               (               ),
        .apb_int                 (               ),
        .mem_rst_n               (mem_rst_n      ),
        .mem_ck                  (mem_ck         ),
        .mem_ck_n                (mem_ck_n       ),
        .mem_cke                 (mem_cke        ),
        .mem_cs_n                (mem_cs_n       ),
        .mem_ras_n               (mem_ras_n      ),
        .mem_cas_n               (mem_cas_n      ),
        .mem_we_n                (mem_we_n       ),
        .mem_odt                 (mem_odt        ),
        .mem_a                   (mem_a          ),
        .mem_ba                  (mem_ba         ),
        .mem_dqs                 (mem_dqs        ),
        .mem_dqs_n               (mem_dqs_n      ),
        .mem_dq                  (mem_dq         ),
        .mem_dm                  (mem_dm         ),
        .debug_data              (               ),
        .debug_slice_state       (               ),
        .debug_calib_ctrl        (               ),
        .ck_dly_set_bin          (               ),
        .force_ck_dly_en         (1'b0           ),
        .force_ck_dly_set_bin    (8'h05          ),
        .dll_step                (               ),
        .dll_lock                (               ),
        .init_read_clk_ctrl      (2'b0           ),
        .init_slip_step          (4'b0           ),
        .force_read_clk_ctrl     (1'b0           ),
        .ddrphy_gate_update_en   (1'b0           ),
        .update_com_val_err_flag (               ),
        .rd_fake_stop            (1'b0           )
    );
    wire fb_init_done;
`ifdef USE_SIM_BUF
    fb_bram #(
        .H_NUM     (H_NUM    ),
        .V_NUM     (V_NUM    ),
        .PIX_WIDTH (PIX_WIDTH)
    ) u_fram_buf (
        .vin_clk   (wr_vin_clk),
        .wr_fsync  (wr_fsync_f),
        .wr_en     (wr_de_f     ),
        .wr_data   (wr_pixel_f  ),
        .vout_clk  (pix_clk     ),
        .rd_fsync  (rd_vs       ),
        .rd_en     (rd_de_re    ),
        .vout_de   (vout_de     ),
        .vout_data (vout_data   ),
        .init_done (fb_init_done),
        .dbg_wr_ptr(             ),
        .dbg_rd_ptr(             ),
        .dbg_shadow(             )
    );
    wire        cam_vout_de;
    wire [15:0] cam_vout_data;
    assign cam_vout_de   = 1'b0;
    assign cam_vout_data = 16'd0;
`else
    localparam [11:0] FB_H_NUM = 12'd1920;
    localparam [11:0] FB_V_NUM = 12'd1080;
    localparam [11:0] CAM_FB_H_NUM = CAM_SRC_W;
    localparam [11:0] CAM_FB_V_NUM = CAM_SRC_H;
    wire        cam_vout_de;
    wire [15:0] cam_vout_data;
    fram_buf_dual #(
        .MEM_ROW_WIDTH    (MEM_ROW_ADDR_WIDTH),
        .MEM_COLUMN_WIDTH (MEM_COL_ADDR_WIDTH),
        .MEM_BANK_WIDTH   (MEM_BADDR_WIDTH   ),
        .MEM_DQ_WIDTH     (MEM_DQ_WIDTH      ),
        .H_NUM            (FB_H_NUM          ),
        .V_NUM            (FB_V_NUM          ),
        .CAM_H_NUM        (CAM_FB_H_NUM      ),
        .CAM_V_NUM        (CAM_FB_V_NUM      ),
        .CAM_ADDR_OFFSET  (28'h080_0000     ),
        .PIX_WIDTH        (PIX_WIDTH         )
    ) u_fram_buf (
        .ddr_clk        (core_clk       ),
        .ddr_rstn       (ddr_init_done  ),
        .vin_clk        (wr_vin_clk),
        .wr_fsync       (wr_fsync_f),
        .wr_en          (wr_de_f        ),
        .wr_data        (wr_pixel_f     ),
        .vout_clk       (pix_clk        ),
        .rd_fsync       (rd_vs          ),
        .rd_en          (rd_de_re       ),
        .vout_de        (vout_de        ),
        .vout_data      (vout_data      ),
        .init_done      (fb_init_done   ),
        .cam_vin_clk    (cam_pclk_16bit ),
        .cam_fsync      (cam_wr_fsync   ),
        .cam_en         (cam_wr_en      ),
        .cam_data       (cam_wr_px      ),
        .cam_rd_de      (rd_de          ),
        .cam_rd_x       (rd_x           ),
        .cam_rd_y       (rd_y           ),
        .cam_vout_de    (cam_vout_de    ),
        .cam_vout_data  (cam_vout_data  ),
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
`endif
    wire [15:0] pixel_expected = {rd_y[3:0], rd_x[7:0], rd_y[7:4]};
    reg        verify_ok;
    reg        frame_had_error;
    reg [31:0] ok_frames;
    reg [31:0] bad_frames;
    reg [31:0] err_pixels;
    reg        rd_active_tgl;
    reg        rd_vs_d;
    always @(posedge pix_clk) begin
        if (!fb_init_done) begin
            verify_ok        <= 1'b0;
            frame_had_error  <= 1'b0;
            ok_frames        <= 32'd0;
            bad_frames       <= 32'd0;
            err_pixels       <= 32'd0;
            rd_active_tgl    <= 1'b0;
            rd_vs_d          <= 1'b0;
        end else begin
            rd_vs_d <= rd_vs;
            if (vout_de) rd_active_tgl <= ~rd_active_tgl;
            if (vout_de && (vout_data !== pixel_expected)) begin
                frame_had_error <= 1'b1;
                if (err_pixels != 32'hFFFFFFFF) err_pixels <= err_pixels + 32'd1;
            end
            if (rd_vs && !rd_vs_d) begin
                if (frame_had_error) begin
                    if (bad_frames != 32'hFFFFFFFF) bad_frames <= bad_frames + 32'd1;
                    verify_ok       <= 1'b0;
                    frame_had_error <= 1'b0;
                end else begin
                    if (ok_frames != 32'hFFFFFFFF) ok_frames <= ok_frames + 32'd1;
                    verify_ok <= 1'b1;
                end
            end
        end
    end
    reg [31:0] wr_beats ;
    reg [31:0] rd_beats ;
    always @(posedge core_clk) begin
        if (!ddr_init_done) begin
            wr_beats <= 32'd0;
            rd_beats <= 32'd0;
        end else begin
            if (axi_awvalid && axi_awready) wr_beats <= wr_beats + 32'd1;
            if (axi_arvalid && axi_arready) rd_beats <= rd_beats + 32'd1;
        end
    end
    wire [0:0] key_deb, key2_deb;
    btn_deb_fix #(
        .BTN_WIDTH (1         ),
        .BTN_DELAY (20'hF_FFFF)
    ) u_deb_k1 (
        .clk         (pix_clk ),
        .btn_in      (key     ),
        .btn_deb_fix (key_deb )
    );
    btn_deb_fix #(
        .BTN_WIDTH (1         ),
        .BTN_DELAY (20'hF_FFFF)
    ) u_deb_k2 (
        .clk         (pix_clk ),
        .btn_in      (rst_n   ),
        .btn_deb_fix (key2_deb)
    );
    wire key_pressed  = ~key_deb[0];
    wire key2_pressed = ~key2_deb[0];
    reg [19:0] por_cnt;
    wire       deb_ready = (por_cnt == 20'hF_FFFF);
    reg  key_d, key2_d;
    reg  [1:0] lane_sel;
    always @(posedge pix_clk or negedge fb_init_done) begin
        if (!fb_init_done) begin
            por_cnt   <= 20'd0;
            key_d     <= 1'b0;
            key2_d    <= 1'b0;
            disp_mode <= {1'b0, 1'b0, PROC_MODE};
            lane_sel  <= 2'd1;
        end else begin
            if (!deb_ready)
                por_cnt <= por_cnt + 20'd1;
            key_d  <= key_pressed;
            key2_d <= key2_pressed;
            if (deb_ready && key_pressed & ~key_d)
                lane_sel <= (lane_sel == 2'd2) ? 2'd0 : lane_sel + 2'd1;
            if (deb_ready && key2_pressed & ~key2_d)
                disp_mode <= (disp_mode == 3'd7) ? 3'd0 : disp_mode + 3'd1;
        end
    end
    wire        rgbg_de, rgbg_hs, rgbg_vs;
    wire [7:0]  rgbg_gray;
    rgb2gray #(
        .IN_FMT (0)
    ) u_rgb2gray (
        .clk    (pix_clk     ),
        .rst_n  (fb_init_done),
        .i_de   (vout_de     ),
        .i_hs   (rd_hs       ),
        .i_vs   (rd_vs       ),
        .i_data ({8'd0, vout_data}),
        .o_de   (rgbg_de     ),
        .o_hs   (rgbg_hs     ),
        .o_vs   (rgbg_vs     ),
        .o_gray (rgbg_gray   )
    );
    wire        gau_de, gau_hs, gau_vs;
    wire [7:0]  gau_gray;
    wire [11:0] gau_xc, gau_yc;
    gauss3x3 #(
        .H_ACT (H_NUM),
        .V_ACT (V_NUM),
        .AW    (11   )
    ) u_gauss (
        .clk    (pix_clk     ),
        .rst_n  (fb_init_done),
        .i_de   (rgbg_de     ),
        .i_hs   (rgbg_hs     ),
        .i_vs   (rgbg_vs     ),
        .i_gray (rgbg_gray   ),
        .o_de   (gau_de      ),
        .o_hs   (gau_hs      ),
        .o_vs   (gau_vs      ),
        .o_gray (gau_gray    ),
        .o_xc   (gau_xc      ),
        .o_yc   (gau_yc      )
    );
    wire        bin_de, bin_hs, bin_vs, bin_bit;
    wire [11:0] bin_xc, bin_yc;
    wire [7:0]  bin_level;
    binarize #(
        .H_ACT  (H_NUM   ),
        .ROI_Y0 (12'd0   ),
        .ROI_Y1 (12'd719 )
    ) u_binarize (
        .clk        (pix_clk     ),
        .rst_n      (fb_init_done),
        .i_de       (gau_de      ),
        .i_hs       (gau_hs      ),
        .i_vs       (gau_vs      ),
        .i_gray     (gau_gray    ),
        .i_xc       (gau_xc      ),
        .i_yc       (gau_yc      ),
        .i_floor    (8'd0        ),
        .i_floor_we (1'b0        ),
        .o_de       (bin_de      ),
        .o_hs       (bin_hs      ),
        .o_vs       (bin_vs      ),
        .o_bin      (bin_bit     ),
        .o_xc       (bin_xc      ),
        .o_yc       (bin_yc      ),
        .o_level    (bin_level   )
    );
    wire        lane_vld;
    wire [1:0]  lane_type_l, lane_type_r;
    wire [11:0] lane_pos_l,  lane_pos_r;
    lane_detect u_lane (
        .clk      (pix_clk     ),
        .rst_n    (fb_init_done),
        .i_de     (bin_de      ),
        .i_hs     (bin_hs      ),
        .i_vs     (bin_vs      ),
        .i_bin    (bin_bit     ),
        .i_xc     (bin_xc      ),
        .i_yc     (bin_yc      ),
        .o_type_l (lane_type_l ),
        .o_type_r (lane_type_r ),
        .o_valid  (lane_vld    ),
        .o_pos_l  (lane_pos_l  ),
        .o_pos_r  (lane_pos_r  )
    );
    wire rd_fv = rd_vs & ~rd_vs_d;
    wire [1:0] fsm_lane, fsm_tl, fsm_line;
    wire       fsm_motion, fsm_alarm;
    wire [2:0] fsm_led_lane;
    wire [1:0] fsm_arrow;
    reg [11:0] tlx_cnt, tly_cnt;
    reg        vout_de_d;
    wire       tlde_fall = vout_de_d & ~vout_de;
    always @(posedge pix_clk or negedge fb_init_done) begin
        if (!fb_init_done) begin
            tlx_cnt   <= 12'd0;
            tly_cnt   <= 12'd0;
            vout_de_d <= 1'b0;
        end else begin
            vout_de_d <= vout_de;
            if (vout_de)                tlx_cnt <= tlx_cnt + 12'd1;
            else                        tlx_cnt <= 12'd0;
            if (tlde_fall)              tly_cnt <= tly_cnt + 12'd1;
            if (rd_vs && !rd_vs_d)      tly_cnt <= 0;
        end
    end
    wire [1:0]  tl_det_state;
    wire        tl_det_valid;
    wire [17:0] tl_det_area, tl_det_box;
    traffic_light #(
        .ROI_X0   (12'd600 ),
        .ROI_X1   (12'd1100),
        .ROI_Y0   (12'd100 ),
        .ROI_Y1   (12'd460 ),
        .MIN_AREA (18'd20  )
    ) u_traffic_light (
        .clk     (pix_clk      ),
        .rst_n   (fb_init_done ),
        .i_de    (vout_de      ),
        .i_hs    (rd_hs        ),
        .i_vs    (rd_vs        ),
        .i_rgb   (vout_data    ),
        .i_xc    (tlx_cnt      ),
        .i_yc    (tly_cnt      ),
        .o_state (tl_det_state ),
        .o_valid (tl_det_valid ),
        .o_area  (tl_det_area  ),
        .o_box   (tl_det_box   )
    );
    wire [1:0] tl_for_fsm = (tl_det_state == 2'd3) ? 2'd2 : tl_det_state;
    ctrl_fsm u_fsm (
        .clk         (pix_clk     ),
        .rst_n       (fb_init_done),
        .i_fv        (rd_fv       ),
        .i_lane_sel  (lane_sel    ),
        .i_line_type (line_type_fsm),
        .i_tl        (tl_for_fsm  ),
        .i_zebra     (1'b0        ),
        .i_obstacle  (1'b0        ),
        .o_lane      (fsm_lane    ),
        .o_arrow     (fsm_arrow   ),
        .o_led_lane  (fsm_led_lane),
        .o_motion    (fsm_motion  ),
        .o_tl        (fsm_tl      ),
        .o_line_type (fsm_line    ),
        .o_alarm     (fsm_alarm   )
    );
    wire        gs_de, gs_hs, gs_vs;
    wire [7:0]  gs_gray;
    gray_scale3x #(
        .IN_FMT (0        ),
        .H_ACT  (H_NUM    ),
        .V_ACT  (V_NUM    )
    ) u_gray (
        .clk    (pix_clk     ),
        .rst_n  (fb_init_done),
        .i_de   (vout_de     ),
        .i_hs   (rd_hs       ),
        .i_vs   (rd_vs       ),
        .i_data ({8'd0, vout_data}),
        .o_de   (gs_de       ),
        .o_hs   (gs_hs       ),
        .o_vs   (gs_vs       ),
        .o_gray (gs_gray     )
    );
    wire [15:0] cam_disp_pix = cam_vout_de ? cam_vout_data : vout_data;
    wire [15:0] disp_pix = cam_disp_pix;
    wire [4:0] src_r5 = disp_pix[15:11];
    wire [5:0] src_g6 = disp_pix[10:5];
    wire [4:0] src_b5 = disp_pix[4:0];
    wire [7:0] q_r8 = {src_r5, src_r5[4:2]};
    wire [7:0] q_g8 = {src_g6, src_g6[5:4]};
    wire [7:0] q_b8 = {src_b5, src_b5[4:2]};
    wire [23:0] q_color = {q_r8, q_g8, q_b8};
    wire [15:0] q_gw = ( ({8'd0,q_r8} << 6) + ({8'd0,q_r8} << 3) + ({8'd0,q_r8} << 2) + {8'd0,q_r8} )
                     + ( ({8'd0,q_g8} << 7) + ({8'd0,q_g8} << 4) + ({8'd0,q_g8} << 2) + ({8'd0,q_g8} << 1) )
                     + ( ({8'd0,q_b8} << 5) - ({8'd0,q_b8} << 2) + {8'd0,q_b8} );
    wire [7:0]  q_gray = q_gw[15:8];
    wire [23:0] q_grey = {q_gray, q_gray, q_gray};
    wire [23:0] q_bin = (q_gray >= 8'd128) ? 24'hFFFFFF : 24'h000000;
    wire [23:0] q_inv = ~q_color;
    wire        quad_de, quad_hs, quad_vs;
    wire [23:0] quad_rgb;
    wire [1:0]  quad_id;
    quad_mux #(
        .H_ACT (H_NUM),
        .V_ACT (V_NUM)
    ) u_quad (
        .i_de   (rd_de   ),
        .i_hs   (rd_hs   ),
        .i_vs   (rd_vs   ),
        .i_x    (rd_x    ),
        .i_y    (rd_y    ),
        .s0_rgb (q_color ),
        .s1_rgb (q_grey  ),
        .s2_rgb (q_bin   ),
        .s3_rgb (q_inv   ),
        .o_de   (quad_de ),
        .o_hs   (quad_hs ),
        .o_vs   (quad_vs ),
        .o_rgb  (quad_rgb),
        .o_quad (quad_id )
    );
    reg [31:0] dbg_frame_cnt;
    wire       dbg_vs_rise = rd_vs & ~rd_vs_d;
    always @(posedge pix_clk or negedge fb_init_done) begin
        if (!fb_init_done) dbg_frame_cnt <= 32'd0;
        else if (dbg_vs_rise) dbg_frame_cnt <= dbg_frame_cnt + 32'd1;
    end
    reg [31:0] dbg_pix_cnt;
    always @(posedge pix_clk or negedge fb_init_done) begin
        if (!fb_init_done) dbg_pix_cnt <= 32'd0;
        else if (vout_de)  dbg_pix_cnt <= dbg_pix_cnt + 32'd1;
    end
    wire        uart_busy;
    wire        uart_valid;
    wire [7:0]  uart_data;
    uart_tx #(.CLK_HZ(50_000_000), .BAUD(115200)) u_uart_tx (
        .clk     (sys_clk   ),
        .rst_n   (rstn_out  ),
        .i_valid (uart_valid),
        .i_data  (uart_data ),
        .o_tx    (uart_txd  ),
        .o_busy  (uart_busy )
    );
    reg [23:0] cam_pclk_cnt;
    always @(posedge cmos1_pclk) cam_pclk_cnt <= cam_pclk_cnt + 24'd1;
    wire cam_nack_many = (cam_nack_cnt > 16'd100);
    wire [15:0] mosaic_de_cycles_dummy = 16'd0;
    wire [31:0] dbg_cam = {
        16'd0,
        q_srcw1
    };
    dbg_print #(.CLK_HZ(50_000_000), .REFRESH_CYC(12_500_000)) u_dbg_print (
        .clk     (sys_clk   ),
        .rst_n   (rstn_out  ),
        .i_v0    (dbg_frame_cnt              ),
        .i_v1    ({30'd0, lane_type_l}       ),
        .i_v2    ({30'd0, lane_type_r}       ),
        .i_v3    ({30'd0, lane_sel}          ),
        .i_v4    ({31'd0, fsm_motion}        ),
        .i_v5    ({31'd0, fsm_alarm}         ),
        .i_v6    ({16'd0, q_camlines}    ),
        .i_v7    ({20'd0, cam_lines_dbg} ),
        .o_valid (uart_valid),
        .o_data  (uart_data ),
        .i_busy  (uart_busy )
    );
    wire        m_a_lock, m_b_lock, m_c_lock, m_d_lock;
    wire        m_a_size, m_b_size, m_c_size, m_d_size;
    wire [11:0] m_a_rx, m_a_ry, m_b_rx, m_b_ry;
    wire [11:0] m_c_rx, m_c_ry, m_d_rx, m_d_ry;
    video_monitor #(.H_ACT_EXP(12'd1920), .V_ACT_EXP(12'd1080),
                    .LOCK_FRAMES(4'd4), .VS_TIMEOUT(32'd4_000_000))
    u_mon_a (.clk(pixclk_in), .rst_n(rstn_out), .en(1'b1),
             .vs(vs_in), .de(de_in),
             .o_locked(m_a_lock), .o_res_x(m_a_rx), .o_res_y(m_a_ry),
             .o_size_ok(m_a_size), .o_frame_cnt(), .o_vs_rise());
    video_monitor #(.H_ACT_EXP(12'd1920), .V_ACT_EXP(12'd1080),
                    .LOCK_FRAMES(4'd4), .VS_TIMEOUT(32'd4_000_000))
    u_mon_b (.clk(pixclk_in), .rst_n(rstn_out), .en(1'b1),
             .vs(vs_in), .de(~de_in),
             .o_locked(m_b_lock), .o_res_x(m_b_rx), .o_res_y(m_b_ry),
             .o_size_ok(m_b_size), .o_frame_cnt(), .o_vs_rise());
    video_monitor #(.H_ACT_EXP(12'd1920), .V_ACT_EXP(12'd1080),
                    .LOCK_FRAMES(4'd4), .VS_TIMEOUT(32'd4_000_000))
    u_mon_c (.clk(pixclk_in), .rst_n(rstn_out), .en(1'b1),
             .vs(~vs_in), .de(de_in),
             .o_locked(m_c_lock), .o_res_x(m_c_rx), .o_res_y(m_c_ry),
             .o_size_ok(m_c_size), .o_frame_cnt(), .o_vs_rise());
    video_monitor #(.H_ACT_EXP(12'd1920), .V_ACT_EXP(12'd1080),
                    .LOCK_FRAMES(4'd4), .VS_TIMEOUT(32'd4_000_000))
    u_mon_d (.clk(pixclk_in), .rst_n(rstn_out), .en(1'b1),
             .vs(~vs_in), .de(~de_in),
             .o_locked(m_d_lock), .o_res_x(m_d_rx), .o_res_y(m_d_ry),
             .o_size_ok(m_d_size), .o_frame_cnt(), .o_vs_rise());
    wire        hdmi_locked_raw = m_a_lock | m_b_lock | m_c_lock | m_d_lock;
    wire        hdmi_size_raw   = m_a_size | m_b_size | m_c_size | m_d_size;
    wire [11:0] hdmi_rx = m_a_lock ? m_a_rx : m_b_lock ? m_b_rx :
                          m_c_lock ? m_c_rx : m_d_rx;
    wire [11:0] hdmi_ry = m_a_lock ? m_a_ry : m_b_lock ? m_b_ry :
                          m_c_lock ? m_c_ry : m_d_ry;
    reg [2:0] in_lk_s, in_sz_s, in_de_s, in_vs_s, in_hs_s;
    always @(posedge pixclk_in or negedge rstn_out) begin
        if (!rstn_out) begin
            in_lk_s <= 3'd0; in_sz_s <= 3'd0;
            in_de_s <= 3'd0; in_vs_s <= 3'd0; in_hs_s <= 3'd0;
        end else begin
            in_lk_s <= {in_lk_s[1:0], hdmi_locked_raw};
            in_sz_s <= {in_sz_s[1:0], hdmi_size_raw  };
            in_de_s <= {in_de_s[1:0], de_in          };
            in_vs_s <= {in_vs_s[1:0], vs_in          };
            in_hs_s <= {in_hs_s[1:0], hs_in          };
        end
    end
    reg [31:0] in_pc_cnt;
    always @(posedge pixclk_in or negedge rstn_out) begin
        if (!rstn_out) in_pc_cnt <= 32'd0;
        else           in_pc_cnt <= in_pc_cnt + 32'd1;
    end
    reg [25:0] dbg_win;
    reg [19:0] in_hs_num, in_vs_num;
    reg        in_de_tgl;
    reg [3:0]  pc_low_d;
    reg        pc_act;
    reg [19:0] live_hs, live_vs;
    reg        live_detgl, live_pcact;
    reg [19:0] hold_hs, hold_vs;
    reg        hold_detgl, hold_pcact;
    reg        hold_lk, hold_sz, hold_init;
    reg [11:0] hold_rx, hold_ry;
    reg  [1:0] auto_cnt;
    reg        k1_s0, k1_s1;
    wire       snap_key  = k1_s0 & ~k1_s1;
    wire       snap_auto = (auto_cnt == 2'd2) && (dbg_win == 26'd49_999_999);
    wire       do_snap   = snap_key | snap_auto;
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            dbg_win    <= 26'd0;
            in_hs_num  <= 20'd0;
            in_vs_num  <= 20'd0;
            in_de_tgl  <= 1'b0;
            pc_low_d   <= 4'd0;
            pc_act     <= 1'b0;
            live_hs    <= 20'd0;
            live_vs    <= 20'd0;
            live_detgl <= 1'b0;
            live_pcact <= 1'b0;
            hold_hs    <= 20'd0;
            hold_vs    <= 20'd0;
            hold_detgl <= 1'b0;
            hold_pcact <= 1'b0;
            hold_lk    <= 1'b0;
            hold_sz    <= 1'b0;
            hold_init  <= 1'b0;
            hold_rx    <= 12'd0;
            hold_ry    <= 12'd0;
            auto_cnt   <= 2'd0;
            k1_s0      <= 1'b0;
            k1_s1      <= 1'b0;
        end else begin
            k1_s0 <= key_pressed;
            k1_s1 <= k1_s0;
            if (in_hs_s[2:1] == 2'b01) in_hs_num <= in_hs_num + 20'd1;
            if (in_vs_s[2:1] == 2'b01) in_vs_num <= in_vs_num + 20'd1;
            if (in_de_s[2] != in_de_s[1]) in_de_tgl <= 1'b1;
            if (in_pc_cnt[3:0] != pc_low_d) begin
                pc_low_d <= in_pc_cnt[3:0];
                pc_act   <= 1'b1;
            end
            if (dbg_win == 26'd49_999_999) begin
                dbg_win    <= 26'd0;
                live_hs    <= in_hs_num;
                live_vs    <= in_vs_num;
                live_detgl <= in_de_tgl;
                live_pcact <= pc_act;
                in_hs_num  <= 20'd0;
                in_vs_num  <= 20'd0;
                in_de_tgl  <= 1'b0;
                pc_act     <= 1'b0;
                if (auto_cnt != 2'd3) auto_cnt <= auto_cnt + 2'd1;
            end else begin
                dbg_win <= dbg_win + 26'd1;
            end
            if (do_snap) begin
                hold_hs    <= live_hs;
                hold_vs    <= live_vs;
                hold_detgl <= live_detgl;
                hold_pcact <= live_pcact;
                hold_lk    <= in_lk_s[2];
                hold_sz    <= in_sz_s[2];
                hold_init  <= init_over_rx;
                hold_rx    <= hdmi_rx;
                hold_ry    <= hdmi_ry;
            end
        end
    end
    reg [2:0] i3_pc_s, i3_hs_s, i3_vs_s, i3_de_s;
    reg [3:0] i3_pc_low_d;
    reg       i3_pc_act, i3_de_tgl;
    reg [19:0] i3_hs_num, i3_vs_num;
    reg [19:0] live3_hs, live3_vs;
    reg        live3_pcact, live3_detgl;
    reg [19:0] hold3_hs, hold3_vs;
    reg        hold3_pcact, hold3_detgl;
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            i3_pc_s <= 3'd0; i3_hs_s <= 3'd0; i3_vs_s <= 3'd0; i3_de_s <= 3'd0;
            i3_pc_low_d <= 4'd0; i3_pc_act <= 1'b0; i3_de_tgl <= 1'b0;
            i3_hs_num <= 20'd0; i3_vs_num <= 20'd0;
            live3_hs <= 20'd0; live3_vs <= 20'd0;
            live3_pcact <= 1'b0; live3_detgl <= 1'b0;
            hold3_hs <= 20'd0; hold3_vs <= 20'd0;
            hold3_pcact <= 1'b0; hold3_detgl <= 1'b0;
        end else begin
            i3_pc_s <= {i3_pc_s[1:0], pixclk_in3};
            i3_hs_s <= {i3_hs_s[1:0], hs_in3    };
            i3_vs_s <= {i3_vs_s[1:0], vs_in3    };
            i3_de_s <= {i3_de_s[1:0], de_in3    };
            if (i3_hs_s[2:1] == 2'b01) i3_hs_num <= i3_hs_num + 20'd1;
            if (i3_vs_s[2:1] == 2'b01) i3_vs_num <= i3_vs_num + 20'd1;
            if (i3_de_s[2]   != i3_de_s[1]) i3_de_tgl <= 1'b1;
            if (i3_pc_s[1] != i3_pc_low_d[3]) begin
                i3_pc_low_d[3] <= i3_pc_s[1];
                i3_pc_act      <= 1'b1;
            end
            if (dbg_win == 26'd49_999_999) begin
                live3_hs    <= i3_hs_num;
                live3_vs    <= i3_vs_num;
                live3_pcact <= i3_pc_act;
                live3_detgl <= i3_de_tgl;
                i3_hs_num   <= 20'd0;
                i3_vs_num   <= 20'd0;
                i3_pc_act   <= 1'b0;
                i3_de_tgl   <= 1'b0;
            end
            if (do_snap) begin
                hold3_hs    <= live3_hs;
                hold3_vs    <= live3_vs;
                hold3_pcact <= live3_pcact;
                hold3_detgl <= live3_detgl;
            end
        end
    end
    wire [7:0] dbg_hs3_byte = hold3_hs[19:12];
    wire [7:0] dbg_status = {
        m_d_size, m_c_size, m_b_size, m_a_size,
        m_d_lock, m_c_lock, m_b_lock, m_a_lock
    };
    wire [7:0] dbg_hs_byte = hold_rx[11:4];
    wire [7:0] dbg_vs_byte = hold_ry[11:4];
    wire [2:0]  dbg_col = rd_x[11:8];
    wire [1:0]  dbg_row = (rd_y <  12'd360) ? 2'd0 :
                          (rd_y <  12'd720) ? 2'd1 : 2'd2;
    reg         dbg_bit;
    always @(*) begin
        case (dbg_row)
            2'd0:   dbg_bit = dbg_status [dbg_col];
            2'd1:   dbg_bit = dbg_hs_byte[dbg_col];
            default: dbg_bit = dbg_vs_byte[dbg_col];
        endcase
    end
    wire dbg_sep = (rd_x[7:0] == 8'd0);
    wire dbg_hor = (rd_y == 12'd360) || (rd_y == 12'd720);
    wire [23:0] dbg_rgb = (dbg_sep || dbg_hor) ? 24'h000000
                                               : (dbg_bit ? 24'h00FF00 : 24'hFF0000);
    wire        sel_gray = (disp_mode[1:0] == 2'd1);
    wire [23:0] gray_rgb = use_road ? q_grey : {3{gs_gray}};
    wire        gray_de  = use_road ? quad_de : gs_de;
    wire        gray_hs  = use_road ? quad_hs : gs_hs;
    wire        gray_vs  = use_road ? quad_vs : gs_vs;
    wire        mix_de  = sel_gray ? gray_de : quad_de;
    wire        mix_hs  = sel_gray ? gray_hs : quad_hs;
    wire        mix_vs  = sel_gray ? gray_vs : quad_vs;
    wire [23:0] mix_rgb = (disp_mode[1:0] == 2'd3) ? dbg_rgb  :
                          sel_gray                  ? gray_rgb :
                          (disp_mode[1:0] == 2'd2) ? quad_rgb :
                                                      q_color;
    wire [7:0] mix_r = mix_rgb[23:16];
    wire [7:0] mix_g = mix_rgb[15:8];
    wire [7:0] mix_b = mix_rgb[7:0];
    wire [1:0] arrow_sel = fsm_arrow;
    reg [23:0] box0_color;
    always @(*) begin
        case (lane_type_r)
            2'd0:    box0_color = 24'hFF2020;
            2'd1:    box0_color = 24'h00FF40;
            default: box0_color = 24'hFFFF00;
        endcase
    end
    reg [25:0] alm_cnt;
    always @(posedge pix_clk or negedge fb_init_done) begin
        if (!fb_init_done) alm_cnt <= 26'd0;
        else               alm_cnt <= alm_cnt + 26'd1;
    end
    wire alm_blink = alm_cnt[25];
    wire [1:0] line_type_fsm = (lane_type_r == 2'd1) ? 2'd0 :
                               (lane_type_r == 2'd2) ? 2'd1 :
                                                       2'd2;
    wire        osd_de, osd_hs, osd_vs;
    wire [23:0] osd_rgb;
    osd_overlay #(
        .MIX_LATENCY (3),
        .BOX1_X0     (12'd667 ),
        .BOX1_Y0     (12'd200 ),
        .BOX1_X1     (12'd1133),
        .BOX1_Y1     (12'd533 ),
        .BOX1_COLOR  (24'h00FF00),
        .BOX1_ALPHA  (5'd6),
        .BOX_TH      (4'd3)
    ) u_osd (
        .clk          (pix_clk           ),
        .rst_n        (fb_init_done      ),
        .i_de         (mix_de            ),
        .i_hs         (mix_hs            ),
        .i_vs         (mix_vs            ),
        .i_rgb        ({mix_r, mix_g, mix_b}),
        .i_box0_en    (1'b0             ),
        .i_box0_x0    (12'd40            ),
        .i_box0_y0    (12'd40            ),
        .i_box0_x1    (12'd200           ),
        .i_box0_y1    (12'd140           ),
        .i_box0_color (box0_color        ),
        .i_box0_alpha (5'd8              ),
        .i_box0_fill  (1'b1              ),
        .i_arrow_en   (fsm_alarm ? alm_blink : fsm_motion),
        .i_arrow_sel  (arrow_sel         ),
        .i_arrow_x    (12'd150           ),
        .i_arrow_y    (12'd530           ),
        .i_arrow_color(24'hFFFFFF        ),
        .i_arrow_alpha(5'd16             ),
        .o_de         (osd_de            ),
        .o_hs         (osd_hs            ),
        .o_vs         (osd_vs            ),
        .o_rgb        (osd_rgb           )
    );
    assign pixclk_out = pix_clk;
    assign vs_out     = osd_vs;
    assign hs_out     = osd_hs;
    assign de_out     = osd_de;
    assign r_out = osd_rgb[23:16];
    assign g_out = osd_rgb[15:8];
    assign b_out = osd_rgb[7:0];
    reg [26:0] hb_cnt;
    reg        heart_beat;
    always @(posedge core_clk) begin
        if (!ddr_init_done)
            hb_cnt <= 27'd0;
        else if (hb_cnt >= TH_1S[26:0])
            hb_cnt <= 27'd0;
        else
            hb_cnt <= hb_cnt + 27'd1;
    end
    always @(posedge core_clk) begin
        if (!ddr_init_done)
            heart_beat <= 1'b1;
        else if (hb_cnt >= TH_1S[26:0])
            heart_beat <= ~heart_beat;
    end
    reg [24:0] gs_cnt;
    always @(posedge pix_clk) begin
        if (!fb_init_done)
            gs_cnt <= 25'd0;
        else if (gs_de)
            gs_cnt <= gs_cnt + 25'd1;
    end
    wire gs_act = gs_cnt[24];
    assign led[0] = ddr_init_done;
    assign led[1] = fb_init_done;
    reg vs_d0, vs_d1;
    always @(posedge cmos1_pclk) begin
        vs_d0 <= cmos1_vsync;
        vs_d1 <= vs_d0;
    end
    reg cam_vs_seen;
    always @(posedge cmos1_pclk) begin
        if (vs_d0 ^ vs_d1) cam_vs_seen <= 1'b1;
    end
    reg cam_href_seen;
    always @(posedge cmos1_pclk) begin
        if (cam_href_d0) cam_href_seen <= 1'b1;
    end
    assign led[2] = cam_init_done;
    reg [7:0] cam_data_or;
    always @(posedge cmos1_pclk) begin
        if (cam_href_d0) cam_data_or <= cam_data_or | cam_d_d0;
    end
    wire lane_any = (lane_type_l != 2'd0) || (lane_type_r != 2'd0);
    assign led[3] = fsm_alarm ? alm_blink : lane_any;
    wire       _unused_vs_seen  = cam_vs_seen;
    wire [31:0] dbg_ok_frames   = ok_frames;
    wire [31:0] dbg_bad_frames  = bad_frames;
    wire [31:0] dbg_err_pixels  = err_pixels;
    wire        dbg_rd_active   = rd_active_tgl;
    wire _unused_ok = &{1'b0, dbg_ok_frames[0], dbg_bad_frames[0],
                        dbg_err_pixels[0], dbg_rd_active,
                        init_over_tx, init_over_rx, wr_de_re, wr_hs,
                        rd_de, rd_hs, pll_lock_ddr, locked,
                        verify_ok, gs_act, gs_hs, gs_vs,
                        fsm_lane, fsm_tl, fsm_line, fsm_motion, fsm_led_lane,
                        fsm_alarm, lane_type_l, lane_pos_l, lane_pos_r,
                        bin_level, sc_de, sc_hs, sc_vs, sc_rgb888[0]};
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
    localparam [11:0] V_ACT = 12'd1080;
    localparam [19:0] GAP_TH = 20'd4400;
    reg  [19:0] gap_cnt;
    reg         de_d;
    reg  [5:0]  vs_pulse;
    wire        de_rise = de & ~de_d;
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
`timescale 1ns / 1ps
module power_on_delay(
    input  clk_50M        ,
    input  reset_n        ,
    output camera1_rstn   ,
    output camera2_rstn   ,
    output camera_pwnd    ,
    output initial_en
);
reg [18:0]cnt1;
reg [15:0]cnt2;
reg [19:0]cnt3;
reg initial_en;
reg camera_rstn_reg;
reg camera_pwnd_reg;
assign camera1_rstn=camera_rstn_reg;
assign camera2_rstn=camera_rstn_reg;
assign camera_pwnd=camera_pwnd_reg;
always@(posedge clk_50M)begin
  if(reset_n==1'b0) begin
	    cnt1<=0;
		camera_pwnd_reg<=1'b1;
  end
  else if(cnt1<19'h40000) begin
       cnt1<=cnt1+1'b1;
       camera_pwnd_reg<=1'b1;
  end
  else
     camera_pwnd_reg<=1'b0;
end
always@(posedge clk_50M)begin
  if(camera_pwnd_reg==1)  begin
	 cnt2<=0;
     camera_rstn_reg<=1'b0;
  end
  else if(cnt2<16'hffff) begin
       cnt2<=cnt2+1'b1;
       camera_rstn_reg<=1'b0;
  end
  else
     camera_rstn_reg<=1'b1;
end
always@(posedge clk_50M)begin
  if(camera_rstn_reg==0) begin
         cnt3<=0;
         initial_en<=1'b0;
  end
  else if(cnt3<20'hfffff) begin
        cnt3<=cnt3+1'b1;
        initial_en<=1'b0;
  end
  else
       initial_en<=1'b1;
end
endmodule
`timescale 1ns / 1ps
`timescale 1ns/1ns
module cmos_8_16bit(
	input 				   pclk 		,
	input 				   rst_n		,
	input				   de_i	        ,
	input	[7:0]	       pdata_i	    ,
    input                  vs_i         ,
    output                 pixel_clk    ,
 	output	reg			   de_o         ,
	output  reg [15:0]	   pdata_o
);
reg			de_out1          ;
reg [15:0]	pdata_out1       ;
reg			de_out2          ;
reg [15:0]	pdata_out2       ;
reg [1:0]   cnt             ;
wire        pclk_IOCLKBUF   ;
reg         vs_i_reg        ;
reg         enble           ;
reg [7:0] pdata_i_reg;
reg de_i_r,de_i_r1;
reg			de_out3          ;
reg [15:0]	pdata_out3       ;
always @(posedge pclk)begin
       vs_i_reg <= vs_i ;
end
    always@(posedge pclk)
        begin
            if(!rst_n)
                enble <= 1'b0;
            else if(!vs_i_reg&&vs_i)
                enble <= 1'b1;
            else
                enble <= enble;
        end
    reg pclk_half;
    always @(negedge pclk)
    begin
        if (!rst_n) pclk_half <= 1'b0;
        else        pclk_half <= ~pclk_half;
    end
    assign pixel_clk = pclk_half;
    always@(posedge pclk)
        begin
            if(!rst_n)
                cnt <= 2'b0;
            else if(de_i == 1'b1 && cnt == 2'd1)
                cnt <= 2'b0;
            else if(de_i == 1'b1)
                cnt <= cnt + 1'b1;
        end
    always@(posedge pclk)
        begin
            if(!rst_n)
                pdata_i_reg <= 8'b0;
            else if(de_i == 1'b1)
                pdata_i_reg <= pdata_i;
        end
    always@(posedge pclk)
        begin
            if(!rst_n)
                pdata_out1 <= 16'b0;
            else if(de_i == 1'b1 && cnt == 2'd1)
                pdata_out1 <= {pdata_i_reg,pdata_i};
        end
   always@(posedge pclk)begin
        de_i_r <= de_i;
        de_i_r1 <= de_i_r;
   end
    always@(posedge pclk)
        begin
            if(!rst_n)
                de_out1 <= 1'b0;
            else if(!de_i_r1 && de_i_r )
                de_out1 <= 1'b1;
            else if(de_i_r1 && !de_i_r )
                de_out1 <= 1'b0;
            else
                de_out1 <= de_out1;
        end
    always@(posedge pixel_clk)begin
        de_out2<=de_out1;
        de_out3<=de_out2;
        de_o   <=de_out3;
    end
    always@(posedge pixel_clk)begin
        pdata_out2<=pdata_out1;
        pdata_out3<=pdata_out2;
        pdata_o   <=pdata_out3;
    end
endmodule
`timescale 1ns / 1ps
 module reg_config(
		  input clk_25M,
		  input camera_rstn,
		  input initial_en,
		  output reg_conf_done,
		  output i2c_sclk,
		  inout i2c_sdat,
		  output reg clock_20k,
		  output reg [8:0]reg_index,
		  output reg [15:0] o_nack_cnt
	  );
     reg [15:0]clock_20k_cnt;
     reg [1:0]config_step;
     reg [31:0]i2c_data;
     reg [23:0]reg_data;
     reg start;
	 reg reg_conf_done_reg;
     i2c_com u1(.clock_i2c(clock_20k),
               .camera_rstn(camera_rstn),
               .ack(ack),
               .i2c_data(i2c_data),
               .start(start),
               .tr_end(tr_end),
               .i2c_sclk(i2c_sclk),
               .i2c_sdat(i2c_sdat));
assign reg_conf_done=reg_conf_done_reg;
	 always @(posedge clk_25M or negedge camera_rstn) begin
	     if (!camera_rstn)      o_nack_cnt <= 16'd0;
	     else if (tr_end && !ack) o_nack_cnt <= o_nack_cnt + 16'd1;
	 end
always@(posedge clk_25M)
begin
   if(!camera_rstn) begin
        clock_20k<=0;
        clock_20k_cnt<=0;
   end
   else if(clock_20k_cnt<1249)
      clock_20k_cnt<=clock_20k_cnt+1'b1;
   else begin
         clock_20k<=!clock_20k;
         clock_20k_cnt<=0;
   end
end
always@(posedge clock_20k)
begin
   if(!camera_rstn) begin
       config_step<=0;
       start<=0;
       reg_index<=0;
		 reg_conf_done_reg<=0;
   end
   else begin
      if(reg_conf_done_reg==1'b0) begin
			  if(reg_index<357) begin
					 case(config_step)
					 0:begin
						i2c_data<={8'h78,reg_data};
						start<=1;
						config_step<=1;
					 end
					 1:begin
						if(tr_end) begin
							 start<=0;
							 config_step<=2;
						end
					 end
					 2:begin
						  reg_index<=reg_index+1'b1;
						  config_step<=0;
					 end
					 endcase
				end
			 else
				reg_conf_done_reg<=1'b1;
      end
   end
 end
always@(reg_index)
 begin
    case(reg_index)
	 0    :reg_data   <=24'h310311 ;
	 1    :reg_data   <=24'h300882 ;
	 102  :reg_data   <=24'h300842 ;
	 103  :reg_data   <=24'h310303 ;
	 104  :reg_data   <=24'h3017ff ;
	 105  :reg_data   <=24'h3018ff ;
	 106  :reg_data   <=24'h30341A ;
	 107  :reg_data   <=24'h303713 ;
	 108  :reg_data   <=24'h310801 ;
	 109  :reg_data   <=24'h363036 ;
	 110  :reg_data  <=24'h36310e ;
	 111  :reg_data  <=24'h3632e2 ;
	 112  :reg_data  <=24'h363312 ;
	 113  :reg_data  <=24'h3621e0 ;
	 114  :reg_data  <=24'h3704a0 ;
	 115  :reg_data  <=24'h37035a ;
	 116  :reg_data  <=24'h371578 ;
	 117  :reg_data  <=24'h371701 ;
	 118  :reg_data  <=24'h370b60 ;
	 119  :reg_data  <=24'h37051a ;
	 120  :reg_data  <=24'h390502 ;
	 121  :reg_data  <=24'h390610 ;
	 122  :reg_data  <=24'h39010a ;
	 123  :reg_data  <=24'h373112 ;
	 124  :reg_data  <=24'h360008 ;
	 125  :reg_data  <=24'h360133 ;
	 126  :reg_data  <=24'h302d60 ;
	 127  :reg_data  <=24'h362052 ;
	 128  :reg_data  <=24'h371b20 ;
	 129  :reg_data  <=24'h471c50 ;
	 130  :reg_data  <=24'h3a1343 ;
	 131  :reg_data  <=24'h3a1800 ;
	 132  :reg_data  <=24'h3a19f8 ;
	 133  :reg_data  <=24'h363513 ;
	 134  :reg_data  <=24'h363603 ;
	 135  :reg_data  <=24'h363440 ;
	 136  :reg_data  <=24'h362201 ;
	 137  :reg_data  <=24'h3c0134 ;
	 138  :reg_data  <=24'h3c0428 ;
	 139  :reg_data  <=24'h3c0598 ;
	 140  :reg_data  <=24'h3c0600 ;
     141  :reg_data  <=24'h3c0708 ;
	 142  :reg_data  <=24'h3c0800 ;
	 143  :reg_data  <=24'h3c091c ;
	 144  :reg_data  <=24'h3c0a9c ;
	 145  :reg_data  <=24'h3c0b40 ;
	 146  :reg_data  <=24'h381000 ;
	 147  :reg_data  <=24'h381110 ;
	 148  :reg_data  <=24'h381200 ;
	 149  :reg_data  <=24'h370864 ;
	 150  :reg_data  <=24'h400102 ;
	 151  :reg_data  <=24'h40051a ;
	 152  :reg_data  <=24'h300000 ;
	 153  :reg_data  <=24'h3004ff ;
	 154  :reg_data  <=24'h300e58 ;
	 155  :reg_data  <=24'h302e00 ;
	 156  :reg_data  <=24'h430060 ;
	 157  :reg_data  <=24'h501f01 ;
	 158  :reg_data  <=24'h440e00 ;
	 159  :reg_data  <=24'h5000a7 ;
	 160  :reg_data  <=24'h3a0f30 ;
	 161  :reg_data  <=24'h3a1028 ;
	 162  :reg_data  <=24'h3a1b30 ;
	 163  :reg_data  <=24'h3a1e26 ;
	 164  :reg_data  <=24'h3a1160 ;
	 165  :reg_data  <=24'h3a1f14 ;
	 166  :reg_data  <=24'h580023 ;
	 167  :reg_data  <=24'h580114 ;
	 168  :reg_data  <=24'h58020f ;
	 169  :reg_data  <=24'h58030f ;
	 170  :reg_data  <=24'h580412 ;
	 171  :reg_data  <=24'h580526 ;
	 172  :reg_data  <=24'h58060c ;
	 173  :reg_data  <=24'h580708 ;
	 174  :reg_data  <=24'h580805 ;
	 175  :reg_data  <=24'h580905 ;
	 176  :reg_data  <=24'h580a08 ;
	 177  :reg_data  <=24'h580b0d ;
	 178  :reg_data  <=24'h580c08 ;
	 179  :reg_data  <=24'h580d03 ;
	 180  :reg_data  <=24'h580e00 ;
	 181  :reg_data  <=24'h580f00 ;
	 182  :reg_data  <=24'h581003 ;
	 183  :reg_data  <=24'h581109 ;
	 184  :reg_data  <=24'h581207 ;
	 185  :reg_data  <=24'h581303 ;
	 186  :reg_data  <=24'h581400 ;
	 187  :reg_data  <=24'h581501 ;
	 188  :reg_data  <=24'h581603 ;
	 189  :reg_data  <=24'h581708 ;
	 190  :reg_data  <=24'h58180d ;
	 191  :reg_data  <=24'h581908 ;
	 192  :reg_data  <=24'h581a05 ;
	 193  :reg_data  <=24'h581b06 ;
	 194  :reg_data  <=24'h581c08 ;
	 195  :reg_data  <=24'h581d0e ;
	 196  :reg_data  <=24'h581e29 ;
	 197  :reg_data  <=24'h581f17 ;
	 198  :reg_data  <=24'h582011 ;
	 199  :reg_data  <=24'h582111 ;
	 200  :reg_data <=24'h582215 ;
	 201  :reg_data <=24'h582328 ;
	 202  :reg_data <=24'h582446 ;
	 203  :reg_data <=24'h582526 ;
	 204  :reg_data <=24'h582608 ;
	 205  :reg_data <=24'h582726 ;
	 206  :reg_data <=24'h582864 ;
	 207  :reg_data <=24'h582926 ;
	 208  :reg_data <=24'h582a24 ;
	 209  :reg_data <=24'h582b22 ;
	 210  :reg_data <=24'h582c24 ;
	 211  :reg_data <=24'h582d24 ;
	 212  :reg_data <=24'h582e06 ;
	 213  :reg_data <=24'h582f22 ;
	 214  :reg_data <=24'h583040 ;
	 215  :reg_data <=24'h583142 ;
	 216  :reg_data <=24'h583224 ;
	 217  :reg_data <=24'h583326 ;
	 218  :reg_data <=24'h583424 ;
	 219  :reg_data <=24'h583522 ;
	 220  :reg_data <=24'h583622 ;
	 221  :reg_data <=24'h583726 ;
	 222  :reg_data <=24'h583844 ;
	 223  :reg_data <=24'h583924 ;
	 224  :reg_data <=24'h583a26 ;
	 225  :reg_data <=24'h583b28 ;
	 226  :reg_data <=24'h583c42 ;
	 227  :reg_data <=24'h583dce ;
	 228  :reg_data <=24'h5180ff ;
	 229  :reg_data <=24'h5181f2 ;
	 230  :reg_data <=24'h518200 ;
	 231  :reg_data <=24'h518314 ;
	 232  :reg_data <=24'h518425 ;
	 233  :reg_data <=24'h518524 ;
	 234  :reg_data <=24'h518609 ;
	 235  :reg_data <=24'h518709 ;
	 236  :reg_data <=24'h518809 ;
	 237  :reg_data <=24'h518975 ;
	 238  :reg_data <=24'h518a54 ;
	 239  :reg_data <=24'h518be0 ;
	 240  :reg_data <=24'h518cb2 ;
	 241  :reg_data <=24'h518d42 ;
	 242  :reg_data <=24'h518e3d ;
	 243  :reg_data <=24'h518f56 ;
	 244  :reg_data <=24'h519046 ;
	 245  :reg_data <=24'h5191f8 ;
	 246  :reg_data <=24'h519204 ;
	 247  :reg_data <=24'h519370 ;
	 248  :reg_data <=24'h5194f0 ;
	 249  :reg_data <=24'h5195f0 ;
	 250  :reg_data <=24'h519603 ;
	 251  :reg_data <=24'h519701 ;
	 252  :reg_data <=24'h519804 ;
	 253  :reg_data <=24'h519912 ;
	 254  :reg_data <=24'h519a04 ;
	 255  :reg_data <=24'h519b00 ;
	 256  :reg_data <=24'h519c06 ;
	 257  :reg_data <=24'h519d82 ;
	 258  :reg_data <=24'h519e38 ;
	 259  :reg_data <=24'h548001 ;
	 260  :reg_data <=24'h548108 ;
	 261  :reg_data <=24'h548214 ;
	 262  :reg_data <=24'h548328 ;
	 263  :reg_data <=24'h548451 ;
	 264  :reg_data <=24'h548565 ;
	 265  :reg_data <=24'h548671 ;
	 266  :reg_data <=24'h54877d ;
	 267  :reg_data <=24'h548887 ;
	 268  :reg_data <=24'h548991 ;
	 269  :reg_data <=24'h548a9a ;
	 270  :reg_data <=24'h548baa ;
	 271  :reg_data <=24'h548cb8 ;
	 272  :reg_data <=24'h548dcd ;
	 273  :reg_data <=24'h548edd ;
	 274  :reg_data <=24'h548fea ;
	 275  :reg_data <=24'h54901d ;
	 276  :reg_data <=24'h53811e ;
	 277  :reg_data <=24'h53825b ;
	 278  :reg_data <=24'h538308 ;
	 279  :reg_data <=24'h53840a ;
	 280  :reg_data <=24'h53857e ;
	 281  :reg_data <=24'h538688 ;
	 282  :reg_data <=24'h53877c ;
	 283  :reg_data <=24'h53886c ;
	 284  :reg_data <=24'h538910 ;
	 285  :reg_data <=24'h538a01 ;
	 286  :reg_data <=24'h538b98 ;
	 287  :reg_data <=24'h558006 ;
	 288  :reg_data <=24'h558340 ;
	 289  :reg_data <=24'h558410 ;
	 290  :reg_data <=24'h558910 ;
	 291  :reg_data <=24'h558a00 ;
	 292  :reg_data <=24'h558bf8 ;
	 293  :reg_data <=24'h501d40 ;
	 294  :reg_data <=24'h530008 ;
	 295  :reg_data <=24'h530130 ;
	 296  :reg_data <=24'h530210 ;
	 297  :reg_data <=24'h530300 ;
	 298  :reg_data <=24'h530408 ;
	 299  :reg_data <=24'h530530 ;
	 300  :reg_data <=24'h530608 ;
	 301  :reg_data <=24'h530716 ;
	 302  :reg_data <=24'h530908 ;
	 303  :reg_data <=24'h530a30 ;
	 304  :reg_data <=24'h530b04 ;
	 305  :reg_data <=24'h530c06 ;
	 306  :reg_data <=24'h502500 ;
	 307  :reg_data <=24'h300802 ;
	 308  :reg_data <=24'h303521 ;
	 309  :reg_data <=24'h303669 ;
     310  :reg_data <=24'h3c0708 ;
	 311  :reg_data <=24'h382047 ;
	 312  :reg_data <=24'h382100 ;
	 313  :reg_data <=24'h381431 ;
	 314  :reg_data <=24'h381531 ;
	 315  :reg_data <=24'h380000 ;
	 316  :reg_data <=24'h380100 ;
	 317  :reg_data <=24'h380200 ;
	 318  :reg_data <=24'h3803fa ;
	 319  :reg_data <=24'h38040a ;
	 320  :reg_data <=24'h38053f ;
	 321  :reg_data <=24'h380606 ;
	 322  :reg_data <=24'h3807a9 ;
	 323  :reg_data <=24'h380805 ;
	 324  :reg_data <=24'h380900 ;
	 325  :reg_data <=24'h380a02 ;
	 326  :reg_data <=24'h380bd0 ;
	 327  :reg_data <=24'h380c07 ;
	 328  :reg_data <=24'h380d64 ;
	 329  :reg_data <=24'h380e02 ;
	 330  :reg_data <=24'h380fe4 ;
	 331  :reg_data <=24'h381304 ;
	 332  :reg_data <=24'h361800 ;
	 333  :reg_data <=24'h361229 ;
	 334  :reg_data <=24'h370952 ;
	 335  :reg_data <=24'h370c03 ;
	 336  :reg_data <=24'h3a0202 ;
	 337  :reg_data <=24'h3a03e0 ;
	 338  :reg_data <=24'h3a0800 ;
	 339  :reg_data <=24'h3a096f ;
	 340  :reg_data <=24'h3a0a00 ;
	 341  :reg_data <=24'h3a0b5c ;
	 342  :reg_data <=24'h3a0e06 ;
	 343  :reg_data <=24'h3a0d08 ;
	 344  :reg_data <=24'h3a1402 ;
	 345  :reg_data <=24'h3a15e0 ;
	 346  :reg_data <=24'h400402 ;
	 347  :reg_data <=24'h30021c ;
	 348  :reg_data <=24'h3006c3 ;
	 349  :reg_data <=24'h471303 ;
	 350  :reg_data <=24'h440704 ;
	 351  :reg_data <=24'h460b37 ;
     352  :reg_data <=24'h460c20 ;
	 353  :reg_data <=24'h483716 ;
	 354  :reg_data <=24'h382404 ;
	 355  :reg_data <=24'h500183 ;
	 356  :reg_data <=24'h350300 ;
	 default:reg_data<=24'hffffff;
    endcase
end
endmodule
module i2c_com(clock_i2c        ,
               camera_rstn      ,
               ack              ,
               i2c_data         ,
               start            ,
               tr_end           ,
               i2c_sclk         ,
               i2c_sdat
);
    input [31:0]i2c_data;
    input camera_rstn;
    input clock_i2c;
    output ack;
    input start;
    output tr_end;
    output i2c_sclk;
    inout i2c_sdat;
    reg [5:0] cyc_count;
    reg reg_sdat;
    reg sclk;
    reg ack1,ack2,ack3;
    reg tr_end;
    wire i2c_sclk;
    wire i2c_sdat;
    wire ack;
    assign ack=ack1|ack2|ack3;
    assign i2c_sclk=sclk|(((cyc_count>=4)&(cyc_count<=39))?~clock_i2c:0);
    assign i2c_sdat=reg_sdat?1'bz:0;
    always@(posedge clock_i2c)
    begin
       if(!camera_rstn)
         cyc_count<=6'b111111;
       else
		   begin
           if(start==0)
             cyc_count<=0;
           else if(cyc_count<6'b111111)
             cyc_count<=cyc_count+1;
         end
    end
    always@(posedge clock_i2c)
    begin
       if(!camera_rstn)
       begin
          tr_end<=0;
          ack1<=1;
          ack2<=1;
          ack3<=1;
          sclk<=1;
          reg_sdat<=1;
       end
       else
          case(cyc_count)
          0:begin ack1<=1;ack2<=1;ack3<=1;tr_end<=0;sclk<=1;reg_sdat<=1;end
          1:reg_sdat<=0;
          2:sclk<=0;
          3:reg_sdat<=i2c_data[31];
          4:reg_sdat<=i2c_data[30];
          5:reg_sdat<=i2c_data[29];
          6:reg_sdat<=i2c_data[28];
          7:reg_sdat<=i2c_data[27];
          8:reg_sdat<=i2c_data[26];
          9:reg_sdat<=i2c_data[25];
          10:reg_sdat<=i2c_data[24];
          11:reg_sdat<=1;
          12:begin reg_sdat<=i2c_data[23];ack1<=i2c_sdat;end
          13:reg_sdat<=i2c_data[22];
          14:reg_sdat<=i2c_data[21];
          15:reg_sdat<=i2c_data[20];
          16:reg_sdat<=i2c_data[19];
          17:reg_sdat<=i2c_data[18];
          18:reg_sdat<=i2c_data[17];
          19:reg_sdat<=i2c_data[16];
          20:reg_sdat<=1;
          21:begin reg_sdat<=i2c_data[15];ack1<=i2c_sdat;end
          22:reg_sdat<=i2c_data[14];
          23:reg_sdat<=i2c_data[13];
          24:reg_sdat<=i2c_data[12];
          25:reg_sdat<=i2c_data[11];
          26:reg_sdat<=i2c_data[10];
          27:reg_sdat<=i2c_data[9];
          28:reg_sdat<=i2c_data[8];
          29:reg_sdat<=1;
          30:begin reg_sdat<=i2c_data[7];ack2<=i2c_sdat;end
          31:reg_sdat<=i2c_data[6];
          32:reg_sdat<=i2c_data[5];
          33:reg_sdat<=i2c_data[4];
          34:reg_sdat<=i2c_data[3];
          35:reg_sdat<=i2c_data[2];
          36:reg_sdat<=i2c_data[1];
          37:reg_sdat<=i2c_data[0];
          38:reg_sdat<=1;
          39:begin ack3<=i2c_sdat;sclk<=0;reg_sdat<=0;end
          40:sclk<=1;
          41:begin reg_sdat<=1;tr_end<=1;end
          endcase
end
endmodule
module quad_wr #(
    parameter integer H_LINE   = 1920,
    parameter integer V_HALF   = 540,
    parameter integer H_CELL   = 960,
    parameter integer LB_AW    = 10
)(
    input  wire        hd_clk,
    input  wire        rst_n,
    input  wire        hd_de,
    input  wire [15:0] hd_pix,
    input  wire        hd_vs,
    input  wire        c1_clk,
    input  wire        c1_de,
    input  wire        c1_vs,
    input  wire [15:0] c1_pix,
    input  wire        c2_clk,
    input  wire        c2_de,
    input  wire        c2_vs,
    input  wire [15:0] c2_pix,
    output reg         o_de,
    output reg  [15:0] o_pix,
    output reg         o_fsync,
    output wire [15:0] dbg_srcw1,
    output wire [15:0] dbg_srcw2,
    output wire [15:0] dbg_srch1,
    output wire [15:0] dbg_srch2
);
    reg [11:0] hd_x, hd_y;
    reg        hd_de_d;
    always @(posedge hd_clk or negedge rst_n) begin
        if (!rst_n) begin
            hd_x <= 12'd0; hd_y <= 12'd0; hd_de_d <= 1'b0;
        end else begin
            hd_de_d <= hd_de;
            if (hd_de) begin
                if (hd_x == H_LINE-1) begin
                    hd_x <= 12'd0;
                    hd_y <= (hd_y == 12'd1079) ? 12'd0 : hd_y + 12'd1;
                end else hd_x <= hd_x + 12'd1;
            end
            if (hd_vs) hd_y <= 12'd0;
        end
    end
    reg [15:0] lb1 [0:(2<<LB_AW)-1];
    reg [15:0] lb2 [0:(2<<LB_AW)-1];
    reg [1:0]  c1_cnt;
    reg [3:0]  c1_vcnt;
    reg [LB_AW-1:0] c1_waddr;
    reg        c1_de_d, c1_vs_d;
    reg        c1_bank, c1_rdy_tgl;
    reg [11:0] c1_srcw, c1_flines, c1_srch;
    reg [11:0] c1_pixcnt;
    reg [6:0] c1_low_cnt;
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n)                    c1_low_cnt <= 7'd0;
        else if (c1_de)                c1_low_cnt <= 7'd0;
        else if (c1_low_cnt != 7'd127) c1_low_cnt <= c1_low_cnt + 7'd1;
    end
    wire c1_line_end = (c1_low_cnt == 7'd63);
    wire       c1_vs_rise  = c1_vs & ~c1_vs_d;
    wire       c1_vkeep    = (c1_vcnt != 4'd3);
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_de_d <= 1'b0; c1_vs_d <= 1'b0;
            c1_cnt  <= 2'd0; c1_vcnt <= 4'd0;
            c1_waddr<= {LB_AW{1'b0}};
            c1_srcw <= 12'd0; c1_flines <= 12'd0; c1_srch <= 12'd0;
        end else begin
            c1_de_d <= c1_de;
            c1_vs_d <= c1_vs;
            if (!c1_de) begin
                c1_cnt   <= 2'd0;
                c1_waddr <= {LB_AW{1'b0}};
                c1_pixcnt<= 12'd0;
            end else begin
                c1_cnt <= c1_cnt + 2'd1;
                c1_pixcnt<= c1_pixcnt + 12'd1;
                if ((c1_cnt != 2'd3) && c1_vkeep && (c1_waddr < H_CELL-1))
                    c1_waddr <= c1_waddr + 1'b1;
            end
            if (c1_line_end) begin
                c1_vcnt   <= (c1_vcnt == 4'd3) ? 4'd0 : c1_vcnt + 4'd1;
                c1_srcw   <= c1_pixcnt;
                c1_flines <= c1_flines + 12'd1;
            end
            if (c1_vs_rise) begin
                c1_srch   <= c1_flines;
                c1_flines <= 12'd0;
            end
        end
    end
    wire c1_wr_en = c1_de & (c1_cnt != 2'd3) & c1_vkeep & (c1_waddr < H_CELL);
    always @(posedge c1_clk) begin
        if (c1_wr_en) lb1[{c1_bank, c1_waddr}] <= c1_pix;
    end
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n) begin c1_bank <= 1'b0; c1_rdy_tgl <= 1'b0; end
        else if (c1_line_end && c1_vkeep) begin
            c1_bank    <= ~c1_bank;
            c1_rdy_tgl <= ~c1_rdy_tgl;
        end
    end
    reg [1:0]  c2_cnt;
    reg [3:0]  c2_vcnt;
    reg [LB_AW-1:0] c2_waddr;
    reg        c2_de_d, c2_vs_d;
    reg        c2_bank, c2_rdy_tgl;
    reg [11:0] c2_srcw, c2_flines, c2_srch;
    reg [11:0] c2_pixcnt;
    reg [6:0] c2_low_cnt;
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n)                    c2_low_cnt <= 7'd0;
        else if (c2_de)                c2_low_cnt <= 7'd0;
        else if (c2_low_cnt != 7'd127) c2_low_cnt <= c2_low_cnt + 7'd1;
    end
    wire c2_line_end = (c2_low_cnt == 7'd63);
    wire       c2_vs_rise  = c2_vs & ~c2_vs_d;
    wire       c2_vkeep    = (c2_vcnt != 4'd3);
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n) begin
            c2_de_d <= 1'b0; c2_vs_d <= 1'b0;
            c2_cnt  <= 2'd0; c2_vcnt <= 4'd0;
            c2_waddr<= {LB_AW{1'b0}};
            c2_srcw <= 12'd0; c2_flines <= 12'd0; c2_srch <= 12'd0;
        end else begin
            c2_de_d <= c2_de;
            c2_vs_d <= c2_vs;
            if (!c2_de) begin
                c2_cnt   <= 2'd0;
                c2_waddr <= {LB_AW{1'b0}};
                c2_pixcnt<= 12'd0;
            end else begin
                c2_cnt <= c2_cnt + 2'd1;
                c2_pixcnt<= c2_pixcnt + 12'd1;
                if ((c2_cnt != 2'd3) && c2_vkeep && (c2_waddr < H_CELL-1))
                    c2_waddr <= c2_waddr + 1'b1;
            end
            if (c2_line_end) begin
                c2_vcnt   <= (c2_vcnt == 4'd3) ? 4'd0 : c2_vcnt + 4'd1;
                c2_srcw   <= c2_pixcnt;
                c2_flines <= c2_flines + 12'd1;
            end
            if (c2_vs_rise) begin
                c2_srch   <= c2_flines;
                c2_flines <= 12'd0;
            end
        end
    end
    wire c2_wr_en = c2_de & (c2_cnt != 2'd3) & c2_vkeep & (c2_waddr < H_CELL);
    always @(posedge c2_clk) begin
        if (c2_wr_en) lb2[{c2_bank, c2_waddr}] <= c2_pix;
    end
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n) begin c2_bank <= 1'b0; c2_rdy_tgl <= 1'b0; end
        else if (c2_line_end && c2_vkeep) begin
            c2_bank    <= ~c2_bank;
            c2_rdy_tgl <= ~c2_rdy_tgl;
        end
    end
    reg [2:0] c1_rdy_s, c2_rdy_s;
    always @(posedge hd_clk or negedge rst_n) begin
        if (!rst_n) begin c1_rdy_s <= 3'd0; c2_rdy_s <= 3'd0; end
        else begin
            c1_rdy_s <= {c1_rdy_s[1:0], c1_rdy_tgl};
            c2_rdy_s <= {c2_rdy_s[1:0], c2_rdy_tgl};
        end
    end
    wire c1_new = (c1_rdy_s[2:1] == 2'b01);
    wire c2_new = (c2_rdy_s[2:1] == 2'b01);
    reg rb1, rb2, rb1_hold, rb2_hold;
    always @(posedge hd_clk or negedge rst_n) begin
        if (!rst_n) begin
            rb1 <= 1'b0; rb2 <= 1'b0;
            rb1_hold <= 1'b0; rb2_hold <= 1'b0;
        end else begin
            if (c1_new) begin rb1_hold <= rb1; rb1 <= ~rb1; end
            if (c2_new) begin rb2_hold <= rb2; rb2 <= ~rb2; end
        end
    end
    wire [11:0] cx = (hd_x >= H_CELL) ? (hd_x - H_CELL) : hd_x;
    wire [LB_AW-1:0] cxa = cx[LB_AW-1:0];
    reg        hd_de_d2;
    reg [11:0] hd_x_d, hd_y_d;
    reg [15:0] hd_pix_d;
    reg [15:0] px1, px2;
    always @(posedge hd_clk) begin
        px1 <= lb1[{rb1_hold, cxa}];
        px2 <= lb2[{rb2_hold, cxa}];
    end
    always @(posedge hd_clk or negedge rst_n) begin
        if (!rst_n) begin
            o_de <= 1'b0; o_pix <= 16'd0; o_fsync <= 1'b0;
            hd_de_d2 <= 1'b0; hd_x_d <= 12'd0; hd_y_d <= 12'd0; hd_pix_d <= 16'd0;
        end else begin
            hd_de_d2 <= hd_de;
            hd_x_d   <= hd_x;
            hd_y_d   <= hd_y;
            hd_pix_d <= hd_pix;
            o_de     <= hd_de_d2;
            o_fsync  <= hd_vs;
            if (hd_y_d < V_HALF)
                o_pix <= hd_pix_d;
            else
                o_pix <= (hd_x_d < H_CELL) ? px1 : px2;
        end
    end
    assign dbg_srcw1 = {4'd0, c1_srcw};
    assign dbg_srcw2 = {4'd0, c2_srcw};
    assign dbg_srch1 = {4'd0, c1_srch};
    assign dbg_srch2 = {4'd0, c2_srch};
endmodule
module mosaic_wr2 #(
    parameter integer H_HALF = 960,
    parameter integer V_NUM  = 540
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        c1_clk,
    input  wire        c1_vs,
    input  wire        c1_de,
    input  wire [15:0] c1_pix,
    input  wire        c2_clk,
    input  wire        c2_de,
    input  wire [15:0] c2_pix,
    output reg         o_fsync,
    output reg         o_de,
    output reg  [15:0] o_pix,
    output wire [15:0] dbg_srcw1,
    output wire [15:0] dbg_srcw2
);
    reg [15:0] lb1 [0:2047];
    reg [15:0] lb2 [0:2047];
    reg [1:0]  c1_cnt;
    reg [9:0]  c1_waddr;
    reg        c1_de_d;
    reg        c1_bank, c1_rdy_tgl;
    reg [11:0] c1_pixcnt, c1_srcw;
    wire       c1_line_end = c1_de_d & ~c1_de;
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_de_d  <= 1'b0;
            c1_cnt   <= 2'd0;
            c1_waddr <= 10'd0;
            c1_pixcnt<= 12'd0;
            c1_srcw  <= 12'd0;
        end else begin
            c1_de_d <= c1_de;
            if (!c1_de) begin
                c1_cnt    <= 2'd0;
                c1_waddr  <= 10'd0;
                c1_pixcnt <= 12'd0;
            end else begin
                c1_cnt    <= c1_cnt + 2'd1;
                c1_pixcnt <= c1_pixcnt + 12'd1;
                if ((c1_cnt != 2'd3) && (c1_waddr < H_HALF-1))
                    c1_waddr <= c1_waddr + 10'd1;
            end
            if (c1_line_end) c1_srcw <= c1_pixcnt;
        end
    end
    wire c1_wr_en = c1_de & (c1_cnt != 2'd3) & (c1_waddr < H_HALF);
    always @(posedge c1_clk) begin
        if (c1_wr_en) lb1[{c1_bank, c1_waddr}] <= c1_pix;
    end
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n) begin c1_bank <= 1'b0; c1_rdy_tgl <= 1'b0; end
        else if (c1_line_end) begin
            c1_bank    <= ~c1_bank;
            c1_rdy_tgl <= ~c1_rdy_tgl;
        end
    end
    reg [1:0]  c2_cnt;
    reg [9:0]  c2_waddr;
    reg        c2_de_d;
    reg        c2_bank, c2_rdy_tgl;
    reg [11:0] c2_pixcnt, c2_srcw;
    wire       c2_line_end = c2_de_d & ~c2_de;
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n) begin
            c2_de_d  <= 1'b0;
            c2_cnt   <= 2'd0;
            c2_waddr <= 10'd0;
            c2_pixcnt<= 12'd0;
            c2_srcw  <= 12'd0;
        end else begin
            c2_de_d <= c2_de;
            if (!c2_de) begin
                c2_cnt    <= 2'd0;
                c2_waddr  <= 10'd0;
                c2_pixcnt <= 12'd0;
            end else begin
                c2_cnt    <= c2_cnt + 2'd1;
                c2_pixcnt <= c2_pixcnt + 12'd1;
                if ((c2_cnt != 2'd3) && (c2_waddr < H_HALF-1))
                    c2_waddr <= c2_waddr + 10'd1;
            end
            if (c2_line_end) c2_srcw <= c2_pixcnt;
        end
    end
    wire c2_wr_en = c2_de & (c2_cnt != 2'd3) & (c2_waddr < H_HALF);
    always @(posedge c2_clk) begin
        if (c2_wr_en) lb2[{c2_bank, c2_waddr}] <= c2_pix;
    end
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n) begin c2_bank <= 1'b0; c2_rdy_tgl <= 1'b0; end
        else if (c2_line_end) begin
            c2_bank    <= ~c2_bank;
            c2_rdy_tgl <= ~c2_rdy_tgl;
        end
    end
    reg [2:0] c1_rdy_s, c2_rdy_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin c1_rdy_s <= 3'd0; c2_rdy_s <= 3'd0; end
        else begin
            c1_rdy_s <= {c1_rdy_s[1:0], c1_rdy_tgl};
            c2_rdy_s <= {c2_rdy_s[1:0], c2_rdy_tgl};
        end
    end
    wire c1_line_new = (c1_rdy_s[2:1] == 2'b01);
    wire c2_line_new = (c2_rdy_s[2:1] == 2'b01);
    reg [1:0] c1_bank_s, c2_bank_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin c1_bank_s <= 2'd0; c2_bank_s <= 2'd0; end
        else begin
            c1_bank_s <= {c1_bank_s[0], c1_bank};
            c2_bank_s <= {c2_bank_s[0], c2_bank};
        end
    end
    wire rbank1 = ~c1_bank_s[1];
    wire rbank2 = ~c2_bank_s[1];
    reg [2:0] c1vs_s;
    always @(posedge clk) c1vs_s <= {c1vs_s[1:0], c1_vs};
    wire c1vs_rise = (c1vs_s[2:1] == 2'b01);
    localparam S_WAIT = 2'd0,
               S_C1   = 2'd1,
               S_C2   = 2'd2,
               S_GAP  = 2'd3;
    reg [3:0] c1_vcnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) c1_vcnt <= 4'd0;
        else if (c1_line_new) c1_vcnt <= (c1_vcnt == 4'd3) ? 4'd0 : c1_vcnt + 4'd1;
    end
    wire v_keep = (c1_vcnt != 4'd3);
    reg [1:0]  st;
    reg [11:0] line_cnt;
    reg [9:0]  raddr;
    reg        rb1_hold, rb2_hold;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_WAIT;
            raddr    <= 10'd0;
            line_cnt <= 12'd0;
            o_de     <= 1'b0;
            o_pix    <= 16'd0;
            o_fsync  <= 1'b0;
            rb1_hold <= 1'b0;
            rb2_hold <= 1'b0;
        end else begin
            o_fsync <= c1vs_rise;
            case (st)
                S_WAIT: begin
                    o_de <= 1'b0;
                    if (c1_line_new && v_keep) begin
                        rb1_hold <= rbank1;
                        rb2_hold <= rbank2;
                        raddr    <= 10'd0;
                        st       <= S_C1;
                    end
                end
                S_C1: begin
                    o_de  <= 1'b1;
                    o_pix <= lb1[{rb1_hold, raddr}];
                    if (raddr == H_HALF-1) begin
                        raddr <= 10'd0;
                        st    <= S_C2;
                    end else raddr <= raddr + 10'd1;
                end
                S_C2: begin
                    o_de  <= 1'b1;
                    o_pix <= lb2[{rb2_hold, raddr}];
                    if (raddr == H_HALF-1) begin
                        o_de <= 1'b0;
                        st   <= S_GAP;
                    end else raddr <= raddr + 10'd1;
                end
                S_GAP: begin
                    o_de     <= 1'b0;
                    line_cnt <= line_cnt + 12'd1;
                    st       <= S_WAIT;
                end
                default: st <= S_WAIT;
            endcase
        end
    end
    assign dbg_srcw1 = {4'd0, c1_srcw};
    assign dbg_srcw2 = {4'd0, c2_srcw};
endmodule
module hdmi_lb #(
    parameter integer H_LINE = 1920
)(
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        wr_de,
    input  wire [15:0] wr_pix,
    input  wire        rd_clk,
    input  wire        rd_rst_n,
    input  wire [11:0] rd_x,
    output reg  [15:0] rd_pix
);
    reg [15:0] mem [0:4095];
    reg [10:0] waddr;
    reg        wbank, wrdy;
    reg        wde_d;
    wire       wline_end = wde_d & ~wr_de;
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wde_d  <= 1'b0;
            waddr  <= 11'd0;
            wbank  <= 1'b0;
            wrdy   <= 1'b0;
        end else begin
            wde_d <= wr_de;
            if (!wr_de)    waddr <= 11'd0;
            else if (waddr < H_LINE-1) waddr <= waddr + 11'd1;
            if (wline_end) begin
                wbank <= ~wbank;
                wrdy  <= ~wrdy;
            end
        end
    end
    always @(posedge wr_clk) begin
        if (wr_de) mem[{wbank, waddr}] <= wr_pix;
    end
    reg [2:0] rdy_s;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) rdy_s <= 3'd0;
        else           rdy_s <= {rdy_s[1:0], wrdy};
    end
    wire line_new = (rdy_s[2:1] == 2'b01);
    reg rbank, rbank_hold;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin rbank <= 1'b0; rbank_hold <= 1'b0; end
        else if (line_new) begin
            rbank_hold <= rbank;
            rbank      <= ~rbank;
        end
    end
    always @(posedge rd_clk) begin
        rd_pix <= mem[{rbank_hold, rd_x[10:0]}];
    end
endmodule
module compose_wr #(
    parameter integer H_LINE  = 1920,
    parameter integer V_TOP   = 540,
    parameter integer V_BOT   = 540,
    parameter integer H_CELL  = 960
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        hd_de,
    input  wire [15:0] hd_pix,
    input  wire        hd_vs,
    input  wire        c1_clk,
    input  wire        c1_vs,
    input  wire        c1_de,
    input  wire [15:0] c1_pix,
    input  wire        c2_clk,
    input  wire        c2_de,
    input  wire [15:0] c2_pix,
    output reg         o_de,
    output reg  [15:0] o_pix,
    output reg         o_fsync,
    output wire [15:0] dbg_srcw1,
    output wire [15:0] dbg_srcw2,
    output wire [15:0] dbg_botline,
    output wire [15:0] dbg_camlines
);
    reg [15:0] lb1 [0:2047];
    reg [15:0] lb2 [0:2047];
    reg [1:0]  c1_cnt;
    reg [9:0]  c1_waddr;
    reg        c1_de_d;
    reg        c1_bank, c1_rdy_tgl;
    reg [11:0] c1_pixcnt, c1_srcw;
    reg [6:0] c1_low_cnt2;
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n)                     c1_low_cnt2 <= 7'd0;
        else if (c1_de)                 c1_low_cnt2 <= 7'd0;
        else if (c1_low_cnt2 != 7'd127) c1_low_cnt2 <= c1_low_cnt2 + 7'd1;
    end
    wire c1_line_end = (c1_low_cnt2 == 7'd3);
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_de_d  <= 1'b0;
            c1_cnt   <= 2'd0;
            c1_waddr <= 10'd0;
            c1_pixcnt<= 12'd0;
            c1_srcw  <= 12'd0;
        end else begin
            c1_de_d <= c1_de;
            if (!c1_de) begin
                c1_cnt    <= 2'd0;
                c1_waddr  <= 10'd0;
                c1_pixcnt <= 12'd0;
            end else begin
                c1_cnt    <= c1_cnt + 2'd1;
                c1_pixcnt <= c1_pixcnt + 12'd1;
                if ((c1_cnt != 2'd3) && (c1_waddr < H_CELL-1))
                    c1_waddr <= c1_waddr + 10'd1;
            end
            if (c1_line_end) c1_srcw <= c1_pixcnt;
        end
    end
    wire c1_wr_en = c1_de & (c1_cnt != 2'd3) & (c1_waddr < H_CELL);
    always @(posedge c1_clk) begin
        if (c1_wr_en) lb1[{c1_bank, c1_waddr}] <= c1_pix;
    end
    always @(posedge c1_clk or negedge rst_n) begin
        if (!rst_n) begin c1_bank <= 1'b0; c1_rdy_tgl <= 1'b0; end
        else if (c1_line_end) begin
            c1_bank    <= ~c1_bank;
            c1_rdy_tgl <= ~c1_rdy_tgl;
        end
    end
    reg [1:0]  c2_cnt;
    reg [9:0]  c2_waddr;
    reg        c2_de_d;
    reg        c2_bank, c2_rdy_tgl;
    reg [11:0] c2_pixcnt, c2_srcw;
    reg [6:0] c2_low_cnt2;
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n)                     c2_low_cnt2 <= 7'd0;
        else if (c2_de)                 c2_low_cnt2 <= 7'd0;
        else if (c2_low_cnt2 != 7'd127) c2_low_cnt2 <= c2_low_cnt2 + 7'd1;
    end
    wire c2_line_end = (c2_low_cnt2 == 7'd3);
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n) begin
            c2_de_d  <= 1'b0;
            c2_cnt   <= 2'd0;
            c2_waddr <= 10'd0;
            c2_pixcnt<= 12'd0;
            c2_srcw  <= 12'd0;
        end else begin
            c2_de_d <= c2_de;
            if (!c2_de) begin
                c2_cnt    <= 2'd0;
                c2_waddr  <= 10'd0;
                c2_pixcnt <= 12'd0;
            end else begin
                c2_cnt    <= c2_cnt + 2'd1;
                c2_pixcnt <= c2_pixcnt + 12'd1;
                if ((c2_cnt != 2'd3) && (c2_waddr < H_CELL-1))
                    c2_waddr <= c2_waddr + 10'd1;
            end
            if (c2_line_end) c2_srcw <= c2_pixcnt;
        end
    end
    wire c2_wr_en = c2_de & (c2_cnt != 2'd3) & (c2_waddr < H_CELL);
    always @(posedge c2_clk) begin
        if (c2_wr_en) lb2[{c2_bank, c2_waddr}] <= c2_pix;
    end
    always @(posedge c2_clk or negedge rst_n) begin
        if (!rst_n) begin c2_bank <= 1'b0; c2_rdy_tgl <= 1'b0; end
        else if (c2_line_end) begin
            c2_bank    <= ~c2_bank;
            c2_rdy_tgl <= ~c2_rdy_tgl;
        end
    end
    reg [2:0] c1_rdy_s, c2_rdy_s;
    reg [1:0] c1_bank_s, c2_bank_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_rdy_s  <= 3'd0; c2_rdy_s  <= 3'd0;
            c1_bank_s <= 2'd0; c2_bank_s <= 2'd0;
        end else begin
            c1_rdy_s  <= {c1_rdy_s[1:0],  c1_rdy_tgl};
            c2_rdy_s  <= {c2_rdy_s[1:0],  c2_rdy_tgl};
            c1_bank_s <= {c1_bank_s[0],   c1_bank};
            c2_bank_s <= {c2_bank_s[0],   c2_bank};
        end
    end
    wire c1_new = (c1_rdy_s[2:1] == 2'b01);
    wire c2_new = (c2_rdy_s[2:1] == 2'b01);
    wire rb1 = ~c1_bank_s[1];
    wire rb2 = ~c2_bank_s[1];
    reg [11:0] hd_y;
    reg        hd_de_d;
    wire       hd_line_end = hd_de_d & ~hd_de;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hd_y <= 12'd0; hd_de_d <= 1'b0;
        end else begin
            hd_de_d <= hd_de;
            if (hd_vs)                 hd_y <= 12'd0;
            else if (hd_line_end)      hd_y <= hd_y + 12'd1;
        end
    end
    reg [15:0] px_rd;
    wire [11:0] cidx = (bot_col < H_CELL) ? bot_col : (bot_col - H_CELL);
    always @(posedge clk) begin
        px_rd <= (st == S_BOTC2) ? lb2[{rb2h, bot_col[9:0]}]
                                     : lb1[{rb1h, bot_col[9:0]}];
    end
    localparam [2:0] S_WAIT = 3'd0, S_TOP = 3'd1, S_CWAIT = 3'd6,
                   S_BOTW = 3'd2, S_BOTC1 = 3'd3, S_BOTC2 = 3'd4, S_BOTG = 3'd5;
    reg hd_vs_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hd_vs_d2 <= 1'b0;
        else        hd_vs_d2 <= hd_vs;
    end
    wire hd_vs_rise = hd_vs & ~hd_vs_d2;
    reg [2:0] c1vs_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) c1vs_s <= 3'd0;
        else        c1vs_s <= {c1vs_s[1:0], c1_vs};
    end
    wire c1vs_rise = (c1vs_s[2:1] == 2'b01);
    reg [2:0]  st;
    reg [11:0] bot_line;
    reg [11:0] bot_col;
    reg [1:0]  c1_vcnt2;
    reg        rb1h, rb2h;
    reg [1:0]  hd_vs_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hd_vs_div <= 2'd0;
        else if (hd_vs_rise) hd_vs_div <= hd_vs_div + 2'd1;
    end
    wire frame_start = hd_vs_rise & (hd_vs_div == 2'd3);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_WAIT;
            bot_line <= 12'd0;
            bot_col  <= 12'd0;
            o_de     <= 1'b0;
            o_pix    <= 16'd0;
            o_fsync  <= 1'b0;
        end else begin
            case (st)
                S_WAIT: begin
                    o_de <= 1'b0;
                    if (frame_start) begin
                        o_fsync  <= 1'b1;
                        bot_line <= 12'd0;
                        bot_col  <= 12'd0;
                        c1_vcnt2 <= 2'd0;
                        st       <= S_TOP;
                    end else begin
                        o_fsync  <= 1'b0;
                    end
                end
                S_TOP: begin
                    o_de    <= hd_de;
                    o_pix   <= hd_pix;
                    o_fsync <= 1'b0;
                    if (hd_line_end && (hd_y >= V_TOP-1)) begin
                        st       <= S_CWAIT;
                        bot_line <= 12'd0;
                    end
                end
                S_CWAIT: begin
                    o_de <= 1'b0;
                    if (c1vs_rise) begin
                        bot_line <= 12'd0;
                        bot_col  <= 12'd0;
                        st       <= S_BOTW;
                    end
                end
                S_BOTW: begin
                    o_de <= 1'b0;
                    if (c1_new) begin
                        rb1h    <= rb1;
                        rb2h    <= rb2;
                        bot_col <= 12'd0;
                        st      <= S_BOTC1;
                    end
                end
                S_BOTC1: begin
                    o_de  <= 1'b1;
                    o_pix <= px_rd;
                    if (bot_col == H_CELL-1) begin
                        bot_col <= 12'd0;
                        st      <= S_BOTC2;
                    end else begin
                        bot_col <= bot_col + 12'd1;
                    end
                end
                S_BOTC2: begin
                    o_de  <= 1'b1;
                    o_pix <= px_rd;
                    if (bot_col == H_CELL-1) begin
                        o_de    <= 1'b0;
                        bot_line<= bot_line + 12'd1;
                        st      <= S_BOTG;
                    end else begin
                        bot_col <= bot_col + 12'd1;
                    end
                end
                S_BOTG: begin
                    o_de <= 1'b0;
                    if (bot_line >= V_BOT) begin
                        st <= S_WAIT;
                    end else begin
                        st <= S_BOTW;
                    end
                end
                default: st <= S_WAIT;
            endcase
        end
    end
    reg [11:0] camline_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              camline_cnt <= 12'd0;
        else if (frame_start)    camline_cnt <= 12'd0;
        else if (c1_new)         camline_cnt <= camline_cnt + 12'd1;
    end
    reg [11:0] lat_botline = 0, lat_camlines = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin lat_botline <= 12'd0; lat_camlines <= 12'd0; end
        else if (frame_start) begin
            lat_botline  <= bot_line;
            lat_camlines <= camline_cnt;
        end
    end
    assign dbg_botline  = {4'd0, lat_botline};
    assign dbg_camlines = {4'd0, lat_camlines};
    assign dbg_srcw1 = {4'd0, c1_srcw};
    assign dbg_srcw2 = {4'd0, c2_srcw};
endmodule
`timescale 1ns / 1ps
module cam_lb1 #(
    parameter integer H_CELL = 960
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        c_clk,
    input  wire        c_de,
    input  wire [15:0] c_pix,
    input  wire [11:0] rd_x,
    output reg  [15:0] rd_pix,
    output wire        o_line_new
);
    reg [15:0] lb [0:2047];
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) begin
            lb[i]        = 16'd0;
            lb[1024 + i] = 16'd0;
        end
    end
    reg [1:0]  cnt;
    reg [9:0]  waddr;
    reg        bank, rdy_tgl;
    reg [6:0]  low_cnt;
    wire line_end = (low_cnt == 7'd3);
    always @(posedge c_clk or negedge rst_n) begin
        if (!rst_n)                 low_cnt <= 7'd0;
        else if (c_de)              low_cnt <= 7'd0;
        else if (low_cnt != 7'd127) low_cnt <= low_cnt + 7'd1;
    end
    always @(posedge c_clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= 2'd0;
            waddr   <= 10'd0;
            bank    <= 1'b0;
            rdy_tgl <= 1'b0;
        end else begin
            if (!c_de) begin
                cnt   <= 2'd0;
                waddr <= 10'd0;
            end else begin
                cnt <= cnt + 2'd1;
                if ((cnt != 2'd3) && (waddr < H_CELL-1))
                    waddr <= waddr + 10'd1;
            end
            if (line_end) begin
                bank    <= ~bank;
                rdy_tgl <= ~rdy_tgl;
            end
        end
    end
    wire wr_en = c_de & (cnt != 2'd3);
    always @(posedge c_clk) begin
        if (wr_en) lb[{bank, waddr}] <= c_pix;
    end
    reg [2:0] rdy_s;
    reg [1:0] bank_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdy_s  <= 3'd0;
            bank_s <= 2'd0;
        end else begin
            rdy_s  <= {rdy_s[1:0], rdy_tgl};
            bank_s <= {bank_s[0],  bank};
        end
    end
    wire rb = ~bank_s[1];
    always @(posedge clk) begin
        rd_pix <= lb[{rb, rd_x[9:0]}];
    end
    assign o_line_new = (rdy_s[2:1] == 2'b01);
endmodule
module fram_buf_dual #(
    parameter MEM_ROW_WIDTH    = 15,
    parameter MEM_COLUMN_WIDTH = 10,
    parameter MEM_BANK_WIDTH   = 3,
    parameter CTRL_ADDR_WIDTH  = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter MEM_DQ_WIDTH     = 32,
    parameter H_NUM            = 12'd1920,
    parameter V_NUM            = 12'd1080,
    parameter PIX_WIDTH        = 16,
    parameter CAM_H_NUM        = 12'd640,
    parameter CAM_V_NUM        = 12'd480,
    parameter [CTRL_ADDR_WIDTH-1:0] CAM_ADDR_OFFSET = 28'h080_0000
)(
    input  wire                       vin_clk,
    input  wire                       wr_fsync,
    input  wire                       wr_en,
    input  wire [PIX_WIDTH-1'b1:0]    wr_data,
    output wire                       init_done,
    input  wire                       ddr_clk,
    input  wire                       ddr_rstn,
    input  wire                       vout_clk,
    input  wire                       rd_fsync,
    input  wire                       rd_en,
    output wire                       vout_de,
    output wire [PIX_WIDTH-1'b1:0]    vout_data,
    input  wire                       cam_vin_clk,
    input  wire                       cam_fsync,
    input  wire                       cam_en,
    input  wire [PIX_WIDTH-1'b1:0]    cam_data,
    input  wire                       cam_rd_de,
    input  wire [11:0]                cam_rd_x,
    input  wire [11:0]                cam_rd_y,
    output wire                       cam_vout_de,
    output wire [PIX_WIDTH-1'b1:0]    cam_vout_data,
    output wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr,
    output wire [3:0]                 axi_awid,
    output wire [3:0]                 axi_awlen,
    output wire [2:0]                 axi_awsize,
    output wire [1:0]                 axi_awburst,
    input  wire                       axi_awready,
    output wire                       axi_awvalid,
    output wire [MEM_DQ_WIDTH*8-1:0]  axi_wdata,
    output wire [MEM_DQ_WIDTH-1:0]    axi_wstrb,
    input  wire                       axi_wlast,
    output wire                       axi_wvalid,
    input  wire                       axi_wready,
    input  wire [3:0]                 axi_bid,
    output wire [CTRL_ADDR_WIDTH-1:0] axi_araddr,
    output wire [3:0]                 axi_arid,
    output wire [3:0]                 axi_arlen,
    output wire [2:0]                 axi_arsize,
    output wire [1:0]                 axi_arburst,
    output wire                       axi_arvalid,
    input  wire                       axi_arready,
    output wire                       axi_rready,
    input  wire [MEM_DQ_WIDTH*8-1:0]  axi_rdata,
    input  wire                       axi_rvalid,
    input  wire                       axi_rlast,
    input  wire [3:0]                 axi_rid
);
    localparam LEN_WIDTH        = 32;
    localparam LINE_ADDR_WIDTH  = 22;
    localparam CAM_LINE_ADDR_W  = 21;
    localparam FRAME_CNT_WIDTH  = CTRL_ADDR_WIDTH - LINE_ADDR_WIDTH;
    reg         cam_rd_bank;
    wire                        dis_wreq;
    wire [CTRL_ADDR_WIDTH-1:0]  dis_waddr;
    wire [LEN_WIDTH-1:0]        dis_wlen;
    wire                        dis_wrdy, dis_wdone, dis_wdata_req, dis_wbac;
    wire [8*MEM_DQ_WIDTH-1:0]   dis_wdata;
    wire [FRAME_CNT_WIDTH-1:0]  dis_wframe;
    wire                        dis_wirq;
    wr_buf #(
        .ADDR_WIDTH       (CTRL_ADDR_WIDTH  ),
        .ADDR_OFFSET      (32'd0            ),
        .H_NUM            (H_NUM            ),
        .V_NUM            (V_NUM            ),
        .DQ_WIDTH         (MEM_DQ_WIDTH     ),
        .LEN_WIDTH        (LEN_WIDTH        ),
        .PIX_WIDTH        (PIX_WIDTH        ),
        .LINE_ADDR_WIDTH  (LINE_ADDR_WIDTH  ),
        .FRAME_CNT_WIDTH  (FRAME_CNT_WIDTH  )
    ) u_wr_dis (
        .ddr_clk       (ddr_clk        ),
        .ddr_rstn      (ddr_rstn       ),
        .wr_clk        (vin_clk        ),
        .wr_fsync      (wr_fsync       ),
        .wr_en         (wr_en          ),
        .wr_data       (wr_data        ),
        .rd_bac        (dis_wbac       ),
        .ddr_wreq      (dis_wreq       ),
        .ddr_waddr     (dis_waddr      ),
        .ddr_wr_len    (dis_wlen       ),
        .ddr_wrdy      (dis_wrdy       ),
        .ddr_wdone     (dis_wdone      ),
        .ddr_wdata     (dis_wdata      ),
        .ddr_wdata_req (dis_wdata_req  ),
        .frame_wcnt    (dis_wframe     ),
        .frame_wirq    (dis_wirq       )
    );
    wire                        cam_wreq;
    wire [CTRL_ADDR_WIDTH-1:0]  cam_waddr;
    wire [LEN_WIDTH-1:0]        cam_wlen;
    wire                        cam_wrdy, cam_wdone, cam_wdata_req, cam_wbac;
    wire [8*MEM_DQ_WIDTH-1:0]   cam_wdata;
    wire [FRAME_CNT_WIDTH-1:0]  cam_wframe;
    wr_buf #(
        .ADDR_WIDTH       (CTRL_ADDR_WIDTH   ),
        .ADDR_OFFSET      (CAM_ADDR_OFFSET   ),
        .H_NUM            (CAM_H_NUM         ),
        .V_NUM            (CAM_V_NUM         ),
        .DQ_WIDTH         (MEM_DQ_WIDTH      ),
        .LEN_WIDTH        (LEN_WIDTH         ),
        .PIX_WIDTH        (PIX_WIDTH         ),
        .LINE_ADDR_WIDTH  (CAM_LINE_ADDR_W   ),
        .FRAME_CNT_WIDTH  (FRAME_CNT_WIDTH   )
    ) u_wr_cam (
        .ddr_clk       (ddr_clk        ),
        .ddr_rstn      (ddr_rstn       ),
        .wr_clk        (cam_vin_clk    ),
        .wr_fsync      (cam_fsync      ),
        .wr_en         (cam_en         ),
        .wr_data       (cam_data       ),
        .rd_bac        (cam_wbac       ),
        .ddr_wreq      (cam_wreq       ),
        .ddr_waddr     (cam_waddr      ),
        .ddr_wr_len    (cam_wlen       ),
        .ddr_wrdy      (cam_wrdy       ),
        .ddr_wdone     (cam_wdone      ),
        .ddr_wdata     (cam_wdata      ),
        .ddr_wdata_req (cam_wdata_req  ),
        .frame_wcnt    (cam_wframe     ),
        .frame_wirq    (               )
    );
    wire                        dis_rreq;
    wire [CTRL_ADDR_WIDTH-1:0]  dis_raddr;
    wire [LEN_WIDTH-1:0]        dis_rlen;
    wire                        dis_rrdy, dis_rdone;
    wire [8*MEM_DQ_WIDTH-1:0]   dis_rdata;
    wire                        dis_rden;
    rd_buf #(
        .ADDR_WIDTH       (CTRL_ADDR_WIDTH  ),
        .ADDR_OFFSET      (32'd0            ),
        .H_NUM            (H_NUM            ),
        .V_NUM            (V_NUM            ),
        .DQ_WIDTH         (MEM_DQ_WIDTH     ),
        .LEN_WIDTH        (LEN_WIDTH        ),
        .PIX_WIDTH        (PIX_WIDTH        ),
        .LINE_ADDR_WIDTH  (LINE_ADDR_WIDTH  ),
        .FRAME_CNT_WIDTH  (FRAME_CNT_WIDTH  ),
        .EXT_FRAME_SEL    (1'b0             )
    ) u_rd_dis (
        .ddr_clk       (ddr_clk        ),
        .ddr_rstn      (ddr_rstn       ),
        .ext_frame_sel (1'b0           ),
        .vout_clk      (vout_clk       ),
        .rd_fsync      (rd_fsync       ),
        .rd_en         (rd_en          ),
        .vout_de       (vout_de        ),
        .vout_data     (vout_data      ),
        .init_done     (init_done      ),
        .ddr_rreq      (dis_rreq       ),
        .ddr_raddr     (dis_raddr      ),
        .ddr_rd_len    (dis_rlen       ),
        .ddr_rrdy      (dis_rrdy       ),
        .ddr_rdone     (dis_rdone      ),
        .ddr_rdata     (dis_rdata      ),
        .ddr_rdata_en  (dis_rden       )
    );
    wire                        cam_rreq;
    wire [CTRL_ADDR_WIDTH-1:0]  cam_raddr;
    wire [LEN_WIDTH-1:0]        cam_rlen;
    wire                        cam_rrdy, cam_rdone;
    wire [8*MEM_DQ_WIDTH-1:0]   cam_rdata;
    wire                        cam_rden;
    cam_line_rd #(
        .SRC_W            (CAM_H_NUM      ),
        .SEG_H            (CAM_V_NUM/2    ),
        .DST_X0           (H_NUM/2        ),
        .DST_W            (H_NUM/2        )
    ) u_rd_cam (
        .vout_clk     (vout_clk        ),
        .ddr_clk      (ddr_clk         ),
        .rst_n        (ddr_rstn        ),
        .rd_fsync     (rd_fsync        ),
        .rd_de        (cam_rd_de       ),
        .rd_x         (cam_rd_x        ),
        .rd_y         (cam_rd_y        ),
        .frame_bank   (cam_rd_bank     ),
        .base_addr    (CAM_ADDR_OFFSET ),
        .ddr_rreq     (cam_rreq        ),
        .ddr_raddr    (cam_raddr       ),
        .ddr_rd_len   (cam_rlen        ),
        .ddr_rdone    (cam_rdone       ),
        .ddr_rdata_en (cam_rden        ),
        .ddr_rdata    (cam_rdata       ),
        .vout_data    (cam_vout_data   ),
        .vout_de      (cam_vout_de     )
    );
    reg  [11:0] cam_line_cnt;
    reg  [2:0]  camfs_s;
    reg         cam_mybank;
    reg         cam_done_tgl;
    reg         cam_done_bank;
    wire        cam_fsync_rise = (camfs_s[2:1] == 2'b01);
    always @(posedge ddr_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            cam_line_cnt  <= 12'd0;
            camfs_s       <= 3'd0;
            cam_mybank    <= 1'b0;
            cam_done_tgl  <= 1'b0;
            cam_done_bank <= 1'b0;
        end else begin
            camfs_s <= {camfs_s[1:0], cam_fsync};
            if (cam_fsync_rise && (cam_line_cnt != 12'd0)) begin
                cam_done_bank <= cam_mybank;
                cam_mybank    <= ~cam_mybank;
                cam_done_tgl  <= ~cam_done_tgl;
                cam_line_cnt  <= 12'd0;
            end else if (cam_wdone) begin
                cam_line_cnt <= cam_line_cnt + 12'd1;
            end
        end
    end
    reg [1:0] cam_done_s;
    reg       cam_bank_s;
    reg       cam_seen;
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            cam_done_s <= 2'd0;
            cam_bank_s <= 1'b0;
        end else begin
            cam_done_s <= {cam_done_s[0], cam_done_tgl};
            cam_bank_s <= cam_done_bank;
        end
    end
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            cam_seen    <= 1'b0;
            cam_rd_bank <= 1'b0;
        end else if (rd_fsync && (cam_done_s[1] != cam_seen)) begin
            cam_seen    <= cam_done_s[1];
            cam_rd_bank <= cam_bank_s;
        end
    end
    wire                       wr_cmd_en, wr_cmd_ready, wr_cmd_done, wr_data_re, wr_bac;
    wire [CTRL_ADDR_WIDTH-1:0] wr_cmd_addr;
    wire [31:0]                wr_cmd_len;
    wire [8*MEM_DQ_WIDTH-1:0]  wr_ctrl_data;
    wire                       rd_cmd_en, rd_cmd_ready, rd_cmd_done, rd_rdata_en;
    wire [CTRL_ADDR_WIDTH-1:0] rd_cmd_addr;
    wire [31:0]                rd_cmd_len;
    wire [8*MEM_DQ_WIDTH-1:0]  rd_rdata;
    reg  wr_busy, wr_sel, wr_go;
    reg  [15:0] wr_wdog;
    wire dis_own = (wr_busy && !wr_sel);
    wire cam_own = (wr_busy &&  wr_sel);
    wire wr_done_now = cam_own ? cam_wdone : dis_wdone;
    always @(posedge ddr_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            wr_busy <= 1'b0;
            wr_sel  <= 1'b0;
            wr_go   <= 1'b0;
            wr_wdog <= 16'd0;
        end else begin
            wr_go <= 1'b0;
            if (!wr_busy) begin
                wr_wdog <= 16'd0;
                if (dis_wreq)      begin wr_busy <= 1'b1; wr_sel <= 1'b0; wr_go <= 1'b1; end
                else if (cam_wreq) begin wr_busy <= 1'b1; wr_sel <= 1'b1; wr_go <= 1'b1; end
            end else begin
                wr_wdog <= wr_wdog + 16'd1;
                if (wr_done_now || (wr_wdog == 16'hFFFF))
                    wr_busy <= 1'b0;
            end
        end
    end
    assign wr_cmd_en    = wr_go;
    assign wr_cmd_addr  = wr_sel ? cam_waddr : dis_waddr;
    assign wr_cmd_len   = wr_sel ? cam_wlen  : dis_wlen;
    assign wr_ctrl_data = wr_sel ? cam_wdata : dis_wdata;
    assign dis_wrdy      = dis_own ? wr_cmd_ready : 1'b0;
    assign dis_wdone     = dis_own ? wr_cmd_done  : 1'b0;
    assign dis_wdata_req = dis_own ? wr_data_re   : 1'b0;
    assign dis_wbac      = dis_own ? wr_bac       : 1'b0;
    assign cam_wdone     = cam_own ? wr_cmd_done  : 1'b0;
    assign cam_wrdy      = cam_own ? wr_cmd_ready : 1'b0;
    assign cam_wdata_req = cam_own ? wr_data_re   : 1'b0;
    assign cam_wbac      = cam_own ? wr_bac       : 1'b0;
    reg  rd_busy, rd_sel, rd_go;
    reg  dis_rpend, cam_rpend;
    reg  [15:0] rd_beat;
    reg  [15:0] rd_wdog;
    wire dis_ask = dis_rreq | dis_rpend;
    wire cam_ask = cam_rreq | cam_rpend;
    wire gnt_dis = (!rd_busy) && dis_ask;
    wire gnt_cam = (!rd_busy) && (!dis_ask) && cam_ask;
    wire dis_ro = (rd_busy && !rd_sel);
    wire cam_ro = (rd_busy &&  rd_sel);
    wire rd_done_now = cam_ro ? cam_rdone : dis_rdone;
    wire [15:0] rd_len_now = rd_sel ? cam_rlen[15:0] : dis_rlen[15:0];
    wire rd_data_done = (rd_beat >= rd_len_now);
    always @(posedge ddr_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            rd_busy   <= 1'b0;
            rd_sel    <= 1'b0;
            rd_go     <= 1'b0;
            dis_rpend <= 1'b0;
            cam_rpend <= 1'b0;
            rd_beat   <= 16'd0;
            rd_wdog   <= 16'd0;
        end else begin
            rd_go <= 1'b0;
            if (!rd_busy)         rd_beat <= 16'd0;
            else if (rd_rdata_en) rd_beat <= rd_beat + 16'd1;
            if (!rd_busy) begin
                rd_wdog <= 16'd0;
                if (dis_ask)      begin rd_busy <= 1'b1; rd_sel <= 1'b0; rd_go <= 1'b1; end
                else if (cam_ask) begin rd_busy <= 1'b1; rd_sel <= 1'b1; rd_go <= 1'b1; end
            end else begin
                rd_wdog <= rd_wdog + 16'd1;
                if ((rd_done_now && rd_data_done) || (rd_wdog == 16'hFFFF))
                    rd_busy <= 1'b0;
            end
            dis_rpend <= dis_ask & ~gnt_dis;
            cam_rpend <= cam_ask & ~gnt_cam;
        end
    end
    assign rd_cmd_en   = rd_go;
    assign rd_cmd_addr = rd_sel ? cam_raddr : dis_raddr;
    assign rd_cmd_len  = rd_sel ? cam_rlen  : dis_rlen;
    assign dis_rrdy  = dis_ro ? rd_cmd_ready : 1'b0;
    assign dis_rdone = dis_ro ? rd_cmd_done  : 1'b0;
    assign dis_rden  = dis_ro ? rd_rdata_en  : 1'b0;
    assign cam_rrdy  = cam_ro ? rd_cmd_ready : 1'b0;
    assign cam_rdone = cam_ro ? rd_cmd_done  : 1'b0;
    assign cam_rden  = cam_ro ? rd_rdata_en  : 1'b0;
    assign dis_rdata = rd_rdata;
    assign cam_rdata = rd_rdata;
    wr_rd_ctrl_top #(
        .CTRL_ADDR_WIDTH (CTRL_ADDR_WIDTH),
        .MEM_DQ_WIDTH    (MEM_DQ_WIDTH   )
    ) u_wr_rd_ctrl (
        .clk           (ddr_clk        ),
        .rstn          (ddr_rstn       ),
        .wr_cmd_en     (wr_cmd_en      ),
        .wr_cmd_addr   (wr_cmd_addr    ),
        .wr_cmd_len    (wr_cmd_len     ),
        .wr_cmd_ready  (wr_cmd_ready   ),
        .wr_cmd_done   (wr_cmd_done    ),
        .wr_bac        (wr_bac         ),
        .wr_ctrl_data  (wr_ctrl_data   ),
        .wr_data_re    (wr_data_re     ),
        .rd_cmd_en     (rd_cmd_en      ),
        .rd_cmd_addr   (rd_cmd_addr    ),
        .rd_cmd_len    (rd_cmd_len     ),
        .rd_cmd_ready  (rd_cmd_ready   ),
        .rd_cmd_done   (rd_cmd_done    ),
        .read_ready    (1'b1           ),
        .read_rdata    (rd_rdata       ),
        .read_en       (rd_rdata_en    ),
        .axi_awaddr    (axi_awaddr     ),
        .axi_awid      (axi_awid       ),
        .axi_awlen     (axi_awlen      ),
        .axi_awsize    (axi_awsize     ),
        .axi_awburst   (axi_awburst    ),
        .axi_awready   (axi_awready    ),
        .axi_awvalid   (axi_awvalid    ),
        .axi_wdata     (axi_wdata      ),
        .axi_wstrb     (axi_wstrb      ),
        .axi_wlast     (axi_wlast      ),
        .axi_wvalid    (axi_wvalid     ),
        .axi_wready    (axi_wready     ),
        .axi_bid       (axi_bid        ),
        .axi_bresp     (2'd0           ),
        .axi_bvalid    (1'b0           ),
        .axi_bready    (               ),
        .axi_araddr    (axi_araddr     ),
        .axi_arid      (axi_arid       ),
        .axi_arlen     (axi_arlen      ),
        .axi_arsize    (axi_arsize     ),
        .axi_arburst   (axi_arburst    ),
        .axi_arvalid   (axi_arvalid    ),
        .axi_arready   (axi_arready    ),
        .axi_rready    (axi_rready     ),
        .axi_rdata     (axi_rdata      ),
        .axi_rvalid    (axi_rvalid     ),
        .axi_rlast     (axi_rlast      ),
        .axi_rid       (axi_rid        ),
        .axi_rresp     (2'd0           )
    );
    reg init_done_r;
    always @(posedge ddr_clk) begin
        if (dis_wirq) init_done_r <= 1'b1;
        else          init_done_r <= init_done_r;
    end
    assign init_done = init_done_r;
endmodule
module cam_fe #(
    parameter integer H_NUM = 640,
    parameter integer DIV   = 4,
    parameter integer KEEP  = 3
)(
    input  wire        c_clk,
    input  wire        rst_n,
    input  wire        vs,
    input  wire        de,
    input  wire [15:0] pix,
    input  wire        r_clk,
    input  wire [1:0]  r_bank,
    input  wire [9:0]  r_addr,
    output reg  [15:0] r_pix,
    output reg         pub_tgl,
    output reg  [11:0] src_lines
);
    reg [15:0] mem [0:4095];
    reg [1:0]  wbank;
    reg [9:0]  waddr;
    reg        pair;
    reg [15:0] p0;
    reg        de_d;
    reg [11:0] ln;
    reg        keep;
    wire [5:0]  ar = ({1'b0, p0[15:11]} + {1'b0, pix[15:11]} + 6'd1) >> 1;
    wire [6:0]  ag = ({1'b0, p0[10:5] } + {1'b0, pix[10:5] } + 7'd1) >> 1;
    wire [5:0]  ab = ({1'b0, p0[4:0]  } + {1'b0, pix[4:0]  } + 6'd1) >> 1;
    wire [15:0] avg = {ar[4:0], ag[5:0], ab[4:0]};
    always @(posedge r_clk) r_pix <= mem[{r_bank, r_addr}];
    reg [2:0] vs_s;
    always @(posedge c_clk or negedge rst_n) begin
        if (!rst_n) vs_s <= 3'd0;
        else        vs_s <= {vs_s[1:0], vs};
    end
    wire vs_rise = (vs_s[2:1] == 2'b01);
    always @(posedge c_clk or negedge rst_n) begin
        if (!rst_n) begin
            wbank     <= 2'd0;
            waddr     <= 10'd0;
            pair      <= 1'b0;
            p0        <= 16'd0;
            de_d      <= 1'b0;
            ln        <= 12'd0;
            keep      <= 1'b0;
            pub_tgl   <= 1'b0;
            src_lines <= 12'd0;
        end else begin
            de_d <= de;
            if (!de) begin
                pair  <= 1'b0;
                waddr <= 10'd0;
            end else begin
                pair <= ~pair;
                if (!pair) begin
                    p0 <= pix;
                end else if (waddr < H_NUM) begin
                    mem[{wbank, waddr}] <= avg;
                    waddr <= waddr + 10'd1;
                end
            end
            if (de && !de_d)
                keep <= (ln[1:0] < KEEP[1:0]);
            if (de_d && !de) begin
                if (keep) begin
                    wbank   <= wbank + 2'd1;
                    pub_tgl <= ~pub_tgl;
                end
                if (ln != 12'd4095) ln <= ln + 12'd1;
            end
            if (vs_rise) begin
                src_lines <= ln;
                ln        <= 12'd0;
                waddr     <= 10'd0;
            end
        end
    end
endmodule
module cam_dual_wr #(
    parameter integer H_NUM  = 640,
    parameter integer SEG_V  = 540
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vs1,
    input  wire        de1,
    input  wire [15:0] pix1,
    input  wire        clk2,
    input  wire        vs2,
    input  wire        de2,
    input  wire [15:0] pix2,
    output reg         o_fsync,
    output reg         o_en,
    output reg  [15:0] o_px,
    output reg  [11:0] o_rows,
    output reg  [11:0] o_lines
);
    wire [15:0] fe1_pix, fe2_pix;
    wire        fe1_tgl, fe2_tgl;
    wire [11:0] fe1_lines, fe2_lines;
    reg  [1:0]  rd_bank1, rd_bank2;
    reg  [9:0]  rd_addr1, rd_addr2;
    cam_fe #(.H_NUM(H_NUM)) u_fe1 (
        .c_clk     (clk),
        .rst_n     (rst_n),
        .vs        (vs1),
        .de        (de1),
        .pix       (pix1),
        .r_clk     (clk),
        .r_bank    (rd_bank1),
        .r_addr    (rd_addr1),
        .r_pix     (fe1_pix),
        .pub_tgl   (fe1_tgl),
        .src_lines (fe1_lines)
    );
    cam_fe #(.H_NUM(H_NUM)) u_fe2 (
        .c_clk     (clk2),
        .rst_n     (rst_n),
        .vs        (vs2),
        .de        (de2),
        .pix       (pix2),
        .r_clk     (clk),
        .r_bank    (rd_bank2),
        .r_addr    (rd_addr2),
        .r_pix     (fe2_pix),
        .pub_tgl   (fe2_tgl),
        .src_lines (fe2_lines)
    );
    reg [2:0] fe2_tgl_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fe2_tgl_s <= 3'd0;
        else        fe2_tgl_s <= {fe2_tgl_s[1:0], fe2_tgl};
    end
    wire fe2_tgl_d = fe2_tgl_s[2];
    reg [3:0] fe2_pub;
    reg       fe2_tgl_dd;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fe2_pub    <= 4'd0;
            fe2_tgl_dd <= 1'b0;
        end else begin
            fe2_tgl_dd <= fe2_tgl_d;
            if (fe2_tgl_d != fe2_tgl_dd) fe2_pub <= fe2_pub + 4'd1;
        end
    end
    localparam [1:0] M_W1   = 2'd0,
                     M_PRE1 = 2'd1,
                     M_O1   = 2'd2,
                     M_W2   = 2'd3;
    reg [1:0]  st;
    reg [1:0]  n1, n2;
    reg        seen1, seen2;
    reg [9:0]  xa;
    reg [2:0] vs1_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vs1_s <= 3'd0;
        else        vs1_s <= {vs1_s[1:0], vs1};
    end
    wire vs1_rise = (vs1_s[2:1] == 2'b01);
    reg        w2_pre;
    reg        last1, last2;
    reg        pf2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= M_W1;
            n1      <= 2'd0;
            n2      <= 2'd0;
            seen1   <= 1'b0;
            seen2   <= 1'b0;
            xa      <= 10'd0;
            w2_pre  <= 1'b0;
            last1   <= 1'b0;
            last2   <= 1'b0;
            pf2     <= 1'b0;
            rd_bank1<= 2'd0;
            rd_bank2<= 2'd0;
            rd_addr1<= 10'd0;
            rd_addr2<= 10'd0;
            o_en    <= 1'b0;
            o_px    <= 16'd0;
            o_fsync <= 1'b0;
            o_rows  <= 12'd0;
            o_lines <= 12'd0;
        end else begin
            o_en    <= 1'b0;
            o_fsync <= vs1_rise;
            if (vs1_rise) begin
                o_lines <= fe1_lines;
                o_rows  <= 12'd0;
            end
            case (st)
            M_W1: begin
                if (fe1_tgl != seen1) begin
                    seen1   <= fe1_tgl;
                    rd_bank1<= n1;
                    rd_addr1<= 10'd0;
                    xa      <= 10'd0;
                    st      <= M_PRE1;
                end
            end
            M_PRE1: begin
                rd_addr1 <= 10'd1;
                st <= M_O1;
            end
            M_O1: begin
                if (!last1) begin
                    o_en <= 1'b1;
                    o_px <= fe1_pix;
                    if (xa == H_NUM-1) begin
                        last1 <= 1'b1;
                        xa    <= 10'd0;
                    end else begin
                        xa      <= xa + 10'd1;
                        rd_addr1<= (xa < H_NUM-2) ? (xa + 10'd2) : (H_NUM-1);
                    end
                end else begin
                    o_en   <= 1'b0;
                    last1  <= 1'b0;
                    n1     <= n1 + 2'd1;
                    o_rows <= o_rows + 12'd1;
                    w2_pre <= 1'b0;
                    st     <= M_W2;
                end
            end
            M_W2: begin
                if (!w2_pre) begin
                    if (fe2_pub != {2'b0, n2}) begin
                        rd_bank2 <= n2;
                        rd_addr2 <= 10'd0;
                        xa       <= 10'd0;
                        w2_pre   <= 1'b1;
                    end
                end else if (!pf2) begin
                    rd_addr2 <= 10'd1;
                    pf2      <= 1'b1;
                end else if (!last2) begin
                    o_en <= 1'b1;
                    o_px <= fe2_pix;
                    if (xa == H_NUM-1) begin
                        last2 <= 1'b1;
                        xa    <= 10'd0;
                    end else begin
                        xa       <= xa + 10'd1;
                        rd_addr2 <= (xa < H_NUM-2) ? (xa + 10'd2) : (H_NUM-1);
                    end
                end else begin
                    o_en   <= 1'b0;
                    last2  <= 1'b0;
                    w2_pre <= 1'b0;
                    pf2    <= 1'b0;
                    n2     <= n2 + 2'd1;
                    o_rows <= o_rows + 12'd1;
                    st     <= M_W1;
                end
            end
            default: st <= M_W1;
            endcase
        end
    end
endmodule
module cam_line_rd #(
    parameter integer SRC_W  = 640,
    parameter integer SEG_H  = 540,
    parameter integer DST_X0 = 960,
    parameter integer DST_W  = 960
)(
    input  wire         vout_clk,
    input  wire         ddr_clk,
    input  wire         rst_n,
    input  wire         rd_fsync,
    input  wire         rd_de,
    input  wire [11:0]  rd_x,
    input  wire [11:0]  rd_y,
    input  wire         frame_bank,
    input  wire [27:0]  base_addr,
    output reg          ddr_rreq,
    output wire [27:0]  ddr_raddr,
    output wire [31:0]  ddr_rd_len,
    input  wire         ddr_rdone,
    input  wire         ddr_rdata_en,
    input  wire [255:0] ddr_rdata,
    output wire [15:0]  vout_data,
    output wire         vout_de
);
    localparam integer BEATS  = SRC_W*16/256;
    localparam integer STRIDE = SRC_W/2;
    localparam [11:0]  SW     = SRC_W;
    localparam [11:0]  SEGW   = SEG_H;
    localparam [11:0]  GEN0   = DST_X0 - 3;
    localparam [11:0]  GEN1   = DST_X0 + DST_W - 1;
    localparam [11:0]  X0     = DST_X0;
    localparam [11:0]  X1     = DST_X0 + DST_W - 1;
    localparam [31:0]  RD_LEN = BEATS;
    reg  [11:0] buf_row;
    reg         req_tgl;
    reg  [11:0] req_row;
    reg  [11:0] sx;
    reg  [12:0] acc_h;
    reg  [11:0] rd_word;
    reg  [11:0] sx_d1, sx_d2;
    reg  [15:0] pix_r;
    wire [31:0] lb_word;
    wire        seg      = (rd_y >= SEGW);
    wire [11:0] yin      = seg ? (rd_y - SEGW) : rd_y;
    wire [11:0] want_row = {yin[10:0], 1'b0} + {11'd0, seg};
    wire line_start = rd_de && (rd_x == 12'd0);
    wire gen        = rd_de && (rd_x >= GEN0) && (rd_x <= GEN1);
    wire [12:0] acc_h_n  = acc_h + SRC_W;
    wire        adv_h    = (acc_h_n >= DST_W);
    wire [11:0] slot_word = {buf_row[2:0], 9'd0};
    wire [11:0] word_addr = slot_word + {2'b0, sx[10:1]};
    always @(posedge vout_clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_row <= 12'hFFF;
            req_tgl <= 1'b0;
            req_row <= 12'd0;
            sx      <= 12'd0;
            acc_h   <= 13'd0;
            rd_word <= 12'd0;
            sx_d1   <= 12'd0;
            sx_d2   <= 12'd0;
            pix_r   <= 16'd0;
        end else begin
            pix_r <= sx_d2[0] ? lb_word[15:0] : lb_word[31:16];
            if (rd_fsync) buf_row <= 12'hFFF;
            if (line_start) begin
                sx    <= 12'd0;
                acc_h <= 13'd0;
                if (want_row != buf_row) begin
                    req_row <= want_row;
                    req_tgl <= ~req_tgl;
                    buf_row <= want_row;
                end
            end
            if (gen) begin
                rd_word <= word_addr;
                sx_d1   <= sx;
                sx_d2   <= sx_d1;
                if (sx < SW - 12'd1) begin
                    if (adv_h) begin
                        sx    <= sx + 12'd1;
                        acc_h <= acc_h_n - DST_W;
                    end else begin
                        acc_h <= acc_h_n;
                    end
                end
            end
        end
    end
    reg         req_s, req_s1;
    reg         pend;
    reg  [11:0] pend_row;
    reg         busy;
    reg  [11:0] row_d;
    reg  [8:0]  fill_addr;
    wire req_pulse = (req_s ^ req_s1);
    always @(posedge ddr_clk or negedge rst_n) begin
        if (!rst_n) begin req_s <= 1'b0; req_s1 <= 1'b0; end
        else        begin req_s <= req_tgl; req_s1 <= req_s; end
    end
    always @(posedge ddr_clk or negedge rst_n) begin
        if (!rst_n) begin
            ddr_rreq  <= 1'b0;
            pend      <= 1'b0;
            pend_row  <= 12'd0;
            busy      <= 1'b0;
            row_d     <= 12'd0;
            fill_addr <= 9'd0;
        end else begin
            ddr_rreq <= 1'b0;
            if (pend && !busy) begin
                busy      <= 1'b1;
                ddr_rreq  <= 1'b1;
                row_d     <= pend_row;
                fill_addr <= {pend_row[2:0], 6'd0};
                pend      <= 1'b0;
            end else if (ddr_rdone) begin
                busy <= 1'b0;
            end
            if (req_pulse) begin
                pend     <= 1'b1;
                pend_row <= req_row;
            end
            if (ddr_rdata_en) fill_addr <= fill_addr + 9'd1;
        end
    end
    wire [27:0] row_off  = row_d * STRIDE;
    wire [27:0] bank_off = {1'b0, frame_bank, 21'd0};
    assign ddr_raddr  = base_addr + bank_off + row_off;
    assign ddr_rd_len = RD_LEN;
    rd_fram_buf u_row_buf (
        .wr_data  (ddr_rdata    ),
        .wr_addr  (fill_addr    ),
        .wr_en    (ddr_rdata_en ),
        .wr_clk   (ddr_clk      ),
        .wr_rst   (~rst_n       ),
        .rd_addr  (rd_word      ),
        .rd_data  (lb_word      ),
        .rd_clk   (vout_clk     ),
        .rd_rst   (~rst_n       )
    );
    assign vout_data = pix_r;
    assign vout_de   = rd_de && (rd_x >= X0) && (rd_x <= X1);
endmodule
