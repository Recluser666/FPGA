module pcie_axis_top #(
    parameter DEBUG_ONE_TLP = 0,
    parameter USE_FIXED_ADDR = 0
)(
    input  wire                 button_rst_n,
    input  wire                 perst_n,
    input  wire                 free_clk,
    output wire                 txd,
    input  wire                 rxd,
    input  wire                 ref_clk_n,
    input  wire                 ref_clk_p,
    input  wire [1:0]           rxn,
    input  wire [1:0]           rxp,
    output wire [1:0]           txn,
    output wire [1:0]           txp,
    output wire                  ref_led,
    output wire                  pclk_led,
    output wire                  pclk_div2_led,
    output wire                 smlh_link_up,
    output wire                 rdlh_link_up,
    input  wire                 vin_clk,
    input  wire                 vin_de,
    input  wire                 vin_hs,
    input  wire                 vin_vs,
    input  wire [15:0]          vin_rgb565           ,
    input  wire [11:0]          dbg_rd_addr          ,
    input  wire [11:0]          dbg_rd_base          ,
    input  wire [15:0]          dbg_rd_hs_cnt        ,
    input  wire [15:0]          dbg_line_had_cnt     ,
    input  wire [15:0]          dbg_lock_fire_cnt    ,
    input  wire [15:0]          dbg_wr_step_bad      ,
    input  wire [15:0]          dbg_ddr_rdone_cnt    ,
    input  wire [15:0]          dbg_wr_line_snap     ,
    input  wire [8:0]           dbg_wr_addr_snap     ,
    input  wire [11:0]          dbg_px_bad_snap      ,
    input  wire [11:0]          dbg_hd_line_snap     ,
    input  wire [15:0]          dbg_x_glitch_snap    ,
    input  wire [1:0]       lane_l,
    input  wire [1:0]       lane_r,
    output wire [255:0]     dbg_bus,
    output wire [319:0]     dbg_diag,
    output wire             line_lock_en,
    output wire             rd_align_en,
    output wire             rd_parity_inv,
    output wire             rc_lag,
    output wire             xclr_robust,
    output wire             waddr_lock,
    output wire             rdp_fallback
);
    wire            pclk;
    wire            pclk_div2;
    wire            ref_clk_out;
    wire            core_rst_n;
    wire sync_button_rst_n;
    wire sync_perst_n;
    hsst_rst_cross_sync_v1_0 #(
        .RST_CNTR_VALUE (16'hC000)
    ) u_buttonrstn_debounce (
        .clk      (ref_clk_out      ),
        .rstn_in  (button_rst_n     ),
        .rstn_out (sync_button_rst_n)
    );
    hsst_rst_cross_sync_v1_0 #(
        .RST_CNTR_VALUE (16'hC000)
    ) u_perstn_debounce (
        .clk      (ref_clk_out      ),
        .rstn_in  (perst_n          ),
        .rstn_out (sync_perst_n     )
    );
    wire            m_tvalid, m_tready, m_tlast;
    wire [127:0]    m_tdata;
    wire [3:0]      m_tkeep;
    wire [7:0]      m_tuser;
    wire            s0_tready, s0_tvalid, s0_tlast, s0_tuser;
    wire [127:0]    s0_tdata;
    wire [7:0]      cfg_pbus_num;
    wire [4:0]      cfg_pbus_dev_num;
    wire            cfg_bus_master_en;
    wire [2:0]      cfg_max_payload_size;
    wire            cfg_mem_space_en;
    wire [2:0]      cfg_max_rd_req_size;
    wire            rbar_ctrl_update;
    wire [4:0]      ltssm_state;
    wire [132:0]    debug_info_mux;
    pcie_test u_pcie (
        .free_clk                   (free_clk               ),
        .pclk                       (pclk                   ),
        .pclk_div2                  (pclk_div2              ),
        .ref_clk                    (ref_clk_out            ),
        .ref_clk_n                  (ref_clk_n              ),
        .ref_clk_p                  (ref_clk_p              ),
        .button_rst_n               (sync_button_rst_n      ),
        .power_up_rst_n             (sync_perst_n           ),
        .perst_n                    (sync_perst_n           ),
        .core_rst_n                 (core_rst_n             ),
        .p_sel                      (1'b0                   ),
        .p_strb                     (4'd0                   ),
        .p_addr                     (16'd0                  ),
        .p_wdata                    (32'd0                  ),
        .p_ce                       (1'b0                   ),
        .p_we                       (1'b0                   ),
        .rxn                        (rxn                    ),
        .rxp                        (rxp                    ),
        .txn                        (txn                    ),
        .txp                        (txp                    ),
        .pcs_nearend_loop           (2'd0                   ),
        .pma_nearend_ploop          (2'd0                   ),
        .pma_nearend_sloop          (2'd0                   ),
        .axis_master_tvalid         (m_tvalid               ),
        .axis_master_tready         (m_tready               ),
        .axis_master_tdata          (m_tdata                ),
        .axis_master_tkeep          (m_tkeep                ),
        .axis_master_tlast          (m_tlast                ),
        .axis_master_tuser          (m_tuser                ),
        .axis_slave0_tready         (s0_tready              ),
        .axis_slave0_tvalid         (s0_tvalid              ),
        .axis_slave0_tdata          (s0_tdata               ),
        .axis_slave0_tlast          (s0_tlast               ),
        .axis_slave0_tuser          (s0_tuser               ),
        .axis_slave1_tvalid         (1'b0                   ),
        .axis_slave1_tdata          (128'd0                 ),
        .axis_slave1_tlast          (1'b0                   ),
        .axis_slave1_tuser          (1'b0                   ),
        .axis_slave2_tvalid         (1'b0                   ),
        .axis_slave2_tdata          (128'd0                 ),
        .axis_slave2_tlast          (1'b0                   ),
        .axis_slave2_tuser          (1'b0                   ),
        .diag_ctrl_bus              (2'd0                   ),
        .app_ras_des_sd_hold_ltssm  (1'b0                   ),
        .app_ras_des_tba_ctrl       (2'd0                   ),
        .dyn_debug_info_sel         (4'd0                   ),
        .debug_info_mux             (debug_info_mux         ),
        .cfg_pbus_num               (cfg_pbus_num           ),
        .cfg_pbus_dev_num           (cfg_pbus_dev_num       ),
        .cfg_bus_master_en          (cfg_bus_master_en      ),
        .cfg_max_payload_size       (cfg_max_payload_size   ),
        .cfg_mem_space_en           (cfg_mem_space_en       ),
        .cfg_max_rd_req_size        (cfg_max_rd_req_size    ),
        .rbar_ctrl_update           (rbar_ctrl_update       ),
        .smlh_link_up               (smlh_link_up           ),
        .rdlh_link_up               (rdlh_link_up           ),
        .smlh_ltssm_state           (ltssm_state            )
    );
    assign txd = 1'b1;
    wire            fifo_full;
    wire [127:0]    fifo_rd_data;
    wire [12:0]     fifo_rd_level;
    wire            fifo_rd_empty;
    wire            fifo_rd_en;
    wire [16:0]     dbg_dma_cnt;
    wire            test_en;
    reg [1:0]  test_en_pix_sync;
    always @(posedge vin_clk or negedge core_rst_n) begin
        if (!core_rst_n) test_en_pix_sync <= 2'd0;
        else             test_en_pix_sync <= {test_en_pix_sync[0], test_en};
    end
    wire test_en_pix = test_en_pix_sync[1];
    reg [10:0] gen_x;
    reg        vin_de_d;
    wire       de_rise = vin_de & ~vin_de_d;
    always @(posedge vin_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            gen_x   <= 11'd0;
            vin_de_d<= 1'b0;
        end
        else begin
            vin_de_d <= vin_de;
            if (vin_vs)          gen_x <= 11'd0;
            else if (vin_de)     gen_x <= de_rise ? 11'd1 : gen_x + 11'd1;
        end
    end
    wire [10:0] gen_x_out = de_rise ? 11'd0 : gen_x;
    wire [15:0] gen_rgb565 = {5'd31, gen_x_out[10:5], gen_x_out[4:0]};
    wire [15:0] pcie_vin_rgb565 = test_en_pix ? gen_rgb565 : vin_rgb565;
    wire        pcie_vin_de     = vin_de;
    reg [2:0] vs_sync_pd2;
    always @(posedge pclk_div2 or negedge core_rst_n) begin
        if (!core_rst_n) vs_sync_pd2 <= 3'd0;
        else             vs_sync_pd2 <= {vs_sync_pd2[1:0], vin_vs};
    end
    wire vs_pd2 = vs_sync_pd2[2];
    pix2tlp_fifo #(.AW(6)) u_pix2tlp (
        .wclk   (vin_clk                     ),
        .wrst   (vin_vs | ~core_rst_n        ),
        .wen    (pcie_vin_de                 ),
        .wdata  (pcie_vin_rgb565             ),
        .wfull  (fifo_full                   ),
        .rclk   (pclk_div2                   ),
        .rrst   (vs_pd2 | ~core_rst_n),
        .ren    (fifo_rd_en                  ),
        .rdata  (fifo_rd_data                ),
        .rlevel (fifo_rd_level               ),
        .rempty (fifo_rd_empty               )
    );
    pcie_dma_ctrl #(
        .DEBUG_ONE_TLP      (DEBUG_ONE_TLP       ),
        .USE_FIXED_ADDR     (USE_FIXED_ADDR      )
    ) u_pcie_dma (
        .clk                (pclk_div2           ),
        .rstn               (core_rst_n          ),
        .axis_master_tvalid (m_tvalid            ),
        .axis_master_tready (m_tready            ),
        .axis_master_tdata  (m_tdata             ),
        .axis_master_tkeep  (m_tkeep             ),
        .axis_master_tlast  (m_tlast             ),
        .axis_master_tuser  (m_tuser             ),
        .ep_bus_num         (cfg_pbus_num        ),
        .ep_dev_num         (cfg_pbus_dev_num    ),
        .cfg_mem_space_en   (cfg_mem_space_en    ),
        .cfg_bus_master_en  (cfg_bus_master_en   ),
        .rbar_ctrl_update   (rbar_ctrl_update    ),
        .AXIS_S_TREADY      (s0_tready           ),
        .AXIS_S_TVALID      (s0_tvalid           ),
        .AXIS_S_TDATA       (s0_tdata            ),
        .AXIS_S_TLAST       (s0_tlast            ),
        .AXIS_S_TUSER       (s0_tuser            ),
        .o_fifo_rd_en       (fifo_rd_en          ),
        .i_fifo_rd_data     (fifo_rd_data        ),
        .i_fifo_rd_level    (fifo_rd_level       ),
        .i_fifo_rd_empty    (fifo_rd_empty       ),
        .vs_in              (vin_vs              ),
        .i_lane_l           (lane_l             ),
        .i_lane_r           (lane_r             ),
        .video_enhance_lightdown_num (           ),
        .video_enhance_lightdown_sw  (           ),
        .video_enhance_darkup_num    (           ),
        .video_enhance_darkup_sw     (           ),
        .dbg_bus                     (dbg_bus     ),
        .i_diag                      (dbg_diag    ),
        .dbg_dma_cnt                 (dbg_dma_cnt ),
        .test_en                     (test_en       ),
        .line_lock_en                (line_lock_en  ),
        .rd_align_en                 (rd_align_en   ),
        .rd_parity_inv               (rd_parity_inv ),
        .rc_lag                      (rc_lag        ),
        .xclr_robust                 (xclr_robust   ),
        .waddr_lock                  (waddr_lock    ),
        .rdp_fallback                (rdp_fallback  )
    );
    reg [26:0] ref_led_cnt, pclk_led_cnt, pclk_div2_led_cnt;
    always @(posedge free_clk or negedge core_rst_n) begin
        if (!core_rst_n) ref_led_cnt <= 27'd0;
        else             ref_led_cnt <= ref_led_cnt + 27'd1;
    end
    assign ref_led = ref_led_cnt[26];
    always @(posedge pclk or negedge core_rst_n) begin
        if (!core_rst_n) pclk_led_cnt <= 27'd0;
        else             pclk_led_cnt <= pclk_led_cnt + 27'd1;
    end
    assign pclk_led = pclk_led_cnt[26];
    always @(posedge pclk_div2 or negedge core_rst_n) begin
        if (!core_rst_n) pclk_div2_led_cnt <= 27'd0;
        else             pclk_div2_led_cnt <= pclk_div2_led_cnt + 27'd1;
    end
    assign pclk_div2_led = pclk_div2_led_cnt[26];
    wire _unused_ok = &{1'b0, cfg_bus_master_en, cfg_mem_space_en, cfg_max_rd_req_size,
                        ltssm_state, debug_info_mux[0],
                        fifo_full, fifo_rd_empty, rxd};
    pcie_diag u_pcie_diag (
        .pix_clk       (vin_clk        ),
        .pix_rst_n     (core_rst_n     ),
        .vin_de        (vin_de         ),
        .vin_hs        (vin_hs         ),
        .vin_vs        (vin_vs         ),
        .fifo_full     (fifo_full      ),
        .dbg_rd_addr   (dbg_rd_addr    ),
        .dbg_rd_base   (dbg_rd_base    ),
        .rd_align_en_i (rd_align_en    ),
        .rd_parity_i   (rd_parity_inv  ),
        .line_lock_i   (line_lock_en   ),
        .rc_lag_i      (rc_lag         ),
        .xclr_robust_i (xclr_robust    ),
        .waddr_lock_i  (waddr_lock     ),
        .rdp_fallback_i(rdp_fallback   ),
        .rd_hs_cnt     (dbg_rd_hs_cnt  ),
        .line_had_cnt  (dbg_line_had_cnt),
        .lock_fire_cnt (dbg_lock_fire_cnt),
        .wr_step_bad   (dbg_wr_step_bad  ),
        .ddr_rdone_cnt (dbg_ddr_rdone_cnt),
        .wr_line_snap  (dbg_wr_line_snap ),
        .wr_addr_snap  (dbg_wr_addr_snap ),
        .px_bad_snap   (dbg_px_bad_snap  ),
        .hd_line_snap  (dbg_hd_line_snap ),
        .x_glitch_snap (dbg_x_glitch_snap),
        .test_x        (gen_x_out      ),
        .pclk          (pclk_div2      ),
        .pclk_rst_n    (core_rst_n     ),
        .fifo_rd_en    (fifo_rd_en     ),
        .fifo_rd_empty (fifo_rd_empty  ),
        .fifo_rd_level (fifo_rd_level  ),
        .tlp_tlast     (s0_tlast       ),
        .dma_cnt       (dbg_dma_cnt    ),
        .diag          (dbg_diag       )
    );
endmodule
