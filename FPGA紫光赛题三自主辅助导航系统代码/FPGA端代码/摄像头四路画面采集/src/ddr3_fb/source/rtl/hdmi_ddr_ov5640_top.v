`timescale 1ns / 1ps
`define UD #1
`define CMOS_1
module hdmi_ddr_ov5640_top#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8
)(
	input                                sys_clk              ,
    output  [1:0]                        cmos_init_done       ,
    inout                                cmos1_scl            ,
    inout                                cmos1_sda            ,
    input                                cmos1_vsync          ,
    input                                cmos1_href           ,
    input                                cmos1_pclk           ,
    input   [7:0]                        cmos1_data           ,
    output                               cmos1_reset          ,
    inout                                cmos2_scl            ,
    inout                                cmos2_sda            ,
    input                                cmos2_vsync          ,
    input                                cmos2_href           ,
    input                                cmos2_pclk           ,
    input   [7:0]                        cmos2_data           ,
    output                               cmos2_reset          ,
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,
    output                               pix_clk                   ,
    output     reg                       vs_out                    ,
    output     reg                       hs_out                    ,
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     ,
    output     reg[7:0]                  g_out                     ,
    output     reg[7:0]                  b_out
);
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;
    parameter TH_1S = 27'd33000000;
    reg  [15:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;
    wire                        cmos_sda            ;
    wire                        cmos_vsync          ;
    wire                        cmos_href           ;
    wire                        cmos_pclk           ;
    wire   [7:0]                cmos_data           ;
    wire                        cmos_reset          ;
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    wire                        cmos1_pclk_16bit    ;
    wire[15:0]                  cmos2_d_16bit       ;
    wire                        cmos2_href_16bit    ;
    reg [7:0]                   cmos2_d_d0          ;
    reg                         cmos2_href_d0       ;
    reg                         cmos2_vsync_d0      ;
    wire                        cmos2_pclk_16bit    ;
    wire[15:0]                  o_rgb565            ;
    wire                        pclk_in_test        ;
    wire                        vs_in_test          ;
    wire                        de_in_test          ;
    wire[15:0]                  i_rgb565            ;
    wire                        de_re               ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;
    wire                        axi_awvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;
    wire                        axi_arvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   ;
    wire                        axi_rvalid                  ;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
    pll u_pll (
        .clkin1   (  sys_clk    ),
        .clkout0  (  pix_clk    ),
        .clkout1  (  cfg_clk    ),
        .clkout2  (  clk_25M    ),
        .pll_lock (  locked     )
    );
    ms72xx_ctl ms72xx_ctl(
        .clk             (  cfg_clk        ),
        .rst_n           (  rstn_out       ),
        .init_over_tx    (  init_over_tx   ),
        .init_over_rx    (  init_over_rx   ),
        .iic_tx_scl      (  iic_tx_scl     ),
        .iic_tx_sda      (  iic_tx_sda     ),
        .iic_scl         (  iic_scl        ),
        .iic_sda         (  iic_sda        )
    );
   assign    hdmi_int_led    =    init_over_tx;
    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    assign rstn_out = (rstn_1ms == 16'h2710);
    power_on_delay	power_on_delay_inst(
    	.clk_50M                 (sys_clk        ),
    	.reset_n                 (1'b1           ),
    	.camera1_rstn            (cmos1_reset    ),
    	.camera2_rstn            (cmos2_reset    ),
    	.camera_pwnd             (               ),
    	.initial_en              (initial_en     )
    );
    reg_config	coms1_reg_config(
    	.clk_25M                 (clk_25M            ),
    	.camera_rstn             (cmos1_reset        ),
    	.initial_en              (initial_en         ),
    	.i2c_sclk                (cmos1_scl          ),
    	.i2c_sdat                (cmos1_sda          ),
    	.reg_conf_done           (cmos_init_done[0]  ),
    	.reg_index               (                   ),
    	.clock_20k               (                   )
    );
    reg_config	coms2_reg_config(
    	.clk_25M                 (clk_25M            ),
    	.camera_rstn             (cmos2_reset        ),
    	.initial_en              (initial_en         ),
    	.i2c_sclk                (cmos2_scl          ),
    	.i2c_sdat                (cmos2_sda          ),
    	.reg_conf_done           (cmos_init_done[1]  ),
    	.reg_index               (                   ),
    	.clock_20k               (                   )
    );
    always@(posedge cmos1_pclk)
        begin
            cmos1_d_d0        <= cmos1_data    ;
            cmos1_href_d0     <= cmos1_href    ;
            cmos1_vsync_d0    <= cmos1_vsync   ;
        end
    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_pclk       ),
    	.rst_n          (cmos_init_done[0]),
    	.pdata_i        (cmos1_d_d0       ),
    	.de_i           (cmos1_href_d0    ),
    	.vs_i           (cmos1_vsync_d0    ),
    	.pixel_clk      (cmos1_pclk_16bit ),
    	.pdata_o        (cmos1_d_16bit    ),
    	.de_o           (cmos1_href_16bit )
    );
    always@(posedge cmos2_pclk)
        begin
            cmos2_d_d0        <= cmos2_data    ;
            cmos2_href_d0     <= cmos2_href    ;
            cmos2_vsync_d0    <= cmos2_vsync   ;
        end
    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_pclk       ),
    	.rst_n          (cmos_init_done[1]),
    	.pdata_i        (cmos2_d_d0       ),
    	.de_i           (cmos2_href_d0    ),
    	.vs_i           (cmos2_vsync_d0    ),
    	.pixel_clk      (cmos2_pclk_16bit ),
    	.pdata_o        (cmos2_d_16bit    ),
    	.de_o           (cmos2_href_16bit )
    );
`ifdef CMOS_1
assign     pclk_in_test    =    cmos1_pclk_16bit    ;
assign     vs_in_test      =    cmos1_vsync_d0      ;
assign     de_in_test      =    cmos1_href_16bit    ;
assign     i_rgb565        =    {cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};
`elsif CMOS_2
assign     pclk_in_test    =    cmos2_pclk_16bit    ;
assign     vs_in_test      =    cmos2_vsync_d0      ;
assign     de_in_test      =    cmos2_href_16bit    ;
assign     i_rgb565        =    {cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};
`endif
    fram_buf fram_buf(
        .ddr_clk        (  core_clk             ),
        .ddr_rstn       (  ddr_init_done        ),
        .vin_clk        (  pclk_in_test         ),
        .wr_fsync       (  vs_in_test           ),
        .wr_en          (  de_in_test           ),
        .wr_data        (  i_rgb565             ),
        .vout_clk       (  pix_clk              ),
        .rd_fsync       (  vs_o               ),
        .rd_en          (  de_re                ),
        .vout_de        (  de_o               ),
        .vout_data      (  o_rgb565             ),
        .init_done      (  init_done            ),
        .axi_awaddr     (  axi_awaddr           ),
        .axi_awid       (  axi_awuser_id        ),
        .axi_awlen      (  axi_awlen            ),
        .axi_awsize     (                       ),
        .axi_awburst    (                       ),
        .axi_awready    (  axi_awready          ),
        .axi_awvalid    (  axi_awvalid          ),
        .axi_wdata      (  axi_wdata            ),
        .axi_wstrb      (  axi_wstrb            ),
        .axi_wlast      (  axi_wusero_last      ),
        .axi_wvalid     (                       ),
        .axi_wready     (  axi_wready           ),
        .axi_bid        (  4'd0                 ),
        .axi_araddr     (  axi_araddr           ),
        .axi_arid       (  axi_aruser_id        ),
        .axi_arlen      (  axi_arlen            ),
        .axi_arsize     (                       ),
        .axi_arburst    (                       ),
        .axi_arvalid    (  axi_arvalid          ),
        .axi_arready    (  axi_arready          ),
        .axi_rready     (                       ),
        .axi_rdata      (  axi_rdata            ),
        .axi_rvalid     (  axi_rvalid           ),
        .axi_rlast      (  axi_rlast            ),
        .axi_rid        (  axi_rid              )
    );
     always@(posedge pix_clk) begin
        r_out<={o_rgb565[15:11],3'b0   };
        g_out<={o_rgb565[10:5],2'b0    };
        b_out<={o_rgb565[4:0],3'b0     };
        vs_out<=vs_o;
        hs_out<=hs_o;
        de_out<=de_o;
     end
     sync_vg sync_vg(
        .clk            (  pix_clk              ),
        .rstn           (  init_done            ),
        .vs_out         (  vs_o                 ),
        .hs_out         (  hs_o                 ),
        .de_out         (                       ),
        .de_re          (  de_re                )
    );
        DDR3_50H u_DDR3_50H (
             .ref_clk                   (sys_clk            ),
             .resetn                    (rstn_out           ),
             .ddr_init_done             (ddr_init_done      ),
             .ddrphy_clkin              (core_clk           ),
             .pll_lock                  (pll_lock           ),
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
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end
     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
endmodule
