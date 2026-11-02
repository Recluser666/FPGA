`timescale 1ns / 1ps
module top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter [1:0]  STAGE   = 2'd1
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
    inout  wire        iic_tx_sda,
    input  wire        pixclk_in,
    input  wire        vs_in,
    input  wire        hs_in,
    input  wire        de_in,
    input  wire [7:0]  r_in,
    input  wire [7:0]  g_in,
    input  wire [7:0]  b_in
);
    wire        pix_clk, cfg_clk, pll_lock;
    wire        vs, hs, de;
    wire [11:0] x_act, y_act;
    timing_gen #(
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_FP(H_FP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_FP(V_FP), .V_ACT(V_ACT)
    ) u_tg (
        .sys_clk  (sys_clk  ),
        .rst_n    (rst_n    ),
        .pix_clk  (pix_clk  ),
        .cfg_clk  (cfg_clk  ),
        .pll_lock (pll_lock ),
        .vs_out   (vs       ),
        .hs_out   (hs       ),
        .de_out   (de       ),
        .x_act    (x_act    ),
        .y_act    (y_act    )
    );
    reg rstn_p1, rstn_p2;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin rstn_p1 <= 1'b0; rstn_p2 <= 1'b0; end
        else        begin rstn_p1 <= 1'b1; rstn_p2 <= rstn_p1; end
    end
    wire rstn_pix = rstn_p2;
    reg [15:0] rstn_1ms;
    always @(posedge cfg_clk) begin
        if (!pll_lock)                  rstn_1ms <= 16'd0;
        else if (rstn_1ms == 16'h2710)  rstn_1ms <= rstn_1ms;
        else                            rstn_1ms <= rstn_1ms + 16'd1;
    end
    wire rstn_cfg = (rstn_1ms == 16'h2710);
    function [23:0] bar_rgb;
        input [11:0] x;
        begin
            if      (x < 12'd240 ) bar_rgb = 24'hFFFFFF;
            else if (x < 12'd480 ) bar_rgb = 24'hFFFF00;
            else if (x < 12'd720 ) bar_rgb = 24'h00FFFF;
            else if (x < 12'd960 ) bar_rgb = 24'h00FF00;
            else if (x < 12'd1200) bar_rgb = 24'hFF00FF;
            else if (x < 12'd1440) bar_rgb = 24'hFF0000;
            else if (x < 12'd1680) bar_rgb = 24'h0000FF;
            else                   bar_rgb = 24'h000000;
        end
    endfunction
    wire [23:0] src_rgb = bar_rgb(x_act);
    wire        osd_de, osd_hs, osd_vs;
    wire [23:0] osd_rgb;
    osd_overlay #(
        .BOX1_X0(12'd700), .BOX1_Y0(12'd120), .BOX1_X1(12'd1200), .BOX1_Y1(12'd520),
        .BOX1_COLOR(24'h00FF00), .BOX1_ALPHA(5'd6)
    ) u_osd (
        .clk            (pix_clk    ),
        .rst_n          (rstn_pix   ),
        .i_de           (de         ),
        .i_hs           (hs         ),
        .i_vs           (vs         ),
        .i_rgb          (src_rgb    ),
        .i_box0_en      (1'b0       ),
        .i_box0_x0      (12'd0      ),
        .i_box0_y0      (12'd0      ),
        .i_box0_x1      (12'd0      ),
        .i_box0_y1      (12'd0      ),
        .i_box0_color   (24'hFF0000 ),
        .i_box0_alpha   (5'd16      ),
        .i_box0_fill    (1'b0       ),
        .i_arrow_en     (1'b0       ),
        .i_arrow_sel    (2'd1       ),
        .i_arrow_x      (12'd150    ),
        .i_arrow_y      (12'd800    ),
        .i_arrow_color  (24'hFFFFFF ),
        .i_arrow_alpha  (5'd16      ),
        .o_de           (osd_de     ),
        .o_hs           (osd_hs     ),
        .o_vs           (osd_vs     ),
        .o_rgb          (osd_rgb    )
    );
    wire init_over;
    hdmi_tx u_tx (
        .cfg_clk    (cfg_clk    ),
        .cfg_rst_n  (rstn_cfg   ),
        .pix_clk    (pix_clk    ),
        .pix_rst_n  (rstn_pix   ),
        .i_de       (osd_de     ),
        .i_hs       (osd_hs     ),
        .i_vs       (osd_vs     ),
        .i_rgb      (osd_rgb    ),
        .o_pixclk   (pixclk_out ),
        .o_de       (de_out     ),
        .o_hs       (hs_out     ),
        .o_vs       (vs_out     ),
        .o_r        (r_out      ),
        .o_g        (g_out      ),
        .o_b        (b_out      ),
        .iic_scl    (iic_scl    ),
        .iic_sda    (iic_sda    ),
        .iic_tx_scl (iic_tx_scl ),
        .iic_tx_sda (iic_tx_sda ),
        .o_init_over(init_over  )
    );
    reg [23:0] act_cnt;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)     act_cnt <= 24'd0;
        else if (de)    act_cnt <= act_cnt + 24'd1;
    end
    assign led[0] = pll_lock;
    assign led[1] = init_over;
    assign led[2] = rstn_p1;
    assign led[3] = act_cnt[23];
endmodule
