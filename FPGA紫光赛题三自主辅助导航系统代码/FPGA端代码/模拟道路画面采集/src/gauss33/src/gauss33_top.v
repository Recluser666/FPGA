`timescale 1ns / 1ps
module gauss33_top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter [7:0]  BG   = 8'd100,
    parameter [7:0]  LINE = 8'd116,
    parameter [11:0] XL   = 12'd960,
    parameter [11:0] YL   = 12'd200
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
    output wire        iic_scl, inout wire iic_sda,
    output wire        iic_tx_scl, inout wire iic_tx_sda
);
    wire pix_clk, cfg_clk, pll_lock;
    pll u_pll (
        .clkin1 (sys_clk), .clkout0 (pix_clk), .clkout1 (cfg_clk), .pll_lock (pll_lock)
    );
    reg rstn_p1, rstn_p2;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin rstn_p1 <= 1'b0; rstn_p2 <= 1'b0; end
        else        begin rstn_p1 <= 1'b1; rstn_p2 <= rstn_p1; end
    end
    wire rstn_pix = rstn_p2;
    reg [15:0] rstn_1ms;
    always @(posedge cfg_clk) begin
        if (!pll_lock)                 rstn_1ms <= 16'd0;
        else if (rstn_1ms == 16'h2710) rstn_1ms <= rstn_1ms;
        else                           rstn_1ms <= rstn_1ms + 16'd1;
    end
    wire rstn_cfg = (rstn_1ms == 16'h2710);
    wire init_over;
    ms72xx_ctl u_cfg (
        .clk (cfg_clk), .rst_n (rstn_cfg), .init_over (init_over),
        .iic_tx_scl (iic_tx_scl), .iic_tx_sda (iic_tx_sda),
        .iic_scl (iic_scl), .iic_sda (iic_sda)
    );
    wire        vs, hs, de;
    wire [11:0] x_act, y_act;
    sync_vg #(
        .X_BITS(12), .Y_BITS(12),
        .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_FP(H_FP), .H_ACT(H_ACT),
        .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_FP(V_FP), .V_ACT(V_ACT),
        .HV_OFFSET(12'd0)
    ) u_sync_vg (
        .clk (pix_clk), .rstn (rstn_pix),
        .vs_out (vs), .hs_out (hs), .de_out (de), .de_re (),
        .x_act (x_act), .y_act (y_act)
    );
    wire [7:0] pat = ((y_act == YL) || (x_act == XL)) ? LINE : BG;
    wire        g_de, g_hs, g_vs;
    wire [7:0]  g_gray;
    wire [11:0] g_xc, g_yc;
    gauss3x3 #(
        .H_ACT (H_ACT), .V_ACT (V_ACT), .AW (11)
    ) u_gauss (
        .clk (pix_clk), .rst_n (rstn_pix),
        .i_de (de), .i_hs (hs), .i_vs (vs), .i_gray (pat),
        .o_de (g_de), .o_hs (g_hs), .o_vs (g_vs), .o_gray (g_gray),
        .o_xc (g_xc), .o_yc (g_yc)
    );
    wire v_in  = (g_xc >= XL - 12'd1) && (g_xc <= XL + 12'd1);
    wire v_mid = (g_xc == XL);
    wire h_in  = (g_yc >= YL - 12'd1) && (g_yc <= YL + 12'd1);
    wire h_mid = (g_yc == YL);
    wire [4:0] Vw = v_in ? (v_mid ? 5'd8 : 5'd4) : 5'd0;
    wire [4:0] Hw = h_in ? (h_mid ? 5'd8 : 5'd4) : 5'd0;
    reg [3:0] Iw;
    always @(*) begin
        if      (!(v_in && h_in)) Iw = 4'd0;
        else if (v_mid && h_mid)  Iw = 4'd4;
        else if (v_mid || h_mid)  Iw = 4'd2;
        else                      Iw = 4'd1;
    end
    wire [7:0] exp_g = BG + {3'b0, Vw} + {3'b0, Hw} - {4'b0, Iw};
    reg [7:0] exp_r, gray_v;
    reg       de_v;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin exp_r <= 8'd0; gray_v <= 8'd0; de_v <= 1'b0; end
        else begin exp_r <= exp_g; gray_v <= g_gray; de_v <= g_de; end
    end
    reg [31:0] chk_cnt, err_cnt;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin
            chk_cnt <= 32'd0;
            err_cnt <= 32'd0;
        end else if (de_v) begin
            if (!chk_cnt[31]) chk_cnt <= chk_cnt + 32'd1;
            if (gray_v !== exp_r)
                if (!err_cnt[31]) err_cnt <= err_cnt + 32'd1;
        end
    end
    reg [7:0] o_r, o_g, o_b;
    reg       o_de, o_hs, o_vs;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin
            o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
            o_r <= 8'd0;  o_g <= 8'd0;  o_b <= 8'd0;
        end else begin
            o_de <= g_de;  o_hs <= g_hs;  o_vs <= g_vs;
            o_r  <= g_gray;  o_g <= g_gray;  o_b <= g_gray;
        end
    end
    assign pixclk_out = pix_clk;
    assign de_out = o_de;  assign hs_out = o_hs;  assign vs_out = o_vs;
    assign r_out = o_r;    assign g_out = o_g;    assign b_out = o_b;
    assign led[0] = pll_lock;
    assign led[1] = init_over;
    assign led[2] = (err_cnt == 32'd0) && (chk_cnt != 32'd0);
    assign led[3] = (err_cnt != 32'd0);
endmodule
