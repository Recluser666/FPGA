`timescale 1ns / 1ps
module rgb2gray_top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter [23:0] READY_CYCLES = 24'hFF_FFFF
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
    reg [15:0] rstn_1ms;
    always @(posedge cfg_clk) begin
        if (!pll_lock)                  rstn_1ms <= 16'd0;
        else if (rstn_1ms == 16'h2710)  rstn_1ms <= rstn_1ms;
        else                            rstn_1ms <= rstn_1ms + 16'd1;
    end
    wire rstn_cfg = (rstn_1ms == 16'h2710);
    wire init_over;
    ms72xx_ctl u_cfg (
        .clk        (cfg_clk    ), .rst_n (rstn_cfg), .init_over (init_over),
        .iic_tx_scl (iic_tx_scl ), .iic_tx_sda (iic_tx_sda),
        .iic_scl    (iic_scl    ), .iic_sda    (iic_sda   )
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
    wire       pat_vs, pat_hs, pat_de;
    wire [7:0] pat_r, pat_g, pat_b;
    pattern_vg #(
        .COCLOR_DEPP (8), .X_BITS (12), .Y_BITS (12), .H_ACT (H_ACT), .V_ACT (V_ACT)
    ) u_pat (
        .rstn (rstn_pix), .pix_clk (pix_clk), .act_x (x_act),
        .vs_in (vs), .hs_in (hs), .de_in (de),
        .vs_out (pat_vs), .hs_out (pat_hs), .de_out (pat_de),
        .r_out (pat_r), .g_out (pat_g), .b_out (pat_b)
    );
    wire [15:0] pat_565 = {pat_r[7:3], pat_g[7:2], pat_b[7:3]};
    wire        gray_de, gray_hs, gray_vs;
    wire [7:0]  gray_data;
    rgb2gray #(
        .IN_FMT (0),
        .K_R    (8'd77),
        .K_G    (8'd150),
        .K_B    (8'd29)
    ) u_rgb2gray (
        .clk    (pix_clk  ), .rst_n (rstn_pix),
        .i_de   (pat_de   ), .i_hs (pat_hs), .i_vs (pat_vs),
        .i_data ({8'h00, pat_565}),
        .o_de   (gray_de  ), .o_hs (gray_hs), .o_vs (gray_vs),
        .o_gray (gray_data)
    );
    wire [2:0] bar_c = (x_act < 12'd240 ) ? 3'd0 :
                       (x_act < 12'd480 ) ? 3'd1 :
                       (x_act < 12'd720 ) ? 3'd2 :
                       (x_act < 12'd960 ) ? 3'd3 :
                       (x_act < 12'd1200) ? 3'd4 :
                       (x_act < 12'd1440) ? 3'd5 :
                       (x_act < 12'd1680) ? 3'd6 : 3'd7;
    function [7:0] exp_gray;
        input [2:0] bar;
        begin
            case (bar)
                3'd0:    exp_gray = 8'd255;
                3'd1:    exp_gray = 8'd226;
                3'd2:    exp_gray = 8'd178;
                3'd3:    exp_gray = 8'd149;
                3'd4:    exp_gray = 8'd105;
                3'd5:    exp_gray = 8'd76;
                3'd6:    exp_gray = 8'd28;
                default: exp_gray = 8'd0;
            endcase
        end
    endfunction
    reg [2:0] bar_p1, bar_p2, bar_p3, bar_p4;
    reg       de_p1,  de_p2,  de_p3,  de_p4;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin
            bar_p1<=3'd0; bar_p2<=3'd0; bar_p3<=3'd0; bar_p4<=3'd0;
            de_p1<=1'b0;  de_p2<=1'b0;  de_p3<=1'b0;  de_p4<=1'b0;
        end else begin
            bar_p1 <= bar_c;  de_p1 <= de;
            bar_p2 <= bar_p1; de_p2 <= de_p1;
            bar_p3 <= bar_p2; de_p3 <= de_p2;
            bar_p4 <= bar_p3; de_p4 <= de_p3;
        end
    end
    reg [31:0] chk_cnt, err_cnt;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin
            chk_cnt <= 32'd0;
            err_cnt <= 32'd0;
        end else if (de_p4) begin
            if (!chk_cnt[31]) chk_cnt <= chk_cnt + 32'd1;
            if (gray_data !== exp_gray(bar_p4))
                if (!err_cnt[31]) err_cnt <= err_cnt + 32'd1;
        end
    end
    reg [7:0] o_r, o_g, o_b;
    reg       o_de, o_hs, o_vs;
    always @(posedge pix_clk or negedge rstn_pix) begin
        if (!rstn_pix) begin
            o_de<=1'b0; o_hs<=1'b0; o_vs<=1'b0;
            o_r<=8'd0; o_g<=8'd0; o_b<=8'd0;
        end else begin
            o_de <= gray_de;
            o_hs <= gray_hs;
            o_vs <= gray_vs;
            o_r  <= gray_data;
            o_g  <= gray_data;
            o_b  <= gray_data;
        end
    end
    assign pixclk_out = pix_clk;
    assign de_out = o_de;
    assign hs_out = o_hs;
    assign vs_out = o_vs;
    assign r_out  = o_r;
    assign g_out  = o_g;
    assign b_out  = o_b;
    reg [23:0] rdy_cnt;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)                      rdy_cnt <= 24'd0;
        else if (rdy_cnt < READY_CYCLES) rdy_cnt <= rdy_cnt + 24'd1;
    end
    assign led[0] = pll_lock;
    assign led[1] = init_over;
    assign led[2] = (err_cnt == 32'd0) && (chk_cnt != 32'd0);
    assign led[3] = (err_cnt != 32'd0);
endmodule
