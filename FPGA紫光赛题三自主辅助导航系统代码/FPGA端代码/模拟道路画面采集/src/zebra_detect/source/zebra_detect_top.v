module zebra_detect_top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter [11:0] ZY0 = 12'd830, ZY1 = 12'd1000,
    parameter [11:0] LX0 = 12'd600, LX1 = 12'd620,
    parameter [11:0] RX0 = 12'd1300, RX1 = 12'd1320,
    parameter [7:0]  ROAD = 8'd100, MARK = 8'd200,
    parameter        PAT_ZEBRA = 1'b1
)(
    input  wire        sys_clk,
    input  wire        rst_n,
    output wire [3:0]  led,
    output wire        pixclk_out,
    output wire        vs_out, hs_out, de_out,
    output wire [7:0]  r_out, g_out, b_out,
    output wire        iic_scl, inout wire iic_sda,
    output wire        iic_tx_scl, inout wire iic_tx_sda
);
wire pix_clk, cfg_clk, pll_lock;
pll u_pll (.clkin1(sys_clk), .clkout0(pix_clk), .clkout1(cfg_clk), .pll_lock(pll_lock));
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
    .clk(cfg_clk), .rst_n(rstn_cfg), .init_over(init_over),
    .iic_tx_scl(iic_tx_scl), .iic_tx_sda(iic_tx_sda),
    .iic_scl(iic_scl), .iic_sda(iic_sda)
);
wire        vs, hs, de;
wire [11:0] x_act, y_act;
sync_vg #(
    .X_BITS(12), .Y_BITS(12),
    .H_TOTAL(H_TOTAL), .H_SYNC(H_SYNC), .H_BP(H_BP), .H_FP(H_FP), .H_ACT(H_ACT),
    .V_TOTAL(V_TOTAL), .V_SYNC(V_SYNC), .V_BP(V_BP), .V_FP(V_FP), .V_ACT(V_ACT),
    .HV_OFFSET(12'd0)
) u_sync_vg (
    .clk(pix_clk), .rstn(rstn_pix),
    .vs_out(vs), .hs_out(hs), .de_out(de), .de_re(),
    .x_act(x_act), .y_act(y_act)
);
wire zband = (y_act >= ZY0) && (y_act <= ZY1);
wire zbar  = x_act[7];
wire in_l  = (x_act >= LX0) && (x_act <= LX1);
wire in_r  = (x_act >= RX0) && (x_act <= RX1);
wire pat_hi = PAT_ZEBRA ? (zband & zbar) : (in_l | in_r);
wire [7:0] pat = pat_hi ? MARK : ROAD;
wire        g_de, g_hs, g_vs;
wire [7:0]  g_gray;
wire [11:0] g_xc, g_yc;
gauss3x3 #(
    .H_ACT(H_ACT), .V_ACT(V_ACT), .AW(11)
) u_gauss (
    .clk(pix_clk), .rst_n(rstn_pix),
    .i_de(de), .i_hs(hs), .i_vs(vs), .i_gray(pat),
    .o_de(g_de), .o_hs(g_hs), .o_vs(g_vs), .o_gray(g_gray),
    .o_xc(g_xc), .o_yc(g_yc)
);
wire        b_bin;
wire        b_de, b_hs, b_vs;
wire [11:0] b_xc, b_yc;
wire [7:0]  b_level;
binarize #(
    .H_ACT(H_ACT), .SHIFT(5'd11), .K_SHIFT(4'd1),
    .FLOOR_INIT(8'd20), .CAP(8'd230),
    .ROI_Y0(12'd0), .ROI_Y1(12'd1079)
) u_bin (
    .clk(pix_clk), .rst_n(rstn_pix),
    .i_de(g_de), .i_hs(g_hs), .i_vs(g_vs), .i_gray(g_gray),
    .i_xc(g_xc), .i_yc(g_yc),
    .i_floor(8'd20), .i_floor_we(1'b0),
    .o_de(b_de), .o_hs(b_hs), .o_vs(b_vs),
    .o_bin(b_bin), .o_xc(b_xc), .o_yc(b_yc), .o_level(b_level)
);
wire        z_zebra, z_valid;
wire [9:0]  z_runmax;
wire [5:0]  z_ntrmax;
zebra_detect #(
    .ROI_Y0(12'd810), .ROI_Y1(12'd1079),
    .NTR_MIN(6'd8), .NTR_MAX(6'd48), .RUN_MIN(10'd5)
) u_zebra (
    .clk(pix_clk), .rst_n(rstn_pix),
    .i_de(b_de), .i_hs(b_hs), .i_vs(b_vs), .i_bin(b_bin),
    .i_xc(b_xc), .i_yc(b_yc),
    .o_zebra(z_zebra), .o_valid(z_valid),
    .o_runmax(z_runmax), .o_ntrmax(z_ntrmax)
);
reg [3:0] frame_cnt;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) frame_cnt <= 4'd0;
    else if (z_valid && frame_cnt != 4'd15) frame_cnt <= frame_cnt + 4'd1;
end
wire warm_ok = (frame_cnt >= 4'd6);
reg [31:0] chk_cnt, err_cnt;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) begin
        chk_cnt <= 32'd0; err_cnt <= 32'd0;
    end else if (z_valid && warm_ok) begin
        if (!chk_cnt[31]) chk_cnt <= chk_cnt + 32'd1;
        if (z_zebra !== PAT_ZEBRA)
            if (!err_cnt[31]) err_cnt <= err_cnt + 32'd1;
    end
end
reg [7:0] o_r, o_g, o_b;
reg       o_de, o_hs, o_vs;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) begin
        o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
        o_r <= 8'd0; o_g <= 8'd0; o_b <= 8'd0;
    end else begin
        o_de <= b_de;  o_hs <= b_hs;  o_vs <= b_vs;
        o_r <= b_bin ? 8'd255 : 8'd0;
        o_g <= b_bin ? 8'd255 : 8'd0;
        o_b <= b_bin ? 8'd255 : 8'd0;
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
