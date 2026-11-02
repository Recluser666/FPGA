module binarize_top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter [11:0] SX0 = 12'd600, SX1 = 12'd620,
    parameter [7:0]  ROAD = 8'd100, LINE = 8'd200
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
wire stripe = (x_act >= SX0) && (x_act <= SX1);
wire [7:0] pat = stripe ? LINE : ROAD;
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
    .i_de(de), .i_hs(hs), .i_vs(vs), .i_gray(pat),
    .i_xc(x_act), .i_yc(y_act),
    .i_floor(8'd20), .i_floor_we(1'b0),
    .o_de(b_de), .o_hs(b_hs), .o_vs(b_vs),
    .o_bin(b_bin), .o_xc(b_xc), .o_yc(b_yc), .o_level(b_level)
);
reg        b_de_d;
reg [11:0] row_cnt, row_last;
wire       de_fall_b = b_de_d & ~b_de;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) begin
        b_de_d <= 1'b0; row_cnt <= 12'd0; row_last <= 12'd0;
    end else begin
        b_de_d <= b_de;
        if (b_de) row_cnt <= b_bin ? (row_cnt + 12'd1) : row_cnt;
        else      row_cnt <= 12'd0;
        if (de_fall_b) row_last <= row_cnt;
    end
end
reg [2:0] frame_cnt;
reg       b_vs_d;
wire      b_vs_rise = b_vs & ~b_vs_d;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) begin frame_cnt <= 3'd0; b_vs_d <= 1'b0; end
    else begin
        b_vs_d <= b_vs;
        if (b_vs_rise && frame_cnt != 3'd7) frame_cnt <= frame_cnt + 3'd1;
    end
end
wire warm_ok = (frame_cnt >= 3'd2);
reg [31:0] chk_cnt, err_cnt;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) begin
        chk_cnt <= 32'd0; err_cnt <= 32'd0;
    end else if (warm_ok) begin
        if (de_fall_b) begin
            if (!chk_cnt[31]) chk_cnt <= chk_cnt + 32'd1;
            if (row_last !== 12'd21)
                if (!err_cnt[31]) err_cnt <= err_cnt + 32'd1;
        end
        if (b_vs_rise) begin
            if (!chk_cnt[31]) chk_cnt <= chk_cnt + 32'd1;
            if (b_level !== 8'd94)
                if (!err_cnt[31]) err_cnt <= err_cnt + 32'd1;
        end
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
