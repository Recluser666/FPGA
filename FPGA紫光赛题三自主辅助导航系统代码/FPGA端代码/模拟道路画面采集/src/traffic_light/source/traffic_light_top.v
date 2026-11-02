module traffic_light_top #(
    parameter [11:0] H_TOTAL = 12'd2200, H_SYNC = 12'd44, H_BP = 12'd148, H_FP = 12'd88, H_ACT = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125, V_SYNC = 12'd5,  V_BP = 12'd36,  V_FP = 12'd4,  V_ACT = 12'd1080,
    parameter [11:0] BX0 = 12'd930, BX1 = 12'd990,
    parameter [11:0] BY0 = 12'd150, BY1 = 12'd210,
    parameter [1:0]  PAT_COL = 2'd0
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
wire [15:0] blob_col = (PAT_COL == 2'd0) ? 16'hF800 :
                       (PAT_COL == 2'd1) ? 16'hFFE0 : 16'h07E0;
wire [15:0] bg_col = 16'h4208;
wire in_blob = (x_act >= BX0) && (x_act <= BX1) && (y_act >= BY0) && (y_act <= BY1);
wire [15:0] pat = in_blob ? blob_col : bg_col;
wire [1:0]  tl_state;
wire        tl_valid;
wire [17:0] tl_area, tl_box;
traffic_light #(
    .ROI_X0(12'd640), .ROI_X1(12'd1279), .ROI_Y0(12'd0), .ROI_Y1(12'd359)
) u_tl (
    .clk(pix_clk), .rst_n(rstn_pix),
    .i_de(de), .i_hs(hs), .i_vs(vs), .i_rgb(pat),
    .i_xc(x_act), .i_yc(y_act),
    .o_state(tl_state), .o_valid(tl_valid),
    .o_area(tl_area), .o_box(tl_box)
);
reg [3:0] frame_cnt;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) frame_cnt <= 4'd0;
    else if (tl_valid && frame_cnt != 4'd15) frame_cnt <= frame_cnt + 4'd1;
end
wire warm_ok = (frame_cnt >= 4'd8);
reg [31:0] chk_cnt, err_cnt;
always @(posedge pix_clk or negedge rstn_pix) begin
    if (!rstn_pix) begin
        chk_cnt <= 32'd0; err_cnt <= 32'd0;
    end else if (tl_valid && warm_ok) begin
        if (!chk_cnt[31]) chk_cnt <= chk_cnt + 32'd1;
        if (tl_state !== PAT_COL)
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
        o_de <= de;  o_hs <= hs;  o_vs <= vs;
        o_r <= (pat[15:11] == 5'd0) ? 8'd0 : 8'd255;
        o_g <= (pat[10:5]  == 6'd0) ? 8'd0 : 8'd255;
        o_b <= (pat[4:0]   == 5'd0) ? 8'd0 : 8'd255;
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
