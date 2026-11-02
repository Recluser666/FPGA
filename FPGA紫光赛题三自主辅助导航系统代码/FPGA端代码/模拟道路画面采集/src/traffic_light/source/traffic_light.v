module traffic_light #(
    parameter [11:0] ROI_X0 = 12'd640, ROI_X1 = 12'd1279,
    parameter [11:0] ROI_Y0 = 12'd0,   ROI_Y1 = 12'd359,
    parameter [7:0]  R_RMIN = 8'd180, R_GMAX = 8'd90,  R_BMAX = 8'd90,
    parameter [7:0]  G_GMIN = 8'd170, G_RMAX = 8'd110, G_BMAX = 8'd110,
    parameter [7:0]  Y_RMIN = 8'd180, Y_GMIN = 8'd150, Y_BMAX = 8'd110,
    parameter [17:0] MIN_AREA = 18'd20
)(
    input  wire        clk, rst_n,
    input  wire        i_de, i_hs, i_vs,
    input  wire [15:0] i_rgb,
    input  wire [11:0] i_xc, i_yc,
    output reg  [1:0]  o_state,
    output reg         o_valid,
    output reg  [17:0] o_area,
    output reg  [17:0] o_box
);
reg  vs_d;
wire vs_rise = i_vs & ~vs_d;
wire [7:0] r8 = {i_rgb[15:11], i_rgb[13:11]};
wire [7:0] g8 = {i_rgb[10:5],  i_rgb[6:5]};
wire [7:0] b8 = {i_rgb[4:0],   i_rgb[2:0]};
wire in_roi = (i_xc >= ROI_X0) && (i_xc <= ROI_X1) &&
              (i_yc >= ROI_Y0) && (i_yc <= ROI_Y1);
wire is_red = (r8 >= R_RMIN) && (g8 <= R_GMAX) && (b8 <= R_BMAX);
wire is_grn = (g8 >= G_GMIN) && (r8 <= G_RMAX) && (b8 <= G_BMAX);
wire is_yel = (r8 >= Y_RMIN) && (g8 >= Y_GMIN) && (b8 <= Y_BMAX);
wire is_col = is_red | is_grn | is_yel;
wire cnt_en = i_de & in_roi & is_col;
reg [17:0] red_cnt, grn_cnt, yel_cnt, col_cnt;
reg [11:0] bxmin, bxmax, bymin, bymax;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) vs_d <= 1'b0;
    else        vs_d <= i_vs;
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        red_cnt <= 18'd0; grn_cnt <= 18'd0; yel_cnt <= 18'd0; col_cnt <= 18'd0;
        bxmin <= 12'd4095; bxmax <= 12'd0; bymin <= 12'd4095; bymax <= 12'd0;
    end else if (vs_rise) begin
        red_cnt <= 18'd0; grn_cnt <= 18'd0; yel_cnt <= 18'd0; col_cnt <= 18'd0;
        bxmin <= 12'd4095; bxmax <= 12'd0; bymin <= 12'd4095; bymax <= 12'd0;
    end else if (cnt_en) begin
        col_cnt <= col_cnt + 18'd1;
        if (is_red) red_cnt <= red_cnt + 18'd1;
        if (is_grn) grn_cnt <= grn_cnt + 18'd1;
        if (is_yel) yel_cnt <= yel_cnt + 18'd1;
        if (i_xc < bxmin) bxmin <= i_xc;
        if (i_xc > bxmax) bxmax <= i_xc;
        if (i_yc < bymin) bymin <= i_yc;
        if (i_yc > bymax) bymax <= i_yc;
    end
end
reg [17:0] red_h, grn_h, yel_h, col_h;
reg [11:0] bxmin_h, bxmax_h, bymin_h, bymax_h;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        red_h <= 18'd0; grn_h <= 18'd0; yel_h <= 18'd0; col_h <= 18'd0;
        bxmin_h <= 12'd0; bxmax_h <= 12'd0; bymin_h <= 12'd0; bymax_h <= 12'd0;
    end else if (vs_rise) begin
        red_h   <= red_cnt;   grn_h   <= grn_cnt;   yel_h   <= yel_cnt;   col_h <= col_cnt;
        bxmin_h <= bxmin;     bxmax_h <= bxmax;
        bymin_h <= bymin;     bymax_h <= bymax;
    end
end
wire [11:0] box_w = bxmax_h - bxmin_h + 12'd1;
wire [11:0] box_h = bymax_h - bymin_h + 12'd1;
reg [11:0] box_w_r, box_h_r;
always @(posedge clk) begin box_w_r <= box_w; box_h_r <= box_h; end
reg [23:0] box_a_r;
always @(posedge clk) box_a_r <= box_w_r * box_h_r;
reg [23:0] box3_r, area5_r;
reg [17:0] maxc_r;
always @(posedge clk) begin
    box3_r  <= box_a_r + (box_a_r << 1);
    area5_r <= ({6'd0, col_h} << 2) + {6'd0, col_h};
    maxc_r  <= (red_h >= grn_h && red_h >= yel_h) ? red_h :
               (grn_h >= yel_h) ? grn_h : yel_h;
end
reg        area_ok_r;
reg [17:0] rc_r, gc_r, yc_r;
always @(posedge clk) begin
    area_ok_r <= (area5_r > box3_r) && (box_a_r != 24'd0);
    rc_r <= red_h; gc_r <= grn_h; yc_r <= yel_h;
end
reg [1:0] fst_r;
always @(posedge clk) begin
    fst_r <= ((rc_r == maxc_r) && (rc_r >= MIN_AREA) && area_ok_r) ? 2'd0 :
             ((yc_r == maxc_r) && (yc_r >= MIN_AREA) && area_ok_r) ? 2'd1 :
             ((gc_r == maxc_r) && (gc_r >= MIN_AREA) && area_ok_r) ? 2'd2 : 2'd3;
end
reg [5:0] vr;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) vr <= 6'd0;
    else        vr <= {vr[4:0], vs_rise};
end
reg [1:0] h0, h1, h2, h3, h4;
wire same5 = (h0 == h1) && (h1 == h2) && (h2 == h3) && (h3 == h4);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        h0 <= 2'd3; h1 <= 2'd3; h2 <= 2'd3; h3 <= 2'd3; h4 <= 2'd3;
        o_state <= 2'd3; o_valid <= 1'b0;
        o_area <= 18'd0; o_box <= 18'd0;
    end else begin
        o_valid <= vr[5];
        if (vr[5]) begin
            h4 <= h3; h3 <= h2; h2 <= h1; h1 <= h0; h0 <= fst_r;
            if (same5) o_state <= h0;
            o_area <= rc_r;
            o_box  <= box_a_r[17:0];
        end
    end
end
endmodule
