module sobel #(parameter H_ACT=1920, V_ACT=1080, AW=11, GH_W=8, SCALE_SHIFT=3)
(
    input  clk, rst_n,
    input  i_de, i_hs, i_vs,
    input [7:0] i_gray,
    output reg o_de, o_hs, o_vs,
    output reg [GH_W-1:0] o_grad,
    output reg [11:0] o_xc, o_yc
);
wire [7:0]  p00,p01,p02,p10,p11,p12,p20,p21,p22;
wire        w_de, w_hs, w_vs;
wire [11:0] w_xc, w_yc;
reg [10:0] s_l, s_r, h_t, h_d;
reg        de1, hs1, vs1;
reg [11:0] xc1, yc1;
reg [10:0] gx_abs, gy_abs;
reg        de2, hs2, vs2;
reg [11:0] xc2, yc2;
line3x3 #(
    .H_ACT  (H_ACT),
    .V_ACT  (V_ACT),
    .AW     (AW),
    .BORDER (4)
) u_win (
    .clk   (clk),
    .rst_n (rst_n),
    .i_de  (i_de),
    .i_hs  (i_hs),
    .i_vs  (i_vs),
    .i_pix (i_gray),
    .o_p00 (p00), .o_p01 (p01), .o_p02 (p02),
    .o_p10 (p10), .o_p11 (p11), .o_p12 (p12),
    .o_p20 (p20), .o_p21 (p21), .o_p22 (p22),
    .o_de  (w_de), .o_hs (w_hs), .o_vs (w_vs),
    .o_xc  (w_xc), .o_yc (w_yc)
);
always @(posedge clk)
begin
    s_l <= ({1'b0,p02} + {1'b0,p22}) + ({1'b0,p12} << 1);
    s_r <= ({1'b0,p00} + {1'b0,p20}) + ({1'b0,p10} << 1);
    h_t <= ({1'b0,p00} + {1'b0,p02}) + ({1'b0,p01} << 1);
    h_d <= ({1'b0,p20} + {1'b0,p22}) + ({1'b0,p21} << 1);
end
always @(posedge clk or negedge rst_n)
begin
    if (!rst_n)
    begin
     de1<=1'b0; hs1<=1'b0; vs1<=1'b0; xc1<=12'd0; yc1<=12'd0;
    end
    else
    begin
     de1<=w_de; hs1<=w_hs; vs1<=w_vs; xc1<=w_xc; yc1<=w_yc;
    end
end
always @(posedge clk)
begin
    gx_abs <= (s_r>s_l) ? (s_r-s_l) : (s_l-s_r);
    gy_abs <= (h_d>h_t) ? (h_d-h_t) : (h_t-h_d);
end
always @(posedge clk or negedge rst_n)
 begin
    if (!rst_n)
     begin
     de2<=1'b0; hs2<=1'b0; vs2<=1'b0; xc2<=12'd0; yc2<=12'd0;
     end
    else
     begin de2<=de1; hs2<=hs1; vs2<=vs1; xc2<=xc1; yc2<=yc1;
     end
end
wire [11:0] mag    = {1'b0, gx_abs} + {1'b0, gy_abs};
wire [11:0] scaled = mag >> SCALE_SHIFT;
wire [7:0]  grad   = (scaled > 12'd255) ? 8'd255 : scaled[7:0];
always @(posedge clk or negedge rst_n)
 begin
    if (!rst_n)
     begin
        o_grad <= {GH_W{1'b0}};
        o_de<=1'b0; o_hs<=1'b0; o_vs<=1'b0; o_xc<=12'd0; o_yc<=12'd0;
     end
else
     begin
        o_grad <= grad[GH_W-1:0];
        o_de   <= de2;
        o_hs   <= hs2;
        o_vs   <= vs2;
        o_xc   <= xc2;
        o_yc   <= yc2;
    end
end
endmodule
