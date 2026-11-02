`timescale 1ns / 1ps
module gauss3x3 #(
    parameter H_ACT = 1920,
    parameter V_ACT = 1080,
    parameter AW    = 11
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [7:0]  i_gray,
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [7:0]  o_gray,
    output reg  [11:0] o_xc,
    output reg  [11:0] o_yc
);
    wire [7:0]  p00,p01,p02,p10,p11,p12,p20,p21,p22;
    wire        w_de, w_hs, w_vs;
    wire [11:0] w_xc, w_yc;
    line3x3 #(
        .H_ACT (H_ACT), .V_ACT (V_ACT), .AW (AW), .BORDER (4)
    ) u_win (
        .clk   (clk),    .rst_n (rst_n),
        .i_de  (i_de),   .i_hs  (i_hs),   .i_vs (i_vs),   .i_pix (i_gray),
        .o_p00 (p00), .o_p01 (p01), .o_p02 (p02),
        .o_p10 (p10), .o_p11 (p11), .o_p12 (p12),
        .o_p20 (p20), .o_p21 (p21), .o_p22 (p22),
        .o_de  (w_de),   .o_hs  (w_hs),   .o_vs (w_vs),
        .o_xc  (w_xc),   .o_yc  (w_yc)
    );
    reg [9:0] a0, a1, a2, b0, b1, b2;
    reg       de_a, hs_a, vs_a;
    reg [11:0] xc_a, yc_a;
    always @(posedge clk) begin
        a0 <= {1'b0, p00} + {1'b0, p02};
        a1 <= {1'b0, p10} + {1'b0, p12};
        a2 <= {1'b0, p20} + {1'b0, p22};
        b0 <= {1'b0, p01} << 1;
        b1 <= {1'b0, p11} << 1;
        b2 <= {1'b0, p21} << 1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin de_a <= 1'b0; hs_a <= 1'b0; vs_a <= 1'b0; xc_a <= 12'd0; yc_a <= 12'd0; end
        else begin
            de_a <= w_de;  hs_a <= w_hs;  vs_a <= w_vs;
            xc_a <= w_xc;  yc_a <= w_yc;
        end
    end
    reg [10:0] h0, h1, h2;
    reg        de_b, hs_b, vs_b;
    reg [11:0] xc_b, yc_b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h0 <= 11'd0; h1 <= 11'd0; h2 <= 11'd0;
            de_b <= 1'b0; hs_b <= 1'b0; vs_b <= 1'b0;
            xc_b <= 12'd0; yc_b <= 12'd0;
        end else begin
            h0 <= {1'b0, a0} + {1'b0, b0};
            h1 <= {1'b0, a1} + {1'b0, b1};
            h2 <= {1'b0, a2} + {1'b0, b2};
            de_b <= de_a;  hs_b <= hs_a;  vs_b <= vs_a;
            xc_b <= xc_a;  yc_b <= yc_a;
        end
    end
    wire [12:0] sum = {2'b0, h0} + {2'b0, h2} + ({2'b0, h1} << 1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_gray <= 8'd0; o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
            o_xc <= 12'd0;  o_yc <= 12'd0;
        end else begin
            o_gray <= sum[12:4];
            o_de   <= de_b;
            o_hs   <= hs_b;
            o_vs   <= vs_b;
            o_xc   <= xc_b;
            o_yc   <= yc_b;
        end
    end
endmodule
