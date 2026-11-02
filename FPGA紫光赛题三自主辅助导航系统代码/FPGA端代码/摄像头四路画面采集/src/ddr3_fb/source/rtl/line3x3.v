`timescale 1ns / 1ps
module line3x3 #(
    parameter H_ACT  = 1920,
    parameter V_ACT  = 1080,
    parameter AW     = 11,
    parameter BORDER = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [7:0]  i_pix,
    output wire [7:0]  o_p00, o_p01, o_p02,
    output wire [7:0]  o_p10, o_p11, o_p12,
    output wire [7:0]  o_p20, o_p21, o_p22,
    output wire        o_de,
    output wire        o_hs,
    output wire        o_vs,
    output wire [11:0] o_xc,
    output wire [11:0] o_yc
);
    localparam LAT = 4;
    reg [11:0] x_cnt, y_cnt;
    reg [1:0]  row_ph;
    reg        de_d, vs_d;
    wire de_fall = de_d & ~i_de;
    wire vs_rise = i_vs & ~vs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt <= 12'd0; y_cnt <= 12'd0; row_ph <= 2'd0;
            de_d <= 1'b0; vs_d <= 1'b0;
        end else begin
            de_d <= i_de;
            vs_d <= i_vs;
            if (i_de) x_cnt <= x_cnt + 12'd1;
            else      x_cnt <= 12'd0;
            if (vs_rise) begin
                y_cnt <= 12'd0; row_ph <= 2'd0;
            end else if (de_fall) begin
                y_cnt  <= y_cnt + 12'd1;
                row_ph <= (row_ph == 2'd2) ? 2'd0 : row_ph + 2'd1;
            end
        end
    end
    reg [7:0] lb0 [0:2047];
    reg [7:0] lb1 [0:2047];
    reg [7:0] lb2 [0:2047];
    reg [7:0] q0, q1, q2;
    wire [AW-1:0] ram_addr = x_cnt[AW-1:0];
    always @(posedge clk) begin
        q0 <= lb0[ram_addr];
        q1 <= lb1[ram_addr];
        q2 <= lb2[ram_addr];
    end
    always @(posedge clk) begin
        if (i_de) begin
            case (row_ph)
                2'd0:    lb0[ram_addr] <= i_pix;
                2'd1:    lb1[ram_addr] <= i_pix;
                default: lb2[ram_addr] <= i_pix;
            endcase
        end
    end
    reg [1:0] rp1;
    reg [7:0] g1;
    reg       de1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rp1 <= 2'd0; g1 <= 8'd0; de1 <= 1'b0; end
        else begin
            rp1 <= row_ph;
            g1  <= i_pix;
            de1 <= i_de;
        end
    end
    wire [7:0] qA = (rp1 == 2'd0) ? q2 : (rp1 == 2'd1) ? q0 : q1;
    wire [7:0] qB = (rp1 == 2'd0) ? q1 : (rp1 == 2'd1) ? q2 : q0;
    reg [7:0] qA_r, qB_r, g2;
    reg       de2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin qA_r <= 8'd0; qB_r <= 8'd0; g2 <= 8'd0; de2 <= 1'b0; end
        else begin
            qA_r <= qA;   qB_r <= qB;   g2 <= g1;   de2 <= de1;
        end
    end
    wire [7:0] v0 = g2;
    wire [7:0] v1 = qA_r;
    wire [7:0] v2 = qB_r;
    reg [7:0] d1_0, d2_0;
    reg [7:0] d1_1, d2_1;
    reg [7:0] d1_2, d2_2;
    always @(posedge clk) begin
        d1_0 <= v0;  d2_0 <= d1_0;
        d1_1 <= v1;  d2_1 <= d1_1;
        d1_2 <= v2;  d2_2 <= d1_2;
    end
    assign o_p00 = v0;    assign o_p01 = d1_0;  assign o_p02 = d2_0;
    assign o_p10 = v1;    assign o_p11 = d1_1;  assign o_p12 = d2_1;
    assign o_p20 = v2;    assign o_p21 = d1_2;  assign o_p22 = d2_2;
    reg [LAT-1:0] de_sr, hs_sr, vs_sr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_sr <= {LAT{1'b0}}; hs_sr <= {LAT{1'b0}}; vs_sr <= {LAT{1'b0}};
        end else begin
            de_sr <= {de_sr[LAT-2:0], i_de};
            hs_sr <= {hs_sr[LAT-2:0], i_hs};
            vs_sr <= {vs_sr[LAT-2:0], i_vs};
        end
    end
    wire [11:0] xc = x_cnt - 12'd3;
    wire [11:0] yc = y_cnt - 12'd3;
    wire bd_ok = (xc >= BORDER) && (xc <= (H_ACT - 1 - BORDER)) &&
                 (yc >= BORDER) && (yc <= (V_ACT - 1 - BORDER));
    assign o_de = de_sr[LAT-1] & bd_ok;
    assign o_hs = hs_sr[LAT-1];
    assign o_vs = vs_sr[LAT-1];
    assign o_xc = xc;
    assign o_yc = yc;
endmodule
