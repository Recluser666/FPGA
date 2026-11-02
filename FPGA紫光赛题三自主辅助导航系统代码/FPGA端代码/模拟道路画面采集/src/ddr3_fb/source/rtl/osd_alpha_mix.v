`timescale 1ns / 1ps
module osd_alpha_mix #(
    parameter MIX_LATENCY = 3
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire [23:0] i_video,
    input  wire [23:0] i_osd,
    input  wire [4:0]  i_alpha,
    output reg         o_de,
    output reg  [23:0] o_rgb
);
    reg        de1;
    reg [23:0] v1, s1;
    reg [4:0]  a1;
    reg        de2;
    reg [12:0] pv_r, pv_g, pv_b;
    reg [12:0] ps_r, ps_g, ps_b;
    wire [4:0] inv_a = 5'd16 - a1;
    wire [13:0] sum_r = {1'b0, pv_r} + {1'b0, ps_r};
    wire [13:0] sum_g = {1'b0, pv_g} + {1'b0, ps_g};
    wire [13:0] sum_b = {1'b0, pv_b} + {1'b0, ps_b};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de1 <= 1'b0;  v1 <= 24'd0;  s1 <= 24'd0;  a1 <= 5'd0;
            de2 <= 1'b0;  pv_r <= 13'd0; pv_g <= 13'd0; pv_b <= 13'd0;
                          ps_r <= 13'd0; ps_g <= 13'd0; ps_b <= 13'd0;
            o_de <= 1'b0; o_rgb <= 24'd0;
        end else begin
            de1 <= i_de;
            v1  <= i_video;
            s1  <= i_osd;
            a1  <= i_alpha;
            de2  <= de1;
            pv_r <= v1[23:16] * inv_a;
            pv_g <= v1[15:8]  * inv_a;
            pv_b <= v1[7:0]   * inv_a;
            ps_r <= s1[23:16] * a1;
            ps_g <= s1[15:8]  * a1;
            ps_b <= s1[7:0]   * a1;
            o_de      <= de2;
            o_rgb[23:16] <= sum_r[11:4];
            o_rgb[15:8]  <= sum_g[11:4];
            o_rgb[7:0]   <= sum_b[11:4];
        end
    end
endmodule
