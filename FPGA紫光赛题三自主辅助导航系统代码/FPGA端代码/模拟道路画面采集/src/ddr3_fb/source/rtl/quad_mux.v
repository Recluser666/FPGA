`timescale 1ns / 1ps
module quad_mux #(
    parameter [11:0] H_ACT = 12'd1920,
    parameter [11:0] V_ACT = 12'd1080
)(
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [11:0] i_x,
    input  wire [11:0] i_y,
    input  wire [23:0] s0_rgb,
    input  wire [23:0] s1_rgb,
    input  wire [23:0] s2_rgb,
    input  wire [23:0] s3_rgb,
    output wire        o_de,
    output wire        o_hs,
    output wire        o_vs,
    output wire [23:0] o_rgb,
    output wire [1:0]  o_quad
);
    wire left = (i_x < (H_ACT >> 1));
    wire top  = (i_y < (V_ACT >> 1));
    assign o_quad = {~top, ~left};
    reg [23:0] rgb_r;
    always @(*) begin
        case (o_quad)
            2'b00 : rgb_r = s0_rgb;
            2'b01 : rgb_r = s1_rgb;
            2'b10 : rgb_r = s2_rgb;
            default: rgb_r = s3_rgb;
        endcase
    end
    assign o_de  = i_de;
    assign o_hs  = i_hs;
    assign o_vs  = i_vs;
    assign o_rgb = rgb_r;
endmodule
