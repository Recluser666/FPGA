`timescale 1ns / 1ps
module scene_gen (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [11:0] x,
    input  wire [11:0] y,
    input  wire [1:0]  tl_color,
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [23:0] o_rgb888,
    output wire [15:0] o_rgb565,
    output reg  [11:0] o_x,
    output reg  [11:0] o_y
);
    wire [11:0] up   = 12'd1079 - y;
    wire [11:0] lc   = 12'd700 + {2'b0, up[11:2]};
    wire [11:0] rc   = 12'd1220 - {2'b0, up[11:2]};
    wire        lline = (x >= lc) && (x < (lc + 12'd16)) && (y >= 12'd592);
    wire        rline = (x >= rc) && (x < (rc + 12'd16)) && (y >= 12'd592);
    wire        zebra = (y >= 12'd860) && (y < 12'd960) && (x[5] == 1'b0);
    wire        tlbox = (x >= 12'd900) && (x < 12'd1020) &&
                        (y >= 12'd120) && (y < 12'd300);
    wire        road  = (y >= 12'd592);
    always @(*) begin
        if (tlbox) begin
            case (tl_color)
                2'd0:    rgb_now = 24'hFF_30_30;
                2'd1:    rgb_now = 24'hFF_FF_30;
                default: rgb_now = 24'h30_FF_30;
            endcase
        end else if (lline || rline || zebra) begin
            rgb_now = 24'hFF_FF_FF;
        end else if (road) begin
            rgb_now = 24'h40_40_40;
        end else begin
            rgb_now = 24'h18_18_20;
        end
    end
    reg [23:0] rgb_now;
    assign o_rgb565 = {o_rgb888[23:19], o_rgb888[15:10], o_rgb888[7:3]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
            o_rgb888 <= 24'd0; o_x <= 12'd0; o_y <= 12'd0;
        end else begin
            o_de     <= i_de;
            o_hs     <= i_hs;
            o_vs     <= i_vs;
            o_rgb888 <= rgb_now;
            o_x      <= x;
            o_y      <= y;
        end
    end
endmodule
