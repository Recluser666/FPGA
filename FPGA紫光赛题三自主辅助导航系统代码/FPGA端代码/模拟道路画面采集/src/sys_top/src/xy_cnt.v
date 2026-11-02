`timescale 1ns / 1ps
module xy_cnt #(
    parameter [11:0] XMAX = 12'd1920,
    parameter [11:0] YMAX = 12'd1080
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_vs,
    output reg  [11:0] qx,
    output reg  [11:0] qy
);
    reg vs_d, de_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qx   <= 12'd0;
            qy   <= 12'd0;
            vs_d <= 1'b0;
            de_d <= 1'b0;
        end else begin
            vs_d <= i_vs;
            de_d <= i_de;
            if (i_vs & ~vs_d) begin
                qx <= 12'd0;
                qy <= 12'd0;
            end else if (~i_de & de_d) begin
                qx <= 12'd0;
                qy <= qy + 12'd1;
            end else if (i_de) begin
                qx <= qx + 12'd1;
            end
        end
    end
endmodule
