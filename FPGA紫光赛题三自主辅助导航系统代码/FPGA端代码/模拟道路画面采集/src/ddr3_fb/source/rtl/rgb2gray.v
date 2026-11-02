`timescale 1ns / 1ps
module rgb2gray #(
    parameter IN_FMT = 0,
    parameter [7:0] K_R = 8'd77,
    parameter [7:0] K_G = 8'd150,
    parameter [7:0] K_B = 8'd29
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_data,
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [7:0]  o_gray
);
    reg [7:0] kR, kG, kB;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kR <= K_R;
            kG <= K_G;
            kB <= K_B;
        end
    end
    reg [23:0] data_s0;
    reg        de_s0, hs_s0, vs_s0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_s0 <= 24'd0;
            de_s0 <= 1'b0; hs_s0 <= 1'b0; vs_s0 <= 1'b0;
        end else begin
            data_s0 <= i_data;
            de_s0   <= i_de;
            hs_s0   <= i_hs;
            vs_s0   <= i_vs;
        end
    end
    wire [7:0] r8, g8, b8;
    generate
        if (IN_FMT == 0) begin : g_565
            wire [4:0] r5 = data_s0[15:11];
            wire [5:0] g6 = data_s0[10:5];
            wire [4:0] b5 = data_s0[4:0];
            assign r8 = {r5, r5[4:2]};
            assign g8 = {g6, g6[5:4]};
            assign b8 = {b5, b5[4:2]};
        end else begin : g_888
            assign r8 = data_s0[23:16];
            assign g8 = data_s0[15:8];
            assign b8 = data_s0[7:0];
        end
    endgenerate
    reg [15:0] wr, wg, wb;
    reg        de_s1, hs_s1, vs_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr <= 16'd0; wg <= 16'd0; wb <= 16'd0;
            de_s1 <= 1'b0; hs_s1 <= 1'b0; vs_s1 <= 1'b0;
        end else begin
            wr <= r8 * kR;
            wg <= g8 * kG;
            wb <= b8 * kB;
            de_s1 <= de_s0;
            hs_s1 <= hs_s0;
            vs_s1 <= vs_s0;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_gray <= 8'd0;
            o_de   <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
        end else begin
            o_gray <= (wr + wg + wb) >> 8;
            o_de   <= de_s1;
            o_hs   <= hs_s1;
            o_vs   <= vs_s1;
        end
    end
endmodule
