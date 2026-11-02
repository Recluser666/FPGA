`timescale 1ns / 1ps
module dly_pix #(
    parameter integer W = 24,
    parameter integer N = 4
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         i_de,
    input  wire         i_hs,
    input  wire         i_vs,
    input  wire [W-1:0] i_data,
    output wire         o_de,
    output wire         o_hs,
    output wire         o_vs,
    output wire [W-1:0] o_data
);
    generate
        if (N <= 0) begin : g_bypass
            assign o_de   = i_de;
            assign o_hs   = i_hs;
            assign o_vs   = i_vs;
            assign o_data = i_data;
        end else begin : g_delay
            reg [W+2:0] sr [0:N-1];
            integer k;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    for (k = 0; k < N; k = k + 1) begin
                        sr[k] <= {(W+3){1'b0}};
                    end
                end else begin
                    sr[0] <= {i_de, i_hs, i_vs, i_data};
                    for (k = 1; k < N; k = k + 1) begin
                        sr[k] <= sr[k-1];
                    end
                end
            end
            assign {o_de, o_hs, o_vs, o_data} = sr[N-1];
        end
    endgenerate
endmodule
