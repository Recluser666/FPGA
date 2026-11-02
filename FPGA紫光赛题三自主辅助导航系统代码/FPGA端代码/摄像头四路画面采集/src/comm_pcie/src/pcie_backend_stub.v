`timescale 1ns / 1ps
module pcie_backend_stub (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_valid,
    input  wire [7:0]  i_data,
    output wire        o_ready,
    output reg  [31:0] byte_cnt,
    output reg  [31:0] frame_cnt,
    output reg         got_eof
);
    assign o_ready = 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt  <= 32'd0;
            frame_cnt <= 32'd0;
            got_eof   <= 1'b0;
        end else begin
            got_eof <= 1'b0;
            if (i_valid) byte_cnt  <= byte_cnt + 32'd1;
            if (got_eof) frame_cnt <= frame_cnt + 32'd1;
        end
    end
endmodule
