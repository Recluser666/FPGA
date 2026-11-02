`timescale 1ns / 1ps
module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       i_valid,
    input  wire [7:0] i_data,
    output reg        o_tx,
    output wire       o_busy
);
    localparam integer DIV = (CLK_HZ + BAUD/2) / BAUD;
    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;
    reg [9:0]  shift;
    reg        sending;
    assign o_busy = sending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_tx     <= 1'b1;
            baud_cnt <= 16'd0;
            bit_idx  <= 4'd0;
            shift    <= 10'h3FF;
            sending  <= 1'b0;
        end else if (!sending) begin
            o_tx <= 1'b1;
            if (i_valid) begin
                shift    <= {1'b1, i_data, 1'b0};
                baud_cnt <= 16'd0;
                bit_idx  <= 4'd0;
                sending  <= 1'b1;
                o_tx     <= 1'b0;
            end
        end else begin
            if (baud_cnt == DIV - 1) begin
                baud_cnt <= 16'd0;
                if (bit_idx == 4'd9) begin
                    sending <= 1'b0;
                    o_tx    <= 1'b1;
                end else begin
                    bit_idx <= bit_idx + 4'd1;
                    o_tx    <= shift[bit_idx + 4'd1];
                end
            end else begin
                baud_cnt <= baud_cnt + 16'd1;
            end
        end
    end
endmodule
