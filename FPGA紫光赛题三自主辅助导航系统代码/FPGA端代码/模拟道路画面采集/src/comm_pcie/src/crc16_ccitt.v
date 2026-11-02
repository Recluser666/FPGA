`timescale 1ns / 1ps
module crc16_ccitt (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init,
    input  wire        valid,
    input  wire [7:0]  data,
    output reg  [15:0] crc
);
    wire [15:0] mixed = crc ^ {data, 8'h00};
    reg  [15:0] next;
    integer     i;
    always @(*) begin
        next = mixed;
        for (i = 0; i < 8; i = i + 1) begin
            next = next[15] ? ((next << 1) ^ 16'h1021) : (next << 1);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc <= 16'hFFFF;
        end else if (init) begin
            crc <= 16'hFFFF;
        end else if (valid) begin
            crc <= next;
        end
    end
endmodule
