`timescale 1ns / 1ps
module road_rom #(
    parameter integer ROM_WORDS = 32400,
    parameter integer ADDR_W    = 15,
    parameter         MEM_FILE  = "road_rom.mem"
)(
    input  wire                clk,
    input  wire [ADDR_W-1:0]   addr,
    output reg  [15:0]         dout
);
    reg [15:0] mem [0:ROM_WORDS-1];
    initial begin
        $readmemh(MEM_FILE, mem);
    end
    always @(posedge clk) begin
        dout <= mem[addr];
    end
endmodule
