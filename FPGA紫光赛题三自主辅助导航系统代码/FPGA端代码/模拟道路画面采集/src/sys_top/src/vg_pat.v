`timescale 1ns / 1ps
module vg_pat #(
    parameter [11:0] H_ACT = 12'd1920,
    parameter [11:0] V_ACT = 12'd1080,
    parameter [1:0]  MODE  = 2'd0
)(
    input  wire [11:0] x,
    input  wire [11:0] y,
    input  wire        de,
    output reg  [23:0] rgb
);
    reg [1:0] bar;
    always @(*) begin
        if (x < H_ACT/8*1)       bar = 2'd0;
        else if (x < H_ACT/8*2)  bar = 2'd1;
        else if (x < H_ACT/8*3)  bar = 2'd2;
        else                     bar = 2'd3;
    end
    wire [23:0] bars = (bar == 2'd0) ? 24'hFFFFFF :
                       (bar == 2'd1) ? 24'hFFFF00 :
                       (bar == 2'd2) ? 24'h00FFFF :
                                       24'hFF0000 ;
    wire [23:0] bars8 = { x[10:8] == 3'd0 ? 8'hFF : 8'h00,
                          x[10:8] == 3'd2 ? 8'hFF : (x[10:8] == 3'd3 ? 8'hFF : 8'h00),
                          x[10:8] == 3'd4 ? 8'hFF : (x[10:8] == 3'd6 ? 8'hFF : 8'h00) };
    wire chk = x[6] ^ y[6];
    wire [23:0] checker = chk ? 24'hFF2020 : 24'hF0F0F0;
    wire [7:0] ramp = x[9:2];
    wire       mark = (x[11:0] > H_ACT/2 - 12'd8) && (x[11:0] < H_ACT/2 + 12'd8) && (y[4:0] < 5'd16);
    wire [23:0] ramp_rgb = mark ? 24'hFFFFFF : {ramp, ramp, ramp};
    always @(*) begin
        case (MODE)
            2'd0 : rgb = bars8;
            2'd1 : rgb = checker;
            default: rgb = ramp_rgb;
        endcase
    end
endmodule
