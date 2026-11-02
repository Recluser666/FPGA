`timescale 1ns / 1ps
module activity_stat #(
    parameter [25:0] SLOT_DIV = 26'h3F_FFFF
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [8:0]  act,
    input  wire [35:0] st,
    output reg  [3:0]  led
);
    reg [19:0] c0, c1, c2, c3, c4, c5, c6, c7, c8;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c0 <= 20'd0; c1 <= 20'd0; c2 <= 20'd0; c3 <= 20'd0; c4 <= 20'd0;
            c5 <= 20'd0; c6 <= 20'd0; c7 <= 20'd0; c8 <= 20'd0;
        end else begin
            if (act[0]) c0 <= c0 + 20'd1;
            if (act[1]) c1 <= c1 + 20'd1;
            if (act[2]) c2 <= c2 + 20'd1;
            if (act[3]) c3 <= c3 + 20'd1;
            if (act[4]) c4 <= c4 + 20'd1;
            if (act[5]) c5 <= c5 + 20'd1;
            if (act[6]) c6 <= c6 + 20'd1;
            if (act[7]) c7 <= c7 + 20'd1;
            if (act[8]) c8 <= c8 + 20'd1;
        end
    end
    reg [3:0]  slot;
    reg [25:0] div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div  <= 26'd0;
            slot <= 4'd0;
        end else if (div == SLOT_DIV) begin
            div  <= 26'd0;
            slot <= (slot == 4'd8) ? 4'd0 : slot + 4'd1;
        end else begin
            div <= div + 26'd1;
        end
    end
    reg [19:0] csel;
    always @(*) begin
        case (slot)
            4'd0:    csel = c0;
            4'd1:    csel = c1;
            4'd2:    csel = c2;
            4'd3:    csel = c3;
            4'd4:    csel = c4;
            4'd5:    csel = c5;
            4'd6:    csel = c6;
            4'd7:    csel = c7;
            default: csel = c8;
        endcase
    end
    wire [3:0] st_sel = st[{slot, 2'b00} +: 4];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) led <= 4'd0;
        else        led <= csel[19:16] ^ st_sel;
    end
endmodule
