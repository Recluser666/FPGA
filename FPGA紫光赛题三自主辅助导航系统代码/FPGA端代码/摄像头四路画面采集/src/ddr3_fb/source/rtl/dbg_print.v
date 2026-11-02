`timescale 1ns / 1ps
module dbg_print #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer REFRESH_CYC = 12_500_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] i_v0,
    input  wire [31:0] i_v1,
    input  wire [31:0] i_v2,
    input  wire [31:0] i_v3,
    input  wire [31:0] i_v4,
    input  wire [31:0] i_v5,
    input  wire [31:0] i_v6,
    input  wire [31:0] i_v7,
    output reg         o_valid,
    output reg  [7:0]  o_data,
    input  wire        i_busy
);
    localparam integer REFRESH = REFRESH_CYC;
    reg [31:0] tick_cnt;
    reg        tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 32'd0;
            tick     <= 1'b0;
        end else if (tick_cnt >= REFRESH) begin
            tick_cnt <= 32'd0;
            tick     <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 32'd1;
            tick     <= 1'b0;
        end
    end
    reg [31:0] cur_val;
    always @(*) begin
        case (line_idx)
            3'd0: cur_val = i_v0;
            3'd1: cur_val = i_v1;
            3'd2: cur_val = i_v2;
            3'd3: cur_val = i_v3;
            3'd4: cur_val = i_v4;
            3'd5: cur_val = i_v5;
            3'd6: cur_val = i_v6;
            default: cur_val = i_v7;
        endcase
    end
    reg [7:0] cur_label;
    always @(*) begin
        case (line_idx)
            3'd0: cur_label = "F";
            3'd1: cur_label = "L";
            3'd2: cur_label = "R";
            3'd3: cur_label = "S";
            3'd4: cur_label = "M";
            3'd5: cur_label = "A";
            3'd6: cur_label = "T";
            default: cur_label = "X";
        endcase
    end
    reg [3:0] nib;
    reg [7:0] hex_char;
    always @(*) begin
        if (nib < 4'd10) hex_char = 8'h30 + {4'd0, nib};
        else             hex_char = 8'h41 + {4'd0, nib} - 8'd10;
    end
    reg [2:0]  line_idx;
    reg [3:0]  byte_idx;
    always @(*) begin
        case (byte_idx)
            4'd1: nib = cur_val[31:28];
            4'd2: nib = cur_val[27:24];
            4'd3: nib = cur_val[23:20];
            4'd4: nib = cur_val[19:16];
            4'd5: nib = cur_val[15:12];
            4'd6: nib = cur_val[11:8];
            4'd7: nib = cur_val[7:4];
            4'd8: nib = cur_val[3:0];
            default: nib = 4'd0;
        endcase
    end
    localparam [1:0] S_IDLE = 2'd0,
                     S_LOAD = 2'd1,
                     S_SEND = 2'd2,
                     S_WAIT = 2'd3;
    reg [1:0] st;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            line_idx <= 3'd0;
            byte_idx <= 4'd0;
            o_valid  <= 1'b0;
            o_data   <= 8'd0;
        end else begin
            case (st)
                S_IDLE: begin
                    o_valid <= 1'b0;
                    if (tick) begin
                        line_idx <= 3'd0;
                        byte_idx <= 4'd0;
                        st       <= S_LOAD;
                    end
                end
                S_LOAD: begin
                    o_valid <= 1'b0;
                    o_data  <= (byte_idx == 4'd0)      ? cur_label :
                               (byte_idx <= 4'd8)      ? hex_char  :
                               (byte_idx == 4'd9)      ? 8'h0D    : 8'h0A;
                    st      <= S_SEND;
                end
                S_SEND: begin
                    o_valid <= 1'b1;
                    if (i_busy) begin
                        o_valid <= 1'b0;
                        if (byte_idx == 4'd10) begin
                            byte_idx <= 4'd0;
                            if (line_idx == 3'd7) begin
                                line_idx <= 3'd0;
                                st       <= S_IDLE;
                            end else begin
                                line_idx <= line_idx + 3'd1;
                                st       <= S_WAIT;
                            end
                        end else begin
                            byte_idx <= byte_idx + 4'd1;
                            st       <= S_WAIT;
                        end
                    end
                end
                S_WAIT: begin
                    o_valid <= 1'b0;
                    if (!i_busy) st <= S_LOAD;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
