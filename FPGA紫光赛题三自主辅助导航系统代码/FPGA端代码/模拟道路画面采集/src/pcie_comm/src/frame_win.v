`timescale 1ns / 1ps
module frame_win #(
    parameter integer IDLE_CLR = 16'd4096
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        s_valid,
    input  wire [7:0]  s_data,
    output wire        s_ready,
    output reg         usr_wr_en,
    output reg  [11:0] usr_wr_addr,
    output reg  [127:0] usr_wr_data,
    output reg  [15:0]  usr_wr_byte_en,
    output reg  [31:0] wr_word_cnt,
    output reg  [31:0] wr_byte_cnt
);
    assign s_ready = 1'b1;
    reg [3:0]   lane;
    reg [11:0]  word_addr;
    reg [119:0] sh;
    reg [15:0]  idle_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane         <= 4'd0;
            word_addr    <= 12'd0;
            sh           <= 120'd0;
            usr_wr_en    <= 1'b0;
            usr_wr_addr  <= 12'd0;
            usr_wr_data  <= 128'd0;
            usr_wr_byte_en <= 16'd0;
            idle_cnt     <= 16'd0;
            wr_word_cnt  <= 32'd0;
            wr_byte_cnt  <= 32'd0;
        end else begin
            usr_wr_en <= 1'b0;
            if (s_valid) begin
                idle_cnt    <= 16'd0;
                wr_byte_cnt <= wr_byte_cnt + 32'd1;
                if (lane == 4'd15) begin
                    usr_wr_en      <= 1'b1;
                    usr_wr_addr    <= word_addr;
                    usr_wr_data    <= {s_data, sh};
                    usr_wr_byte_en <= 16'hFFFF;
                    word_addr      <= word_addr + 12'd1;
                    wr_word_cnt    <= wr_word_cnt + 32'd1;
                    lane           <= 4'd0;
                    sh             <= 120'd0;
                end else begin
                    sh <= {s_data, sh[119:8]};
                    lane <= lane + 4'd1;
                end
            end else if (lane == 4'd0) begin
                if (idle_cnt == IDLE_CLR) begin
                    word_addr <= 12'd0;
                end else begin
                    idle_cnt <= idle_cnt + 16'd1;
                end
            end
        end
    end
endmodule
