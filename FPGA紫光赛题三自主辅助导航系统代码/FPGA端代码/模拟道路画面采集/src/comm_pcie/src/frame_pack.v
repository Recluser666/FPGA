`timescale 1ns / 1ps
module frame_pack #(
    parameter [7:0] TYPE       = 8'h01,
    parameter       HDR_LEN    = 5'd14
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [15:0] frame_id,
    input  wire [15:0] width,
    input  wire [15:0] height,
    input  wire [31:0] ts_us,
    input  wire [7:0]  flag,
    output reg         payload_rd,
    input  wire [7:0]  payload_data,
    input  wire        payload_valid,
    output reg  [7:0]  out_data,
    output reg         out_valid,
    output reg         out_sof,
    output reg         out_eof,
    output reg         busy,
    output reg         done,
    output wire  [15:0] crc_out
);
    localparam S_IDLE  = 3'd0,
               S_HDR   = 3'd1,
               S_REQ   = 3'd2,
               S_EMIT  = 3'd3,
               S_CRCW  = 3'd4,
               S_CRC   = 3'd5,
               S_CRC2  = 3'd6,
               S_DONE  = 3'd7;
    reg [2:0]  st;
    reg [4:0]  hdr_idx;
    reg [31:0] px_left;
    reg        crc_init, crc_valid;
    reg  [7:0] crc_data;
    crc16_ccitt u_crc (
        .clk   (clk),
        .rst_n (rst_n),
        .init  (crc_init),
        .valid (crc_valid),
        .data  (crc_data),
        .crc   (crc_out)
    );
    function [7:0] hdr_byte;
        input [4:0] idx;
        begin
            case (idx)
                5'd0 : hdr_byte = 8'hA5;
                5'd1 : hdr_byte = 8'h5A;
                5'd2 : hdr_byte = TYPE;
                5'd3 : hdr_byte = flag;
                5'd4 : hdr_byte = frame_id[7:0];
                5'd5 : hdr_byte = frame_id[15:8];
                5'd6 : hdr_byte = width[7:0];
                5'd7 : hdr_byte = width[15:8];
                5'd8 : hdr_byte = height[7:0];
                5'd9 : hdr_byte = height[15:8];
                5'd10: hdr_byte = ts_us[7:0];
                5'd11: hdr_byte = ts_us[15:8];
                5'd12: hdr_byte = ts_us[23:16];
                default: hdr_byte = ts_us[31:24];
            endcase
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            hdr_idx    <= 5'd0;
            px_left    <= 32'd0;
            out_data   <= 8'h00;
            out_valid  <= 1'b0;
            out_sof    <= 1'b0;
            out_eof    <= 1'b0;
            payload_rd <= 1'b0;
            busy       <= 1'b0;
            done       <= 1'b0;
            crc_init   <= 1'b0;
            crc_valid  <= 1'b0;
            crc_data   <= 8'h00;
        end else begin
            out_valid  <= 1'b0;
            out_sof    <= 1'b0;
            out_eof    <= 1'b0;
            done       <= 1'b0;
            crc_init   <= 1'b0;
            crc_valid  <= 1'b0;
            case (st)
                S_IDLE: begin
                    payload_rd <= 1'b0;
                    if (start) begin
                        px_left <= {16'b0, width} * {16'b0, height};
                        hdr_idx <= 5'd0;
                        busy    <= 1'b1;
                        st      <= S_HDR;
                    end
                end
                S_HDR: begin
                    if (hdr_idx == HDR_LEN) begin
                        st <= (px_left == 32'd0) ? S_CRCW : S_REQ;
                    end else begin
                        out_data <= hdr_byte(hdr_idx);
                        out_valid<= 1'b1;
                        out_sof  <= (hdr_idx == 5'd0);
                        if (hdr_idx == 5'd0) begin
                            crc_init <= 1'b1;
                        end else if (hdr_idx >= 5'd2) begin
                            crc_valid <= 1'b1;
                            crc_data  <= hdr_byte(hdr_idx);
                        end
                        hdr_idx <= hdr_idx + 5'd1;
                    end
                end
                S_REQ: begin
                    payload_rd <= 1'b1;
                    st         <= S_EMIT;
                end
                S_EMIT: begin
                    payload_rd <= 1'b0;
                    if (payload_valid) begin
                        out_data  <= payload_data;
                        out_valid <= 1'b1;
                        crc_valid <= 1'b1;
                        crc_data  <= payload_data;
                        px_left   <= px_left - 32'd1;
                        st        <= (px_left == 32'd1) ? S_CRCW : S_REQ;
                    end
                end
                S_CRCW: begin
                    st <= S_CRC;
                end
                S_CRC: begin
                    out_data  <= crc_out[7:0];
                    out_valid <= 1'b1;
                    st        <= S_CRC2;
                end
                S_CRC2: begin
                    out_data  <= crc_out[15:8];
                    out_valid <= 1'b1;
                    out_eof   <= 1'b1;
                    st        <= S_DONE;
                end
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
