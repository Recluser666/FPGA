`timescale 1ns / 1ps
module comm_pcie_top #(
    parameter [15:0] WIDTH    = 16'd16,
    parameter [15:0] HEIGHT   = 16'd8,
    parameter [15:0] FRAME_ID = 16'h0001,
    parameter [31:0] TS_US    = 32'h0000FFFF,
    parameter [7:0]  FLAG     = 8'h03,
    parameter [7:0]  TYPE_B   = 8'h01,
    parameter [15:0] CRC_GOLD = 16'hC9F5,
    parameter [23:0] GAP_MAX  = 24'hFF_FFFF
)(
    input  wire       sys_clk,
    input  wire       rst_n,
    output wire [3:0] led
);
    reg [1:0] bdiv;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) bdiv <= 2'd0; else bdiv <= bdiv + 2'd1;
    end
    wire bus_clk = bdiv[1];
    reg [23:0] gap_cnt;
    wire       start = (gap_cnt == GAP_MAX);
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)        gap_cnt <= 24'd0;
        else if (start)    gap_cnt <= 24'd0;
        else               gap_cnt <= gap_cnt + 24'd1;
    end
    reg [15:0] pl_cnt;
    reg [7:0]  pl_data_r;
    reg        pl_valid_r;
    wire [7:0] payload_data;
    wire       payload_valid;
    wire       payload_rd;
    wire [7:0] pat_next = (8'd3 + ({3'd0, pl_cnt[4:0]} * 8'd7));
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            pl_cnt     <= 16'd0;
            pl_data_r  <= 8'd0;
            pl_valid_r <= 1'b0;
        end else begin
            pl_valid_r <= payload_rd;
            if (payload_rd) begin
                pl_data_r <= pat_next;
                pl_cnt    <= pl_cnt + 16'd1;
            end
            if (start) pl_cnt <= 16'd0;
        end
    end
    assign payload_data  = pl_data_r;
    assign payload_valid = pl_valid_r;
    wire        busy, done, crc_out_v;
    wire [15:0] crc_out;
    wire        o_valid, o_full, o_empty;
    wire [7:0]  o_data;
    wire [9:0]  o_count;
    comm_link #(.TYPE(TYPE_B), .HDR_LEN(5'd14), .FIFO_AW(9)) u_link (
        .clk(sys_clk), .rst_n(rst_n),
        .start(start), .frame_id(FRAME_ID), .width(WIDTH), .height(HEIGHT),
        .ts_us(TS_US), .flag(FLAG),
        .payload_rd(payload_rd), .payload_data(payload_data), .payload_valid(payload_valid),
        .busy(busy), .done(done), .crc_out(crc_out),
        .bus_clk(bus_clk), .rstn_bus(rst_n),
        .o_valid(o_valid), .o_data(o_data), .i_ready(1'b1),
        .o_full(o_full), .o_empty(o_empty), .o_count(o_count)
    );
    assign crc_out_v = 1'b0;
    wire        be_ready;
    wire [31:0] be_byte_cnt;
    wire [31:0] be_frame_cnt;
    wire        be_got_eof;
    pcie_backend_stub u_be (
        .clk(sys_clk), .rst_n(rst_n),
        .i_valid(o_valid), .i_data(o_data), .o_ready(be_ready),
        .byte_cnt(be_byte_cnt), .frame_cnt(be_frame_cnt), .got_eof(be_got_eof)
    );
    localparam [31:0] EXP_BYTES = 32'd14 + (WIDTH * HEIGHT) + 32'd2;
    reg seen_done, crc_ok, cnt_ok, any_err;
    reg [31:0] cap_bytes;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            seen_done <= 1'b0;
            crc_ok    <= 1'b0;
            cnt_ok    <= 1'b0;
            any_err   <= 1'b0;
            cap_bytes <= 32'd0;
        end else begin
            if (done) begin
                seen_done <= 1'b1;
                if (crc_out == CRC_GOLD) crc_ok <= 1'b1; else any_err <= 1'b1;
                cap_bytes <= be_byte_cnt;
                if (be_byte_cnt == EXP_BYTES) cnt_ok <= 1'b1; else any_err <= 1'b1;
            end
        end
    end
    assign led[0] = seen_done;
    assign led[1] = crc_ok;
    assign led[2] = cnt_ok;
    assign led[3] = any_err;
endmodule
