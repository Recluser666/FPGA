`timescale 1ns / 1ps
module comm_link #(
    parameter [7:0]  TYPE    = 8'h01,
    parameter        HDR_LEN = 5'd14,
    parameter [8:0]  FIFO_AW = 9
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [15:0] frame_id,
    input  wire [15:0] width,
    input  wire [15:0] height,
    input  wire [31:0] ts_us,
    input  wire [7:0]  flag,
    output wire        payload_rd,
    input  wire [7:0]  payload_data,
    input  wire        payload_valid,
    output wire        busy,
    output wire        done,
    output wire [15:0] crc_out,
    input  wire        bus_clk,
    input  wire        rstn_bus,
    output wire        o_valid,
    output wire [7:0]  o_data,
    input  wire        i_ready,
    output wire        o_full,
    output wire        o_empty,
    output wire [FIFO_AW:0] o_count
);
    wire [7:0] fp_data;
    wire       fp_valid, fp_sof, fp_eof;
    frame_pack #(.TYPE(TYPE), .HDR_LEN(HDR_LEN)) u_frame_pack (
        .clk           (clk          ),
        .rst_n         (rst_n        ),
        .start         (start        ),
        .frame_id      (frame_id     ),
        .width         (width        ),
        .height        (height       ),
        .ts_us         (ts_us        ),
        .flag          (flag         ),
        .payload_rd    (payload_rd   ),
        .payload_data  (payload_data ),
        .payload_valid (payload_valid),
        .out_data      (fp_data      ),
        .out_valid     (fp_valid     ),
        .out_sof       (fp_sof       ),
        .out_eof       (fp_eof       ),
        .busy          (busy         ),
        .done          (done         ),
        .crc_out       (crc_out      )
    );
    wire [7:0]       sf_data;
    wire             sf_full, sf_empty;
    wire [FIFO_AW:0] sf_count;
    wire [7:0] af_data;
    wire       af_full, af_empty;
    wire af_accept = ~sf_empty & ~af_full;
    wire af_pop    = ~af_empty & i_ready;
    sync_fifo #(.DW(8), .AW(FIFO_AW)) u_sync_fifo (
        .clk     (clk      ),
        .rst_n   (rst_n    ),
        .wr_en   (fp_valid ),
        .wr_data (fp_data  ),
        .rd_en   (af_accept),
        .rd_data (sf_data  ),
        .full    (sf_full  ),
        .empty   (sf_empty ),
        .count   (sf_count )
    );
    async_fifo #(.DW(8), .AW(FIFO_AW)) u_async_fifo (
        .wclk   (clk      ),
        .wrst_n (rst_n    ),
        .winc   (af_accept),
        .wdata  (sf_data  ),
        .wfull  (af_full  ),
        .rclk   (bus_clk  ),
        .rrst_n (rstn_bus ),
        .rinc   (af_pop   ),
        .rdata  (af_data  ),
        .rempty (af_empty )
    );
    assign o_valid = ~af_empty;
    assign o_data  = af_data;
    assign o_full  = af_full;
    assign o_empty = af_empty;
    assign o_count = {1'b0, sf_count[FIFO_AW:1]};
endmodule
