`timescale 1ns / 1ps
`define UD #1
module rd_buf #(
    parameter                     ADDR_WIDTH      = 6'd27,
    parameter                     ADDR_OFFSET     = 32'h0000_0000,
    parameter                     H_NUM           = 12'd1920,
    parameter                     V_NUM           = 12'd1080,
    parameter                     DQ_WIDTH        = 12'd32,
    parameter                     LEN_WIDTH       = 12'd16,
    parameter                     PIX_WIDTH       = 12'd24,
    parameter                     LINE_ADDR_WIDTH = 16'd19,
    parameter                     FRAME_CNT_WIDTH = 16'd8,
    parameter                     EXT_FRAME_SEL   = 1'b0
)  (
    input                         ddr_clk,
    input                         ddr_rstn,
    input                         ext_frame_sel,
    input                         vout_clk,
    input                         rd_fsync,
input                         rd_hs,
input                         line_lock_en,
input                         rd_parity_inv,
input                         rc_lag,
input                         ring_sel,
input                         wr_frame_bit,
input                         rd_en,
    output                        vout_de,
    output [PIX_WIDTH- 1'b1 : 0]  vout_data,
output [11:0]                 dbg_rd_addr,
output [11:0]                 dbg_rd_base,
output [15:0]                 dbg_rd_hs_cnt,
output [15:0]                 dbg_line_had_cnt,
output [15:0]                 dbg_lock_fire_cnt,
output [15:0]                 dbg_wr_step_bad,
output [15:0]                 dbg_ddr_rdone_cnt,
output [15:0]                 dbg_wr_line_snap,
output [8:0]                  dbg_wr_addr_snap,
    input                         init_done,
    output                        ddr_rreq,
    output [ADDR_WIDTH- 1'b1 : 0] ddr_raddr,
    output [LEN_WIDTH- 1'b1 : 0]  ddr_rd_len,
    input                         ddr_rrdy,
    input                         ddr_rdone,
    input [8*DQ_WIDTH- 1'b1 : 0]  ddr_rdata,
    input                         ddr_rdata_en
);
    localparam SIM            = 1'b0;
    localparam RAM_WIDTH      = 16'd32;
    localparam DDR_DATA_WIDTH = DQ_WIDTH * 8;
    localparam WR_LINE_NUM    = H_NUM * PIX_WIDTH/RAM_WIDTH;
    localparam RD_LINE_NUM    = WR_LINE_NUM * RAM_WIDTH/DDR_DATA_WIDTH;
    localparam DDR_ADDR_OFFSET= RD_LINE_NUM*DDR_DATA_WIDTH/DQ_WIDTH;
    reg       rd_fsync_1d;
    reg       rd_en_1d,rd_en_2d;
    wire      rd_rst;
    reg       ddr_rstn_1d,ddr_rstn_2d;
    always @(posedge vout_clk)
    begin
        rd_fsync_1d <= rd_fsync;
        rd_en_1d <= rd_en;
        rd_en_2d <= rd_en_1d;
        ddr_rstn_1d <= ddr_rstn;
        ddr_rstn_2d <= ddr_rstn_1d;
    end
    assign rd_rst = ~rd_fsync_1d &rd_fsync;
    reg      wr_fsync_1d,wr_fsync_2d,wr_fsync_3d;
    wire     wr_rst;
    reg      wr_en_1d,wr_en_2d,wr_en_3d;
    reg      wr_trig;
    reg [11:0] wr_line;
    always @(posedge ddr_clk)
    begin
        wr_fsync_1d <= rd_fsync;
        wr_fsync_2d <= wr_fsync_1d;
        wr_fsync_3d <= wr_fsync_2d;
        wr_en_1d <= rd_en;
        wr_en_2d <= wr_en_1d;
        wr_en_3d <= wr_en_2d;
        wr_trig <= wr_rst || (~wr_en_3d && wr_en_2d && wr_line != V_NUM);
    end
    always @(posedge ddr_clk)
    begin
        if(wr_rst || (~ddr_rstn))
            wr_line <= 12'd1;
        else if(wr_trig)
            wr_line <= wr_line + 12'd1;
    end
    assign wr_rst = ~wr_fsync_3d && wr_fsync_2d;
    reg [FRAME_CNT_WIDTH - 1'b1 :0] wr_frame_cnt=0;
    always @(posedge ddr_clk)
    begin
        if(wr_rst)
            wr_frame_cnt <= wr_frame_cnt + 1'b1;
        else
            wr_frame_cnt <= wr_frame_cnt;
    end
    reg [LINE_ADDR_WIDTH - 1'b1 :0] wr_cnt;
    always @(posedge ddr_clk)
    begin
        if(wr_rst)
            wr_cnt <= 9'd0;
        else if(ddr_rdone)
            wr_cnt <= wr_cnt + DDR_ADDR_OFFSET;
        else
            wr_cnt <= wr_cnt;
    end
    assign ddr_rreq = wr_trig;
reg [1:0] rd_parity_sync;
always @(posedge ddr_clk or negedge ddr_rstn)
begin
if(!ddr_rstn) rd_parity_sync <= 2'd0;
else rd_parity_sync <= {rd_parity_sync[0], rd_parity_inv};
end
wire rd_parity = rd_parity_sync[1];
reg  rd_frame_bit_lat;
always @(posedge ddr_clk or negedge ddr_rstn) begin
    if (!ddr_rstn) rd_frame_bit_lat <= 1'b0;
    else if (wr_rst) rd_frame_bit_lat <= wr_frame_bit;
end
wire rd_frame_bit = (EXT_FRAME_SEL != 1'b0) ? ext_frame_sel
                   : (rd_frame_bit_lat ^ rd_parity ^ 1'b1);
assign ddr_raddr = {rd_frame_bit,wr_cnt} + ADDR_OFFSET;
    assign ddr_rd_len = RD_LINE_NUM;
    reg  [ 8:0]           wr_addr;
    reg  [11:0]           rd_addr;
    wire [RAM_WIDTH-1:0]  rd_data;
    always @(posedge ddr_clk)
    begin
        if(wr_rst)
            wr_addr <= (SIM == 1'b1) ? 9'd180 : 9'd0;
        else if(ddr_rdata_en) begin
            if(ring_d && wr_addr == 9'd479)
                wr_addr <= 9'd0;
            else
                wr_addr <= wr_addr + 9'd1;
        end
        else
            wr_addr <= wr_addr;
    end
    rd_fram_buf rd_fram_buf (
        .wr_data    (  ddr_rdata       ),
        .wr_addr    (  wr_addr         ),
        .wr_en      (  ddr_rdata_en    ),
        .wr_clk     (  ddr_clk         ),
        .wr_rst     (  ~ddr_rstn       ),
        .rd_addr    (  rd_addr         ),
        .rd_data    (  rd_data         ),
        .rd_clk     (  vout_clk        ),
        .rd_rst     (  ~ddr_rstn_2d    )
    );
    reg [1:0] rd_cnt;
    wire      read_en;
reg [11:0] rd_base;
reg        line_had_de;
reg        rd_hs_d;
wire       rd_hs_rise = rd_hs & ~rd_hs_d;
wire [12:0] rd_ring      = ring_v ? 13'd3840 : 13'd4096;
wire [12:0] rd_base_plus = {1'b0, rd_base} + 13'd960;
wire [11:0] rd_base_nxt  = (rd_base_plus >= rd_ring) ? (rd_base_plus - rd_ring) : rd_base_plus[11:0];
reg [1:0]  line_lock_sync;
wire       line_lock = line_lock_sync[1];
reg [1:0]  rc_lag_sync;
wire       rc_lag_pix = rc_lag_sync[1];
reg [1:0]  ring_sel_v;
reg [1:0]  ring_sel_d;
wire       ring_v = ring_sel_v[1];
wire       ring_d = ring_sel_d[1];
reg [15:0] rd_hs_cnt;
reg [15:0] line_had_cnt;
reg [15:0] lock_fire_cnt;
reg [15:0] wr_step_bad;
reg [15:0] ddr_rdone_cnt;
reg [15:0] wr_line_snap;
reg [8:0]  wr_addr_snap;
reg [8:0]  wr_addr_prev;
wire [8:0] wr_addr_delta = wr_addr - wr_addr_prev;
always @(posedge vout_clk or negedge ddr_rstn_2d) begin
if (!ddr_rstn_2d) begin
rd_hs_cnt     <= 16'd0;
line_had_cnt  <= 16'd0;
lock_fire_cnt <= 16'd0;
end
else begin
if (rd_hs_rise)
rd_hs_cnt <= rd_hs_cnt + 16'd1;
if (rd_hs_rise && line_had_de)
line_had_cnt <= line_had_cnt + 16'd1;
if (rd_hs_rise && line_had_de && line_lock)
lock_fire_cnt <= lock_fire_cnt + 16'd1;
end
end
always @(posedge ddr_clk or negedge ddr_rstn) begin
if (!ddr_rstn) begin
wr_step_bad    <= 16'd0;
ddr_rdone_cnt  <= 16'd0;
wr_line_snap   <= 16'd0;
wr_addr_snap   <= 9'd0;
wr_addr_prev   <= 9'd0;
end
else begin
if (ddr_rdone)
ddr_rdone_cnt <= ddr_rdone_cnt + 16'd1;
if (wr_rst) begin
wr_step_bad  <= 16'd0;
wr_addr_prev <= wr_addr;
wr_line_snap  <= {4'd0, wr_line};
wr_addr_snap  <= wr_addr;
end
else if (ddr_rdone) begin
if (wr_addr_delta != 9'd120)
wr_step_bad <= wr_step_bad + 16'd1;
wr_addr_prev <= wr_addr;
end
end
end
    always @(posedge vout_clk)
    begin
        if(rd_en)
            rd_cnt <= rd_cnt + 1'b1;
        else
            rd_cnt <= 2'd0;
    end
    always @(posedge vout_clk or negedge ddr_rstn_2d)
begin
if(!ddr_rstn_2d) begin
line_lock_sync <= 2'd0;
rc_lag_sync    <= 2'd0;
ring_sel_v     <= 2'd0;
end
else begin
line_lock_sync <= {line_lock_sync[0], line_lock_en};
rc_lag_sync    <= {rc_lag_sync[0], rc_lag};
ring_sel_v     <= {ring_sel_v[0], ring_sel};
end
end
always @(posedge ddr_clk or negedge ddr_rstn)
begin
if(!ddr_rstn) ring_sel_d <= 2'd0;
else          ring_sel_d <= {ring_sel_d[0], ring_sel};
end
always @(posedge vout_clk)
begin
if(rd_rst) begin
rd_addr <= rc_lag_pix ? (ring_v ? 12'd2880 : 12'd3136) : 12'd0;
rd_base <= rc_lag_pix ? (ring_v ? 12'd2880 : 12'd3136) : 12'd0;
line_had_de <= 1'b0;
rd_hs_d <= 1'b0;
end
else begin
rd_hs_d <= rd_hs;
if (rd_en) line_had_de <= 1'b1;
if (rd_hs_rise) begin
if (line_had_de) begin
rd_base <= rd_base_nxt;
if (line_lock) rd_addr <= rd_base_nxt;
end
line_had_de <= 1'b0;
end
else if(read_en) rd_addr <= rd_addr + 1'b1;
end
end
reg [PIX_WIDTH- 1'b1 : 0] read_data;
    reg [RAM_WIDTH-1:0]       rd_data_1d;
    always @(posedge vout_clk)
    begin
        rd_data_1d <= rd_data;
    end
    generate
    if(PIX_WIDTH == 6'd24)
    begin
        assign read_en = rd_en && (rd_cnt != 2'd3);
        always @(posedge vout_clk)
        begin
            if(rd_en_1d)
            begin
                if(rd_cnt[1:0] == 2'd1)
                    read_data <= rd_data[PIX_WIDTH-1:0];
                else if(rd_cnt[1:0] == 2'd2)
                    read_data <= {rd_data[15:0],rd_data_1d[31:PIX_WIDTH]};
                else if(rd_cnt[1:0] == 2'd3)
                    read_data <= {rd_data[7:0],rd_data_1d[31:16]};
                else
                    read_data <= rd_data_1d[31:8];
            end
            else
                read_data <= 'd0;
        end
    end
    else if(PIX_WIDTH == 6'd16)
    begin
        assign read_en = rd_en && (rd_cnt[0] != 1'b1);
        always @(posedge vout_clk)
        begin
            if(rd_en_1d)
            begin
                if(rd_cnt[0])
                    read_data <= rd_data[15:0];
                else
                    read_data <= rd_data_1d[31:16];
            end
            else
                read_data <= 'd0;
        end
    end
    else
    begin
        assign read_en = rd_en;
        always @(posedge vout_clk)
        begin
            read_data <= rd_data;
        end
    end
endgenerate
    assign vout_de = rd_en_2d;
    assign vout_data = read_data;
assign dbg_rd_addr = rd_addr[11:0];
assign dbg_rd_base = {rd_base[9:0], wr_frame_bit, rd_frame_bit_lat};
assign dbg_rd_hs_cnt = rd_hs_cnt;
assign dbg_line_had_cnt = line_had_cnt;
assign dbg_lock_fire_cnt = lock_fire_cnt;
assign dbg_wr_step_bad = wr_step_bad;
assign dbg_ddr_rdone_cnt = ddr_rdone_cnt;
assign dbg_wr_line_snap = wr_line_snap;
assign dbg_wr_addr_snap = wr_addr_snap;
endmodule
