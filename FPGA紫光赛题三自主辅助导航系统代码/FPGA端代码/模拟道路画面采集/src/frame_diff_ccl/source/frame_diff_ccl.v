module frame_diff_ccl #(
    parameter [5:0]  DS_VOTE  = 6'd16,
    parameter [15:0] MIN_AREA = 16'd6
)(
    input  wire        clk, rst_n,
    input  wire        i_de, i_hs, i_vs,
    input  wire        i_bin,
    input  wire [11:0] i_xc, i_yc,
    output reg         o_box_valid,
    output reg  [11:0] o_x0, o_x1, o_y0, o_y1,
    output reg  [15:0] o_cnt
);
localparam [7:0] DS_COLS = 8'd239;
localparam [7:0] DS_ROWS = 8'd134;
reg  vs_d;
wire vs_rise = i_vs & ~vs_d;
wire blk_last = (i_xc[2:0] == 3'd7);
wire blk_done = i_de & blk_last;
reg [5:0] blk_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)          blk_cnt <= 6'd0;
    else if (!i_de)      blk_cnt <= 6'd0;
    else if (blk_last)   blk_cnt <= 6'd0;
    else                 blk_cnt <= blk_cnt + (i_bin ? 6'd1 : 6'd0);
end
wire [6:0] blk_vote = {1'b0, blk_cnt} + {6'd0, i_bin};
wire       ds_bin   = (blk_vote >= {1'b0, DS_VOTE});
reg [14:0] ds_addr;
reg [7:0]  ds_col, ds_row;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin ds_addr <= 15'd0; ds_col <= 8'd0; ds_row <= 8'd0; end
    else if (vs_rise) begin ds_addr <= 15'd0; ds_col <= 8'd0; ds_row <= 8'd0; end
    else if (blk_done) begin
        ds_addr <= ds_addr + 15'd1;
        if (ds_col == DS_COLS) begin ds_col <= 8'd0; ds_row <= ds_row + 8'd1; end
        else                         ds_col <= ds_col + 8'd1;
    end
end
reg ds_m0 [0:32767];
reg ds_m1 [0:32767];
reg sel;
reg wr_en_d;
reg [14:0] wr_addr_d;
reg ds_bin_d1, ds_bin_d2, ds_prev;
reg [7:0] dv_col1, dv_col2, dv_row1, dv_row2;
reg diff_v1, diff_v2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sel <= 1'b0;
    else if (vs_rise) sel <= ~sel;
end
always @(posedge clk) begin
    wr_en_d   <= blk_done;
    wr_addr_d <= ds_addr;
    ds_bin_d1 <= ds_bin;
end
always @(posedge clk) begin
    if (wr_en_d) begin
        if (!sel) begin ds_m0[wr_addr_d] <= ds_bin_d1; ds_prev <= ds_m1[wr_addr_d]; end
        else      begin ds_m1[wr_addr_d] <= ds_bin_d1; ds_prev <= ds_m0[wr_addr_d]; end
    end
    ds_bin_d2 <= ds_bin_d1;
    dv_col1 <= ds_col;   dv_row1 <= ds_row;
    dv_col2 <= dv_col1;  dv_row2 <= dv_row1;
    diff_v1 <= blk_done;
    diff_v2 <= diff_v1;
end
wire [7:0] dv_col = dv_col2;
wire [7:0] dv_row = dv_row2;
wire       diff_v = diff_v2;
wire diff = ds_bin_d2 ^ ds_prev;
wire hit  = diff_v & diff;
reg [11:0] row_xmin, row_xmax;
reg [15:0] row_cnt;
reg        row_any;
wire [11:0] row_xmin_f = hit ? ((!row_any || ({4'd0, dv_col} < row_xmin)) ? {4'd0, dv_col} : row_xmin) : row_xmin;
wire [11:0] row_xmax_f = hit ? (({4'd0, dv_col} > row_xmax) ? {4'd0, dv_col} : row_xmax) : row_xmax;
wire [15:0] row_cnt_f  = row_cnt + (hit ? 16'd1 : 16'd0);
wire        row_any_f  = row_any | hit;
wire        row_end    = diff_v & (dv_col == DS_COLS);
reg [11:0] rowf_xmin, rowf_xmax;
reg [15:0] rowf_cnt;
reg        rowf_any, row_end_r;
reg [7:0]  dv_row_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row_xmin <= 12'd0; row_xmax <= 12'd0; row_cnt <= 16'd0; row_any <= 1'b0;
        rowf_xmin <= 12'd0; rowf_xmax <= 12'd0; rowf_cnt <= 16'd0; rowf_any <= 1'b0;
        row_end_r <= 1'b0; dv_row_r <= 8'd0;
    end else begin
        rowf_xmin <= row_xmin_f; rowf_xmax <= row_xmax_f;
        rowf_cnt  <= row_cnt_f;  rowf_any  <= row_any_f;
        row_end_r <= row_end;
        dv_row_r  <= dv_row;
        if (row_end) begin
            row_xmin <= 12'd0; row_xmax <= 12'd0; row_cnt <= 16'd0; row_any <= 1'b0;
        end else begin
            row_xmin <= row_xmin_f; row_xmax <= row_xmax_f;
            row_cnt  <= row_cnt_f;  row_any  <= row_any_f;
        end
    end
end
reg [11:0] band_xmin, band_xmax, band_ymin, band_ymax;
reg [15:0] band_cnt;
reg        band_any;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        band_xmin <= 12'd0; band_xmax <= 12'd0; band_ymin <= 12'd0; band_ymax <= 12'd0;
        band_cnt <= 16'd0; band_any <= 1'b0;
        o_box_valid <= 1'b0; o_x0 <= 12'd0; o_x1 <= 12'd0; o_y0 <= 12'd0; o_y1 <= 12'd0; o_cnt <= 16'd0;
    end else begin
        o_box_valid <= 1'b0;
        if (row_end_r) begin
            if (rowf_any) begin
                if (!band_any) band_ymin <= {4'd0, dv_row_r};
                if (!band_any || (rowf_xmin < band_xmin)) band_xmin <= rowf_xmin;
                if (rowf_xmax > band_xmax) band_xmax <= rowf_xmax;
                band_ymax <= {4'd0, dv_row_r};
                band_cnt  <= band_cnt + rowf_cnt;
                band_any  <= 1'b1;
            end else if (band_any) begin
                if (band_cnt >= MIN_AREA) begin
                    o_box_valid <= 1'b1;
                    o_x0 <= band_xmin << 3;
                    o_x1 <= (band_xmax << 3) + 12'd7;
                    o_y0 <= band_ymin << 3;
                    o_y1 <= (band_ymax << 3) + 12'd7;
                    o_cnt <= band_cnt;
                end
                band_any <= 1'b0; band_cnt <= 16'd0; band_xmax <= 12'd0; band_xmin <= 12'd0;
            end
        end
        if (vs_rise) begin
            if (band_any && (band_cnt >= MIN_AREA)) begin
                o_box_valid <= 1'b1;
                o_x0 <= band_xmin << 3;
                o_x1 <= (band_xmax << 3) + 12'd7;
                o_y0 <= band_ymin << 3;
                o_y1 <= (band_ymax << 3) + 12'd7;
                o_cnt <= band_cnt;
            end
            band_any <= 1'b0; band_cnt <= 16'd0;
        end
    end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) vs_d <= 1'b0;
    else        vs_d <= i_vs;
end
endmodule
