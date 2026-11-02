module pcie_diag (
    input  wire         pix_clk,
    input  wire         pix_rst_n,
    input  wire         vin_de,
    input  wire         vin_hs,
    input  wire         vin_vs,
    input  wire         fifo_full,
    input  wire [11:0]  dbg_rd_addr,
    input  wire [11:0]  dbg_rd_base,
    input  wire         rd_align_en_i,
    input  wire         rd_parity_i,
    input  wire         line_lock_i,
    input  wire         rc_lag_i,
    input  wire         xclr_robust_i,
    input  wire         waddr_lock_i,
    input  wire         rdp_fallback_i,
    input  wire [15:0]  rd_hs_cnt,
    input  wire [15:0]  line_had_cnt,
    input  wire [15:0]  lock_fire_cnt,
    input  wire [15:0]  wr_step_bad,
    input  wire [15:0]  ddr_rdone_cnt,
    input  wire [15:0]  wr_line_snap,
    input  wire [8:0]   wr_addr_snap,
    input  wire [11:0]  px_bad_snap,
    input  wire [11:0]  hd_line_snap,
    input  wire [15:0]  x_glitch_snap,
    input  wire [10:0]  test_x,
    input  wire         pclk,
    input  wire         pclk_rst_n,
    input  wire         fifo_rd_en,
    input  wire         fifo_rd_empty,
    input  wire [12:0]  fifo_rd_level,
    input  wire         tlp_tlast,
    input  wire [16:0]  dma_cnt,
    output wire [319:0] diag
);
    reg [15:0] de_line;
    reg [15:0] wr_word_line;
    reg [15:0] line_cnt;
    reg [15:0] wr_total;
    reg [15:0] full_cycles;
    reg [15:0] drop_words;
    reg [2:0]  wr_pix;
    wire [15:0] rd_addr_raw = {4'b0000, dbg_rd_addr};
    wire [15:0] rd_col = rd_addr_raw;
    reg  [12:0] col_min, col_max;
    reg  [15:0] de_first_dly;
    reg  [15:0] hs_gap_cnt;
    reg         de_seen;
    reg  [15:0] first_col_cur, first_col_prev;
    reg  [15:0] first_col_frame, first_col_prev_frame;
    reg  [10:0] x_hs_latch;
    reg [63:0] info_snap;
    reg        info_toggle;
    reg        vin_hs_d, vin_vs_d;
    wire       vin_hs_rise = vin_hs & ~vin_hs_d;
    wire       vin_vs_rise = vin_vs & ~vin_vs_d;
    reg [31:0] line_snap;
    reg        line_toggle;
    reg [63:0] frame_snap;
    reg        frame_toggle;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            de_line       <= 16'd0;
            wr_word_line  <= 16'd0;
            line_cnt      <= 16'd0;
            wr_total      <= 16'd0;
            full_cycles   <= 16'd0;
            drop_words    <= 16'd0;
            wr_pix        <= 3'd0;
            vin_hs_d      <= 1'b0;
            vin_vs_d      <= 1'b0;
            line_snap     <= 32'd0;
            line_toggle   <= 1'b0;
            frame_snap    <= 64'd0;
            frame_toggle  <= 1'b0;
            col_min       <= 13'h1FFF;
            col_max       <= 13'd0;
            de_first_dly  <= 16'd0;
            hs_gap_cnt    <= 16'd0;
            de_seen       <= 1'b0;
            first_col_cur <= 16'd0;
            first_col_prev<= 16'd0;
            first_col_frame <= 16'd0;
            first_col_prev_frame <= 16'd0;
            x_hs_latch    <= 11'd0;
            info_snap     <= 64'd0;
            info_toggle   <= 1'b0;
        end
        else begin
            vin_hs_d <= vin_hs;
            vin_vs_d <= vin_vs;
            if (vin_vs_rise) begin
                first_col_frame      <= first_col_cur;
                first_col_prev_frame <= first_col_prev;
                frame_snap  <= {drop_words, full_cycles, wr_total, line_cnt};
                frame_toggle <= ~frame_toggle;
                line_cnt     <= 16'd0;
                wr_total     <= 16'd0;
                full_cycles  <= 16'd0;
                drop_words   <= 16'd0;
                de_line      <= 16'd0;
                wr_word_line <= 16'd0;
                wr_pix       <= 3'd0;
                col_min      <= 13'h1FFF;
                col_max      <= 13'd0;
                de_first_dly <= 16'd0;
                hs_gap_cnt   <= 16'd0;
                de_seen      <= 1'b0;
            end
            else if (vin_hs_rise) begin
                line_snap   <= {wr_word_line, de_line};
                line_toggle <= ~line_toggle;
                info_snap   <= {line_cnt,
                                {5'd0, x_hs_latch},
                                first_col_prev_frame,
                                first_col_frame};
                info_toggle <= ~info_toggle;
                x_hs_latch  <= {5'd0, test_x};
                if (de_line != 16'd0)
                    line_cnt <= line_cnt + 16'd1;
                de_line      <= 16'd0;
                wr_word_line <= 16'd0;
                wr_pix       <= 3'd0;
                col_min      <= 13'h1FFF;
                col_max      <= 13'd0;
                de_first_dly <= 16'd0;
                hs_gap_cnt   <= 16'd0;
                de_seen      <= 1'b0;
            end
            if (vin_de) begin
                de_line <= de_line + 16'd1;
                if (wr_pix == 3'd7) begin
                    wr_word_line <= wr_word_line + 16'd1;
                    wr_total     <= wr_total + 16'd1;
                    wr_pix       <= 3'd0;
                    if (fifo_full)
                        drop_words <= drop_words + 16'd1;
                end
                else begin
                    wr_pix <= wr_pix + 3'd1;
                end
                if (fifo_full)
                    full_cycles <= full_cycles + 16'd1;
                if (rd_col < col_min) col_min <= rd_col;
                if (rd_col > col_max) col_max <= rd_col;
            end
            if (!vin_hs_rise)
                hs_gap_cnt <= hs_gap_cnt + 16'd1;
            if (!de_seen) begin
                if (vin_de) begin
                    first_col_prev <= first_col_cur;
                    first_col_cur  <= rd_addr_raw;
                    de_seen        <= 1'b1;
                end
                else
                    de_first_dly <= de_first_dly + 16'd1;
            end
        end
    end
    reg [2:0]  line_tog_sync, frame_tog_sync, info_tog_sync;
    reg [31:0] line_snap_sync;
    reg [63:0] frame_snap_sync;
    reg [63:0] info_snap_sync;
    always @(posedge pclk or negedge pclk_rst_n) begin
        if (!pclk_rst_n) begin
            line_tog_sync  <= 3'd0;
            frame_tog_sync <= 3'd0;
            info_tog_sync  <= 3'd0;
            line_snap_sync <= 32'd0;
            frame_snap_sync<= 64'd0;
            info_snap_sync <= 64'd0;
        end
        else begin
            line_tog_sync  <= {line_tog_sync[1:0],  line_toggle};
            frame_tog_sync <= {frame_tog_sync[1:0], frame_toggle};
            info_tog_sync  <= {info_tog_sync[1:0],  info_toggle};
            if (line_tog_sync[2] ^ line_tog_sync[1])
                line_snap_sync <= line_snap;
            if (frame_tog_sync[2] ^ frame_tog_sync[1])
                frame_snap_sync <= frame_snap;
            if (info_tog_sync[2] ^ info_tog_sync[1])
                info_snap_sync <= info_snap;
        end
    end
    wire [15:0] de_line_sync      = line_snap_sync[15:0];
    wire [15:0] wr_word_line_sync = line_snap_sync[31:16];
    wire [15:0] line_cnt_sync     = frame_snap_sync[15:0];
    wire [15:0] wr_total_sync     = frame_snap_sync[31:16];
    wire [15:0] full_cycles_sync  = frame_snap_sync[47:32];
    wire [15:0] drop_words_sync   = frame_snap_sync[63:48];
    wire [15:0] first_col_sync    = info_snap_sync[15:0];
    wire [15:0] first_col_prev_sync = info_snap_sync[31:16];
    wire [15:0] x_hs_sync         = info_snap_sync[47:32];
    wire [15:0] gen_line_sync     = info_snap_sync[63:48];
    reg [2:0]  hs_sync;
    reg [2:0]  vs_sync_pd;
    wire       hs_pd2_rise = hs_sync[2] & ~hs_sync[1];
    wire       vs_pd2      = vs_sync_pd[2];
    always @(posedge pclk or negedge pclk_rst_n) begin
        if (!pclk_rst_n) vs_sync_pd <= 3'd0;
        else             vs_sync_pd <= {vs_sync_pd[1:0], vin_vs};
    end
    reg        tlast_d;
    wire       tlast_rise = tlp_tlast & ~tlast_d;
    reg [15:0] rd_word_line;
    reg [15:0] tlp_line;
    reg [15:0] tlast_high_line;
    reg [15:0] rd_total;
    reg [15:0] underflow;
    reg [15:0] tlast_total;
    reg [15:0] tlast_high_total;
    reg [15:0] hs_cnt;
    reg [12:0] level_hs;
    reg [15:0] dma_hs;
    reg [12:0] level_min;
    reg [12:0] level_max;
    reg [15:0] rd_word_line_snap;
    reg [15:0] tlp_line_snap;
    reg [15:0] tlast_high_line_snap;
    always @(posedge pclk or negedge pclk_rst_n) begin
        if (!pclk_rst_n) begin
            hs_sync             <= 3'd0;
            tlast_d             <= 1'b0;
            rd_word_line        <= 16'd0;
            tlp_line            <= 16'd0;
            tlast_high_line     <= 16'd0;
            rd_total            <= 16'd0;
            underflow           <= 16'd0;
            tlast_total         <= 16'd0;
            tlast_high_total    <= 16'd0;
            hs_cnt              <= 16'd0;
            level_hs            <= 13'd0;
            dma_hs              <= 16'd0;
            level_min           <= 13'h1FFF;
            level_max           <= 13'd0;
            rd_word_line_snap   <= 16'd0;
            tlp_line_snap       <= 16'd0;
            tlast_high_line_snap<= 16'd0;
        end
        else begin
            hs_sync <= {hs_sync[1:0], vin_hs};
            tlast_d <= tlp_tlast;
            if (vs_pd2) begin
                rd_total         <= 16'd0;
                underflow        <= 16'd0;
                tlast_total      <= 16'd0;
                tlast_high_total <= 16'd0;
                hs_cnt           <= 16'd0;
            end
            if (tlp_tlast && !tlast_d)
                tlp_line <= tlp_line + 16'd1;
            if (tlp_tlast) begin
                tlast_high_line  <= tlast_high_line + 16'd1;
                tlast_high_total <= tlast_high_total + 16'd1;
            end
            if (fifo_rd_en) begin
                rd_word_line <= rd_word_line + 16'd1;
                rd_total     <= rd_total + 16'd1;
                if (fifo_rd_empty)
                    underflow <= underflow + 16'd1;
            end
            if (fifo_rd_level < level_min)
                level_min <= fifo_rd_level;
            if (fifo_rd_level > level_max)
                level_max <= fifo_rd_level;
            if (hs_pd2_rise) begin
                rd_word_line_snap    <= rd_word_line;
                tlp_line_snap        <= tlp_line;
                tlast_high_line_snap <= tlast_high_line;
                level_hs             <= fifo_rd_level;
                dma_hs               <= dma_cnt[15:0];
                hs_cnt               <= hs_cnt + 16'd1;
                rd_word_line    <= 16'd0;
                tlp_line        <= 16'd0;
                tlast_high_line <= 16'd0;
                level_min       <= 13'h1FFF;
                level_max       <= 13'd0;
            end
        end
    end
    assign diag[15:0]    = de_line_sync;
    assign diag[31:16]   = tlp_line_snap;
    assign diag[47:32]   = rd_word_line_snap;
    assign diag[63:48]   = wr_word_line_sync;
    assign diag[79:64]   = {3'd0, level_hs};
    assign diag[95:80]   = tlast_high_line_snap;
    assign diag[111:96]  = dma_hs;
    assign diag[127:112] = line_cnt_sync;
    assign diag[143:128] = wr_total_sync;
    assign diag[159:144] = rd_total;
    assign diag[175:160] = underflow;
    assign diag[191:176] = drop_words_sync;
    assign diag[207:192] = full_cycles_sync;
    assign diag[223:208] = px_bad_snap;
    assign diag[239:224] = hd_line_snap;
    assign diag[255:240] = x_glitch_snap;
    assign diag[271:256] = first_col_sync;
    assign diag[287:272] = first_col_prev_sync;
    assign diag[303:288] = {5'd0, dbg_rd_base[1], dbg_rd_base[0], wr_addr_snap};
    assign diag[319:304] = {8'd0, rdp_fallback_i, waddr_lock_i, xclr_robust_i,
                            rc_lag_i, 1'b0, rd_parity_i, rd_align_en_i, line_lock_i};
endmodule
