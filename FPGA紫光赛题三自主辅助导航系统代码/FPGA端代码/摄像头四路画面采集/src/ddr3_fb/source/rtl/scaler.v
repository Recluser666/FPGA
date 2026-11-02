`timescale 1ns / 1ps
module scaler #(
    parameter IN_FMT = 0,
    parameter H_ACT  = 1920,
    parameter V_ACT  = 1080
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_data,
    output wire        o_de,
    output wire [7:0]  o_data,
    output wire [10:0] o_x,
    output wire [9:0]  o_y,
    output reg         o_hs,
    output reg         o_vs
);
    reg [23:0] data_s0;
    reg        de_s0, vs_s0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_s0 <= 24'd0; de_s0 <= 1'b0; vs_s0 <= 1'b0;
        end else begin
            data_s0 <= i_data; de_s0 <= i_de; vs_s0 <= i_vs;
        end
    end
    wire [7:0] r8, g8, b8;
    generate
        if (IN_FMT == 0) begin : g_in565
            wire [4:0] r5 = data_s0[15:11];
            wire [5:0] g6 = data_s0[10:5];
            wire [4:0] b5 = data_s0[4:0];
            assign r8 = {r5, r5[4:2]};
            assign g8 = {g6, g6[5:4]};
            assign b8 = {b5, b5[4:2]};
        end else begin : g_in888
            assign r8 = data_s0[23:16];
            assign g8 = data_s0[15:8];
            assign b8 = data_s0[7:0];
        end
    endgenerate
    reg [15:0] gwr_r, gwg_r, gwb_r;
    reg        de_s1, vs_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gwr_r <= 16'd0; gwg_r <= 16'd0; gwb_r <= 16'd0;
            de_s1 <= 1'b0;  vs_s1 <= 1'b0;
        end else begin
            gwr_r <= (r8 << 6) + (r8 << 3) + (r8 << 2) + r8;
            gwg_r <= (g8 << 7) + (g8 << 4) + (g8 << 2) + (g8 << 1);
            gwb_r <= (b8 << 5) - (b8 << 2) + b8;
            de_s1 <= de_s0;  vs_s1 <= vs_s0;
        end
    end
    reg [7:0] gray_r;
    reg       de_s2, vs_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gray_r <= 8'd0; de_s2 <= 1'b0; vs_s2 <= 1'b0;
        end else begin
            gray_r <= (gwr_r + gwg_r + gwb_r) >> 8;
            de_s2  <= de_s1;  vs_s2 <= vs_s1;
        end
    end
    reg [11:0] x_cnt, y_cnt;
    reg [1:0]  row_ph;
    reg        de_d, vs_d;
    wire de_fall = de_d & ~de_s2;
    wire vs_rise = vs_s2 & ~vs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt <= 12'd0; y_cnt <= 12'd0; row_ph <= 2'd0;
            de_d  <= 1'b0;  vs_d  <= 1'b0;
        end else begin
            de_d <= de_s2;  vs_d <= vs_s2;
            if (de_s2) x_cnt <= x_cnt + 12'd1;
            else       x_cnt <= 12'd0;
            if (vs_rise) begin
                y_cnt <= 12'd0; row_ph <= 2'd0;
            end else if (de_fall) begin
                y_cnt  <= y_cnt + 12'd1;
                row_ph <= (row_ph == 2'd2) ? 2'd0 : row_ph + 2'd1;
            end
        end
    end
    reg [7:0] lb0 [0:2047];
    reg [7:0] lb1 [0:2047];
    reg [7:0] lb2 [0:2047];
    reg [7:0] q0, q1, q2;
    wire [10:0] ram_addr = x_cnt[10:0];
    always @(posedge clk) begin
        q0 <= lb0[ram_addr];
        q1 <= lb1[ram_addr];
        q2 <= lb2[ram_addr];
    end
    always @(posedge clk) begin
        if (de_s2) begin
            case (row_ph)
                2'd0:    lb0[ram_addr] <= gray_r;
                2'd1:    lb1[ram_addr] <= gray_r;
                default: lb2[ram_addr] <= gray_r;
            endcase
        end
    end
    reg [1:0] rp1;
    reg [7:0] g1;
    reg       d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rp1 <= 2'd0; g1 <= 8'd0; d1 <= 1'b0; end
        else begin
            rp1 <= row_ph;
            g1  <= gray_r;
            d1  <= de_s2;
        end
    end
    wire [7:0] qA = (rp1 == 2'd0) ? q1 : (rp1 == 2'd1) ? q2 : q0;
    wire [7:0] qB = (rp1 == 2'd0) ? q2 : (rp1 == 2'd1) ? q0 : q1;
    reg [7:0] qA_r, qB_r, g2;
    reg [1:0] rp2;
    reg       d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qA_r <= 8'd0; qB_r <= 8'd0; g2 <= 8'd0; rp2 <= 2'd0; d2 <= 1'b0;
        end else begin
            qA_r <= qA;  qB_r <= qB;  g2 <= g1;  rp2 <= rp1;  d2 <= d1;
        end
    end
    wire [9:0] vsum_c = {2'b0, g2} + {2'b0, qA_r} + {2'b0, qB_r};
    reg [9:0] vsum_r;
    reg [1:0] rp3;
    reg       d3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin vsum_r <= 10'd0; rp3 <= 2'd0; d3 <= 1'b0; end
        else begin
            vsum_r <= vsum_c;  rp3 <= rp2;  d3 <= d2;
        end
    end
    reg [1:0]  col_ph;
    reg [11:0] hacc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin col_ph <= 2'd0; hacc <= 12'd0; end
        else if (d3) begin
            if (col_ph == 2'd2) begin col_ph <= 2'd0; hacc <= 12'd0; end
            else begin col_ph <= col_ph + 2'd1; hacc <= hacc + vsum_r; end
        end
    end
    wire [11:0] hsum = hacc + vsum_r;
    wire blk_valid = d3 & (col_ph == 2'd2) & (rp3 == 2'd2);
    reg [10:0] ox_cnt;
    reg [9:0]  oy_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ox_cnt <= 11'd0; oy_cnt <= 10'd0; end
        else begin
            if (blk_valid)                 ox_cnt <= ox_cnt + 11'd1;
            if (de_fall && row_ph == 2'd2) oy_cnt <= oy_cnt + 10'd1;
            if (vs_rise) begin ox_cnt <= 11'd0; oy_cnt <= 10'd0; end
        end
    end
    reg [10:0] ox1, ox2, ox3;
    reg [9:0]  oy1, oy2, oy3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ox1 <= 11'd0; ox2 <= 11'd0; ox3 <= 11'd0;
            oy1 <= 10'd0; oy2 <= 10'd0; oy3 <= 10'd0;
        end else begin
            if (blk_valid) begin ox1 <= ox_cnt; oy1 <= oy_cnt; end
            ox2 <= ox1;  oy2 <= oy1;
            ox3 <= ox2;  oy3 <= oy2;
        end
    end
    reg [11:0] hsum_r;
    reg        blk_v1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin hsum_r <= 12'd0; blk_v1 <= 1'b0; end
        else begin
            hsum_r <= hsum;
            blk_v1 <= blk_valid;
        end
    end
    wire [20:0] s    = {9'd0, hsum_r};
    reg [20:0] m_hi_r, m_lo_r;
    reg        blk_v2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin m_hi_r <= 21'd0; m_lo_r <= 21'd0; blk_v2 <= 1'b0; end
        else begin
            m_hi_r <= (s << 9) - (s << 6);
            m_lo_r <= (s << 3) - s + 21'd2048;
            blk_v2 <= blk_v1;
        end
    end
    reg [7:0]  g9_r;
    reg        blk_v3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin g9_r <= 8'd0; blk_v3 <= 1'b0; end
        else begin
            g9_r   <= (m_hi_r + m_lo_r) >> 12;
            blk_v3 <= blk_v2;
        end
    end
    assign o_de   = blk_v3;
    assign o_data = g9_r;
    assign o_x    = ox3;
    assign o_y    = oy3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin o_hs <= 1'b0; o_vs <= 1'b0; end
        else begin
            o_hs <= (rp3 == 2'd2);
            o_vs <= vs_rise;
        end
    end
endmodule
