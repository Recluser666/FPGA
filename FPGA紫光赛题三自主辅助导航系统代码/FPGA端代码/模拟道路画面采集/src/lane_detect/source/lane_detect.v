module lane_detect #(
    parameter [11:0] ROI_Y0    = 12'd592,
    parameter [11:0] ROI_Y1    = 12'd1079,
    parameter [11:0] MID_X     = 12'd960,
    parameter [11:0] WIN       = 12'd1024,
    parameter [5:0]  SOLID_MIN = 6'd20,
    parameter [5:0]  GAP_MIN   = 6'd4,
    parameter [5:0]  GAP_MAX   = 6'd20,
    parameter [5:0]  NONE_MAX  = 6'd7
)(
    input  wire        clk, rst_n,
    input  wire        i_de, i_hs, i_vs,
    input  wire        i_bin,
    input  wire [11:0] i_xc, i_yc,
    output reg  [1:0]  o_type_l,
    output reg  [1:0]  o_type_r,
    output reg         o_valid,
    output reg  [11:0] o_pos_l,
    output reg  [11:0] o_pos_r
);
reg  vs_d, de_d, samp_d;
reg  first_l, first_r, hit_l, hit_r;
reg  [11:0] pos_l, pos_r;
reg  [31:0] occ_l, occ_r, occ_l_h, occ_r_h;
wire vs_rise = i_vs & ~vs_d;
wire de_fall = de_d & ~i_de;
wire in_roi  = (i_yc >= ROI_Y0) && (i_yc <= ROI_Y1);
wire samp    = in_roi && (i_yc[3:0] == ROI_Y0[3:0]);
wire left    = (i_xc < MID_X);
wire in_win_l = !first_l || ((i_xc + WIN >= pos_l) && (i_xc <= pos_l + WIN));
wire in_win_r = !first_r || ((i_xc + WIN >= pos_r) && (i_xc <= pos_r + WIN));
wire hit_l_now = i_de & samp &  left & i_bin & in_win_l;
wire hit_r_now = i_de & samp & ~left & i_bin & in_win_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin vs_d <= 1'b0; de_d <= 1'b0; samp_d <= 1'b0; end
    else        begin vs_d <= i_vs; de_d <= i_de; samp_d <= samp; end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        first_l <= 1'b0; first_r <= 1'b0;
        hit_l   <= 1'b0; hit_r   <= 1'b0;
        pos_l   <= 12'd0; pos_r  <= 12'd0;
    end else if (vs_rise) begin
        first_l <= 1'b0; first_r <= 1'b0;
        hit_l   <= 1'b0; hit_r   <= 1'b0;
    end else begin
        if (hit_l_now) begin
            hit_l <= 1'b1;
            if (!first_l) begin pos_l <= i_xc; first_l <= 1'b1; end
        end
        if (hit_r_now) begin
            hit_r <= 1'b1;
            if (!first_r) begin pos_r <= i_xc; first_r <= 1'b1; end
        end
    end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        occ_l <= 32'd0; occ_r <= 32'd0; occ_l_h <= 32'd0; occ_r_h <= 32'd0;
    end else if (vs_rise) begin
        occ_l_h <= occ_l;  occ_r_h <= occ_r;
        occ_l   <= 32'd0;  occ_r   <= 32'd0;
    end else if (de_fall & samp_d) begin
        occ_l <= {occ_l[30:0], hit_l};
        occ_r <= {occ_r[30:0], hit_r};
    end
end
function [3:0] pc8;
    input [7:0] v;
    integer i;
    begin
        pc8 = 4'd0;
        for (i = 0; i < 8; i = i + 1) pc8 = pc8 + {3'd0, v[i]};
    end
endfunction
wire [31:0] xor_l = occ_l_h ^ (occ_l_h >> 1);
wire [31:0] xor_r = occ_r_h ^ (occ_r_h >> 1);
wire [5:0] pc_l = {2'd0, pc8(occ_l_h[7:0])}   + {2'd0, pc8(occ_l_h[15:8])}
                + {2'd0, pc8(occ_l_h[23:16])} + {2'd0, pc8(occ_l_h[31:24])};
wire [5:0] pc_r = {2'd0, pc8(occ_r_h[7:0])}   + {2'd0, pc8(occ_r_h[15:8])}
                + {2'd0, pc8(occ_r_h[23:16])} + {2'd0, pc8(occ_r_h[31:24])};
wire [5:0] tp_l = {2'd0, pc8(xor_l[7:0])}     + {2'd0, pc8(xor_l[15:8])}
                + {2'd0, pc8(xor_l[23:16])}   + {2'd0, pc8(xor_l[31:24])};
wire [5:0] tp_r = {2'd0, pc8(xor_r[7:0])}     + {2'd0, pc8(xor_r[15:8])}
                + {2'd0, pc8(xor_r[23:16])}   + {2'd0, pc8(xor_r[31:24])};
reg [5:0] n1_l, n1_r, nt_l, nt_r;
reg       v1, v2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        n1_l <= 6'd0; n1_r <= 6'd0; nt_l <= 6'd0; nt_r <= 6'd0;
        v1 <= 1'b0; v2 <= 1'b0;
    end else begin
        n1_l <= pc_l; n1_r <= pc_r; nt_l <= tp_l; nt_r <= tp_r;
        v1 <= vs_rise; v2 <= v1;
    end
end
function [1:0] cls;
    input [5:0] n1;
    input [5:0] nt;
    begin
        if (n1 <= NONE_MAX)
            cls = 2'd0;
        else if ((n1 >= SOLID_MIN) && (nt <= 6'd2))
            cls = 2'd1;
        else if ((n1 >= 6'd8) && (nt >= GAP_MIN) && (nt <= GAP_MAX))
            cls = 2'd2;
        else
            cls = 2'd0;
    end
endfunction
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_type_l <= 2'd0; o_type_r <= 2'd0;
        o_valid  <= 1'b0;
        o_pos_l  <= 12'd0; o_pos_r <= 12'd0;
    end else begin
        o_valid <= v2;
        if (v2) begin
            o_type_l <= cls(n1_l, nt_l);
            o_type_r <= cls(n1_r, nt_r);
            o_pos_l  <= pos_l;
            o_pos_r  <= pos_r;
        end
    end
end
endmodule
