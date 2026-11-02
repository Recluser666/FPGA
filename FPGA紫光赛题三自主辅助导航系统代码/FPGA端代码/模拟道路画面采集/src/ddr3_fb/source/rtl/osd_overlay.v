`timescale 1ns / 1ps
module osd_overlay #(
    parameter        MIX_LATENCY = 3,
    parameter [11:0] BOX1_X0    = 12'd700,
    parameter [11:0] BOX1_Y0    = 12'd120,
    parameter [11:0] BOX1_X1    = 12'd1200,
    parameter [11:0] BOX1_Y1    = 12'd520,
    parameter [23:0] BOX1_COLOR = 24'h00FF00,
    parameter [4:0]  BOX1_ALPHA = 5'd6,
    parameter        BOX1_EN   = 1'b0,
    parameter [3:0]  BOX_TH     = 4'd3,
    parameter [1:0]  ARROW_SCALE_SHIFT = 2'd2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_rgb,
    input  wire        i_box0_en,
    input  wire [11:0] i_box0_x0,
    input  wire [11:0] i_box0_y0,
    input  wire [11:0] i_box0_x1,
    input  wire [11:0] i_box0_y1,
    input  wire [23:0] i_box0_color,
    input  wire [4:0]  i_box0_alpha,
    input  wire        i_box0_fill,
    input  wire        i_arrow_en,
    input  wire [1:0]  i_arrow_sel,
    input  wire [11:0] i_arrow_x,
    input  wire [11:0] i_arrow_y,
    input  wire [23:0] i_arrow_color,
    input  wire [4:0]  i_arrow_alpha,
    output wire        o_de,
    output wire        o_hs,
    output wire        o_vs,
    output wire [23:0] o_rgb
);
    reg        de1, hs1, vs1;
    reg [23:0] rgb1;
    reg        de2, hs2, vs2;
    reg [23:0] rgb2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de1 <= 1'b0; hs1 <= 1'b0; vs1 <= 1'b0; rgb1 <= 24'd0;
            de2 <= 1'b0; hs2 <= 1'b0; vs2 <= 1'b0; rgb2 <= 24'd0;
        end else begin
            de1 <= i_de;  hs1 <= i_hs;  vs1 <= i_vs;  rgb1 <= i_rgb;
            de2 <= de1;   hs2 <= hs1;   vs2 <= vs1;   rgb2 <= rgb1;
        end
    end
    reg [11:0] x_cnt, y_cnt;
    reg        de2_d, vs2_d;
    wire vs2_rise = vs2 & ~vs2_d;
    wire de2_fall = ~de2 & de2_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt <= 12'd0; y_cnt <= 12'd0; de2_d <= 1'b0; vs2_d <= 1'b0;
        end else begin
            de2_d <= de2;
            vs2_d <= vs2;
            if (de2) x_cnt <= x_cnt + 12'd1;
            else     x_cnt <= 12'd0;
            if (vs2_rise)      y_cnt <= 12'd0;
            else if (de2_fall) y_cnt <= y_cnt + 12'd1;
        end
    end
    wire in_box0 = (x_cnt >= i_box0_x0) && (x_cnt <= i_box0_x1) &&
                   (y_cnt >= i_box0_y0) && (y_cnt <= i_box0_y1);
    wire on_border0 = (x_cnt <= (i_box0_x0 + BOX_TH - 4'd1)) ||
                      (x_cnt >= (i_box0_x1 - BOX_TH + 4'd1)) ||
                      (y_cnt <= (i_box0_y0 + BOX_TH - 4'd1)) ||
                      (y_cnt >= (i_box0_y1 - BOX_TH + 4'd1));
    wire box0_hit = i_box0_en & in_box0 & (i_box0_fill | on_border0);
    wire in_box1 = (x_cnt >= BOX1_X0) && (x_cnt <= BOX1_X1) &&
                   (y_cnt >= BOX1_Y0) && (y_cnt <= BOX1_Y1);
    wire on_border1 = (x_cnt <= (BOX1_X0 + BOX_TH - 4'd1)) ||
                      (x_cnt >= (BOX1_X1 - BOX_TH + 4'd1)) ||
                      (y_cnt <= (BOX1_Y0 + BOX_TH - 4'd1)) ||
                      (y_cnt >= (BOX1_Y1 - BOX_TH + 4'd1));
    wire box1_hit = BOX1_EN & in_box1 & on_border1;
    wire [11:0] arrow_lx_raw = x_cnt - i_arrow_x;
    wire [11:0] arrow_ly_raw = y_cnt - i_arrow_y;
    wire [11:0] lx = (ARROW_SCALE_SHIFT == 2'd0) ? arrow_lx_raw : (arrow_lx_raw >> ARROW_SCALE_SHIFT);
    wire [11:0] ly = (ARROW_SCALE_SHIFT == 2'd0) ? arrow_ly_raw : (arrow_ly_raw >> ARROW_SCALE_SHIFT);
    wire in_cell = (lx <= 12'd31) && (ly <= 12'd31);
    reg        de3, hs3, vs3;
    reg [23:0] rgb3;
    reg [11:0] lx3, ly3;
    reg        in_cell3;
    reg        box0_hit3, box1_hit3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de3 <= 1'b0; hs3 <= 1'b0; vs3 <= 1'b0; rgb3 <= 24'd0;
            lx3 <= 12'd0; ly3 <= 12'd0; in_cell3 <= 1'b0;
            box0_hit3 <= 1'b0; box1_hit3 <= 1'b0;
        end else begin
            de3 <= de2;  hs3 <= hs2;  vs3 <= vs2;  rgb3 <= rgb2;
            lx3 <= lx;   ly3 <= ly;   in_cell3 <= in_cell;
            box0_hit3 <= box0_hit;
            box1_hit3 <= box1_hit;
        end
    end
    wire [5:0] dx3 = (lx3 >= 12'd16) ? lx3[5:0] - 6'd16 : 6'd16 - lx3[5:0];
    wire [5:0] dy3 = (ly3 >= 12'd16) ? ly3[5:0] - 6'd16 : 6'd16 - ly3[5:0];
    wire head_left3  = (lx3 <= 12'd11) && (dy3 <= lx3[5:0]);
    wire head_right3 = (lx3 >= 12'd20) && (dy3 <= (12'd31 - lx3));
    wire head_up3    = (ly3 <= 12'd11) && (dx3 <= ly3[5:0]);
    wire seg_h_left3   = (lx3 >= 12'd6)  && (lx3 <= 12'd21) && (dy3 <= 6'd3);
    wire tail_v_left3  = (lx3 >= 12'd16) && (lx3 <= 12'd21) && (ly3 >= 12'd15) && (ly3 <= 12'd29);
    wire seg_h_right3  = (lx3 >= 12'd10) && (lx3 <= 12'd25) && (dy3 <= 6'd3);
    wire tail_v_right3 = (lx3 >= 12'd10) && (lx3 <= 12'd15) && (ly3 >= 12'd15) && (ly3 <= 12'd29);
    wire shaft_v_up3   = (ly3 >= 12'd8)  && (ly3 <= 12'd31) && (dx3 <= 6'd3);
    wire arrow_left_hit3  = head_left3  || seg_h_left3  || tail_v_left3;
    wire arrow_up_hit3    = head_up3    || shaft_v_up3;
    wire arrow_right_hit3 = head_right3 || seg_h_right3 || tail_v_right3;
    wire arrow_shape_hit3 = (i_arrow_sel == 2'd0) ? arrow_left_hit3  :
                            (i_arrow_sel == 2'd1) ? arrow_up_hit3    :
                                                    arrow_right_hit3 ;
    wire arrow_hit3 = i_arrow_en & in_cell3 & arrow_shape_hit3;
    reg        de4, hs4, vs4;
    reg [23:0] rgb4;
    reg        osd_en_r;
    reg [23:0] osd_col_r;
    reg [4:0]  osd_a_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de4 <= 1'b0; hs4 <= 1'b0; vs4 <= 1'b0; rgb4 <= 24'd0;
            osd_en_r <= 1'b0; osd_col_r <= 24'd0; osd_a_r <= 5'd0;
        end else begin
            de4 <= de3;  hs4 <= hs3;  vs4 <= vs3;  rgb4 <= rgb3;
            if (arrow_hit3) begin
                osd_en_r  <= 1'b1;
                osd_col_r <= i_arrow_color;
                osd_a_r   <= i_arrow_alpha;
            end else if (box0_hit3) begin
                osd_en_r  <= 1'b1;
                osd_col_r <= i_box0_color;
                osd_a_r   <= i_box0_alpha;
            end else if (box1_hit3) begin
                osd_en_r  <= 1'b1;
                osd_col_r <= BOX1_COLOR;
                osd_a_r   <= BOX1_ALPHA;
            end else begin
                osd_en_r  <= 1'b0;
                osd_col_r <= 24'd0;
                osd_a_r   <= 5'd0;
            end
        end
    end
    wire [4:0] mix_a = osd_en_r ? osd_a_r : 5'd0;
    wire        mix_de;
    wire [23:0] mix_rgb;
    osd_alpha_mix #(
        .MIX_LATENCY (MIX_LATENCY)
    ) u_mix (
        .clk     (clk       ),
        .rst_n   (rst_n     ),
        .i_de    (de4       ),
        .i_video (rgb4      ),
        .i_osd   (osd_col_r ),
        .i_alpha (mix_a     ),
        .o_de    (mix_de    ),
        .o_rgb   (mix_rgb   )
    );
    reg [6:0] vs_sr, hs_sr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_sr <= 7'd0;
            hs_sr <= 7'd0;
        end else begin
            vs_sr <= {vs_sr[5:0], i_vs};
            hs_sr <= {hs_sr[5:0], i_hs};
        end
    end
    assign o_de  = mix_de;
    assign o_rgb = mix_rgb;
    assign o_vs  = vs_sr[6];
    assign o_hs  = hs_sr[6];
endmodule
