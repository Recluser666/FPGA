module binarize #(
    parameter H_ACT = 1920,
    parameter [4:0]  SHIFT      = 5'd11,
    parameter [3:0]  K_SHIFT    = 4'd1,
    parameter [7:0]  FLOOR_INIT = 8'd20,
    parameter [7:0]  CAP        = 8'd230,
    parameter [11:0] ROI_Y0     = 12'd0,
    parameter [11:0] ROI_Y1     = 12'd1079
)(
    input  wire        clk, rst_n,
    input  wire        i_de, i_hs, i_vs,
    input  wire [7:0]  i_gray,
    input  wire [11:0] i_xc, i_yc,
    input  wire [7:0]  i_floor,
    input  wire        i_floor_we,
    output reg         o_de, o_hs, o_vs,
    output reg         o_bin,
    output reg  [11:0] o_xc, o_yc,
    output reg  [7:0]  o_level
);
reg  de_d, vs_d;
reg  [31:0] sum_row;
reg  [7:0]  floor_reg, level;
wire de_fall = de_d & ~i_de;
wire vs_rise = i_vs & ~vs_d;
wire roi_ok  = (i_yc >= ROI_Y0) && (i_yc <= ROI_Y1);
wire [15:0] th_k   = {8'd0, level} + ({8'd0, level} >> K_SHIFT);
wire [15:0] th_raw = (th_k < {8'd0, floor_reg}) ? {8'd0, floor_reg} : th_k;
wire [7:0]  th     = (th_raw > {8'd0, CAP}) ? CAP : th_raw[7:0];
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin de_d <= 1'b0; vs_d <= 1'b0; end
    else        begin de_d <= i_de; vs_d <= i_vs; end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)          floor_reg <= FLOOR_INIT;
    else if (i_floor_we) floor_reg <= i_floor;
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                  sum_row <= 32'd0;
    else if (de_fall | vs_rise)  sum_row <= 32'd0;
    else if (i_de & roi_ok)      sum_row <= sum_row + {24'd0, i_gray};
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)      level <= 8'd0;
    else if (de_fall) level <= sum_row[31:0] >> SHIFT;
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_bin <= 1'b0;
        o_de  <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
        o_xc  <= 12'd0; o_yc <= 12'd0;
        o_level <= 8'd0;
    end else begin
        o_bin <= (i_gray > th);
        o_de  <= i_de;  o_hs <= i_hs;  o_vs <= i_vs;
        o_xc  <= i_xc;  o_yc <= i_yc;
        o_level <= level;
    end
end
endmodule
