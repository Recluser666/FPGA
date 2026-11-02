module zebra_detect #(
    parameter [11:0] ROI_Y0  = 12'd810,
    parameter [11:0] ROI_Y1  = 12'd1079,
    parameter [5:0]  NTR_MIN = 6'd8,
    parameter [5:0]  NTR_MAX = 6'd48,
    parameter [9:0]  RUN_MIN = 10'd5
)(
    input  wire        clk, rst_n,
    input  wire        i_de, i_hs, i_vs,
    input  wire        i_bin,
    input  wire [11:0] i_xc, i_yc,
    output reg         o_zebra,
    output reg         o_valid,
    output reg  [9:0]  o_runmax,
    output reg  [5:0]  o_ntrmax
);
reg  de_d, vs_d, bin_d;
wire de_fall = de_d & ~i_de;
wire vs_rise = i_vs & ~vs_d;
wire in_roi  = (i_yc >= ROI_Y0) && (i_yc <= ROI_Y1);
wire bin_edge = (i_bin ^ bin_d) & i_de & de_d & in_roi;
reg [5:0] ntr;
reg [9:0] run, runmax;
reg [5:0] ntrmax;
wire row_ok = (ntr >= NTR_MIN) && (ntr <= NTR_MAX);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin de_d <= 1'b0; vs_d <= 1'b0; bin_d <= 1'b0; end
    else begin
        de_d  <= i_de;
        vs_d  <= i_vs;
        bin_d <= i_de ? i_bin : 1'b0;
    end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)               ntr <= 6'd0;
    else if (de_fall | vs_rise) ntr <= 6'd0;
    else if (bin_edge)            ntr <= (ntr == 6'd63) ? ntr : (ntr + 6'd1);
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        run <= 10'd0; runmax <= 10'd0; ntrmax <= 6'd0;
        o_zebra <= 1'b0; o_valid <= 1'b0;
        o_runmax <= 10'd0; o_ntrmax <= 6'd0;
    end else begin
        o_valid <= 1'b0;
        if (vs_rise) begin
            o_zebra  <= (runmax >= RUN_MIN);
            o_runmax <= runmax;
            o_ntrmax <= ntrmax;
            o_valid  <= 1'b1;
            run <= 10'd0; runmax <= 10'd0; ntrmax <= 6'd0;
        end else if (de_fall) begin
            if (row_ok) begin
                run <= run + 10'd1;
                if ((run + 10'd1) > runmax) runmax <= run + 10'd1;
                if (ntr > ntrmax) ntrmax <= ntr;
            end else begin
                run <= 10'd0;
            end
        end
    end
end
endmodule
