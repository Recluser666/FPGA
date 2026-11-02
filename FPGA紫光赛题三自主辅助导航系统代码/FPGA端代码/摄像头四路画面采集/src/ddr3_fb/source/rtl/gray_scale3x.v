`timescale 1ns / 1ps
module gray_scale3x #(
    parameter IN_FMT = 0,
    parameter [11:0] H_ACT = 12'd1920,
    parameter [11:0] V_ACT = 12'd1080
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_data,
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [7:0]  o_gray
);
    wire        sc_de;
    wire [7:0]  sc_data;
    wire [10:0] sc_x;
    wire [9:0]  sc_y;
    scaler #(
        .IN_FMT (IN_FMT),
        .H_ACT  (H_ACT ),
        .V_ACT  (V_ACT )
    ) u_scaler (
        .clk    (clk     ),
        .rst_n  (rst_n   ),
        .i_de   (i_de    ),
        .i_hs   (i_hs    ),
        .i_vs   (i_vs    ),
        .i_data (i_data  ),
        .o_de   (sc_de   ),
        .o_data (sc_data ),
        .o_x    (sc_x    ),
        .o_y    (sc_y    ),
        .o_hs   (        ),
        .o_vs   (        )
    );
    reg [1:0] col3;
    reg [9:0] rd_addr;
    reg [9:0] wr_cnt;
    reg       rd_sel;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_cnt <= 10'd0;
            rd_sel <= 1'b0;
        end else if (sc_de) begin
            if (wr_cnt == 10'd639) begin
                wr_cnt <= 10'd0;
                rd_sel <= ~rd_sel;
            end else begin
                wr_cnt <= wr_cnt + 10'd1;
            end
        end
    end
    wire wr_sel = ~rd_sel;
    reg [7:0] ls0 [0:1023];
    reg [7:0] ls1 [0:1023];
    always @(posedge clk) begin
        if (sc_de) begin
            if (wr_sel) ls1[sc_x[9:0]] <= sc_data;
            else        ls0[sc_x[9:0]] <= sc_data;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col3    <= 2'd0;
            rd_addr <= 10'd0;
        end else if (i_de) begin
            col3 <= (col3 == 2'd2) ? 2'd0 : col3 + 2'd1;
            if (col3 == 2'd2)
                rd_addr <= (rd_addr == 10'd639) ? 10'd0 : rd_addr + 10'd1;
        end else begin
            col3    <= 2'd0;
            rd_addr <= 10'd0;
        end
    end
    reg [7:0] rd_data_r;
    always @(posedge clk) rd_data_r <= rd_sel ? ls1[rd_addr] : ls0[rd_addr];
    reg de_d1, hs_d1, vs_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d1  <= 1'b0; hs_d1  <= 1'b0; vs_d1  <= 1'b0;
            o_de   <= 1'b0; o_hs   <= 1'b0; o_vs   <= 1'b0;
            o_gray <= 8'd0;
        end else begin
            de_d1  <= i_de;   hs_d1  <= i_hs;   vs_d1  <= i_vs;
            o_de   <= de_d1;  o_hs   <= hs_d1;  o_vs   <= vs_d1;
            o_gray <= rd_data_r;
        end
    end
endmodule
