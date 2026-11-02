`timescale 1ns / 1ps
module hdmi_tx (
    input  wire        cfg_clk,
    input  wire        cfg_rst_n,
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_rgb,
    output wire        o_pixclk,
    output wire        o_de,
    output wire        o_hs,
    output wire        o_vs,
    output wire [7:0]  o_r,
    output wire [7:0]  o_g,
    output wire [7:0]  o_b,
    output wire        iic_scl,
    inout  wire        iic_sda,
    output wire        iic_tx_scl,
    inout  wire        iic_tx_sda,
    output wire        o_init_over
);
    ms72xx_ctl u_cfg (
        .clk        (cfg_clk    ),
        .rst_n      (cfg_rst_n  ),
        .init_over  (o_init_over),
        .iic_tx_scl (iic_tx_scl ),
        .iic_tx_sda (iic_tx_sda ),
        .iic_scl    (iic_scl    ),
        .iic_sda    (iic_sda    )
    );
    reg        de1, hs1, vs1;
    reg [23:0] rgb1;
    reg        de2, hs2, vs2;
    reg [23:0] rgb2;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            de1 <= 1'b0; hs1 <= 1'b0; vs1 <= 1'b0; rgb1 <= 24'd0;
            de2 <= 1'b0; hs2 <= 1'b0; vs2 <= 1'b0; rgb2 <= 24'd0;
        end else begin
            de1 <= i_de;  hs1 <= i_hs;  vs1 <= i_vs;  rgb1 <= i_rgb;
            de2 <= de1;   hs2 <= hs1;   vs2 <= vs1;   rgb2 <= rgb1;
        end
    end
    assign o_pixclk = pix_clk;
    assign o_de     = de2;
    assign o_hs     = hs2;
    assign o_vs     = vs2;
    assign o_r      = rgb2[23:16];
    assign o_g      = rgb2[15:8];
    assign o_b      = rgb2[7:0];
endmodule
