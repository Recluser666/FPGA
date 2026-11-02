`timescale 1ns / 1ps
module timing_gen #(
    parameter [11:0] H_TOTAL = 12'd2200,
    parameter [11:0] H_SYNC  = 12'd44,
    parameter [11:0] H_BP    = 12'd148,
    parameter [11:0] H_FP    = 12'd88,
    parameter [11:0] H_ACT   = 12'd1920,
    parameter [11:0] V_TOTAL = 12'd1125,
    parameter [11:0] V_SYNC  = 12'd5,
    parameter [11:0] V_BP    = 12'd36,
    parameter [11:0] V_FP    = 12'd4,
    parameter [11:0] V_ACT   = 12'd1080
)(
    input  wire        sys_clk,
    input  wire        rst_n,
    output wire        pix_clk,
    output wire        cfg_clk,
    output wire        pll_lock,
    output wire        vs_out,
    output wire        hs_out,
    output wire        de_out,
    output wire [11:0] x_act,
    output wire [11:0] y_act
);
    pll u_pll (
        .clkin1   (sys_clk  ),
        .clkout0  (pix_clk  ),
        .clkout1  (cfg_clk  ),
        .pll_lock (pll_lock )
    );
    reg rstn_pix_r1, rstn_pix_r2;
    always @(posedge pix_clk or negedge rst_n)
     begin
        if (!rst_n)
         begin
            rstn_pix_r1 <= 1'b0;
            rstn_pix_r2 <= 1'b0;
         end
        else
         begin
            rstn_pix_r1 <= 1'b1;
            rstn_pix_r2 <= rstn_pix_r1;
         end
     end
    wire rstn_pix = rstn_pix_r2;
    sync_vg #(
        .X_BITS   (12        ),
        .Y_BITS   (12        ),
        .H_TOTAL  (H_TOTAL   ),
        .H_SYNC   (H_SYNC    ),
        .H_BP     (H_BP      ),
        .H_FP     (H_FP      ),
        .H_ACT    (H_ACT     ),
        .V_TOTAL  (V_TOTAL   ),
        .V_SYNC   (V_SYNC    ),
        .V_BP     (V_BP      ),
        .V_FP     (V_FP      ),
        .V_ACT    (V_ACT     ),
        .HV_OFFSET(12'd0     )
    ) u_sync_vg (
        .clk    (pix_clk  ),
        .rstn   (rstn_pix ),
        .vs_out (vs_out   ),
        .hs_out (hs_out   ),
        .de_out (de_out   ),
        .de_re  (         ),
        .x_act  (x_act    ),
        .y_act  (y_act    )
    );
endmodule
