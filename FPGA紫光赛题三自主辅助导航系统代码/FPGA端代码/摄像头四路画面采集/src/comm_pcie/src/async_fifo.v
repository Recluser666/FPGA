`timescale 1ns / 1ps
module async_fifo #(
    parameter DW = 8,
    parameter AW = 9
)(
    input  wire          wclk,
    input  wire          wrst_n,
    input  wire          winc,
    input  wire [DW-1:0] wdata,
    output wire          wfull,
    input  wire          rclk,
    input  wire          rrst_n,
    input  wire          rinc,
    output wire [DW-1:0] rdata,
    output wire          rempty
);
    localparam DEPTH = (1 << AW);
    reg [DW-1:0] mem [0:DEPTH-1];
    reg [AW:0] wbin, wgray;
    reg [AW:0] rbin, rgray;
    reg [AW:0] wq1_rg, wq2_rg;
    reg [AW:0] rq1_wg, rq2_wg;
    function [AW:0] b2g;
        input [AW:0] b;
        begin b2g = b ^ (b >> 1); end
    endfunction
    wire [AW:0] wbin_nxt  = wbin + {{AW{1'b0}}, (winc & ~wfull)};
    wire [AW:0] wgray_nxt = b2g(wbin_nxt);
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin wbin <= {(AW+1){1'b0}}; wgray <= {(AW+1){1'b0}}; end
        else         begin wbin <= wbin_nxt;       wgray <= wgray_nxt;       end
    end
    wire [AW:0] wgray_cmp = {~wq2_rg[AW:AW-1], wq2_rg[AW-2:0]};
    assign wfull = (wgray_nxt == wgray_cmp);
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin wq1_rg <= {(AW+1){1'b0}}; wq2_rg <= {(AW+1){1'b0}}; end
        else         begin wq1_rg <= rgray;          wq2_rg <= wq1_rg;          end
    end
    wire [AW:0] rbin_nxt  = rbin + {{AW{1'b0}}, (rinc & ~rempty)};
    wire [AW:0] rgray_nxt = b2g(rbin_nxt);
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin rbin <= {(AW+1){1'b0}}; rgray <= {(AW+1){1'b0}}; end
        else         begin rbin <= rbin_nxt;       rgray <= rgray_nxt;       end
    end
    assign rempty = (rgray_nxt == rq2_wg);
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin rq1_wg <= {(AW+1){1'b0}}; rq2_wg <= {(AW+1){1'b0}}; end
        else         begin rq1_wg <= wgray;          rq2_wg <= rq1_wg;          end
    end
    always @(posedge wclk) begin
        if (winc & ~wfull) mem[wbin[AW-1:0]] <= wdata;
    end
    assign rdata = mem[rbin[AW-1:0]];
endmodule
module pix2tlp_fifo #(
    parameter AW = 6
)(
    input  wire         wclk,
    input  wire         wrst,
    input  wire         wen,
    input  wire [15:0]  wdata,
    output wire         wfull,
    input  wire         rclk,
    input  wire         rrst,
    input  wire         ren,
    output wire [127:0] rdata,
    output wire [12:0]  rlevel,
    output wire         rempty
);
    localparam DEPTH = (1 << AW);
    reg [127:0] acc_sr;
    reg [2:0]   acc_cnt;
    reg [127:0] wr_word;
    reg         wr_push;
    always @(posedge wclk) begin
        if (wrst) begin
            acc_sr  <= 128'd0;
            acc_cnt <= 3'd0;
            wr_word <= 128'd0;
            wr_push <= 1'b0;
        end else begin
            wr_push <= 1'b0;
            if (wen) begin
                acc_sr <= {acc_sr[111:0], wdata};
                if (acc_cnt == 3'd7) begin
                    acc_cnt <= 3'd0;
                    wr_word <= {wdata, acc_sr[111:0]};
                    wr_push <= 1'b1;
                end else begin
                    acc_cnt <= acc_cnt + 3'd1;
                end
            end
        end
    end
    reg [127:0] mem [0:DEPTH-1];
    reg [AW:0] wbin, wgray;
    reg [AW:0] rbin, rgray;
    reg [AW:0] wq1_rg, wq2_rg;
    reg [AW:0] rq1_wg, rq2_wg;
    function [AW:0] b2g;
        input [AW:0] b;
        begin b2g = b ^ (b >> 1); end
    endfunction
    function [AW:0] g2b;
        input [AW:0] g;
        integer i;
        begin
            g2b[AW] = g[AW];
            for (i = AW-1; i >= 0; i = i - 1)
                g2b[i] = g2b[i+1] ^ g[i];
        end
    endfunction
    wire [AW:0] wgray_inc  = b2g(wbin + 1'b1);
    wire [AW:0] wgray_cmp  = {~wq2_rg[AW:AW-1], wq2_rg[AW-2:0]};
    assign wfull = (wgray_inc == wgray_cmp);
    wire        wr_ok     = wr_push & ~wfull;
    wire [AW:0] wbin_nxt  = wbin + {{AW{1'b0}}, wr_ok};
    wire [AW:0] wgray_nxt = b2g(wbin_nxt);
    always @(posedge wclk) begin
        if (wrst) begin wbin <= {(AW+1){1'b0}}; wgray <= {(AW+1){1'b0}}; end
        else      begin wbin <= wbin_nxt;       wgray <= wgray_nxt;       end
    end
    always @(posedge wclk) begin
        if (wrst) begin wq1_rg <= {(AW+1){1'b0}}; wq2_rg <= {(AW+1){1'b0}}; end
        else      begin wq1_rg <= rgray;          wq2_rg <= wq1_rg;          end
    end
    always @(posedge wclk) begin
        if (wr_ok) mem[wbin[AW-1:0]] <= wr_word;
    end
    wire [AW:0] rgray_inc = b2g(rbin + 1'b1);
    assign rempty = (rgray_inc == rq2_wg);
    wire        rd_ok     = ren & ~rempty;
    wire [AW:0] rbin_nxt  = rbin + {{AW{1'b0}}, rd_ok};
    wire [AW:0] rgray_nxt = b2g(rbin_nxt);
    always @(posedge rclk) begin
        if (rrst) begin rbin <= {(AW+1){1'b0}}; rgray <= {(AW+1){1'b0}}; end
        else      begin rbin <= rbin_nxt;       rgray <= rgray_nxt;       end
    end
    always @(posedge rclk) begin
        if (rrst) begin rq1_wg <= {(AW+1){1'b0}}; rq2_wg <= {(AW+1){1'b0}}; end
        else      begin rq1_wg <= wgray;          rq2_wg <= rq1_wg;          end
    end
    assign rdata  = mem[rbin[AW-1:0]];
    wire [AW:0] wlevel_bin = g2b(rq2_wg);
    assign rlevel = {6'd0, (wlevel_bin - rbin)};
endmodule
