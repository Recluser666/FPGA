`timescale 1ns / 1ps
module sync_fifo #(
    parameter DW = 8,
    parameter AW = 9
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          wr_en,
    input  wire [DW-1:0] wr_data,
    input  wire          rd_en,
    output wire [DW-1:0] rd_data,
    output wire          full,
    output wire          empty,
    output reg  [AW:0]   count
);
    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW-1:0] wptr, rptr;
    assign rd_data = mem[rptr];
    assign full    = (count == (1 << AW));
    assign empty   = (count == {AW+1{1'b0}});
    wire do_wr = wr_en && !full;
    wire do_rd = rd_en && !empty;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr  <= {AW{1'b0}};
            rptr  <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else begin
            if (do_wr) begin
                mem[wptr] <= wr_data;
                wptr      <= wptr + 1'b1;
            end
            if (do_rd) begin
                rptr <= rptr + 1'b1;
            end
            if (do_wr && !do_rd) begin
                count <= count + 1'b1;
            end else if (do_rd && !do_wr) begin
                count <= count - 1'b1;
            end
        end
    end
endmodule
