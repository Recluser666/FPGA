module cam_frm_wr #(
    parameter integer H_PIX     = 960,
    parameter integer SRC_LINES = 720,
    parameter integer BEATS     = 60
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         line_new,
    input  wire [15:0]  lb_px,
    output reg  [11:0]  lb_addr,
    input  wire         vs_in,
    input  wire         ddr_clk,
    output reg  [27:0]  o_waddr,
    output reg  [31:0]  o_wlen,
    output reg          o_wreq,
    input  wire         o_wrdy,
    input  wire         o_wdone,
    input  wire         o_wdata_req,
    output wire [255:0] o_wdata,
    output reg  [11:0]  o_line,
    output reg          o_wbank,
    output reg          o_done_tgl,
    output reg  [11:0]  o_lines
);
    reg  [11:0] c;
    reg  [15:0] px_hold;
    reg  [11:0] wr_idx;
    reg         wr_we;
    reg  [31:0] wr_data32;
    reg         line_ready;
    reg  [2:0]  st;
    localparam S_IDLE = 3'd0;
    localparam S_RD   = 3'd1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            c          <= 12'd0;
            px_hold    <= 16'd0;
            wr_idx     <= 12'd0;
            wr_we      <= 1'b0;
            wr_data32  <= 32'd0;
            line_ready <= 1'b0;
            lb_addr    <= 12'd0;
        end else begin
            wr_we <= 1'b0;
            case (st)
            S_IDLE: begin
                if (line_new && !line_ready) begin
                    lb_addr <= 12'd0;
                    c       <= 12'd0;
                    wr_idx  <= 12'd0;
                    st      <= S_RD;
                end
            end
            S_RD: begin
                lb_addr <= c + 12'd1;
                if (c[0] == 1'b0) begin
                    px_hold <= lb_px;
                end else begin
                    wr_data32 <= {lb_px, px_hold};
                    wr_we     <= 1'b1;
                    wr_idx    <= wr_idx + 12'd1;
                end
                if (c == H_PIX[11:0] - 12'd1) begin
                    c          <= 12'd0;
                    st         <= S_IDLE;
                    line_ready <= 1'b1;
                end else begin
                    c <= c + 12'd1;
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end
    reg  [5:0] beat_idx;
    reg  [31:0] rd_q;
    reg  [255:0] lb_q;
    reg        req_d;
    wire [8:0]  lb_waddr = wr_idx[8:0] >> 3;
    wire [11:0] lb_raddr = {5'd0, beat_idx, 3'b000};
    always @(posedge ddr_clk or negedge rst_n) begin
        if (!rst_n) begin
            o_wreq     <= 1'b0;
            o_waddr    <= 28'd0;
            o_wlen     <= 32'd0;
            o_line     <= 12'd0;
            o_wbank    <= 1'b0;
            o_done_tgl <= 1'b0;
            o_lines    <= 12'd0;
            beat_idx   <= 6'd0;
            req_d      <= 1'b0;
        end else begin
            req_d <= o_wdata_req;
            if (line_ready && !o_wreq) begin
                o_wreq  <= 1'b1;
                o_waddr <= { {(28-12-21){1'b0}}, o_wbank, o_line } * 28'd1
                           + (o_line * 28'd480);
                o_wlen  <= BEATS;
                beat_idx<= 6'd0;
            end
            if (o_wdata_req) beat_idx <= beat_idx + 6'd1;
            if (o_wdone) begin
                o_wreq <= 1'b0;
                beat_idx <= 6'd0;
                if (o_line == SRC_LINES[11:0] - 12'd1) begin
                    o_line     <= 12'd0;
                    o_wbank    <= ~o_wbank;
                    o_done_tgl <= ~o_done_tgl;
                    o_lines    <= SRC_LINES[11:0];
                end else begin
                    o_line <= o_line + 12'd1;
                end
            end
        end
    end
    wire [255:0] lb_rdata;
    assign o_wdata = (o_wdata_req && !req_d) ? lb_rdata_d : lb_rdata;
    reg [255:0] lb_rdata_d;
    reg [2:0]   rd_pipe;
    always @(posedge ddr_clk or negedge rst_n) begin
        if (!rst_n) begin
            lb_rdata_d <= 256'd0;
            rd_pipe    <= 3'd0;
        end else begin
            lb_rdata_d <= lb_rdata;
            rd_pipe    <= lb_raddr[10:8];
        end
    end
    wr_fram_buf u_lb (
        .wr_data (wr_data32),
        .wr_addr (lb_waddr ),
        .wr_en   (wr_we    ),
        .wr_clk  (clk      ),
        .wr_rst  (~rst_n   ),
        .rd_addr (lb_raddr ),
        .rd_data (lb_rdata ),
        .rd_clk  (ddr_clk  ),
        .rd_rst  (~rst_n   )
    );
    reg [2:0] lr_s;
    always @(posedge ddr_clk or negedge rst_n) begin
        if (!rst_n) lr_s <= 3'd0;
        else        lr_s <= {lr_s[1:0], line_ready};
    end
endmodule
