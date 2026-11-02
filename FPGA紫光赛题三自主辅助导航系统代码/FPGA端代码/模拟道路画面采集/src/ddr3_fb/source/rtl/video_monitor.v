`timescale 1ns / 1ps
module video_monitor #(
    parameter [11:0] H_ACT_EXP    = 12'd1920,
    parameter [11:0] V_ACT_EXP    = 12'd1080,
    parameter [11:0] ACT_TOL       = 12'd8,
    parameter [3:0]  LOCK_FRAMES  = 4'd4,
    parameter [31:0] VS_TIMEOUT   = 32'd4_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire        vs,
    input  wire        de,
    output reg         o_locked,
    output reg  [11:0] o_res_x,
    output reg  [11:0] o_res_y,
    output reg         o_size_ok,
    output reg  [15:0] o_frame_cnt,
    output reg         o_vs_rise
);
    reg vs_d, de_d;
    always @(posedge clk) begin
        if (!rst_n) begin
            vs_d <= 1'b0;
            de_d <= 1'b0;
        end else begin
            vs_d <= vs;
            de_d <= de;
        end
    end
    wire vs_rise = vs & ~vs_d;
    wire de_rise = de & ~de_d;
    wire de_fall = ~de &  de_d;
    reg [11:0] h_cnt;
    reg [11:0] h_max;
    reg [11:0] v_cnt;
    always @(posedge clk) begin
        if (!rst_n) begin
            h_cnt <= 12'd0;
            h_max <= 12'd0;
            v_cnt <= 12'd0;
        end else if (!en) begin
            h_cnt <= 12'd0;
            h_max <= 12'd0;
            v_cnt <= 12'd0;
        end else begin
            if (de_rise)      h_cnt <= 12'd1;
            else if (de)      h_cnt <= h_cnt + 12'd1;
            if (de_fall && (h_cnt > h_max)) h_max <= h_cnt;
            if (de_rise) v_cnt <= v_cnt + 12'd1;
            if (vs_rise) begin
                h_max <= 12'd0;
                v_cnt <= 12'd0;
                h_cnt <= 12'd0;
            end
        end
    end
    reg [3:0]  lock_cnt;
    reg [31:0] vs_idle_cnt;
    always @(posedge clk) begin
        if (!rst_n) begin
            lock_cnt    <= 4'd0;
            vs_idle_cnt <= 32'd0;
        end else if (!en) begin
            lock_cnt    <= 4'd0;
            vs_idle_cnt <= 32'd0;
        end else begin
            if (vs_rise)                    vs_idle_cnt <= 32'd0;
            else if (vs_idle_cnt < VS_TIMEOUT) vs_idle_cnt <= vs_idle_cnt + 32'd1;
            if (vs_rise) begin
                if ((h_max >= (H_ACT_EXP - ACT_TOL)) && (h_max <= (H_ACT_EXP + ACT_TOL)) && (v_cnt >= (V_ACT_EXP - ACT_TOL)) && (v_cnt <= (V_ACT_EXP + ACT_TOL))) begin
                    if (lock_cnt < LOCK_FRAMES) lock_cnt <= lock_cnt + 4'd1;
                end else begin
                    lock_cnt <= 4'd0;
                end
            end
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            o_res_x     <= 12'd0;
            o_res_y     <= 12'd0;
            o_size_ok   <= 1'b0;
            o_frame_cnt <= 16'd0;
            o_vs_rise   <= 1'b0;
            o_locked    <= 1'b0;
        end else begin
            o_vs_rise <= vs_rise;
            if (!en) begin
                o_res_x     <= 12'd0;
                o_res_y     <= 12'd0;
                o_size_ok   <= 1'b0;
                o_frame_cnt <= 16'd0;
                o_locked    <= 1'b0;
            end else if (vs_rise) begin
                o_res_x   <= h_max;
                o_res_y   <= v_cnt;
                o_size_ok <= (h_max >= (H_ACT_EXP - ACT_TOL)) && (h_max <= (H_ACT_EXP + ACT_TOL)) && (v_cnt >= (V_ACT_EXP - ACT_TOL)) && (v_cnt <= (V_ACT_EXP + ACT_TOL));
                if (o_locked) o_frame_cnt <= o_frame_cnt + 16'd1;
            end
            o_locked <= en &&
                        (lock_cnt + 4'd1 >= LOCK_FRAMES) &&
                        (vs_idle_cnt < VS_TIMEOUT);
        end
    end
endmodule
