module ctrl_fsm #(
    parameter [1:0] LANE_DEFAULT = 2'd1,
    parameter [7:0] SLOW_FRAMES  = 8'd8,
    parameter [7:0] MIN_STOP     = 8'd8,
    parameter [1:0] LINE_SOLID   = 2'd0,
    parameter [1:0] LINE_NONE    = 2'd2,
    parameter [1:0] TL_RED       = 2'd0,
    parameter [1:0] TL_YELLOW    = 2'd1,
    parameter [1:0] TL_GREEN     = 2'd2
)(
    input  wire       clk, rst_n,
    input  wire       i_fv,
    input  wire [1:0] i_lane_sel,
    input  wire [1:0] i_line_type,
    input  wire [1:0] i_tl,
    input  wire       i_zebra,
    input  wire       i_obstacle,
    output reg  [1:0] o_lane,
    output reg  [1:0] o_arrow,
    output reg  [2:0] o_led_lane,
    output reg        o_motion,
    output reg  [1:0] o_tl,
    output reg  [1:0] o_line_type,
    output reg        o_alarm
);
localparam [1:0] S_MOVE = 2'd0;
localparam [1:0] S_SLOW = 2'd1;
localparam [1:0] S_STOP = 2'd2;
reg [1:0] st;
reg [7:0] cnt;
reg       solid_viol;
reg [1:0] lane_sel_d;
wire lane_chg       = (i_lane_sel != lane_sel_d);
wire lane_lost      = (i_line_type == LINE_NONE) && !i_zebra;
wire solid_vio_now  = lane_chg && (i_line_type == LINE_SOLID);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st <= S_MOVE; cnt <= 8'd0;
        o_lane <= LANE_DEFAULT; o_arrow <= LANE_DEFAULT; o_led_lane <= 3'b010;
        o_motion <= 1'b0; o_tl <= 2'd3; o_line_type <= LINE_NONE;
        o_alarm <= 1'b0; solid_viol <= 1'b0; lane_sel_d <= LANE_DEFAULT;
    end else if (i_fv) begin
        o_lane      <= i_lane_sel;
        o_arrow     <= i_lane_sel;
        o_led_lane  <= (i_lane_sel == 2'd0) ? 3'b001 :
                       (i_lane_sel == 2'd1) ? 3'b010 : 3'b100;
        o_tl        <= i_tl;
        o_line_type <= i_line_type;
        lane_sel_d  <= i_lane_sel;
        if (solid_vio_now) solid_viol <= 1'b1;
        case (st)
            S_MOVE: begin
                cnt <= 8'd0;
                if ((i_tl == TL_RED) || (i_tl == TL_YELLOW)) st <= S_SLOW;
            end
            S_SLOW: begin
                if (i_tl == TL_GREEN) begin
                    st <= S_MOVE; cnt <= 8'd0;
                end else if (cnt >= SLOW_FRAMES) begin
                    st <= S_STOP; cnt <= 8'd0;
                end else begin
                    cnt <= cnt + 8'd1;
                end
            end
            default: begin
                if (i_tl == TL_GREEN) begin
                    if (cnt >= MIN_STOP) begin st <= S_MOVE; cnt <= 8'd0; end
                    else cnt <= cnt + 8'd1;
                end else begin
                    cnt <= 8'd0;
                end
            end
        endcase
        o_motion <= (st == S_MOVE);
        o_alarm  <= lane_lost | solid_viol | solid_vio_now | i_obstacle;
    end
end
endmodule
