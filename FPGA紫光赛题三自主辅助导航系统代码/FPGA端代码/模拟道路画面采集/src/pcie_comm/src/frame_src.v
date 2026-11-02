`timescale 1ns / 1ps
module frame_src #(
    parameter integer W   = 64,
    parameter integer H   = 64,
    parameter integer GAP = 24'h0F_0000,
    parameter integer US_DIV = 125
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg         start,
    output reg  [15:0] frame_id,
    output wire [15:0] width,
    output wire [15:0] height,
    output reg  [31:0] ts_us,
    output reg  [7:0]  flag,
    input  wire        payload_rd,
    output wire [7:0]  payload_data,
    output reg         payload_valid
);
    assign width       = W[15:0];
    assign height      = H[15:0];
    assign payload_data = (x_cnt + y_cnt) & 8'hFF;
    reg [23:0] gap_cnt;
    reg [10:0] x_cnt, y_cnt;
    reg        run;
    reg [6:0]  us_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin us_div <= 7'd0; ts_us <= 32'd0; end
        else if (us_div == US_DIV - 1) begin us_div <= 7'd0; ts_us <= ts_us + 32'd1; end
        else us_div <= us_div + 7'd1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start <= 1'b0; frame_id <= 16'd0; flag <= 8'd0;
            gap_cnt <= 24'd0; x_cnt <= 11'd0; y_cnt <= 11'd0;
            run <= 1'b0; payload_valid <= 1'b0;
        end else if (!run) begin
            start <= 1'b0;
            payload_valid <= 1'b0;
            if (gap_cnt == GAP) begin
                gap_cnt   <= 24'd0;
                frame_id  <= frame_id + 16'd1;
                start     <= 1'b1;
                x_cnt     <= 11'd0;
                y_cnt     <= 11'd0;
                run       <= 1'b1;
                payload_valid <= 1'b1;
            end else begin
                gap_cnt <= gap_cnt + 24'd1;
            end
        end else begin
            start <= 1'b0;
            if (payload_rd) begin
                if ((y_cnt == H - 1) && (x_cnt == W - 1)) begin
                    run           <= 1'b0;
                    payload_valid <= 1'b0;
                    x_cnt         <= 11'd0;
                    y_cnt         <= 11'd0;
                end else if (x_cnt == W - 1) begin
                    x_cnt <= 11'd0;
                    y_cnt <= y_cnt + 11'd1;
                end else begin
                    x_cnt <= x_cnt + 11'd1;
                end
            end
        end
    end
endmodule
module pcie_dma_ctrl(
    input wire             clk                ,
    input wire             pix_clk_out        ,
    input wire             rstn               ,
    input wire             axis_master_tvalid ,
    output wire            axis_master_tready ,
    input wire    [127:0]  axis_master_tdata  ,
    input wire    [3:0]    axis_master_tkeep  ,
    input wire             axis_master_tlast  ,
    input wire    [7:0]    axis_master_tuser  ,
    input  [7 : 0]            ep_bus_num      ,
    input  [4 : 0]            ep_dev_num      ,
    input                     AXIS_S_TREADY   ,
    output                    AXIS_S_TVALID   ,
    output [127:0]            AXIS_S_TDATA    ,
    output                    AXIS_S_TLAST    ,
    output                    AXIS_S_TUSER    ,
    output wire            o_fifo_rd_en   ,
    input  wire [127:0]    i_fifo_rd_data ,
    input  wire [12:0]     i_fifo_rd_level,
    input  wire            i_fifo_rd_empty,
    input                  vs_in                ,
    input      [1:0]      i_lane_l            ,
    input      [1:0]      i_lane_r            ,
    output wire [255:0]    dbg_bus             ,
    input  wire [319:0]    i_diag              ,
    output wire [16:0]     dbg_dma_cnt         ,
    output reg             test_en             ,
    output reg             line_lock_en        ,
    output reg             rd_align_en         ,
    output reg             rd_parity_inv       ,
    output reg             rc_lag              ,
    output reg             xclr_robust         ,
    output reg             waddr_lock          ,
    output reg             rdp_fallback        ,
    input  wire            cfg_mem_space_en    ,
    input  wire            cfg_bus_master_en   ,
    input  wire            rbar_ctrl_update    ,
    output reg [7 : 0]        video_enhance_lightdown_num    ,
    output reg                video_enhance_lightdown_sw     ,
    output reg [7 : 0]        video_enhance_darkup_num       ,
    output reg                video_enhance_darkup_sw
   );
wire [12 : 0]    rd_water_level = i_fifo_rd_level;
wire [127:0]    pcie_dma_data  = i_fifo_rd_data;
localparam MWR_32    = 8'h40;
localparam DMA_CMD_L_ADDR                   = 12'h110;
localparam DMA_CMD_CLEAR_ADDR               = 12'h130;
    localparam DMA_CMD_TEST_ADDR                = 12'h100;
localparam DMA_CMD_VIDEO_HIGHTLIGHT_SUB     = 12'h140;
localparam DMA_CMD_VIDEO_LOWLIGHT_ADD       = 12'h150;
localparam DMA_CMD_VIDEO_ENHANCE_CLEAR      = 12'h160;
parameter  ALLOC_ADDR_1   = 4'd1;
parameter  ALLOC_ADDR_2   = 4'd2;
parameter  ALLOC_ADDR_3   = 4'd3;
parameter  ALLOC_ADDR_4   = 4'd4;
parameter  MWR_IDLE       = 2'd0;
parameter  MWR_TLP_HEADER = 2'd1;
parameter  MWR_TLP_DATA   = 2'd2;
parameter  TLP_LENGTH     = 10'd16;
    parameter  FRAME_DIV2      = 1;
parameter  FRAME_W        = 1920;
parameter  FRAME_H        = 1080;
parameter  PIX_BYTES      = 2;
localparam [31:0] FRAME_PIX     = FRAME_W*FRAME_H*PIX_BYTES;
localparam [31:0] BYTES_PER_TLP = TLP_LENGTH*4;
localparam [31:0] DATA_TLPS     = FRAME_PIX/BYTES_PER_TLP;
parameter  DEBUG_ONE_TLP = 0;
parameter  USE_FIXED_ADDR = 0;
localparam [31:0] BUF0_ADDR = 32'h0200_0000;
localparam [31:0] BUF1_ADDR = 32'h0300_0000;
localparam [31:0] BUF2_ADDR = 32'h0340_0000;
localparam [31:0] BUF3_ADDR = 32'h0380_0000;
localparam [31:0] DMA_TRAN_TIMES = (DEBUG_ONE_TLP != 0) ? 32'd2 : (DATA_TLPS + 1);
localparam [31:0] FOOTER_MAGIC  = 32'h53414441;
localparam [15:0] FOOTER_W      = FRAME_W;
localparam [15:0] FOOTER_H      = FRAME_H;
localparam [15:0] FOOTER_FMT    = 16'd1;
wire [15:0] FOOTER_FLAGS = { 5'd0, i_lane_r, i_lane_l, 7'd0 };
localparam [15:0] FOOTER_PIXB   = PIX_BYTES;
reg [1:0]   mwr_state ;
reg         r_axis_s_tvalid;
reg [127:0] r_axis_s_tdata;
reg         r_axis_s_tlast;
reg [10 :0] r_tlp_length_cnt;
reg [127:0] axis_master_tdata_d0    ;
wire [31:0] cmd_word = {axis_master_tdata_d0[7:0],
                        axis_master_tdata_d0[15:8],
                        axis_master_tdata_d0[23:16],
                        axis_master_tdata_d0[31:24]};
reg         axis_master_tvalid_d0   ;
reg         axis_master_tvalid_d1   ;
reg [2 : 0] tlp_fmt             ;
reg [4 : 0] tlp_type            ;
reg         mwr32               ;
reg [31: 0] mwr_addr            ;
reg [11: 0] cmd_reg_addr        ;
reg [9 : 0] tlp_lenght           ;
reg         rc_cfg_ep_flag      ;
reg [31: 0] alloc_addrl         ;
reg [3 : 0] alloc_addr_state    ;
reg [1 : 0] cur_buf             ;
reg [1 : 0] fixed_buf_sel       ;
reg [31: 0] dma_addr0           ;
reg [31: 0] dma_addr1           ;
reg [31: 0] dma_addr2           ;
reg [31: 0] dma_addr3           ;
reg         dma_addr_cfg_flag   ;
reg         dma_stop_flag       ;
reg [1 : 0] rc_cfg_cnt          ;
reg [16: 0] dma_cnt             ;
reg         rc_cfg_ep_flag_d0   ;
reg         vs_in_d0            ;
reg         vs_in_d1            ;
reg         r_pre_rd_flag        ;
reg         fram_start        ;
reg         dma_start        ;
reg         pcie_rd_en        ;
assign o_fifo_rd_en = pcie_rd_en && (mwr_state == MWR_TLP_DATA) && AXIS_S_TREADY;
reg [31: 0]  fram_seq    ;
wire fixed_addr  = (USE_FIXED_ADDR != 0);
wire addrs_valid = fixed_addr ? 1'b1 :
                   ((dma_addr0 != 32'd0) && (dma_addr1 != 32'd0) &&
                    (dma_addr2 != 32'd0) && (dma_addr3 != 32'd0));
reg        axis_master_tvalid_d;
reg [15:0] dbg_tvalid_cnt;
always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        axis_master_tvalid_d <= 1'b0;
        dbg_tvalid_cnt       <= 16'd0;
    end else begin
        axis_master_tvalid_d <= axis_master_tvalid;
        if(axis_master_tvalid && !axis_master_tvalid_d)
            dbg_tvalid_cnt <= dbg_tvalid_cnt + 16'd1;
    end
end
reg        rbar_ctrl_update_d0;
reg        rbar_ctrl_update_d1;
reg [7:0]  rbar_update_cnt;
always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        rbar_ctrl_update_d0 <= 1'b0;
        rbar_ctrl_update_d1 <= 1'b0;
        rbar_update_cnt     <= 8'd0;
    end else begin
        rbar_ctrl_update_d0 <= rbar_ctrl_update;
        rbar_ctrl_update_d1 <= rbar_ctrl_update_d0;
        if(rbar_ctrl_update_d0 && !rbar_ctrl_update_d1)
            rbar_update_cnt <= rbar_update_cnt + 8'd1;
    end
end
reg [127:0] hdr_snap;
always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        hdr_snap <= 128'd0;
    end else if((mwr_state == MWR_TLP_DATA) && (r_tlp_length_cnt == 0) && r_axis_s_tvalid) begin
        hdr_snap <= r_axis_s_tdata;
    end
end
wire addr_safe = (alloc_addrl != 32'd0);
assign dbg_bus[31:0]    = {16'd0, dbg_tvalid_cnt};
assign dbg_bus[63:32]   = hdr_snap[31:0];
assign dbg_bus[95:64]   = hdr_snap[63:32];
assign dbg_bus[127:96]  = hdr_snap[95:64];
assign dbg_bus[159:128] = hdr_snap[127:96];
assign dbg_bus[191:160] = {15'd0, dma_cnt};
assign dbg_bus[223:192] = {rbar_update_cnt, 4'd0,
                           cfg_bus_master_en, cfg_mem_space_en, rbar_ctrl_update_d0,
                           dma_start, rc_cfg_ep_flag, addrs_valid, dma_stop_flag,
                           rd_water_level[12:0]};
assign dbg_bus[255:224] = hdr_snap[95:64];
assign dbg_dma_cnt       = dma_cnt;
assign axis_master_tready = 'd1;
assign AXIS_S_TVALID = r_axis_s_tvalid;
assign AXIS_S_TDATA  = r_axis_s_tdata ;
assign AXIS_S_TLAST  = r_axis_s_tlast ;
assign AXIS_S_TUSER  = 'd0;
always  @(posedge clk) begin
    if(!rstn) begin
        axis_master_tdata_d0 <= 'd0;
        axis_master_tvalid_d0 <= 'd0;
        rc_cfg_ep_flag_d0 <= 'd0;
        vs_in_d0 <= 'd0;
        vs_in_d1 <= 'd0;
    end
    else begin
        axis_master_tdata_d0 <= axis_master_tdata;
        axis_master_tvalid_d0 <= axis_master_tvalid;
        axis_master_tvalid_d1 <= axis_master_tvalid_d0;
        rc_cfg_ep_flag_d0 <= rc_cfg_ep_flag;
        vs_in_d0 <= vs_in;
        vs_in_d1 <= vs_in_d0;
    end
end
wire vs_fall_raw = (!vs_in_d0 && vs_in_d1);
wire vs_rise_raw = ( vs_in_d0 && !vs_in_d1);
reg        vs_skip;
reg        frame_do;
always @(posedge clk) begin
    if (!rstn) begin
        vs_skip  <= 1'b0;
        frame_do <= 1'b0;
    end
    else if (vs_fall_raw) begin
        frame_do <= ~vs_skip;
        vs_skip  <= ~vs_skip;
    end
end
wire vs_fall = (FRAME_DIV2 != 0) ? (vs_fall_raw && ~vs_skip) : vs_fall_raw;
wire vs_rise = (FRAME_DIV2 != 0) ? (vs_rise_raw &&  frame_do) : vs_rise_raw;
always  @(posedge clk) begin
    if(!rstn) begin
        tlp_fmt  <= 'd0;
        tlp_type <= 'd0;
        mwr32    <= 'd0;
        mwr_addr <= 'd0;
        cmd_reg_addr <= 'd0;
        tlp_lenght <= 'd0;
    end
    else if(axis_master_tvalid_d0 && !(axis_master_tvalid_d1))begin
        tlp_fmt  <= axis_master_tdata_d0[31:29];
        tlp_type <= axis_master_tdata_d0[28:24];
        mwr_addr <= axis_master_tdata_d0[95:64];
        cmd_reg_addr <= axis_master_tdata_d0[75:64];
        tlp_lenght <= axis_master_tdata_d0[9:0];
    end
    else begin
        tlp_fmt  <= 'd0;
        tlp_type <= 'd0;
        mwr_addr <= 'd0;
        cmd_reg_addr <= 'd0;
        tlp_lenght <= 'd0;
    end
end
always  @(posedge clk) begin
    if(!rstn) begin
        alloc_addr_state  <= ALLOC_ADDR_1;
        dma_addr0         <= fixed_addr ? BUF0_ADDR : 32'd0;
        dma_addr1         <= fixed_addr ? BUF1_ADDR : 32'd0;
        dma_addr2         <= fixed_addr ? BUF2_ADDR : 32'd0;
        dma_addr3         <= fixed_addr ? BUF3_ADDR : 32'd0;
        dma_stop_flag     <= 'd0;
        test_en           <= 1'b0;
        line_lock_en      <= 1'b0;
        rd_align_en       <= 1'b0;
        rd_parity_inv     <= 1'b0;
        rc_lag            <= 1'b0;
        xclr_robust       <= 1'b0;
        waddr_lock        <= 1'b0;
        rdp_fallback      <= 1'b0;
        video_enhance_lightdown_num <= 'd0;
        video_enhance_lightdown_sw  <= 'd0;
        video_enhance_darkup_num    <= 'd0;
        video_enhance_darkup_sw     <= 'd0;
    end
    else if(!fixed_addr && ({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_CLEAR_ADDR)) begin
        alloc_addr_state  <= ALLOC_ADDR_1;
        dma_addr0         <= 'd0;
        dma_addr1         <= 'd0;
        dma_addr2         <= 'd0;
        dma_addr3         <= 'd0;
        dma_stop_flag     <= 1'b1;
        video_enhance_lightdown_num <= 'd0;
        video_enhance_lightdown_sw  <= 'd0;
        video_enhance_darkup_num    <= 'd0;
        video_enhance_darkup_sw     <= 'd0;
    end
    else if(!fixed_addr && ({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_TEST_ADDR)) begin
        test_en       <= cmd_word[0];
        line_lock_en  <= cmd_word[1];
        rd_align_en   <= cmd_word[2];
        rd_parity_inv <= cmd_word[3];
        rc_lag        <= cmd_word[4];
        xclr_robust   <= cmd_word[5];
        waddr_lock    <= cmd_word[6];
        rdp_fallback  <= cmd_word[7];
    end
    else if(!fixed_addr && ({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_L_ADDR)) begin
        case(alloc_addr_state)
            ALLOC_ADDR_1:
            begin
                dma_addr0  <= {axis_master_tdata_d0[7:0],axis_master_tdata_d0[15:8],axis_master_tdata_d0[23:16],axis_master_tdata_d0[31:24]};
                alloc_addr_state <= ALLOC_ADDR_2;
            end
            ALLOC_ADDR_2:
            begin
                dma_addr1  <= {axis_master_tdata_d0[7:0],axis_master_tdata_d0[15:8],axis_master_tdata_d0[23:16],axis_master_tdata_d0[31:24]};
                alloc_addr_state <= ALLOC_ADDR_3;
            end
            ALLOC_ADDR_3:
            begin
                dma_addr2  <= {axis_master_tdata_d0[7:0],axis_master_tdata_d0[15:8],axis_master_tdata_d0[23:16],axis_master_tdata_d0[31:24]};
                alloc_addr_state <= ALLOC_ADDR_4;
            end
            ALLOC_ADDR_4:
            begin
                dma_addr3  <= {axis_master_tdata_d0[7:0],axis_master_tdata_d0[15:8],axis_master_tdata_d0[23:16],axis_master_tdata_d0[31:24]};
                alloc_addr_state <= ALLOC_ADDR_1;
                dma_stop_flag <= 1'b0;
            end
        endcase
    end
    else if(({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_VIDEO_LOWLIGHT_ADD)) begin
        video_enhance_darkup_num <= axis_master_tdata_d0[31:24];
        video_enhance_darkup_sw <= 1'b1;
    end
    else if(({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_VIDEO_HIGHTLIGHT_SUB)) begin
        video_enhance_lightdown_num <= axis_master_tdata_d0[31:24];
        video_enhance_lightdown_sw <= 1'b1;
    end
    else if(({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_VIDEO_ENHANCE_CLEAR)) begin
        video_enhance_lightdown_sw <= 1'b0;
        video_enhance_darkup_sw    <= 1'b0;
        video_enhance_lightdown_num <= 'd0;
        video_enhance_darkup_num   <= 'd0;
    end
    else begin
        alloc_addr_state  <=  alloc_addr_state ;
        dma_addr0         <=  dma_addr0        ;
        dma_addr1         <=  dma_addr1        ;
        dma_addr2         <=  dma_addr2        ;
        dma_addr3         <=  dma_addr3        ;
        dma_addr_cfg_flag <=  dma_addr_cfg_flag;
        video_enhance_lightdown_num <= video_enhance_lightdown_num;
        video_enhance_lightdown_sw  <= video_enhance_lightdown_sw ;
        video_enhance_darkup_num    <= video_enhance_darkup_num   ;
        video_enhance_darkup_sw     <= video_enhance_darkup_sw    ;
    end
end
always  @(posedge clk) begin
    if(!rstn || dma_stop_flag || !addrs_valid || !addr_safe) begin
        rc_cfg_ep_flag  <= 'd0;
        dma_cnt         <= 'd0;
        fram_start      <= 'd0;
        dma_start       <= 'd0;
    end
    else begin
        if(addrs_valid) begin
            rc_cfg_ep_flag <= 'd1;
        end
        if(vs_fall && rc_cfg_ep_flag && addrs_valid) begin
            fram_start <= 'd1;
            dma_start  <= 'd0;
        end
        if(fram_start && vs_rise) begin
            fram_start <= 'd0;
        end
        if(AXIS_S_TLAST) begin
            dma_start <= 'd0;
            if(dma_cnt >= DMA_TRAN_TIMES - 1) begin
                dma_cnt <= 'd0;
            end
            else begin
                dma_cnt <= dma_cnt +'d1;
            end
        end
        else if( addrs_valid && ( ((dma_cnt == 0)&&(rd_water_level >= TLP_LENGTH/4 + 1))|| ((dma_cnt != 0)&&(rd_water_level >= TLP_LENGTH/4)) || ((rd_water_level >= TLP_LENGTH/4 - 1) &&( dma_cnt == DMA_TRAN_TIMES - 2)) || dma_cnt == DMA_TRAN_TIMES-1) ) begin
            dma_start <= 'd1;
        end
    end
end
always @(posedge clk) begin
    if(!rstn || dma_stop_flag || !addrs_valid) begin
        r_axis_s_tvalid  <= 'd0;
        r_axis_s_tdata   <= 'd0;
        r_axis_s_tlast   <= 'd0;
        r_tlp_length_cnt <= 'd0;
        r_pre_rd_flag    <= 'd0;
        pcie_rd_en       <= 'd0;
        alloc_addrl      <= fixed_addr ? BUF0_ADDR : 32'd0;
        cur_buf          <= 'd0;
        fixed_buf_sel    <= 'd0;
        fram_seq         <= 'd1;
    end
    else if (vs_rise) begin
        r_axis_s_tvalid  <= 'd0;
        r_axis_s_tdata   <= 'd0;
        r_axis_s_tlast   <= 'd0;
        r_tlp_length_cnt <= 'd0;
        r_pre_rd_flag    <= 'd0;
        pcie_rd_en       <= 'd0;
    end
    else if (vs_fall) begin
        if (fixed_addr) begin
            case (fixed_buf_sel)
                2'd0: alloc_addrl <= BUF0_ADDR;
                2'd1: alloc_addrl <= BUF1_ADDR;
                2'd2: alloc_addrl <= BUF2_ADDR;
                2'd3: alloc_addrl <= BUF3_ADDR;
            endcase
            fixed_buf_sel <= fixed_buf_sel + 2'd1;
        end
        else if (addrs_valid && (mwr_state == MWR_IDLE) && (dma_cnt == 0)) begin
            case (cur_buf)
                2'd0: alloc_addrl <= dma_addr0;
                2'd1: alloc_addrl <= dma_addr1;
                2'd2: alloc_addrl <= dma_addr2;
                2'd3: alloc_addrl <= dma_addr3;
            endcase
        end
    end
    else if (fram_start && dma_start && rc_cfg_ep_flag) begin
        case(mwr_state)
            MWR_IDLE:
            begin
                r_axis_s_tvalid <= 'd0;
                r_axis_s_tdata  <= 'd0;
                r_axis_s_tlast  <= 'd0;
                if(r_pre_rd_flag == 0) begin
                    pcie_rd_en <= 'd1;
                    r_pre_rd_flag <= 'd1;
                end
                else begin
                    pcie_rd_en <= 'd0;
                end
            end
            MWR_TLP_HEADER:
            begin
                r_axis_s_tvalid <= 'd1;
                r_axis_s_tlast  <= 'd0;
                r_axis_s_tdata[9  :  0]  <= TLP_LENGTH;
                r_axis_s_tdata[11 : 10]  <= 'h0;
                r_axis_s_tdata[13 : 12]  <= 'h0;
                r_axis_s_tdata[14]       <= 'h0;
                r_axis_s_tdata[15]       <= 'h0;
                r_axis_s_tdata[16]       <= 'h0;
                r_axis_s_tdata[17]       <= 'h0;
                r_axis_s_tdata[18]       <= 'h0;
                r_axis_s_tdata[19]       <= 'h0;
                r_axis_s_tdata[22 : 20]  <= 'h0;
                r_axis_s_tdata[23]       <= 'h0;
                r_axis_s_tdata[28 : 24]  <= 'h0;
                r_axis_s_tdata[31 : 29]  <= 3'b010;
                r_axis_s_tdata[35 : 32]  <= 4'hf;
                r_axis_s_tdata[39 : 36]  <= 4'hf;
                r_axis_s_tdata[47 : 40]  <= 8'h01;
                r_axis_s_tdata[63 : 48]  <= {ep_bus_num,ep_dev_num,3'b0};
                r_axis_s_tdata[95 : 64]  <= alloc_addrl;
                r_axis_s_tdata[127: 96]  <= 32'd0;
                pcie_rd_en <= 'd1;
            end
            MWR_TLP_DATA:
            begin
                if(AXIS_S_TREADY) begin
                    if(dma_cnt <= DMA_TRAN_TIMES - 2) begin
                        r_axis_s_tdata <= {
                                            pcie_dma_data[103: 96],pcie_dma_data[111:104],pcie_dma_data[119:112],pcie_dma_data[127:120],
                                            pcie_dma_data[71 : 64],pcie_dma_data[79 : 72],pcie_dma_data[87 : 80],pcie_dma_data[95 : 88],
                                            pcie_dma_data[39 : 32],pcie_dma_data[47 : 40],pcie_dma_data[55 : 48],pcie_dma_data[63 : 56],
                                            pcie_dma_data[7  :  0],pcie_dma_data[15 :  8],pcie_dma_data[23 : 16],pcie_dma_data[31 : 24] };
                        pcie_rd_en <= 'd1;
                    end
                    else if(dma_cnt >= DMA_TRAN_TIMES - 1) begin
                        case (r_tlp_length_cnt[1:0])
                            2'd0: r_axis_s_tdata <= { {FOOTER_FLAGS, FOOTER_FMT},
                                                      {FOOTER_H,    FOOTER_W  },
                                                      fram_seq,
                                                      FOOTER_MAGIC };
                            2'd1: r_axis_s_tdata <= { {i_diag[63:48], i_diag[47:32]},
                                                      {i_diag[31:16], i_diag[15:0]},
                                                      FRAME_PIX,
                                                      {16'h0, FOOTER_PIXB} };
                            2'd2: r_axis_s_tdata <= { {15'd0, test_en, i_diag[175:160]},
                                                      {i_diag[159:144], i_diag[143:128]},
                                                      {i_diag[127:112], i_diag[111:96]},
                                                      {i_diag[95:80],   i_diag[79:64]} };
                            2'd3: r_axis_s_tdata <= {
                                                      {i_diag[319:304], i_diag[303:288]},
                                                      {i_diag[287:272], i_diag[271:256]},
                                                      {i_diag[255:240], i_diag[239:224]},
                                                      {i_diag[223:208], 15'd0, line_lock_en} };
                            default: r_axis_s_tdata <= 128'h0;
                        endcase
                        pcie_rd_en <= 'd0;
                    end
                    r_tlp_length_cnt <= r_tlp_length_cnt + 1'd1;
                    if(AXIS_S_TLAST) begin
                        r_tlp_length_cnt <= 'd0;
                        r_axis_s_tvalid  <= 'd0;
                        r_axis_s_tdata   <= 'd0;
                        r_axis_s_tlast   <= 'd0;
                        if(dma_cnt >= DMA_TRAN_TIMES - 1) begin
                            fram_seq <= fram_seq + 32'd1;
                            cur_buf  <= cur_buf + 2'd1;
                            case (cur_buf + 2'd1)
                                2'd0: alloc_addrl <= dma_addr0;
                                2'd1: alloc_addrl <= dma_addr1;
                                2'd2: alloc_addrl <= dma_addr2;
                                2'd3: alloc_addrl <= dma_addr3;
                            endcase
                        end
                        else begin
                            alloc_addrl <= alloc_addrl + TLP_LENGTH*4;
                        end
                    end
                    else if(((TLP_LENGTH <= 4) || ((TLP_LENGTH > 4) && (r_tlp_length_cnt >= TLP_LENGTH/4 - 1)) && AXIS_S_TREADY)) begin
                        r_axis_s_tlast  <= 'd1;
                        pcie_rd_en       <= 'd0;
                    end
                end
                else begin
                    pcie_rd_en <= 'd0;
                end
            end
            default:
            begin
                r_axis_s_tvalid <= 'd0;
                r_axis_s_tdata  <= 'd0;
                r_axis_s_tlast  <= 'd0;
                pcie_rd_en      <='d0 ;
            end
        endcase
    end
end
always @(posedge clk) begin
    if(!rstn || vs_rise || dma_stop_flag || !addrs_valid || !addr_safe) begin
        mwr_state <= 'd0;
    end
    else if(fram_start && dma_start && rc_cfg_ep_flag) begin
        case(mwr_state)
            MWR_IDLE:
            begin
                if(AXIS_S_TREADY &&
                   ((dma_cnt >= DMA_TRAN_TIMES - 1) || (rd_water_level >= 13'd16))) begin
                    mwr_state <= MWR_TLP_HEADER;
                end
                else begin
                    mwr_state <= mwr_state;
                end
            end
            MWR_TLP_HEADER:
            begin
                if(AXIS_S_TREADY) begin
                    mwr_state <= MWR_TLP_DATA;
                end
                else begin
                    mwr_state <= mwr_state;
                end
            end
            MWR_TLP_DATA:
            begin
                if(AXIS_S_TLAST) begin
                    mwr_state <= MWR_IDLE;
                end
                else begin
                    mwr_state <= mwr_state;
                end
            end
            default:
            begin
                    mwr_state <= mwr_state;
            end
        endcase
    end
    else begin
        mwr_state <= mwr_state;
    end
end
endmodule
