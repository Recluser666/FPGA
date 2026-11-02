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
    input                  vs_in                ,
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
parameter  FRAME_W        = 1920;
parameter  FRAME_H        = 1080;
parameter  PIX_BYTES      = 2;
localparam [31:0] FRAME_PIX     = FRAME_W*FRAME_H*PIX_BYTES;
localparam [31:0] BYTES_PER_TLP = TLP_LENGTH*4;
localparam [31:0] DATA_TLPS     = FRAME_PIX/BYTES_PER_TLP;
parameter  DMA_TRAN_TIMES = DATA_TLPS + 1;
localparam [31:0] FOOTER_MAGIC  = 32'h53414441;
localparam [15:0] FOOTER_W      = FRAME_W;
localparam [15:0] FOOTER_H      = FRAME_H;
localparam [15:0] FOOTER_FMT    = 16'd1;
localparam [15:0] FOOTER_FLAGS  = 16'h0000;
localparam [15:0] FOOTER_PIXB   = PIX_BYTES;
reg [1:0]   mwr_state ;
reg         r_axis_s_tvalid;
reg [127:0] r_axis_s_tdata;
reg         r_axis_s_tlast;
reg [10 :0] r_tlp_length_cnt;
reg [127:0] axis_master_tdata_d0    ;
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
reg [1 : 0] addr_page           ;
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
    assign o_fifo_rd_en = pcie_rd_en;
reg [31: 0]  fram_seq    ;
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
    if(!rstn || dma_stop_flag) begin
        alloc_addr_state  <= ALLOC_ADDR_1;
        dma_addr0         <= 'd0;
        dma_addr1         <= 'd0;
        dma_addr2         <= 'd0;
        dma_addr3         <= 'd0;
        dma_stop_flag     <= 'd0;
        video_enhance_lightdown_num <= 'd0;
        video_enhance_lightdown_sw  <= 'd0;
        video_enhance_darkup_num    <= 'd0;
        video_enhance_darkup_sw     <= 'd0;
    end
    else if(({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_L_ADDR)) begin
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
            end
        endcase
    end
    else if(({tlp_fmt,tlp_type} == MWR_32 )&& (tlp_lenght == 1) && (cmd_reg_addr == DMA_CMD_CLEAR_ADDR)) begin
        dma_stop_flag <= 1'b1;
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
    if(!rstn  || dma_stop_flag) begin
        rc_cfg_ep_flag  <= 'd0;
        dma_cnt         <= 'd0;
        fram_start      <= 'd0;
        dma_start       <= 'd0;
    end
    else begin
        if((dma_addr0 != 32'b0) && (dma_addr1 != 32'b0) && (dma_addr2 != 32'b0) && (dma_addr3 != 32'b0)) begin
            rc_cfg_ep_flag <= 'd1;
        end
        if(!vs_in_d0 && vs_in_d1 && rc_cfg_ep_flag) begin
            fram_start <= 'd1;
            dma_start  <= 'd0;
        end
        if(fram_start && (vs_in_d0 && !vs_in_d1)) begin
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
        else if( ((dma_cnt == 0)&&(rd_water_level >= TLP_LENGTH/4 + 1))|| ((dma_cnt != 0)&&(rd_water_level >= TLP_LENGTH/4)) || ((rd_water_level >= TLP_LENGTH/4 - 1) &&( dma_cnt == DMA_TRAN_TIMES - 2)) || dma_cnt == DMA_TRAN_TIMES-1) begin
            dma_start <= 'd1;
        end
    end
end
always @(posedge clk) begin
    if(!rstn  || dma_stop_flag) begin
        r_axis_s_tvalid  <= 'd0;
        r_axis_s_tdata   <= 'd0;
        r_axis_s_tlast   <= 'd0;
        r_tlp_length_cnt <= 'd0;
        r_pre_rd_flag    <= 'd0;
        pcie_rd_en       <= 'd0;
        alloc_addrl      <= 'd0;
        addr_page        <= 'd0;
        fram_seq         <= 'd1;
    end
    else if (vs_in_d0 && !vs_in_d1) begin
        r_axis_s_tvalid  <= 'd0;
        r_axis_s_tdata   <= 'd0;
        r_axis_s_tlast   <= 'd0;
        r_tlp_length_cnt <= 'd0;
        r_pre_rd_flag    <= 'd0;
        pcie_rd_en       <= 'd0;
        addr_page        <= addr_page + 'd1;
        if(addr_page == 0) begin
            alloc_addrl      <= dma_addr0;
        end
        else if(addr_page == 1) begin
            alloc_addrl      <= dma_addr1;
        end
        else if(addr_page == 2) begin
            alloc_addrl      <= dma_addr2;
        end
        else if(addr_page == 3) begin
            alloc_addrl      <= dma_addr3;
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
                    end
                    else if(dma_cnt >= DMA_TRAN_TIMES - 1) begin
                        case (r_tlp_length_cnt[1:0])
                            2'd0: r_axis_s_tdata <= { {FOOTER_FLAGS, FOOTER_FMT},
                                                      {FOOTER_H,    FOOTER_W  },
                                                      fram_seq,
                                                      FOOTER_MAGIC };
                            2'd1: r_axis_s_tdata <= { 32'h0, 32'h0,
                                                      FRAME_PIX,
                                                      {16'h0, FOOTER_PIXB} };
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
                        alloc_addrl <= alloc_addrl + TLP_LENGTH*4;
                        if(dma_cnt >= DMA_TRAN_TIMES - 1) begin
                            fram_seq <= fram_seq + 32'd1;
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
    if(!rstn || (vs_in_d0 && !vs_in_d1)  || dma_stop_flag) begin
        mwr_state <= 'd0;
    end
    else if(fram_start && dma_start && rc_cfg_ep_flag) begin
        case(mwr_state)
            MWR_IDLE:
            begin
                if(AXIS_S_TREADY) begin
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
