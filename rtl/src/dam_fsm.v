// Project: DMA Controller
// Author: czz
// Date: 2025-06-07
// Description: This module implements a finite state machine (FSM) for a DMA controller.
// 1.manages the state transitions between idle, check, run, and done states.
// 2.handles DMA descriptor configurations, including source and destination addresses, number of bytes, and read/write modes.
// 3.manages the DMA slice indices for read and write operations.one by one run the DMA descriptors, and when all descriptors are done, it sets the done signal.
// 4.checks for configuration errors and handles DMA operations based on the provided descriptors.
 
module dma_fsm #(
parameter DMA_ADDR_WIDTH  = 32, // Address width for DMA
parameter DMA_DATA_WIDTH  = 64, // Data width for DMA (32 or 64 bits)
parameter DMA_BYTES_WIDTH = 32, // Bytes width for DMA
parameter DMA_NUM_DESC    = 8,  // Number of DMA descriptors
parameter DESC_IDX_WIDTH = $clog2(DMA_NUM_DESC) // Descriptor index width
)(
input                                               clk,
input                                               rst_n,
// From/To CSRs

input                                               csr_desc_enable             [DMA_NUM_DESC-1:0], // Enable for each descriptor address
input   [DMA_ADDR_WIDTH-1:0]                        csr_desc_src_addr           [DMA_NUM_DESC-1:0],
input   [DMA_ADDR_WIDTH-1:0]                        csr_desc_dst_addr           [DMA_NUM_DESC-1:0],
input   [DMA_BYTES_WIDTH-1:0]                       csr_desc_num_bytes          [DMA_NUM_DESC-1:0],
input                                               csr_desc_write_mode         [DMA_NUM_DESC-1:0],// 0:increment, 1:jump; when jump, busrt length is 0,the address must aligned to 4 bytes/8bytes.
input                                               csr_desc_read_mode          [DMA_NUM_DESC-1:0],// 0:increment, 1:jump; when jump, busrt length is 0,the address must aligned to 4 bytes/8bytes.
input   [DMA_BYTES_WIDTH-1:0]                       csr_desc_write_jump_bytes   [DMA_NUM_DESC-1:0],
input   [DMA_BYTES_WIDTH-1:0]                       csr_desc_read_jump_bytes    [DMA_NUM_DESC-1:0],

input                                               csr_dma_start,
output                                              csr_dma_done,    
output  [1:0]                                       csr_dma_status, // 00: idle, 01: check, 10: run, 11: done   
input                                               csr_dma_err_clr,
output                                              csr_dma_err, 
output  [1:0]                                       csr_dma_err_type, // 2'b10: config error; 2'b00: aix read resp error,2'b01: axi write resp error
output  [DMA_ADDR_WIDTH-1:0]                        csr_dma_err_addr,

// To/From axi2fifo
input                                               dma_axi_err_valid,
input   [DMA_ADDR_WIDTH-1:0]                        dma_axi_err_addr,
input                                               dma_axi_err_type, // 1'b0 for read, 1'b1 for write
// To/From DMA slice rx
output  reg [DESC_IDX_WIDTH-1:0]                    dma_rd_slice_idx,
output                                              dma_rd_slice_valid,
input                                               dma_rd_slice_done,

// To/From DMA slice tx
output  reg [DESC_IDX_WIDTH-1:0]                    dma_tx_slice_idx,
output                                              dma_tx_slice_valid,
input                                               dma_tx_slice_done,

// To/From axi2fifo
input                                               dma_axi_pending
);


localparam LSB_MASK   = (DMA_DATA_WIDTH == 32) ? 2 : 3; // LSB mask for 32-bit or 64-bit data width

localparam DMA_IDLE    = 2'b00;
localparam DMA_CHECK   = 2'b01;
localparam DMA_RUN     = 2'b10;
localparam DMA_DONE    = 2'b11;

reg [1:0] curt_state;
reg [1:0] next_state;

reg       dma_config_error;
wire      dma_desc_pending;

always@ (posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        curt_state  <= 'b0;
    end else begin
        curt_state  <= next_state;
    end
end

always@(*) begin
    case (curt_state)
        DMA_IDLE:       next_state = csr_dma_start ? DMA_CHECK : DMA_IDLE;
        DMA_CHECK:      next_state = dma_config_error ? DMA_DONE : DMA_RUN;
        DMA_RUN:        next_state = (dma_axi_pending | dma_desc_pending) ? DMA_RUN : DMA_DONE;
        DMA_DONE:       next_state = csr_dma_start ? DMA_DONE : DMA_IDLE;
        default:        next_state = DMA_IDLE;
    endcase
end

// DMA dma_rd_slice indx process according to the descriptor configuration
// This process finds the next valid read descriptor that is not yet done.
reg [DMA_NUM_DESC-1:0] dma_rd_need_trans;
always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        dma_rd_need_trans <= 'b0;
    end else begin
        if (curt_state == DMA_DONE) begin
            dma_rd_need_trans <= 'b0;
        end else if (curt_state == DMA_CHECK && (dma_config_error == 1'b0)) begin//lock enable in this state
            for(int i = 0;i < DMA_NUM_DESC;i++)begin
                if(csr_desc_enable[i])begin
                    dma_rd_need_trans[i] <= 1'b1;
                end
            end
        end else if (dma_rd_slice_done) begin
            dma_rd_need_trans[dma_rd_slice_idx] <= 1'b0;
        end
    end
end

always @(*) begin
    dma_rd_slice_idx = 'b0;
    for (int i = 0; i < DMA_NUM_DESC; i++) begin
        if (dma_rd_need_trans[i]) begin
            dma_rd_slice_idx = i;
            break; // Exit loop once a valid descriptor is found
        end
    end    
end
assign dma_rd_slice_valid = |dma_rd_need_trans;

//DMA dma_tx_slice indx process according to the descriptor configuration
reg [DMA_NUM_DESC-1:0] dma_tx_need_trans;
always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        dma_tx_need_trans <= 'b0;
    end else begin
        if (curt_state == DMA_DONE) begin
            dma_tx_need_trans <= 'b0;
        end else if ((curt_state == DMA_CHECK) && (dma_config_error == 1'b0)) begin
            for(int i = 0;i < DMA_NUM_DESC;i++)begin//lock enable in this state
                if(csr_desc_enable[i])begin
                    dma_tx_need_trans[i] <= 1'b1;
                end
            end
        end else if (dma_tx_slice_done) begin
            dma_tx_need_trans[dma_tx_slice_idx] <= 1'b0;
        end
    end
end

always @(*) begin
    dma_tx_slice_idx = 'b0;
    for (int i = 0; i < DMA_NUM_DESC; i++) begin
        if (dma_tx_need_trans[i]) begin
            dma_tx_slice_idx = i;
            break; // Exit loop once a valid descriptor is found
        end
    end    
end
assign dma_tx_slice_valid = |dma_tx_need_trans;


assign dma_desc_pending = dma_tx_slice_valid | dma_rd_slice_valid | dma_rd_slice_done | dma_tx_slice_done;
assign csr_dma_done = (curt_state == DMA_DONE);
assign csr_dma_status = curt_state;


//check dma config error
// If any descriptor is enabled, check if the configuration is valid
// If both read and write modes are increment, check if source and destination last addresses are same.for example, if source address is 0x0001 and destination address is 0x0002, then the last addresses are not same.
// If write mode is jump, check if jump bytes are valid,and if the jump address is aligned to 4 bytes/8 bytes.
// If read mode is jump, check if jump bytes are valid,and if the jump address is aligned to 4 bytes/8 bytes.
// If read or write modes is jump , check if source and destination addresses are aligned to 4 bytes/8 bytes.
reg err_flg;
always@(*) begin
    err_flg = 1'b0;
    for (int i = 0; i < DMA_NUM_DESC; i++) begin
        if (csr_desc_enable[i]) begin
            if( csr_desc_num_bytes[i] == 0 
            || ((~csr_desc_write_mode[i]) && (~csr_desc_read_mode[i]) && (csr_desc_src_addr[i][LSB_MASK-1:0] != csr_desc_dst_addr[i][LSB_MASK-1:0]))             
            || (csr_desc_write_mode[i] && (csr_desc_write_jump_bytes[i] == 0 || csr_desc_write_jump_bytes[i][LSB_MASK-1:0] != 0))
            || (csr_desc_read_mode[i] && (csr_desc_read_jump_bytes[i] == 0 || csr_desc_read_jump_bytes[i][LSB_MASK-1:0] != 0))
            || ((csr_desc_write_mode[i] || csr_desc_read_mode[i]) && (csr_desc_src_addr[i][LSB_MASK-1:0] != 0 || csr_desc_dst_addr[i][LSB_MASK-1:0] != 0)))begin
                err_flg = 1'b1;   
            end
        end       
    end
end
always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        dma_config_error <= 1'b0;
    end else begin
        if(csr_dma_err_clr)begin
            dma_config_error <= 1'b0;
        end else if ((curt_state == DMA_IDLE) && csr_dma_start) begin
            dma_config_error <= err_flg;
        end
    end
end

assign csr_dma_err = dma_config_error | dma_axi_err_valid;
assign csr_dma_err_type = {dma_config_error,dma_axi_err_type};
assign csr_dma_err_addr = dma_config_error ? 'b0 : dma_axi_err_addr;

endmodule


