// Project: DMA Controller
// Author: czz
// Date: 2025-06-07
// Description: This module implements a descriptor-based finite state machine (FSM) for a DMA controller.
// it generates axi requests based on the descriptor

module dma_slice #(
parameter DMA_ADDR_WIDTH  = 32, // Address width for DMA
parameter DMA_DATA_WIDTH  = 64, // Data width for DMA (32 or 64 bits)
parameter DMA_BYTES_WIDTH = 32, // Bytes width for DMA
parameter DMA_NUM_DESC    = 8,  // Number of DMA descriptors
parameter AXI_STRB_WIDTH  = DMA_DATA_WIDTH/8,  // AXI write strobe width
parameter DESC_IDX_WIDTH  = $clog2(DMA_NUM_DESC)// Descriptor index width
)(
input                                           clk,
input                                           rst_n,
// From/To CSRs

input   [DMA_ADDR_WIDTH-1:0]                    csr_desc_addr       [DMA_NUM_DESC-1:0],
input                                           csr_desc_mode       [DMA_NUM_DESC-1:0],// 0:increment, 1:jump; when jump, busrt length is 0,the address must aligned to 4 bytes/8bytes.
input   [DMA_BYTES_WIDTH-1:0]                   csr_desc_num_bytes  [DMA_NUM_DESC-1:0],
input   [DMA_BYTES_WIDTH-1:0]                   csr_desc_jump_bytes [DMA_NUM_DESC-1:0],
input   [7:0]                                   csr_dma_maxburst,

// From/To AXI I/F
output  reg                                     dma_axi_req_valid,
output  reg  [DMA_ADDR_WIDTH-1:0]               dma_axi_req_addr,
output  reg  [AXI_STRB_WIDTH-1:0]               dma_axi_req_strb,
output  reg  [2:0]                              dma_axi_req_size,
output  reg  [7:0]                              dma_axi_req_alen,

input                                           dma_axi_resp_ready,
// To/From DMA FSM
input        [DESC_IDX_WIDTH-1:0]               dma_slice_idx,
input                                           dma_slice_valid,
output                                          dma_slice_done
);

localparam   DMA_ST_SM_IDLE = 1'b0;
localparam   DMA_ST_SM_RUN  = 1'b1;

wire                                    dma_slice_start;
reg                                     mode; // 0:increment, 1:jump
reg     [DMA_BYTES_WIDTH-1:0]           jump_bytes;
reg                                     curt_state;
reg                                     next_state;
wire    [7:0]                           burst_len; // Burst length in bytes    
wire    [AXI_STRB_WIDTH-1:0]            wstrb;     // Write strobe for current transaction
reg     [DMA_BYTES_WIDTH-1:0]           send_byte; // Bytes to be sent in current transaction
reg     [DMA_ADDR_WIDTH-1:0]            current_addr; 
reg     [DMA_BYTES_WIDTH-1:0]           remain_bytes; // Remaining bytes to be sent in current transaction

always@(*) begin
    case (curt_state)
        DMA_ST_SM_IDLE: begin
            if (dma_slice_valid) begin
                next_state = DMA_ST_SM_RUN;
            end else begin
                next_state = DMA_ST_SM_IDLE;
            end
        end
        DMA_ST_SM_RUN: begin
            if (remain_bytes == 0) begin
                next_state = DMA_ST_SM_IDLE;
            end else begin
                next_state = DMA_ST_SM_RUN;
            end
        end
    endcase
end

always@ (posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        curt_state  <= 'b0;
    end else begin
        curt_state  <= next_state;
    end
end


assign dma_slice_start = (curt_state == DMA_ST_SM_IDLE) && (next_state == DMA_ST_SM_RUN);
assign dma_slice_done = (curt_state == DMA_ST_SM_RUN) && (next_state == DMA_ST_SM_IDLE);

assign dma_axi_req_size = (DMA_DATA_WIDTH == 32) ? 3'b010 : 3'b011; // 4 bytes or 8 bytes


always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        dma_axi_req_valid <= 1'b0;
    end else begin
        if(curt_state == DMA_ST_SM_RUN) begin
            if((dma_axi_resp_ready & dma_axi_req_valid) || (remain_bytes == 0)) begin
                dma_axi_req_valid <= 1'b0;
            end else begin
                dma_axi_req_valid <= 1'b1;
            end
        end
    end
end

always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        mode       <= 'h0;
        jump_bytes <= 'h0;
    end else begin
        if(dma_slice_start) begin
            mode       <= csr_desc_mode[dma_slice_idx];
            jump_bytes <= csr_desc_jump_bytes[dma_slice_idx];
        end 
    end
end

always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        current_addr <= 'h0;
        remain_bytes <= 'h0;
    end else begin
        if(dma_slice_start) begin
            current_addr <= csr_desc_addr[dma_slice_idx][DMA_ADDR_WIDTH-1:0];
            remain_bytes <= csr_desc_num_bytes[dma_slice_idx][DMA_BYTES_WIDTH-1:0];
        end else if (curt_state == DMA_ST_SM_RUN) begin
            if (dma_axi_resp_ready & dma_axi_req_valid) begin
                if (mode == 1'b0) begin // Increment mode
                    current_addr <= current_addr + send_byte;
                end else begin // Jump mode
                    current_addr <= current_addr + jump_bytes;
                end
                remain_bytes <= remain_bytes - send_byte;
            end
        end 
    end
end

assign burst_len = calc_burst_len(current_addr, remain_bytes,mode,csr_dma_maxburst);
assign wstrb = calc_wstrb(current_addr, remain_bytes);


always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        dma_axi_req_addr <= 'h0;
        dma_axi_req_alen <= 'h0;
        dma_axi_req_strb <= 'h0;
        send_byte        <= 'h0;
    end else begin
        if (curt_state == DMA_ST_SM_RUN) begin
            if (~dma_axi_req_valid) begin
                dma_axi_req_addr <= aligned_addr(current_addr);
                dma_axi_req_alen <= burst_len;
                dma_axi_req_strb <= wstrb;
                send_byte        <= calc_send_byte(burst_len, wstrb);
            end 
        end 
    end
end









//----------------------------------------------------------------------------------------------------------
// Function definitions
// These functions are used to calculate aligned address, burst length, write strobe, and send byte
//----------------------------------------------------------------------------------------------------------
function [DMA_ADDR_WIDTH-1:0] aligned_addr;
    input [DMA_ADDR_WIDTH-1:0] addr;
    if(DMA_DATA_WIDTH == 32) begin
        aligned_addr = {addr[DMA_ADDR_WIDTH-1:2],2'b00};
    end else if(DMA_DATA_WIDTH == 64) begin
        aligned_addr = {addr[DMA_ADDR_WIDTH-1:3],3'b000};
    end
endfunction

function [7:0] calc_burst_len;
    input [DMA_ADDR_WIDTH-1:0]   addr;
    input [DMA_BYTES_WIDTH-1:0]  bytes;
    input                        mode;
    input [7:0]                  csr_dma_maxburst;
    reg   [12:0]                 offset_4k;//for 4096
    reg   [10:0]                 max_burst_tmp;
    reg   [DMA_BYTES_WIDTH-1:0]  bytes_burst;
    if(DMA_DATA_WIDTH == 32)begin

        offset_4k = {1'b0,~addr[11:0]} + 1'b1; // 4K offset
        max_burst_tmp = offset_4k[12:2]; // 4K offset in 4 bytes
        if(max_burst_tmp > csr_dma_maxburst) 
            max_burst_tmp = {3'b0,csr_dma_maxburst};
        // Calculate burst length,cut off the last 2 bits,if the last 2 bits not 0, add 1 traction
        bytes_burst = {2'b0,bytes[DMA_BYTES_WIDTH-1:2]} ;
        if((addr[1:0] != 2'b0) || (bytes_burst == 'b0) || (mode == 1'b1)) // split first unaligned access or last access,and if mode is jump,burst length is 0
            calc_burst_len = 8'b0;
        else if(bytes_burst < max_burst_tmp)
            calc_burst_len = bytes_burst - 1;
        else
            calc_burst_len = max_burst_tmp - 1;

    end else if(DMA_DATA_WIDTH == 64)begin

        offset_4k = {1'b0,~addr[11:0]} + 1'b1; // 4K offset
        //offset_4k = 13'd4096 - addr[11:0]; // 4K offset
        max_burst_tmp = {1'b0,offset_4k[12:3]}; // 4K offset in 4 bytes
        if(max_burst_tmp > csr_dma_maxburst) 
            max_burst_tmp = {3'b0,csr_dma_maxburst};
        // Calculate burst length,cut off the last 3 bits,if the last 3 bits not 0, add 1 traction
        bytes_burst = {3'b0,bytes[DMA_BYTES_WIDTH-1:3]} ;
        if((addr[2:0] != 3'b0) || (bytes_burst == 'b0)|| (mode == 1'b1)) // split first unaligned access or last access,and if mode is jump,burst length is 0
            calc_burst_len = 8'b0;
        else if(bytes_burst < max_burst_tmp)
            calc_burst_len = bytes_burst - 1;
        else
            calc_burst_len = max_burst_tmp - 1;
    end
endfunction

function [AXI_STRB_WIDTH-1:0] calc_wstrb;
    input [DMA_ADDR_WIDTH-1:0]  addr;
    input [DMA_BYTES_WIDTH-1:0] bytes;
    if(DMA_DATA_WIDTH == 32)begin
        if (bytes[DMA_BYTES_WIDTH-1:2] != 0) begin
            calc_wstrb = 4'b1111<<addr[1:0]; // 4 bytes aligned, all bytes are valid
        end else begin
            case(bytes[1:0])
                2'b00: calc_wstrb = 4'b1111<<addr[1:0]; // byte >= 4
                2'b01: calc_wstrb = 4'b0001<<addr[1:0]; // byte = 1
                2'b10: calc_wstrb = 4'b0011<<addr[1:0]; // byte = 2
                2'b11: calc_wstrb = 4'b0111<<addr[1:0]; // byte = 3
                default: calc_wstrb = 4'b1111; 
            endcase
        end
    end else if(DMA_DATA_WIDTH == 64)begin
        if (bytes[DMA_BYTES_WIDTH-1:3] != 0) begin
            calc_wstrb = 8'b11111111<<addr[2:0]; // 8 bytes aligned, all bytes are valid
        end else begin
            case(bytes[2:0])
                3'b000: calc_wstrb = 8'b11111111<<addr[2:0]; // byte >= 8
                3'b001: calc_wstrb = 8'b00000001<<addr[2:0]; // byte = 1
                3'b010: calc_wstrb = 8'b00000011<<addr[2:0]; // byte = 2
                3'b011: calc_wstrb = 8'b00000111<<addr[2:0]; // byte = 3
                3'b100: calc_wstrb = 8'b00001111<<addr[2:0]; // byte = 4
                3'b101: calc_wstrb = 8'b00011111<<addr[2:0]; // byte = 5
                3'b110: calc_wstrb = 8'b00111111<<addr[2:0]; // byte = 6
                3'b111: calc_wstrb = 8'b01111111<<addr[2:0]; // byte = 7
                default: calc_wstrb = 8'b11111111; 
            endcase
        end 
        
    end
endfunction

function [DMA_BYTES_WIDTH-1:0] calc_send_byte;
    input [7:0]                  burst_len;
    input [AXI_STRB_WIDTH-1:0]  wstrb;
    if(DMA_DATA_WIDTH == 32)begin
        if (burst_len != 0) begin
            calc_send_byte = {{burst_len+1'b1},2'b0};//muiltiply by 4
        end else begin
            calc_send_byte = wstrb[0] + wstrb[1] + wstrb[2] + wstrb[3];
        end
    end else if(DMA_DATA_WIDTH == 64)begin
        if (burst_len != 0) begin
            calc_send_byte = {{burst_len+1'b1},3'b0};//muiltiply by 8
        end else begin
            calc_send_byte = wstrb[0] + wstrb[1] + wstrb[2] + wstrb[3] + wstrb[4] + wstrb[5] + wstrb[6] + wstrb[7];
        end
    end
endfunction

endmodule
