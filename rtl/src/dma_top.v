// Project: DMA Controller
// Author: czz
// Date: 2025-06-07
// Description: Top-level module for DMA controller
// this module integrates the DMA CSR and core, handling AXI transactions and control signals.
// suported features include:
// - APB interface for control and status registers
// - AXI4 interface for data transfer，supporting 32-bit and 64-bit data widths
// - Multiple DMA descriptors for flexible data transfer
// - Error handling and status reporting
// - Burst transfer support
// - support incremental and jump mode
// This module is designed to be used in a system-on-chip (SoC) environment, where it can be connected to other peripherals and memory.

module dma_top (
input           clk,           // main clk ,same as APB clock and AXI clock
input           rst_n,         // main rst_n ,same as APB reset and AXI reset

output          intr_dma_done,  // DMA done interrupt
output          intr_dma_err,   // DMA error interrupt

input           psel,
input           penable,
input           pwrite,
input  [11:0]   paddr,
input  [31:0]   pwdata,
output [31:0]   prdata,
output          pready,
output          pslverr,       // APB slave error,not used in this design
output          axim_awlock,    // Lock signal        1'b0
output  [3:0]   axim_awcache,   // Cache type         4'b0011
output  [2:0]   axim_awprot,    // Protection type    3'b0
output  [3:0]   axim_awqos,     // Quality of Service 4'b0
output  [31:0]  axim_awaddr,    // Write address
output  [7:0]   axim_awlen,     // Burst length
output  [2:0]   axim_awsize,    // Burst size
output  [1:0]   axim_awburst,   // Burst type
output          axim_awvalid,   // Write address valid
input           axim_awready,   // Write address ready
output  [63:0]  axim_wdata,     // Write data
output  [7:0]   axim_wstrb,     // Write strobes
output          axim_wlast,     // Write last
output          axim_wvalid,    // Write data valid
input           axim_wready,    // Write data ready
input   [1:0]   axim_bresp,     // Write response
input           axim_bvalid,    // Write response valid
output          axim_bready,    // Write response ready
output          axim_arlock,    // Lock signal         1'b0
output  [3:0]   axim_arcache,   // Cache type          4'b0011
output  [2:0]   axim_arprot,    // Protection type     3'b0
output  [3:0]   axim_arqos,     // Quality of Service  4'b0
output  [31:0]  axim_araddr,    // Read address
output  [7:0]   axim_arlen,     // Burst length
output  [2:0]   axim_arsize,    // Burst size
output  [1:0]   axim_arburst,   // Burst type
output          axim_arvalid,   // Read address valid
input           axim_arready,   // Read address ready
input   [63:0]  axim_rdata,     // Read data
input   [1:0]   axim_rresp,     // Read response
input           axim_rlast,     // Read last
input           axim_rvalid,    // Read data valid
output          axim_rready,    // Read data ready
output  [7:0]   axim_awid,      // Write ID,not used
input   [7:0]   axim_bid,       // Write response ID,not used 
output  [7:0]   axim_arid,      // Read ID,not used
input   [7:0]   axim_rid        // Read response ID,not used

);
// Parameters for DMA controller
parameter DMA_ADDR_WIDTH  = 32;                     // Address width for DMA
parameter DMA_DATA_WIDTH  = 64;                     // Data width for DMA (32 or 64 bits)
parameter DMA_BYTES_WIDTH = 32;                     // Bytes width for DMA
parameter DMA_NUM_DESC    = 2;                      // Number of DMA descriptors
parameter AXI_ID_WIDTH    = 8;                      // AXI ID width for 8-bit or 16-bit ID.not used in this design
parameter AXI_STRB_WIDTH  = DMA_DATA_WIDTH/8;       // AXI strobe width for 32-bit or 64-bit data
parameter DESC_IDX_WIDTH  = $clog2(DMA_NUM_DESC);   // Descriptor index width
parameter RD_ADDR_BUFF_DEPTH  = 8;                  // Depth of read address buffer
parameter WR_ADDR_BUFF_DEPTH  = 8;                  // Depth of write address buffer
parameter DATA_BUFF_DEPTH     = 64;                 // Depth of data buffer

wire [07:0]   csr_dma_maxburst;
wire          csr_dma_start;
wire [15:0]   csr_dma_version;
wire [01:0]   csr_dma_status;
wire          csr_dma_err;
wire          csr_dma_done;
wire          csr_dma_err_clr;
wire [01:0]   csr_dma_err_type;
wire [31:0]   csr_dma_err_addr;
wire          csr_desc_read_mode[1:0];
wire          csr_desc_write_mode[1:0];
wire          csr_desc_enable[1:0];
wire [31:0]   csr_desc_num_bytes[1:0];
wire [31:0]   csr_desc_src_addr[1:0];
wire [31:0]   csr_desc_dst_addr[1:0];
wire [31:0]   csr_desc_read_jump_bytes[1:0];
wire [31:0]   csr_desc_write_jump_bytes[1:0];

assign csr_dma_version = 16'hABCD;
assign pslverr = 1'b0; // No slave error in this design
assign intr_dma_done = csr_dma_done;
assign intr_dma_err = csr_dma_err;

dma_csr u_dma_csr(
.pclk                           (clk),
.presetn                        (rst_n),
.psel                           (psel),
.penable                        (penable),
.pwrite                         (pwrite),
.paddr                          (paddr),
.pwdata                         (pwdata),
.prdata                         (prdata),
.pready                         (pready),
.csr_dma_maxburst               (csr_dma_maxburst),
.csr_dma_start                  (csr_dma_start),
.csr_dma_version                (csr_dma_version),
.csr_dma_status                 (csr_dma_status),
.csr_dma_err                    (csr_dma_err),
.csr_dma_done                   (csr_dma_done),
.csr_dma_err_clr                (csr_dma_err_clr),
.csr_dma_err_type               (csr_dma_err_type),
.csr_dma_err_addr               (csr_dma_err_addr),
.csr_desc_read_mode             (csr_desc_read_mode),
.csr_desc_write_mode            (csr_desc_write_mode),
.csr_desc_enable                (csr_desc_enable),
.csr_desc_num_bytes             (csr_desc_num_bytes),
.csr_desc_src_addr              (csr_desc_src_addr),
.csr_desc_dst_addr              (csr_desc_dst_addr),
.csr_desc_read_jump_bytes       (csr_desc_read_jump_bytes),
.csr_desc_write_jump_bytes      (csr_desc_write_jump_bytes)
);

// DMA core instance
dma_core #(
.DMA_ADDR_WIDTH                 (DMA_ADDR_WIDTH),
.DMA_DATA_WIDTH                 (DMA_DATA_WIDTH),
.DMA_BYTES_WIDTH                (DMA_BYTES_WIDTH),
.DMA_NUM_DESC                   (DMA_NUM_DESC),
.AXI_ID_WIDTH                   (AXI_ID_WIDTH),
.AXI_STRB_WIDTH                 (AXI_STRB_WIDTH),
.DESC_IDX_WIDTH                 (DESC_IDX_WIDTH),
.RD_ADDR_BUFF_DEPTH             (RD_ADDR_BUFF_DEPTH),
.WR_ADDR_BUFF_DEPTH             (WR_ADDR_BUFF_DEPTH),
.DATA_BUFF_DEPTH                (DATA_BUFF_DEPTH)
) u_dma_core (
.clk                            (clk),
.rst_n                          (rst_n),
.csr_desc_enable                (csr_desc_enable),
.csr_desc_src_addr              (csr_desc_src_addr),
.csr_desc_dst_addr              (csr_desc_dst_addr),
.csr_desc_num_bytes             (csr_desc_num_bytes),
.csr_desc_write_mode            (csr_desc_write_mode),
.csr_desc_read_mode             (csr_desc_read_mode),
.csr_desc_write_jump_bytes      (csr_desc_write_jump_bytes),
.csr_desc_read_jump_bytes       (csr_desc_read_jump_bytes),
.csr_dma_maxburst               (csr_dma_maxburst),
.csr_dma_start                  (csr_dma_start),
.csr_dma_done                   (csr_dma_done),    
.csr_dma_status                 (csr_dma_status),   
.csr_dma_err_clr                (csr_dma_err_clr),
.csr_dma_err                    (csr_dma_err), 
.csr_dma_err_type               (csr_dma_err_type), 
.csr_dma_err_addr               (csr_dma_err_addr),
.axim_awlock                    (axim_awlock),
.axim_awcache                   (axim_awcache),
.axim_awprot                    (axim_awprot),
.axim_awqos                     (axim_awqos),
.axim_awaddr                    (axim_awaddr),
.axim_awlen                     (axim_awlen),
.axim_awsize                    (axim_awsize),
.axim_awburst                   (axim_awburst),
.axim_awvalid                   (axim_awvalid),
.axim_awready                   (axim_awready),
.axim_wdata                     (axim_wdata),
.axim_wstrb                     (axim_wstrb),
.axim_wlast                     (axim_wlast),
.axim_wvalid                    (axim_wvalid),
.axim_wready                    (axim_wready),
.axim_bresp                     (axim_bresp),
.axim_bvalid                    (axim_bvalid),
.axim_bready                    (axim_bready),
.axim_arlock                    (axim_arlock),
.axim_arcache                   (axim_arcache),
.axim_arprot                    (axim_arprot),
.axim_arqos                     (axim_arqos),
.axim_araddr                    (axim_araddr),
.axim_arlen                     (axim_arlen),
.axim_arsize                    (axim_arsize),
.axim_arburst                   (axim_arburst),
.axim_arvalid                   (axim_arvalid),
.axim_arready                   (axim_arready),
.axim_rdata                     (axim_rdata),
.axim_rresp                     (axim_rresp),
.axim_rlast                     (axim_rlast),
.axim_rvalid                    (axim_rvalid),
.axim_rready                    (axim_rready),
.axim_awid                      (axim_awid),                 
.axim_bid                       (axim_bid), 
.axim_arid                      (axim_arid),
.axim_rid                       (axim_rid)
);

endmodule

