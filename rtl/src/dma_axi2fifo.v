// Project: DMA Controller
// Author: czz
// Date: 2025-06-07
// Description: axi2fifo module for DMA controller
// This module handles the AXI transactions and buffers data for DMA operations.

module dma_axi2fifo#(
parameter DMA_ADDR_WIDTH      = 32, // Address width for DMA
parameter DMA_DATA_WIDTH      = 64, // Data width for DMA (32 or 64 bits)
parameter DMA_NUM_DESC        = 8,  // Number of DMA descriptors
parameter AXI_ID_WIDTH        = 4,  // AXI ID width
parameter AXI_STRB_WIDTH      = DMA_DATA_WIDTH/8,  // AXI write strobe width
parameter RD_ADDR_BUFF_DEPTH  = 8, // Depth of read address buffer
parameter WR_ADDR_BUFF_DEPTH  = 8, // Depth of write address buffer
parameter DATA_BUFF_DEPTH     = 64   // Depth of data buffer
)(
input                                           clk,
input                                           rst_n,

// axi master interface
output wire                                     axim_awlock,    // Lock signal        1'b0
output wire [3:0]                               axim_awcache,   // Cache type         4'b0011
output wire [2:0]                               axim_awprot,    // Protection type    3'b0
output wire [3:0]                               axim_awqos,     // Quality of Service 4'b0
output wire [DMA_ADDR_WIDTH-1:0]                axim_awaddr,    // Write address
output wire [7:0]                               axim_awlen,     // Burst length
output wire [2:0]                               axim_awsize,    // Burst size
output wire [1:0]                               axim_awburst,   // Burst type
output wire                                     axim_awvalid,   // Write address valid
input  wire                                     axim_awready,   // Write address ready
output wire [DMA_DATA_WIDTH-1:0]                axim_wdata,     // Write data
output wire [AXI_STRB_WIDTH-1:0]                axim_wstrb,     // Write strobes
output wire                                     axim_wlast,     // Write last
output wire                                     axim_wvalid,    // Write data valid
input  wire                                     axim_wready,    // Write data ready
input  wire [1:0]                               axim_bresp,     // Write response
input  wire                                     axim_bvalid,    // Write response valid
output wire                                     axim_bready,    // Write response ready
output wire                                     axim_arlock,    // Lock signal         1'b0
output wire [3:0]                               axim_arcache,   // Cache type          4'b0011
output wire [2:0]                               axim_arprot,    // Protection type     3'b0
output wire [3:0]                               axim_arqos,     // Quality of Service  4'b0
output wire [DMA_ADDR_WIDTH-1:0]                axim_araddr,    // Read address
output wire [7:0]                               axim_arlen,     // Burst length
output wire [2:0]                               axim_arsize,    // Burst size
output wire [1:0]                               axim_arburst,   // Burst type
output wire                                     axim_arvalid,   // Read address valid
input  wire                                     axim_arready,   // Read address ready
input  wire [DMA_DATA_WIDTH-1:0]                axim_rdata,     // Read data
input  wire [1:0]                               axim_rresp,     // Read response
input  wire                                     axim_rlast,     // Read last
input  wire                                     axim_rvalid,    // Read data valid
output wire                                     axim_rready,    // Read data ready
output wire [AXI_ID_WIDTH-1:0]                  axim_awid,      // Write ID
input  wire [AXI_ID_WIDTH-1:0]                  axim_bid,       // Write response ID 
output wire [AXI_ID_WIDTH-1:0]                  axim_arid,      // Read ID
input  wire [AXI_ID_WIDTH-1:0]                  axim_rid,       // Read response ID
// From/To dma slice
input                                           dma_axi_rd_req_valid,
input       [DMA_ADDR_WIDTH-1:0]                dma_axi_rd_req_addr,
input       [AXI_STRB_WIDTH-1:0]                dma_axi_rd_req_strb,
input       [2:0]                               dma_axi_rd_req_size,
input       [7:0]                               dma_axi_rd_req_alen,
output                                          dma_axi_rd_resp_ready,

input                                           dma_axi_wr_req_valid,
input       [DMA_ADDR_WIDTH-1:0]                dma_axi_wr_req_addr,
input       [AXI_STRB_WIDTH-1:0]                dma_axi_wr_req_strb,
input       [2:0]                               dma_axi_wr_req_size,
input       [7:0]                               dma_axi_wr_req_alen,
output                                          dma_axi_wr_resp_ready,

output  reg                                     axi_pending,
input                                           dma_axi_err_clr,
output  reg                                     dma_axi_err_valid,
output  reg [DMA_ADDR_WIDTH-1:0]                dma_axi_err_addr,// Address of the error
output  reg                                     dma_axi_err_type // 1'b0 for read, 1'b1 for write
);

wire                        rd_addr_fifo_wr_en;
wire                        rd_addr_fifo_rd_en;
wire [DMA_ADDR_WIDTH-1:0]   rd_addr_fifo_dout;
wire                        rd_addr_fifo_full;
wire                        rd_addr_fifo_empty;

wire                        wr_addr_fifo_wr_en;
wire                        wr_addr_fifo_rd_en;
wire [DMA_ADDR_WIDTH-1:0]   wr_addr_fifo_dout;
wire                        wr_addr_fifo_full;
wire                        wr_addr_fifo_empty;

wire                        wr_alen_fifo_wr_en;
wire                        wr_alen_fifo_rd_en;
wire [8+AXI_STRB_WIDTH-1:0] wr_alen_fifo_dout;


wire                        data_fifo_wr_en;
wire                        data_fifo_rd_en;
wire [DMA_DATA_WIDTH-1:0]   data_fifo_dout;
wire                        data_fifo_full;
wire                        data_fifo_empty;

assign axim_bready = 1'b1;
assign axim_arid    = 'b0; // AXI ID for read address
assign axim_awid    = 'b0; // AXI ID for write address
assign axim_arburst = 2'b01; // INCR burst type
assign axim_arlock = 1'b0; // Normal access
assign axim_arcache = 4'b0011; // Normal cacheable access
assign axim_arprot = 3'b000; // No special protection
assign axim_arqos = 4'b0000; // No special QoS
assign axim_awburst = 2'b01; // INCR burst type
assign axim_awlock  = 1'b0; // Normal access
assign axim_awcache = 4'b0011; // Normal cacheable access
assign axim_awprot  = 3'b000; // No special protection
assign axim_awqos   = 4'b0000; // No special QoS


reg [7:0] wr_data_count;
always@ (posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        wr_data_count <= 8'b0;
    end else begin
        if (axim_wvalid && axim_wready) begin
            wr_data_count <= wr_data_count + 1'b1;
        end else if (axim_bvalid && axim_bready) begin
            wr_data_count <= 8'b0; // Reset count after write response
        end
    end
end


// Stores the read address requests
dma_fifo #(
  .DEPTH        (RD_ADDR_BUFF_DEPTH),
  .WIDTH        (DMA_ADDR_WIDTH)
) u_fifo_rd_addr_error (
  .clk          (clk),
  .rst_n        (rst_n),
  .wr_en        (rd_addr_fifo_wr_en),
  .rd_en        (rd_addr_fifo_rd_en),
  .din          (dma_axi_rd_req_addr),
  .dout         (rd_addr_fifo_dout),
  .error        (),
  .full         (rd_addr_fifo_full),
  .empty        (rd_addr_fifo_empty),
  .ocup_cnt     (),
  .clear        (1'b0),
  .free_cnt     ()
);

// Stores the rd data to tx 
dma_fifo #(
  .DEPTH        (DATA_BUFF_DEPTH),
  .WIDTH        (DMA_DATA_WIDTH)
) u_fifo_data_error (
  .clk          (clk),
  .rst_n        (rst_n),
  .wr_en        (data_fifo_wr_en),
  .rd_en        (data_fifo_rd_en),
  .din          (axim_rdata),
  .dout         (data_fifo_dout),
  .error        (),
  .full         (data_fifo_full),
  .empty        (data_fifo_empty),
  .ocup_cnt     (),
  .clear        (1'b0),
  .free_cnt     ()
);

// Stores the write address requests
dma_fifo #(
  .DEPTH        (WR_ADDR_BUFF_DEPTH),
  .WIDTH        (DMA_ADDR_WIDTH)
) u_fifo_wd_addr_error (
  .clk          (clk),
  .rst_n        (rst_n),
  .wr_en        (wr_addr_fifo_wr_en),
  .rd_en        (wr_addr_fifo_rd_en),
  .din          (dma_axi_wr_req_addr),
  .dout         (wr_addr_fifo_dout),
  .error        (),
  .full         (wr_addr_fifo_full),
  .empty        (wr_addr_fifo_empty),
  .ocup_cnt     (),
  .clear        (1'b0),
  .free_cnt     ()
);

// Stores the write length
dma_fifo #(
  .DEPTH        (WR_ADDR_BUFF_DEPTH),
  .WIDTH        ((8+AXI_STRB_WIDTH))
) u_fifo_wd_alen_error (
  .clk          (clk),
  .rst_n        (rst_n),
  .wr_en        (wr_alen_fifo_wr_en),
  .rd_en        (wr_alen_fifo_rd_en),
  .din          ({dma_axi_wr_req_strb,dma_axi_wr_req_alen}),
  .dout         (wr_alen_fifo_dout),
  .error        (),
  .full         (),
  .empty        (),
  .ocup_cnt     (),
  .clear        (1'b0),
  .free_cnt     ()
);


//----------------------
//axi rd addr process
//----------------------
assign rd_addr_fifo_wr_en = dma_axi_rd_resp_ready;
assign rd_addr_fifo_rd_en = axim_rvalid & axim_rready & axim_rlast;

assign axim_arvalid = dma_axi_rd_req_valid & (!rd_addr_fifo_full);
assign axim_arlen   = dma_axi_rd_req_alen;
assign axim_araddr  = dma_axi_rd_req_addr;
assign axim_arsize  = dma_axi_rd_req_size;
assign dma_axi_rd_resp_ready = axim_arvalid & axim_arready;


//----------------------
//axi rd data process
//----------------------
assign axim_rready = !data_fifo_full;
assign data_fifo_wr_en = axim_rvalid & axim_rready;
assign data_fifo_rd_en = axim_wvalid & axim_wready; // Read data from FIFO when write is valid


//----------------------
//axi wr addr process
//----------------------
assign axim_awvalid = dma_axi_wr_req_valid & (!wr_addr_fifo_full);// Check if FIFO is not full
assign axim_awlen   = dma_axi_wr_req_alen;
assign axim_awaddr  = dma_axi_wr_req_addr;
assign axim_awsize  = dma_axi_wr_req_size;
assign dma_axi_wr_resp_ready = axim_awvalid & axim_awready;

assign wr_addr_fifo_wr_en = dma_axi_wr_resp_ready;
assign wr_addr_fifo_rd_en = axim_bvalid & axim_bready;


//----------------------
//axi wr data process
//----------------------
assign  wr_alen_fifo_wr_en = dma_axi_wr_resp_ready;
assign  wr_alen_fifo_rd_en = axim_wvalid & axim_wready & axim_wlast;

assign axim_wvalid = (!wr_addr_fifo_empty) & (!data_fifo_empty);
assign axim_wdata = data_fifo_dout; // Use the data from the FIFO
assign axim_wstrb = wr_alen_fifo_dout[8+:AXI_STRB_WIDTH]; // Use the strobe from the FIFO
assign axim_wlast = axim_wvalid & axim_wready & (wr_data_count == wr_alen_fifo_dout[7:0]); // Last signal when all data is sent


//----------------------
//axi resp process
//----------------------
wire                        rd_err_flg;
wire                        wr_err_flg;

assign rd_err_flg = axim_rvalid & axim_rready &  axim_rlast & (axim_rresp == 2'b10 || axim_rresp == 2'b11); // Check for read error
assign wr_err_flg = axim_bvalid & axim_bready & (axim_bresp == 2'b10 || axim_bresp == 2'b11); // Check for write error

always@ (posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        dma_axi_err_valid  <= 1'b0;
        dma_axi_err_addr   <= 'b0;
        dma_axi_err_type   <= 1'b0;
    end else begin
        if(dma_axi_err_clr)begin
            dma_axi_err_valid  <= 1'b0;
            dma_axi_err_addr   <= 'b0;
            dma_axi_err_type   <= 1'b0;
        end else if (rd_err_flg) begin
            dma_axi_err_valid <= 1'b1;
            dma_axi_err_addr  <= rd_addr_fifo_dout; // Store the address from the FIFO
            dma_axi_err_type  <= 1'b0; // Read error
        end else if (wr_err_flg) begin
            dma_axi_err_valid <= 1'b1;
            dma_axi_err_addr  <= wr_addr_fifo_dout; // Store the address from the FIFO
            dma_axi_err_type  <= 1'b1; // write error
        end
    end
end


always@ (posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        axi_pending  <= 1'b0;
    end else begin
        axi_pending  <= (~rd_addr_fifo_empty)|(~wr_addr_fifo_empty);
    end
end


endmodule
