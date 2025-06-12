// Project: DMA Controller
// Author: czz
// Date: 2025-06-07
// Description: APB to Register Interface Module
// This module converts APB signals to register interface signals for the DMA controller
module apb2reg_if (
    input         pclk,
    input         presetn,
    input         psel,
    input         penable,
    input         pwrite,
    input  [11:0] paddr,
    input  [31:0] pwdata,
    output [31:0] prdata,
    output        pready,

    // reg_if
    output [11:0] reg_addr,
    output        wr_en,
    output        rd_en,
    output [3:0]  wr_msk,
    output [31:0] wr_data,
    input  [31:0] rd_data
);

assign wr_en = psel & penable & pwrite;
assign rd_en = psel & ~penable & ~pwrite;
assign reg_addr = paddr;
assign wr_msk   = 4'b1111;
assign wr_data  = pwdata;
assign prdata = rd_data;
assign pready = 1'b1; // no wait states

endmodule