// Project: DMA Controller
// Author: czz
// Date: 2025-06-07
// Description: This module implements a DMA FIFO with configurable depth and width.
// Note: the read data delay is 0 clock cycle, so the read data is available in the same clock cycle as the read enable signal.

module dma_fifo #(
parameter DEPTH = 16,
parameter WIDTH = 64
)(
input                                       clk,
input                                       rst_n,
input                                       clear,
input                                       wr_en,
input                                       rd_en,
input         [WIDTH-1:0]                   din,
output        [WIDTH-1:0]                   dout,
output                                      error,
output                                      full,
output                                      empty,
output        [$clog2(DEPTH>1?DEPTH:2):0]   ocup_cnt,
output        [$clog2(DEPTH>1?DEPTH:2):0]   free_cnt
);
localparam MSB_DEPTH = $clog2(DEPTH>1?DEPTH:2);

reg  [WIDTH-1:0]            ram_regfile    [DEPTH-1:0];
reg  [MSB_DEPTH:0]          write_ptr;
reg  [MSB_DEPTH:0]          read_ptr;

always@(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        write_ptr <= 'h0;
        read_ptr  <= 'h0;
    end else begin
        if (clear) begin
            write_ptr <= 'h0;
            read_ptr  <= 'h0;
        end else begin
            if (wr_en && ~full)begin
                write_ptr <= write_ptr + 'd1;
            end
            if (rd_en && ~empty)begin
                read_ptr <= read_ptr + 'd1;
            end
        end
    end
end

always@(posedge clk) begin
    if (wr_en && ~full) begin
        ram_regfile[write_ptr[MSB_DEPTH-1:0]] <= din;
    end
end

assign dout = empty ? 'h0 :  ram_regfile[read_ptr[MSB_DEPTH-1:0]];

assign empty = (write_ptr == read_ptr);

assign full  = (DEPTH == 1) ? (write_ptr[0] != read_ptr[0]) :
               ((write_ptr[MSB_DEPTH-1:0] == read_ptr[MSB_DEPTH-1:0]) &&
                (write_ptr[MSB_DEPTH] != read_ptr[MSB_DEPTH]));

assign ocup_cnt = write_ptr - read_ptr;

assign free_cnt = DEPTH - ocup_cnt;

assign error = (wr_en && full) || (rd_en && empty);

`ifndef NO_ASSERTIONS
    initial begin
        if(2**$clog2(DEPTH) != DEPTH)
            $error("FIFO DEPTH must be power of 2");

        if(DEPTH < 1)
            $error("FIFO size of DEPTH defined is illegal!");
  end
`endif

endmodule
