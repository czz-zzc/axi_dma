`timescale 1ns / 100ps

module tb_top;

reg         pclk;          // APB clock
reg         presetn;      // APB reset (active low)

// Clock generation
initial begin
    pclk = 0;
    forever #5 pclk = ~pclk; // 100MHz clock
end

// Reset generation
initial begin
    presetn = 0;
    #20;
    presetn = 1;
end

initial begin
    $fsdbDumpfile("./tb_top.fsdb");
    $fsdbDumpvars("+all");
    $fsdbDumpMDA(0,tb_top);
end

wire intr_dma_done;
wire intr_dma_err;
reg          psel;
reg          penable;
reg          pwrite;
reg [11:0]   paddr;
reg [31:0]   pwdata;
wire [31:0]   prdata;
wire          pready;
wire          pslverr;
wire         axim_awlock;    
wire [3:0]   axim_awcache;   
wire [2:0]   axim_awprot;    
wire [3:0]   axim_awqos;     
wire [31:0]  axim_awaddr;    
wire [7:0]   axim_awlen;     
wire [2:0]   axim_awsize;    
wire [1:0]   axim_awburst;   
wire         axim_awvalid;   
wire         axim_awready;   
wire [63:0]  axim_wdata;     
wire [7:0]   axim_wstrb;     
wire         axim_wlast;     
wire         axim_wvalid;    
wire         axim_wready;    
wire [1:0]   axim_bresp;     
wire         axim_bvalid;    
wire         axim_bready;    
wire         axim_arlock;    
wire [3:0]   axim_arcache;
wire [2:0]   axim_arprot;    
wire [3:0]   axim_arqos;     
wire [31:0]  axim_araddr;    
wire [7:0]   axim_arlen;     
wire [2:0]   axim_arsize;    
wire [1:0]   axim_arburst;   
wire         axim_arvalid;   
wire         axim_arready;   
wire [63:0]  axim_rdata;     
wire [1:0]   axim_rresp;     
wire         axim_rlast;     
wire         axim_rvalid;    
wire         axim_rready;    
wire [7:0]   axim_awid;      
wire [7:0]   axim_bid;        
wire [7:0]   axim_arid;      
wire [7:0]   axim_rid;       

dma_top u_dma_top (
    .clk(pclk),
    .rst_n(presetn),
    .intr_dma_done(intr_dma_done),
    .intr_dma_err(intr_dma_err),
    .psel(psel),
    .penable(penable),
    .pwrite(pwrite),
    .paddr(paddr),
    .pwdata(pwdata),
    .prdata(prdata),
    .pready(pready),
    .pslverr(pslverr),
    .axim_awlock(axim_awlock),
    .axim_awcache(axim_awcache),
    .axim_awprot(axim_awprot),
    .axim_awqos(axim_awqos),
    .axim_awaddr(axim_awaddr),
    .axim_awlen(axim_awlen),
    .axim_awsize(axim_awsize),
    .axim_awburst(axim_awburst),
    .axim_awvalid(axim_awvalid),
    .axim_awready(axim_awready),
    .axim_wdata(axim_wdata),
    .axim_wstrb(axim_wstrb),
    .axim_wlast(axim_wlast),
    .axim_wvalid(axim_wvalid),
    .axim_wready(axim_wready),
    .axim_bresp(axim_bresp),
    .axim_bvalid(axim_bvalid),
    .axim_bready(axim_bready),
    .axim_arlock(axim_arlock),
    .axim_arcache(axim_arcache),
    .axim_arprot(axim_arprot),
    .axim_arqos(axim_arqos),
    .axim_araddr(axim_araddr),
    .axim_arlen(axim_arlen),
    .axim_arsize(axim_arsize),
    .axim_arburst(axim_arburst),
    .axim_arvalid(axim_arvalid),
    .axim_arready(axim_arready),
    .axim_rdata(axim_rdata),
    .axim_rresp(axim_rresp),
    .axim_rlast(axim_rlast),
    .axim_rvalid(axim_rvalid),
    .axim_rready(axim_rready),
    .axim_awid(axim_awid),
    .axim_bid(axim_bid),    
    .axim_arid(axim_arid),
    .axim_rid(axim_rid)
);
axi_ram #(
    .DATA_WIDTH(64),
    .ADDR_WIDTH(20),
    .STRB_WIDTH(8),
    .ID_WIDTH(8),
    .PIPELINE_OUTPUT(6) 
) u_axi_ram (
    .clk(pclk),
    .rst(~presetn),
    .s_axi_awid(axim_awid),
    .s_axi_awaddr(axim_awaddr[19:0]),
    .s_axi_awlen(axim_awlen),
    .s_axi_awsize(axim_awsize),
    .s_axi_awburst(axim_awburst),
    .s_axi_awlock(axim_awlock),
    .s_axi_awcache(axim_awcache),
    .s_axi_awprot(axim_awprot),
    .s_axi_awvalid(axim_awvalid),
    .s_axi_awready(axim_awready),
    .s_axi_wdata(axim_wdata),
    .s_axi_wstrb(axim_wstrb),
    .s_axi_wlast(axim_wlast),
    .s_axi_wvalid(axim_wvalid),
    .s_axi_wready(axim_wready),
    .s_axi_bid(axim_bid),
    .s_axi_bresp(axim_bresp),
    .s_axi_bvalid(axim_bvalid),
    .s_axi_bready(axim_bready),
    .s_axi_arid(axim_arid),
    .s_axi_araddr(axim_araddr[19:0]),
    .s_axi_arlen(axim_arlen),
    .s_axi_arsize(axim_arsize),
    .s_axi_arburst(axim_arburst),
    .s_axi_arlock(axim_arlock),
    .s_axi_arcache(axim_arcache),
    .s_axi_arprot(axim_arprot),
    .s_axi_arvalid(axim_arvalid),
    .s_axi_arready(axim_arready),
    .s_axi_rid(axim_rid),
    .s_axi_rdata(axim_rdata),
    .s_axi_rresp(axim_rresp),
    .s_axi_rlast(axim_rlast),
    .s_axi_rvalid(axim_rvalid),
    .s_axi_rready(axim_rready)
);


// Test sequence
initial begin
    // Initialize signals
    paddr = 0;
    pwrite = 0;
    psel = 0;
    penable = 0;
    pwdata = 0;

    // Wait for reset deassertion
    wait (presetn == 1'b1);
    
    // Read 0x4
    //apb_read(12'h004);

    //normal incr dma
    config_desc0_mode0();

    //start dma
    dma_start();

    #100000
    apb_read(12'h004);
    apb_read(12'h00c);
    apb_read(12'h010);
    // Test complete
    $finish;
end

task dma_start();
    begin
        apb_write(12'h0,16<<8 | 0);//clr last dma
        apb_write(12'h0,16<<8 | 1);//start
        $display("-------------------------- ---------------");
        $display("-----------------DMA START ---------------");
        $display("-------------------------- ---------------");
    end    
endtask

task config_desc0_mode0();
    begin
        apb_write(12'h20,1);//mode
        apb_write(12'h24,8041);//length 
        apb_write(12'h28,'h0ff1);//src addr
        apb_write(12'h2c,'h10001);//dst addr
        //apb_write(12'h30,'h10);//rd jump byte
        //apb_write(12'h34,'h0);//wr jump byte


        apb_write(12'h40,5);//mode
        apb_write(12'h44,2048);//length 
        apb_write(12'h48,'h2000);//src addr
        apb_write(12'h4c,'h6000);//dst addr
        apb_write(12'h50,'h18);//rd jump byte
        //apb_write(12'h54,'h10);//wr jump byte
    end    
endtask

task apb_write(input [12-1:0] addr, input [32-1:0] data);
    begin
        @(posedge pclk);
        paddr <= addr;
        pwdata <= data;
        pwrite <= 1;
        psel <= 1;
        penable <= 0;
        @(posedge pclk);
        penable <= 1;
        wait (pready);
        @(posedge pclk);
        if (pslverr) begin
            $display("Write Error at address: 0x%h", addr);
        end
        paddr <= 0;
        pwdata <= 0;
        pwrite <= 0;
        psel <= 0;
        penable <= 0;
    end
endtask

task apb_read(input [12-1:0] addr);
    begin
        @(posedge pclk);
        paddr <= addr;
        pwrite <= 0;
        psel <= 1;
        penable <= 0;
        @(posedge pclk);
        penable <= 1;
        wait (pready);
        @(posedge pclk);
        if (pslverr) begin
            $display("Read Error at address: 0x%h", addr);
        end else begin
            if(paddr == 12'h4)begin
                $display("DMA STATUS:fsm-2'b%b err-1'b%b done-1'b%b",prdata[3:2],prdata[1],prdata[0]);
            end else if(paddr == 12'hc)begin
                $display("DMA ERR STATUS:2'b%b,config_err-2'b10 axi_read_err-2'b00 axi_write_err-2'b01",prdata[1:0]);
            end else if(paddr == 12'h10)begin
                $display("DMA ERR ADDR:32'h%h",prdata);
            end else begin
                $display("Read Data from address 0x%h: 0x%h", addr, prdata);
            end
            
        end
        paddr <= 0;
        psel <= 0;
        penable <= 0;
    end
endtask

endmodule
