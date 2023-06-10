module soc(
    input clk, rstn,
    output [7:0] LED
);

wire [31:0] dbg;
wire [31:0] mem_rdata, mem_wdata, addr, rdata;
wire [3:0] wmask;
wire rstrb;
assign LED = ~dbg[7:0];

refemv refemv_inst(
    .clk(clk),
    .rstn(rstn),
    .s7(dbg),
    .mem_rdata(mem_rdata),
    .mem_wdata(mem_wdata),
    .mem_addr(addr),
    .mem_rstrb(rstrb),
    .mem_wmask(wmask)
);

endian_converter32 endian_converter32_inst0(
    .in(rdata),
    .out(mem_rdata)
);

//* Notice: input mem_wdata and mem_mask should be big-endian!
mem mem_inst(
    .clk(clk),
    .rstn(rstn),
    .strb(rstrb),
    .addr(addr),
    .rdata(rdata),
    .wdata(mem_wdata),
    .wmask(wmask)
);

endmodule