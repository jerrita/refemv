module soc(
    input clk, rstn,
    output [7:0] LED
);

wire [31:0] a0;
wire [31:0] mem_rdata, mem_wdata, addr, rdata;
wire rstrb;
assign LED = a0[7:0];

mcu mcu_inst(
    .clk(clk),
    .rstn(rstn),
    .a0(a0),
    .mem_rdata(mem_rdata),
    .mem_wdata(wdata),
    .mem_addr(addr),
    .mem_rstrb(rstrb)
);

endian_converter32 endian_converter32_inst(
    .in(rdata),
    .out(mem_rdata)
);

mem mem_inst(
    .clk(clk),
    .rstn(rstn),
    .strb(rstrb),
    .addr(addr),
    .rdata(rdata)
);

endmodule