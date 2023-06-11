module soc(
    input clk, rstn,
    output reg [7:0] LED
);

wire [31:0] dbg;
wire [31:0] mem_rdata, mem_wdata, addr, rdata;
wire [29:0] mem_wordaddr = addr[31:2];
wire [3:0] wmask;
wire rstrb;

wire isIO = addr[22];
wire isMem = ~isIO;
wire mem_wstrb = |wmask;

localparam IO_LED_bit = 0;

always @(posedge clk) begin
    if (isIO && mem_wstrb && mem_wordaddr[IO_LED_bit]) begin
        LED <= mem_wdata;
    end
end

mcu mcu_inst(
    .clk(clk),
    .rstn(rstn),
    // .s7(dbg),
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

mem mem_inst(
    .clk(clk),
    .rstn(rstn),
    .strb(isMem && rstrb),
    .addr(addr),
    .rdata(rdata),
    .wdata(mem_wdata),
    .wmask({4{isMem}} & wmask)
);

endmodule