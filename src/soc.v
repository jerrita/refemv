// clk is 12MHz in icesugar

module soc(
    input clk, rstn,
    input RX,
    output reg [7:0] LED,
    output TX
);

initial LED = 0;

wire [31:0] mem_rdata, mem_wdata, addr, rdata;
wire [29:0] mem_wordaddr = addr[31:2];
wire [3:0] wmask;
wire rstrb;

wire isIO = addr[22];
wire isMem = ~isIO;
wire mem_wstrb = |wmask;

// Memory-mapped IO, 1-hot encoded
localparam IO_LED_bit = 0;
localparam IO_UartDat_bit = 1;
localparam IO_UartCntl_bit = 2;

wire uart_valid = isIO && mem_wstrb && mem_wordaddr[IO_UartDat_bit];
wire uart_ready;
wire [31:0] io_rdata = mem_wordaddr[IO_UartCntl_bit] ? { 22'b0, ~uart_ready, 9'b0} : 32'b0;

emitter_uart #(
    .clk_freq_hz(12 * 1000000),
    .baud_rate(115200)
) emitter_uart_inst(
    .i_clk(clk),
    .i_rst(~rstn),
    .i_data(mem_wdata[7:0]),
    .i_valid(uart_valid),
    .o_ready(uart_ready),
    .o_uart_tx(TX)
);

always @(posedge clk) begin
    if (isIO && mem_wstrb && mem_wordaddr[IO_LED_bit]) begin
        LED <= ~(mem_wdata[7:0]);
    end
end

/*
    Debugger
*/
`ifdef BENCH
always @(posedge clk) begin
    case (1'b1)
        isIO: $display("IO (#%h): %h <= %h(%h)", addr, io_rdata, mem_wdata, uart_valid);
    endcase
end
`endif


/*
    CPU and memory
*/

refemv refemv_inst(
    .clk(clk),
    .rstn(rstn),
    // .s7(dbg),
    .mem_rdata(isMem ? mem_rdata : io_rdata),
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
    .strb(isMem && rstrb),
    .addr(addr),
    .rdata(rdata),
    .wdata(mem_wdata),
    .wmask({4{isMem}} & wmask)
);

endmodule