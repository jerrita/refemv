// clk is 12MHz in icesugar
`include "inc/define.vh"

module soc(
    input clk,
    input RX,
    input [1:8] SW3,
    output reg [7:0] LED,
    output TX
);

wire rstn = SW3[8];

//! mem_rdata and mem_wdata are all little-endian
wire [`BUS] mem_rdata, mem_rdata_be, mem_wdata;
wire [`BUS_ADDR] mem_addr;
wire [`BUS_ADDRWT-2:0] mem_wordaddr = mem_addr[`BUS_ADDRWT:2];
wire [3:0] wmask;
wire rstrb;

wire isIO = mem_addr[22];
wire isMem = ~isIO;
wire mem_wstrb = |wmask;

reg [`BUS] timer;
initial timer = 0;

// Memory-mapped IO, 1-hot encoded
localparam IO_LED_bit = 0;
localparam IO_UartDat_bit = 1;
localparam IO_UartCntl_bit = 2;
localparam IO_Switch_bit = 3;
localparam IO_Timer = 4;

wire uart_valid = isIO && mem_wstrb && mem_wordaddr[IO_UartDat_bit];
wire uart_ready;

wire [31:0] io_rdata = mem_wordaddr[IO_UartCntl_bit] ? { 22'b0, !uart_ready, 9'b0} :
                       mem_wordaddr[IO_Switch_bit] ? { 24'b0, SW3[1:8] } :
                       mem_wordaddr[IO_Timer] ? timer :
                       32'b0;

emitter_uart #(
    .clk_freq_hz(12 * 1000000),
    .baud_rate(115200)
) emitter_uart_inst(
    .i_clk(clk),
    .i_rst(~rstn),
    .i_data(mem_wdata[31:24]),
    .i_valid(uart_valid),
    .o_ready(uart_ready),
    .o_uart_tx(TX)
);

always @(posedge clk) begin
    timer <= timer + 1;

    if (isIO && mem_wstrb && mem_wordaddr[IO_LED_bit]) begin
        LED <= ~(mem_wdata[31:24]);
    end
end

/*
    Debugger
*/
`ifdef BENCH
always @(posedge clk) begin
    if (rstrb) begin
        #1 $display("[%6d] %h <= mem[%h]", $time, isMem ? mem_rdata : io_rdata, mem_addr);
    end

    if (mem_wstrb) begin
        $display("mem[%h] <= %h", mem_addr, mem_wdata);
    end

    case (1'b1)
        isIO & mem_wstrb: $display("IO (#%h): %h <= %h(uart: %h)", mem_addr, io_rdata, mem_wdata, uart_valid);
    endcase
end
`endif

/*
    CPU and memory
*/

//! output mem is little-endian, but input mem needs to be big-endian
refemv refemv_inst(
    .clk(clk),
    .rstn(rstn),
    // .s7(dbg),
    .mem_rdata(isMem ? mem_rdata_be : io_rdata),
    .mem_wdata(mem_wdata),
    .mem_addr(mem_addr),
    .mem_rstrb(rstrb),
    .mem_wmask(wmask)
);

endian_converter32 ec_inst(
    .in(mem_rdata),
    .out(mem_rdata_be)
);

mem mem_inst(
    .clk(clk),
    .rstn(rstn),
    .strb(isMem && rstrb),
    .addr(mem_addr),
    .rdata(mem_rdata),
    .wdata(mem_wdata),
    .wmask({4{isMem}} & wmask)
);

endmodule