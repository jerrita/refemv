`timescale 1ps/1ps
`include "inc/define.vh"

module mmm_tb ();

reg clk;
reg RST_N;
wire [7:0] LED;

initial begin
    $dumpfile("build/bench.vcd");
    $dumpvars(0, mmm_tb);
end

initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
end

initial begin
    RST_N = 1'b0;
    #10 RST_N = 1'b1;
    #500 RST_N = 1'b0;
    #503 RST_N = 1'b1;
    #1000 $finish;
end

wire [`BUS] rdata;
reg [`BUS] wdata;
reg [4:0] cnt;
reg we;

initial begin
    wdata = 0;
    cnt = 0;
    we = 0;
end

wire [`BUS] addr;
assign addr = {27'b0, cnt[1:0]};
mmm mmm_inst(
    .clk(clk),
    .rdata(rdata),
    .addr(addr),
    .waddr(addr),
    .wdata(wdata),
    .we(we)
);

always @(posedge clk) begin
    cnt <= cnt + 1;
    we <= ~we;
    wdata <= wdata + 1;
end


endmodule