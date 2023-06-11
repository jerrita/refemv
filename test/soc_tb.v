`timescale 1ns/1ps
`include "inc/define.vh"

module soc_tb ();

reg clk;
reg RST_N;
wire [7:0] LED;

initial begin
    $dumpfile("build/bench.vcd");
    $dumpvars(0, soc_tb);
end

initial begin
    clk = 1'b0;
    forever #2 clk = ~clk;
end

initial begin
    RST_N = 1'b0;
    #10 RST_N = 1'b1;
    // #500 RST_N = 1'b0;
    // #503 RST_N = 1'b1;
    #1000 $finish;
end

soc soc_inst(
    .clk(clk),
    .rstn(RST_N),
    .LED(LED)
);


endmodule