`timescale 1ns/1ps
`include "inc/define.vh"

module soc_tb ();

reg clk;
reg RST_N;
wire [7:0] LED;
wire [1:8] SW3;

assign SW3[8] = RST_N;

initial begin
    $dumpfile("build/bench.vcd");
    $dumpvars(0, soc_tb);
end

initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
end

initial begin
    RST_N = 1'b0;
    #10 RST_N = 1'b1;
    // #500 RST_N = 1'b0;
    // #503 RST_N = 1'b1;
    #10000 $finish;
end

soc soc_inst(
    .clk(clk),
    // .rstn(RST_N),
    .SW3(SW3),
    .LED(LED)
);


endmodule