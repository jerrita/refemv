`timescale 1ns/1ps

module rv_tb ();

reg CLOCK_50;
reg RST_N;
wire [7:0] LED;

initial begin
    $dumpfile("build/bench.vcd");
    $dumpvars(0, rv_tb);
end

initial begin
    CLOCK_50 = 1'b0;
    forever #10 CLOCK_50 = ~CLOCK_50;
end

initial begin
    RST_N = 1'b0;
    #10 RST_N = 1'b1;
    #500 RST_N = 1'b0;
    #503 RST_N = 1'b1;
    #1000 $finish;
end

soc mycpu(
    .clk(CLOCK_50),
    .rst(~RST_N)
);

endmodule