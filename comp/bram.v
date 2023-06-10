`include "inc/define.vh"

module bram(
    input clk, we,
    input [31:2] addr,
    input [`BUS] wdata,
    output reg [`BUS] rdata
);

reg [31:0] mem [0:4095];
initial $readmemh("build/prog.hex", mem);

// integer i;
// initial for (i = 0; i < 32768; i = i + 1) begin
//     mem[i] = 0;
// end

always @(posedge clk) begin
    rdata <= mem[addr];
end

always @(negedge clk) begin
    if (we) begin
        mem[addr] <= wdata;
    end
end

endmodule