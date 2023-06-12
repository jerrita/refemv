`include "inc/define.vh"

module mem(
    input clk, rstn,
    input strb,
    input [`BUS] addr, wdata,
    input [3:0] wmask,
    output reg [`BUS] rdata
);

reg [`BUS] mem [0:511];
initial begin
    $readmemh("build/prog.hex", mem);
end

wire [29:0] word_addr = addr[31:2];

always @(posedge clk) begin
    if (!rstn) begin
        rdata <= 0;
    end else begin
        if (strb) begin
            rdata <= mem[word_addr];
        end
    end
end

always @(posedge clk) begin
    if (wmask[0]) mem[word_addr][7:0] <= wdata[7:0];
    if (wmask[1]) mem[word_addr][15:8] <= wdata[15:8];
    if (wmask[2]) mem[word_addr][23:16] <= wdata[23:16];
    if (wmask[3]) mem[word_addr][31:24] <= wdata[31:24];
end

endmodule