`include "inc/define.vh"

module mem(
    input clk, rstn,
    input strb,
    input [`BUS] addr,
    output reg [`BUS] rdata
);

reg [`BUS] mem [0:1023];
initial begin
    $readmemh("build/prog.hex", mem);
end

always @(posedge clk) begin
    if (!rstn) begin
        rdata <= 0;
    end else begin
        if (strb) begin
            rdata <= mem[addr[`BUSWT:2]];
        end
    end
end

endmodule