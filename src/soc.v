module soc(
    input clk,
    output [7:0] LED
);

wire [31:0] a0;
assign LED = a0[7:0];

mcu mcu_instr(
    .clk(clk),
    .rstn(1'b1),
    .a0(a0)
);

endmodule