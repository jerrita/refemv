`include "inc/define.vh"

module mux2(
    input [`BUS] a, b,
    input sel,
    output [`BUS] out
);

assign out = sel ? b : a;

endmodule