`include "inc/define.vh"

module mux3(
    input [`BUS] a, b, c,
    input [1:0] sel,
    output [`BUS] out
);

assign out = (sel == 'b00) ? a :
             (sel == 'b01) ? b :
             (sel == 'b10) ? c :
             0;

endmodule