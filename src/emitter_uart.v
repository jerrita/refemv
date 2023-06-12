// simp: https://twitter.com/OlofKindgren/status/1409634477135982598
// https://gist.github.com/olofk/e91fba2572396f55525f8814f05fb33d

module emitter_uart
  #(
    parameter clk_freq_hz = 12 * 1000000,
    parameter baud_rate = 57600)
  (
   input wire 	    i_clk,
   input wire 	    i_rst,
   input wire [7:0] i_data,
   input wire 	    i_valid,
   output reg 	    o_ready,
   output wire 	    o_uart_tx);

   localparam START_VALUE = clk_freq_hz/baud_rate;
   localparam WIDTH = $clog2(START_VALUE);
   
   reg [WIDTH:0]  cnt;
   reg [9:0] 	    data;

   initial begin
      o_ready <= 1'b1;
      cnt <= {1'b0,START_VALUE[WIDTH-1:0]};
      data <= {1'b0, 9'b0, 1'b0};
   end

   assign o_uart_tx = data[0] | !(|data);

   always @(posedge i_clk) begin
      if (cnt[WIDTH] & !(|data))
	      o_ready <= 1'b1;
      else if (i_valid & o_ready)
	      o_ready <= 1'b0;

      if (o_ready | cnt[WIDTH])
	      cnt <= {1'b0,START_VALUE[WIDTH-1:0]};
      else
	cnt <= cnt-1;
      
      if (cnt[WIDTH])
	data <= {1'b0, data[9:1]};
      else if (i_valid & o_ready)
	data <= {1'b1, i_data, 1'b0};
   end
      
endmodule