`default_nettype none

module UARTHandler #(
    parameter baud_clk_div = 868
) (
    input clk,
    input reset,

    input  tx_start,
    output tx_ready,
    output rx_done,

    input  [7:0] data_in,
    output [7:0] data_out,

    input  rx_line,
    output tx_line
);
  TXHandler #(baud_clk_div) uart_tx (
      .clk(clk),
      .reset(reset),
      .tx_start(tx_start),
      .tx_ready(tx_ready),
      .data_in(data_in),
      .tx_line(tx_line),
  );

  RXHandler #(baud_clk_div) uart_rx (
      .clk(clk),
      .reset(reset),
      .rx_done(rx_done),
      .data_out(data_out),
      .rx_line(rx_line),
  );

endmodule
