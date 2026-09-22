`default_nettype none

module main (
    input clk,
    input [1:0] btn,
    inout [3:0] jb,
    inout [2:0] jc
);
  // UART signals
  reg tx_start_sig;
  reg tx_ready_sig;
  reg rx_done_sig;
  reg [7:0] data_to_send;
  reg [7:0] data_recived;

  // RHD signals
  reg rhd_has_data;
  reg rhd_command;
  reg [15:0] rhd_buffer_d;
  reg [15:0] rhd_buffer_a;
  reg [15:0] rhd_buffer_b;

  UARTHandler #(
      .baud_clk_div(17)
  ) uart_joe (
      .clk(clk),
      .reset(btn[0]),
      .tx_start(tx_start_sig),
      .tx_ready(tx_ready_sig),
      .rx_done(rx_done_sig),
      .data_in(data_to_send),
      .data_out(data_recived),
      .rx_line(jc[0]),
      .tx_line(jc[1])
  );

  RHD2164Handler #(
      .SPI_CLK_TICKS(10)
  ) rhd_joe (
      .sys_clk(clk),
      .reset(btn[0]),
      .spi_line_miso(jb[0]),
      .spi_line_mosi(jb[1]),
      .spi_line_sclk(jb[2]),
      .spi_line_cs(jb[3]),
      .send_data_buffer(rhd_buffer_d),
      .send_command(rhd_command),
      .a_channel_buffer(rhd_buffer_a),
      .b_channel_buffer(rhd_buffer_b),
      .new_data(rhd_has_data)
  );

  PacketMaker9000 pm9k (
      .sys_clk(clk),
      .reset(btn[0]),
      .uart_data_out(data_to_send),
      .uart_tx_ready(tx_ready_sig),
      .uart_tx_enable(tx_start_sig),
      .a_channel_buffer(rhd_buffer_a),
      .b_channel_buffer(rhd_buffer_b),
      .rhd_has_data(rhd_has_data)
  );

  CommandHandler joe_command (
      .sys_clk(clk),
      .reset(btn[0]),
      .uart_buffer(data_recived),
      .uart_rx_done(rx_done_sig),
      .rhd_command_buffer(rhd_buffer_d),
      .command_buffer_full(rhd_command)
  );

endmodule
