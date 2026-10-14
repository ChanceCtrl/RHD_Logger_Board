`default_nettype none

module RXHandler #(
    parameter integer BAUD_CLK_TICKS = 868
) (
    input wire clk,
    input wire reset,

    output reg       rx_done,
    output reg [7:0] data_out,

    input wire rx_line
);
  // Statemachine defs
  localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;
  reg [1:0] rx_state;

  integer baud_count;

  reg [7:0] data_buffer;
  integer data_index;

  // 8N1 UART RX controller fella
  always @(posedge clk) begin
    if (reset) begin
      rx_state    <= IDLE;

      rx_done    <= 1'b0;

      data_buffer <= 8'b0;
      data_index  <= 0;
    end else begin
      case (rx_state)
        IDLE: begin
          baud_count <= 0;
          data_index <= 0;
          rx_done <= 1'b0;

          // The start bit is 0
          if (rx_line == 1'b0) rx_state <= START;
        end

        START: begin
          // Make sure its a real 0 start bit fella by waiting half a baud_clk
          if (rx_line == 1'b0) begin
            if (baud_count == (BAUD_CLK_TICKS / 2)) begin
              baud_count <= 0;
              data_index <= 0;
              rx_state   <= DATA;
            end else baud_count <= baud_count + 1;
          end else rx_state <= IDLE;
        end

        DATA: begin
          // Now that we are half a baud_clk off, we can count in full baud_clk
          // cycles and capture the middle of the data bits
          if (baud_count == BAUD_CLK_TICKS) begin
            baud_count <= 0;

            data_buffer[data_index] <= rx_line;

            // GG go next until the data buffer is full
            if (data_index == 7) rx_state <= STOP;
            else data_index <= data_index + 1;

          end else baud_count <= baud_count + 1;
        end

        STOP: begin
          if (baud_count == BAUD_CLK_TICKS) begin
            baud_count <= 0;
            rx_done <= 1'b1;
            data_out <= data_buffer;
            rx_state <= IDLE;
          end else baud_count <= baud_count + 1;
        end

        // Same as reset logic
        default: begin
          rx_state    <= IDLE;

          rx_done    <= 1'b0;

          data_buffer <= 8'b0;
          data_index  <= 0;
        end
      endcase
    end
  end
endmodule
