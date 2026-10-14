`default_nettype none

module TXHandler #(
    parameter integer BAUD_CLK_TICKS = 868
) (
    input wire clk,
    input wire reset,

    input wire       tx_start,
    input wire [7:0] data_in,

    output reg tx_line,
    output reg tx_ready
);
  // Statemachine defs
  localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;
  reg [1:0] tx_state;

  integer baud_count;

  reg [7:0] data_buffer;
  integer data_index;

  // 8N1 UART TX controller fella
  always @(posedge clk) begin
    if (reset) begin
      tx_state    <= IDLE;

      tx_ready    <= 1'b0;
      tx_line     <= 1'b1;

      data_buffer <= 8'b0;
      data_index  <= 0;
    end else begin
      case (tx_state)
        IDLE: begin
          // UART is meant to idle at a logic level high
          tx_line  <= 1'b1;

          // Ready to accept a new byte
          tx_ready <= 1'b1;

          if (tx_start) begin
            tx_ready <= 1'b0;
            data_buffer <= data_in;

            baud_count <= 0;
            tx_state <= START;
          end
        end

        START: begin
          // UART start bit
          tx_line <= 1'b0;

          // Wait for 1 baud_clk cycle
          if (baud_count == BAUD_CLK_TICKS) begin
            tx_line <= data_buffer[0];
            data_index <= 1;

            baud_count <= 0;
            tx_state <= DATA;
          end else begin
            baud_count <= baud_count + 1;
          end
        end

        DATA: begin
          // Change the bit after the baud_clk time
          if (baud_count == BAUD_CLK_TICKS) begin
            if (data_index == 8) begin
              baud_count <= 0;
              tx_state   <= STOP;
            end else data_index <= data_index + 1;

            baud_count <= 0;
            tx_line <= data_buffer[data_index];
          end else begin
            baud_count <= baud_count + 1;
          end
        end

        STOP: begin
          // UART stop bit
          tx_line <= 1'b1;

          // Wait for 1 baud_clk cycle
          if (baud_count == BAUD_CLK_TICKS) begin
            baud_count <= 0;
            tx_state   <= IDLE;
          end else begin
            baud_count <= baud_count + 1;
          end
        end

        // Same as reset logic
        default: begin
          tx_state    <= IDLE;

          tx_ready    <= 1'b0;
          tx_line     <= 1'b1;

          data_buffer <= 8'b0;
          data_index  <= 0;
        end
      endcase
    end
  end
endmodule
