`default_nettype none

module CommandHandler (
    // Basic system things
    input wire sys_clk,
    input wire reset,

    // UART related inputs
    input wire [7:0] uart_buffer,
    input wire       uart_rx_done,

    // RHD outputs
    output reg [15:0] rhd_command_buffer,
    output reg        command_buffer_full
);

  // ------------------------------------------------------------
  // State machine
  // ------------------------------------------------------------

  localparam [1:0] IDLE = 2'd0, RECV = 2'd1, HALT = 2'd2;

  reg [1:0] packet_state;

  // ------------------------------------------------------------
  // Packet buffer
  // ------------------------------------------------------------

  reg [15:0] out_buffer;

  // 0 = receiving first byte
  // 1 = receiving second byte
  reg packet_count;

  // ------------------------------------------------------------
  // Main packet FSM
  // ------------------------------------------------------------

  always @(posedge sys_clk or posedge reset) begin

    if (reset) begin
      packet_count        <= 1'b0;
      packet_state        <= IDLE;

      command_buffer_full <= 1'b0;
      rhd_command_buffer  <= 16'b0;

      out_buffer          <= 16'b0;
    end else begin

      case (packet_state)

        // ------------------------------------------------
        // IDLE
        // ------------------------------------------------

        IDLE: begin

          command_buffer_full <= 1'b0;

          packet_state <= RECV;
        end


        // ------------------------------------------------
        // RECV
        // ------------------------------------------------

        RECV: begin

          if (uart_rx_done) begin

            case (packet_count)

              // First byte = upper 8 bits
              1'b0: begin

                out_buffer[15:8] <= uart_buffer;

                packet_state <= HALT;
              end


              // Second byte = lower 8 bits
              1'b1: begin

                out_buffer[7:0] <= uart_buffer;

                /*
                                 * NOTE:
                                 *
                                 * The original VHDL does:
                                 *
                                 *   out_buffer(7 downto 0) <= uart_buffer;
                                 *   rhd_command_buffer <= out_buffer;
                                 *
                                 * Because these are signal assignments,
                                 * rhd_command_buffer receives the OLD
                                 * value of out_buffer[7:0].
                                 *
                                 * To preserve the apparent intended
                                 * behavior, we explicitly construct the
                                 * complete word here.
                                 */
                rhd_command_buffer <= {out_buffer[15:8], uart_buffer};

                command_buffer_full <= 1'b1;

                packet_state <= HALT;
              end

            endcase
          end
        end


        // ------------------------------------------------
        // HALT
        // ------------------------------------------------

        HALT: begin

          if (packet_count == 1'b1) begin

            packet_count <= 1'b0;

            packet_state <= IDLE;
          end else if (!uart_rx_done) begin

            packet_count <= packet_count + 1'b1;

            packet_state <= RECV;
          end
        end


        // ------------------------------------------------
        // Safety/default state
        // ------------------------------------------------

        default: begin

          packet_count <= 1'b0;
          packet_state <= IDLE;

        end

      endcase
    end
  end

endmodule
