`default_nettype none

module PacketMaker9000 (
    // Generic system inputs
    input wire sys_clk,
    input wire reset,

    // UART related things
    output reg [7:0] uart_data_out,

    input  wire uart_tx_ready,
    output reg  uart_tx_enable,

    // RHD related things
    input wire [15:0] a_channel_buffer,
    input wire [15:0] b_channel_buffer,

    input wire rhd_has_data
);

  // ------------------------------------------------------------
  // State machine
  // ------------------------------------------------------------

  localparam [1:0] IDLE = 2'd0, SEND = 2'd1, HALT = 2'd2;

  reg [ 1:0] packet_state;

  // ------------------------------------------------------------
  // Channel copies
  //
  // Capture both channels when rhd_has_data is asserted so the
  // values remain stable while the four UART bytes are transmitted.
  // ------------------------------------------------------------

  reg [15:0] a_channel_copy;
  reg [15:0] b_channel_copy;

  // ------------------------------------------------------------
  // Packet counter
  //
  // 0 = A[7:0]
  // 1 = A[15:8]
  // 2 = B[7:0]
  // 3 = B[15:8]
  // ------------------------------------------------------------

  reg [ 1:0] packet_count;

  // ------------------------------------------------------------
  // Main packet-maker FSM
  // ------------------------------------------------------------

  always @(posedge sys_clk or posedge reset) begin

    if (reset) begin
      packet_count   <= 2'd0;
      packet_state   <= IDLE;

      uart_tx_enable <= 1'b0;
      uart_data_out  <= 8'b0;

      a_channel_copy <= 16'b0;
      b_channel_copy <= 16'b0;
    end else begin

      case (packet_state)

        // ------------------------------------------------
        // IDLE
        // ------------------------------------------------

        IDLE: begin

          uart_tx_enable <= 1'b0;

          if (rhd_has_data) begin

            // Snapshot the RHD data before transmitting
            a_channel_copy <= a_channel_buffer;
            b_channel_copy <= b_channel_buffer;

            packet_count   <= 2'd0;

            packet_state   <= SEND;
          end
        end


        // ------------------------------------------------
        // SEND
        // ------------------------------------------------

        SEND: begin

          // TX enable is normally low. It is pulsed high
          // for one sys_clk cycle when a byte is accepted.
          uart_tx_enable <= 1'b0;

          if (uart_tx_ready) begin

            case (packet_count)

              // ------------------------------------
              // Byte 0: A channel low byte
              // ------------------------------------

              2'd0: begin

                uart_data_out  <= a_channel_copy[7:0];
                uart_tx_enable <= 1'b1;

                packet_state   <= HALT;
              end


              // ------------------------------------
              // Byte 1: A channel high byte
              // ------------------------------------

              2'd1: begin

                uart_data_out  <= a_channel_copy[15:8];
                uart_tx_enable <= 1'b1;

                packet_state   <= HALT;
              end


              // ------------------------------------
              // Byte 2: B channel low byte
              // ------------------------------------

              2'd2: begin

                uart_data_out  <= b_channel_copy[7:0];
                uart_tx_enable <= 1'b1;

                packet_state   <= HALT;
              end


              // ------------------------------------
              // Byte 3: B channel high byte
              // ------------------------------------

              2'd3: begin

                uart_data_out  <= b_channel_copy[15:8];
                uart_tx_enable <= 1'b1;

                packet_state   <= HALT;
              end

            endcase
          end
        end


        // ------------------------------------------------
        // HALT
        //
        // Wait for the UART to acknowledge/consume the
        // previous byte by bringing uart_tx_ready low.
        // ------------------------------------------------

        HALT: begin

          uart_tx_enable <= 1'b0;

          // Last byte has been sent
          if (packet_count == 2'd3) begin

            packet_count <= 2'd0;

            packet_state <= IDLE;
          end  // UART has accepted the current byte
          else if (!uart_tx_ready) begin

            packet_count <= packet_count + 1'b1;

            packet_state <= SEND;
          end
        end


        // ------------------------------------------------
        // Safety/default state
        // ------------------------------------------------

        default: begin

          packet_count   <= 2'd0;
          packet_state   <= IDLE;

          uart_tx_enable <= 1'b0;
        end

      endcase
    end
  end

endmodule
