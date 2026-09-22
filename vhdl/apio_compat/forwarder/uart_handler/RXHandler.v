`default_nettype none

module RXHandler #(
    parameter integer BAUD_CLK_TICKS = 868
) (
    input wire clk,
    input wire reset,

    input wire rx_line,

    output reg       rx_done,
    output reg [7:0] data_out,

    // Debug outputs
    output reg [ 1:0] debug_state,
    output reg [ 2:0] debug_bit_count,
    output reg [31:0] debug_bit_timer
);

  // ============================================================
  // STATES
  // ============================================================

  localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2;


  // ============================================================
  // TIMING
  // ============================================================

  localparam integer HALF_BIT_COUNT = BAUD_CLK_TICKS / 2;


  // ============================================================
  // REGISTERS
  // ============================================================

  reg [1:0] rx_state;

  reg [7:0] rx_stored_data;

  reg [2:0] bit_count;

  integer bit_timer;


  // ============================================================
  // RECEIVER
  // ============================================================

  always @(posedge clk) begin

    if (reset) begin

      rx_state        <= IDLE;

      rx_stored_data  <= 8'h00;

      data_out        <= 8'h00;

      rx_done         <= 1'b0;

      bit_count       <= 3'd0;

      bit_timer       <= 0;

      debug_state     <= IDLE;
      debug_bit_count <= 3'd0;
      debug_bit_timer <= 32'd0;

    end else begin

      // ----------------------------------------------------
      // DEBUG OUTPUTS
      // ----------------------------------------------------

      debug_state     <= rx_state;
      debug_bit_count <= bit_count;
      debug_bit_timer <= bit_timer;


      case (rx_state)


        // =================================================
        // IDLE
        // =================================================

        IDLE: begin

          bit_timer <= 0;
          bit_count <= 0;

          // Wait for start bit
          if (rx_line == 1'b0) begin

            rx_state  <= START;

            bit_timer <= 0;

          end

        end


        // =================================================
        // START
        // =================================================
        //
        // We detected LOW.
        //
        // Wait half a bit and check that the line is
        // still LOW.
        //
        // =================================================

        START: begin

          if (bit_timer >= HALF_BIT_COUNT - 1) begin

            if (rx_line == 1'b0) begin

              // -------------------------------------
              // Valid start bit.
              //
              // We are now at approximately the
              // center of the start bit.
              //
              // Wait one full bit to reach the center
              // of DATA bit 0.
              // -------------------------------------

              rx_state  <= DATA;

              bit_timer <= 0;

              bit_count <= 0;

            end else begin

              // False start

              rx_state  <= IDLE;

              bit_timer <= 0;

            end

          end else begin

            bit_timer <= bit_timer + 1;

          end

        end


        // =================================================
        // DATA
        // =================================================
        //
        // Sample one bit every BAUD_CLK_TICKS clocks.
        //
        // =================================================

        DATA: begin

          if (bit_timer >= BAUD_CLK_TICKS - 1) begin

            // -----------------------------------------
            // Sample the current data bit
            // -----------------------------------------

            rx_stored_data[bit_count] <= rx_line;

            bit_timer <= 0;


            if (bit_count == 3'd7) begin

              // -------------------------------------
              // 8th bit has just been sampled.
              //
              // IMPORTANT:
              //
              // rx_stored_data[7] is updated with a
              // NON-BLOCKING assignment, so we cannot
              // use rx_stored_data directly for
              // data_out here.
              //
              // Construct the final byte explicitly.
              // -------------------------------------

              data_out  <= {rx_line, rx_stored_data[6:0]};

              // -------------------------------------
              // BYTE COMPLETE
              //
              // Keep rx_done HIGH until reset.
              // -------------------------------------

              rx_done   <= 1'b1;

              // Return to idle
              rx_state  <= IDLE;

              bit_count <= 0;

            end else begin

              bit_count <= bit_count + 1'b1;

            end

          end else begin

            bit_timer <= bit_timer + 1;

          end

        end


        // =================================================
        // DEFAULT
        // =================================================

        default: begin

          rx_state  <= IDLE;

          bit_timer <= 0;

          bit_count <= 0;

        end

      endcase

    end

  end

endmodule

`default_nettype wire
