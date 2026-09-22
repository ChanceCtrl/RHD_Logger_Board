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

  // ------------------------------------------------------------
  // TX state machine
  // ------------------------------------------------------------

  localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

  reg [1:0] tx_state;

  // ------------------------------------------------------------
  // Baud-rate clock
  // ------------------------------------------------------------

  reg baud_clk;
  integer baud_count;

  // ------------------------------------------------------------
  // Transmit data
  // ------------------------------------------------------------

  reg [7:0] stored_data;

  // ------------------------------------------------------------
  // Data bit index
  //
  // 0 through 7, LSB first
  // ------------------------------------------------------------

  reg [2:0] data_index;

  // ------------------------------------------------------------
  // Start detection
  // ------------------------------------------------------------

  reg start_detected;

  // ------------------------------------------------------------
  // Internal control signals
  // ------------------------------------------------------------

  reg data_index_reset;
  reg start_reset;


  // ============================================================
  // Baud-rate clock generator
  // ============================================================
  //
  // With:
  //
  //   clk        = 100 MHz
  //   BAUD_CLK_TICKS = 868
  //
  // baud_clk pulses approximately every 868 clk cycles.
  //
  // 100,000,000 / 868 = ~115,207 baud
  //
  // ============================================================

  always @(posedge clk) begin

    if (reset) begin
      baud_clk   <= 1'b0;
      baud_count <= BAUD_CLK_TICKS - 1;
    end else begin

      if (baud_count == 0) begin

        baud_clk   <= 1'b1;
        baud_count <= BAUD_CLK_TICKS - 1;

      end else begin

        baud_clk   <= 1'b0;
        baud_count <= baud_count - 1;
      end
    end
  end


  // ============================================================
  // TX start detector
  // ============================================================
  //
  // Captures tx_start even if it is shorter than one baud period.
  //
  // Also captures tx_data_in at the same time so the data remains
  // stable throughout the transmission.
  //
  // ============================================================

  always @(posedge clk) begin

    if (reset || start_reset) begin

      start_detected <= 1'b0;

    end else begin

      if (tx_start && !start_detected) begin

        start_detected <= 1'b1;
        stored_data    <= data_in;

      end
    end
  end


  // ============================================================
  // Data bit counter
  // ============================================================
  //
  // Advances once per baud tick while DATA is active.
  //
  // ============================================================

  always @(posedge clk) begin

    if (reset || data_index_reset) begin

      data_index <= 3'd0;

    end else if (baud_clk) begin

      data_index <= data_index + 1'b1;

    end
  end


  // ============================================================
  // UART TX state machine
  // ============================================================
  //
  // UART format:
  //
  //   IDLE  : 1
  //   START : 0
  //   DATA  : 8 bits, LSB first
  //   STOP  : 1
  //
  // ============================================================

  always @(posedge clk) begin

    if (reset) begin

      tx_state         <= IDLE;

      tx_ready         <= 1'b0;
      tx_line          <= 1'b1;

      data_index_reset <= 1'b1;
      start_reset      <= 1'b1;

    end else begin

      // The TX FSM only advances on baud ticks.
      if (baud_clk) begin

        case (tx_state)

          // ------------------------------------------------
          // IDLE
          // ------------------------------------------------

          IDLE: begin

            data_index_reset <= 1'b1;
            start_reset      <= 1'b0;

            // UART idle level is HIGH
            tx_line          <= 1'b1;

            // Ready to accept a new byte
            tx_ready         <= 1'b1;

            if (start_detected) begin
              tx_state <= START;
            end
          end


          // ------------------------------------------------
          // START
          // ------------------------------------------------

          START: begin

            tx_ready <= 1'b0;

            // Enable bit counter
            data_index_reset <= 1'b0;

            // UART start bit
            tx_line <= 1'b0;

            tx_state <= DATA;
          end


          // ------------------------------------------------
          // DATA
          // ------------------------------------------------

          DATA: begin

            // UART sends LSB first
            tx_line <= stored_data[data_index];

            if (data_index == 3'd7) begin

              // Stop incrementing the data counter
              data_index_reset <= 1'b1;

              tx_state <= STOP;
            end
          end


          // ------------------------------------------------
          // STOP
          // ------------------------------------------------

          STOP: begin

            // UART stop bit
            tx_line <= 1'b1;

            // Clear start detector so another TX request
            // can be accepted.
            start_reset <= 1'b1;

            tx_state <= IDLE;
          end


          // ------------------------------------------------
          // Safety/default state
          // ------------------------------------------------

          default: begin

            tx_state         <= IDLE;

            tx_line          <= 1'b1;
            tx_ready         <= 1'b0;

            data_index_reset <= 1'b1;
            start_reset      <= 1'b1;
          end

        endcase
      end
    end
  end

endmodule
