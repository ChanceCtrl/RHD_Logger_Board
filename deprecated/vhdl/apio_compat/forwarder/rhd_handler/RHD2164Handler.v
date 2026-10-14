`default_nettype none

module RHD2164Handler #(
    parameter integer SYS_CLK_SPEED = 100_000_000,

    parameter integer SPI_CLK_TICKS = 100,
    parameter integer T_CS1_TICKS   = 3,
    parameter integer T_CS2_TICKS   = 3,
    parameter integer T_CSOFF_TICKS = 20
) (
    input wire sys_clk,
    input wire reset,

    input  wire spi_line_miso,
    output wire spi_line_mosi,
    output wire spi_line_sclk,
    output wire spi_line_cs,

    input wire [15:0] send_data_buffer,
    input wire        send_command,

    output reg [15:0] a_channel_buffer,
    output reg [15:0] b_channel_buffer,

    output reg rhd_is_ready,
    output reg new_data
);

  // ------------------------------------------------------------
  // State machine
  // ------------------------------------------------------------

  localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

  reg     [ 1:0] rhd_state;

  // ------------------------------------------------------------
  // SPI clock generation
  // ------------------------------------------------------------

  integer        spi_clk_count;
  reg            spi_sclk;
  reg            spi_cs;
  reg            spi_mosi;

  integer        spi_pulse_count;

  // ------------------------------------------------------------
  // Timing counters
  // ------------------------------------------------------------

  integer        t_cs1_count;
  integer        t_cs2_count;
  integer        t_csoff_count;

  // ------------------------------------------------------------
  // Data buffers
  // ------------------------------------------------------------

  reg     [15:0] send_buff;
  reg     [15:0] a_buff;
  reg     [15:0] b_buff;

  reg            a_channel_done;
  reg            b_channel_done;

  // ------------------------------------------------------------
  // Internal edge detection
  // ------------------------------------------------------------

  reg            spi_sclk_prev;
  reg            spi_cs_prev;

  // ------------------------------------------------------------
  // MOSI bit counter
  // ------------------------------------------------------------

  integer        mosi_bit_count;

  // ------------------------------------------------------------
  // MISO bit counters
  // ------------------------------------------------------------

  integer        a_bit_count;
  integer        b_bit_count;

  // ------------------------------------------------------------
  // New-data edge detection
  // ------------------------------------------------------------

  reg            has_new_data;
  reg            new_data_prev;

  // ------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------

  assign spi_line_sclk = spi_sclk;
  assign spi_line_cs   = spi_cs;
  assign spi_line_mosi = spi_mosi;

  // ------------------------------------------------------------
  // Main SPI state machine
  // ------------------------------------------------------------

  always @(posedge sys_clk or posedge reset) begin
    if (reset) begin
      rhd_state        <= IDLE;

      spi_clk_count    <= 0;
      spi_pulse_count  <= 0;

      spi_sclk         <= 1'b0;
      spi_cs           <= 1'b1;
      spi_mosi         <= 1'b0;

      t_cs1_count      <= 0;
      t_cs2_count      <= 0;
      t_csoff_count    <= 0;

      send_buff        <= 16'b0;

      rhd_is_ready     <= 1'b0;

      mosi_bit_count   <= 14;

      spi_sclk_prev    <= 1'b0;
      spi_cs_prev      <= 1'b0;

      a_buff           <= 16'b0;
      b_buff           <= 16'b0;

      a_bit_count      <= 15;
      b_bit_count      <= 16;

      a_channel_done   <= 1'b0;
      b_channel_done   <= 1'b0;

      a_channel_buffer <= 16'b0;
      b_channel_buffer <= 16'b0;

      has_new_data     <= 1'b0;
      new_data_prev    <= 1'b0;
      new_data         <= 1'b0;
    end else begin

      // ----------------------------------------------------
      // Save previous SPI levels for edge detection
      // ----------------------------------------------------

      spi_sclk_prev <= spi_sclk;
      spi_cs_prev <= spi_cs;

      // ----------------------------------------------------
      // Default pulse behavior
      // ----------------------------------------------------

      new_data <= 1'b0;

      // ----------------------------------------------------
      // Main state machine
      // ----------------------------------------------------

      case (rhd_state)

        // ------------------------------------------------
        // IDLE
        // ------------------------------------------------

        IDLE: begin
          spi_cs        <= 1'b1;
          spi_sclk      <= 1'b0;
          spi_mosi      <= 1'b0;

          rhd_is_ready  <= 1'b1;

          t_cs1_count   <= 0;
          t_cs2_count   <= 0;
          t_csoff_count <= 0;

          if (send_command) begin
            rhd_is_ready <= 1'b0;

            send_buff <= send_data_buffer;

            rhd_state <= START;
          end
        end

        // ------------------------------------------------
        // START
        // ------------------------------------------------

        START: begin
          spi_cs <= 1'b0;

          if (t_cs1_count == T_CS1_TICKS) begin
            t_cs1_count     <= 0;

            spi_clk_count   <= 0;
            spi_sclk        <= 1'b0;
            spi_pulse_count <= 0;

            rhd_state       <= DATA;
          end else begin
            t_cs1_count <= t_cs1_count + 1;
          end
        end

        // ------------------------------------------------
        // DATA
        // ------------------------------------------------

        DATA: begin

          if (spi_clk_count == (SPI_CLK_TICKS / 2)) begin

            spi_clk_count <= 0;

            spi_sclk <= ~spi_sclk;

            if (spi_pulse_count == 31) begin
              // Final SCLK edge has just been generated.
              // Stay in DATA for one more sys_clk so the
              // edge detector can see/process it.
              spi_pulse_count <= 32;
            end else if (spi_pulse_count == 32) begin
              spi_pulse_count <= 0;
              rhd_state <= STOP;
            end else begin
              spi_pulse_count <= spi_pulse_count + 1;
            end
          end else begin
            spi_clk_count <= spi_clk_count + 1;
          end
        end

        // ------------------------------------------------
        // STOP
        // ------------------------------------------------

        STOP: begin

          spi_clk_count <= 0;
          spi_sclk      <= 1'b0;

          if (t_cs2_count == T_CS2_TICKS) begin

            spi_cs <= 1'b1;

            if (t_csoff_count == T_CSOFF_TICKS) begin

              rhd_state <= IDLE;

              t_cs2_count <= 0;
              t_csoff_count <= 0;
            end else begin
              t_csoff_count <= t_csoff_count + 1;
            end
          end else begin
            t_cs2_count <= t_cs2_count + 1;
          end
        end

        default: begin
          rhd_state <= IDLE;
        end

      endcase

      // ----------------------------------------------------
      // MOSI handling
      //
      // The VHDL changes MOSI on the falling edge of SCLK.
      // Since everything is synchronous to sys_clk, detect
      // that edge here.
      // ----------------------------------------------------

      case (rhd_state)

        START: begin
          spi_mosi <= send_buff[15];
          mosi_bit_count <= 14;
        end

        DATA: begin
          if (spi_sclk_prev && !spi_sclk) begin

            spi_mosi <= send_buff[mosi_bit_count];

            if (mosi_bit_count == 0) mosi_bit_count <= 14;
            else mosi_bit_count <= mosi_bit_count - 1;
          end
        end

        default: begin
          spi_mosi <= 1'b0;
          mosi_bit_count <= 14;
        end

      endcase

      // ----------------------------------------------------
      // A-channel MISO capture
      //
      // Capture on falling SCLK edge.
      // ----------------------------------------------------

      if (rhd_state == DATA) begin

        if (spi_sclk_prev && !spi_sclk) begin

          a_buff[a_bit_count] <= spi_line_miso;

          if (a_bit_count == 0) begin
            a_channel_done <= 1'b1;
            a_bit_count <= 15;
          end else begin
            a_channel_done <= 1'b0;
            a_bit_count <= a_bit_count - 1;
          end
        end
      end

      // ----------------------------------------------------
      // B-channel MISO capture
      //
      // Capture on rising SCLK edge.
      // The first rising edge is skipped, matching the VHDL.
      // ----------------------------------------------------

      if (rhd_state == DATA) begin

        b_channel_done <= 1'b0;

        if (!spi_sclk_prev && spi_sclk) begin

          if (b_bit_count == 1) begin

            b_buff[b_bit_count] <= spi_line_miso;
            b_bit_count <= 16;
          end else if (b_bit_count == 16) begin

            b_bit_count <= 15;
          end else begin

            b_buff[b_bit_count] <= spi_line_miso;
            b_bit_count <= b_bit_count - 1;
          end
        end
      end

      // ----------------------------------------------------
      // Last B-channel bit.
      //
      // The RHD2164 puts the final bit on MISO when CS rises.
      // ----------------------------------------------------

      if (rhd_state == STOP) begin

        if (spi_cs && !spi_cs_prev) begin

          b_buff[0] <= spi_line_miso;
          b_channel_done <= 1'b1;
        end
      end

      // ----------------------------------------------------
      // Reset bit counters when outside DATA
      // ----------------------------------------------------

      if (rhd_state != DATA) begin
        b_bit_count <= 16;
        a_bit_count <= 15;
      end

      // ----------------------------------------------------
      // Generate new_data when both channels are complete.
      // ----------------------------------------------------

      has_new_data <= a_channel_done & b_channel_done;

      if ((a_channel_done & b_channel_done) && !new_data_prev) begin

        a_channel_buffer <= a_buff;
        b_channel_buffer <= b_buff;

        new_data <= 1'b1;
      end

      new_data_prev <= a_channel_done & b_channel_done;

    end
  end

endmodule
