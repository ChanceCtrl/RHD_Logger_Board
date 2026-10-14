`timescale 1ns / 1ps
`default_nettype none


module UART_tb;

  initial begin
    $dumpvars(0, UART_tb);
  end

  // ----------------------------------------------------------------
  // Simulation parameters
  // ----------------------------------------------------------------

  // Keep this small so the simulation runs quickly.
  // For example, with a 10 ns clock:
  // 20 clocks/bit = 5 Mbps equivalent baud.
  localparam integer BAUD_CLK_TICKS = 20;

  localparam integer CLK_PERIOD_NS  = 10;

  // ----------------------------------------------------------------
  // Testbench signals
  // ----------------------------------------------------------------

  reg        clk;
  reg        reset;

  reg        tx_start;
  reg  [7:0] data_in;

  wire       tx_line;
  wire       tx_ready;

  wire       rx_done;
  wire [7:0] data_out;

  // ----------------------------------------------------------------
  // Device under test
  // ----------------------------------------------------------------

  TXHandler #(
      .BAUD_CLK_TICKS(BAUD_CLK_TICKS)
  ) tx (
      .clk     (clk),
      .reset   (reset),
      .tx_start(tx_start),
      .data_in (data_in),
      .tx_line (tx_line),
      .tx_ready(tx_ready)
  );

  RXHandler #(
      .BAUD_CLK_TICKS(BAUD_CLK_TICKS)
  ) rx (
      .clk     (clk),
      .reset   (reset),
      .rx_done (rx_done),
      .data_out(data_out),
      .rx_line (tx_line)
  );

  // ----------------------------------------------------------------
  // Clock
  // ----------------------------------------------------------------

  initial begin
    clk = 1'b0;

    forever #(CLK_PERIOD_NS / 2) clk = ~clk;
  end

  // ----------------------------------------------------------------
  // Test bookkeeping
  // ----------------------------------------------------------------

  integer tests_sent;
  integer tests_received;
  integer tests_failed;

  // ----------------------------------------------------------------
  // Send one byte
  // ----------------------------------------------------------------

  task send_byte(input [7:0] value);
    begin
      // Wait until TX says it is ready.
      @(posedge clk);

      while (!tx_ready) @(posedge clk);

      data_in  <= value;
      tx_start <= 1'b1;

      // tx_start is asserted for exactly one clock.
      @(posedge clk);
      tx_start <= 1'b0;

      tests_sent = tests_sent + 1;

      $display("[TB] Sending 0x%02h at time %0t", value, $time);
    end
  endtask

  // ----------------------------------------------------------------
  // Wait for and verify received byte
  // ----------------------------------------------------------------

  task wait_for_byte(input [7:0] expected);
    integer timeout;
    begin
      timeout = 0;

      // Wait for RX to report completion.
      while (!rx_done && timeout < (BAUD_CLK_TICKS * 20)) begin
        @(posedge clk);
        timeout = timeout + 1;
      end

      if (timeout >= (BAUD_CLK_TICKS * 20)) begin
        $error("[TB] TIMEOUT waiting for 0x%02h at time %0t", expected, $time);

        tests_failed = tests_failed + 1;
      end else begin
        tests_received = tests_received + 1;

        if (data_out !== expected) begin
          $error("[TB] WRONG DATA: expected 0x%02h, got 0x%02h at time %0t", expected, data_out,
                 $time);

          tests_failed = tests_failed + 1;
        end else begin
          $display("[TB] PASS: received 0x%02h at time %0t", data_out, $time);
        end
      end
    end
  endtask

  // ----------------------------------------------------------------
  // Test sequence
  // ----------------------------------------------------------------

  initial begin

    tests_sent     = 0;
    tests_received = 0;
    tests_failed   = 0;

    reset          = 1'b1;
    tx_start       = 1'b0;
    data_in        = 8'h00;

    // Hold reset for a few clocks.
    repeat (5) @(posedge clk);

    reset = 1'b0;

    $display("");
    $display("========================================");
    $display(" UART TX/RX TEST");
    $display(" BAUD_CLK_TICKS = %0d", BAUD_CLK_TICKS);
    $display("========================================");
    $display("");

    // ------------------------------------------------------------
    // Test 1: all zeros
    // ------------------------------------------------------------

    send_byte(8'h00);
    wait_for_byte(8'h00);

    // ------------------------------------------------------------
    // Test 2: all ones
    // ------------------------------------------------------------

    send_byte(8'hFF);
    wait_for_byte(8'hFF);

    // ------------------------------------------------------------
    // Test 3: alternating bits
    // ------------------------------------------------------------

    send_byte(8'hAA);
    wait_for_byte(8'hAA);

    send_byte(8'h55);
    wait_for_byte(8'h55);

    // ------------------------------------------------------------
    // Test 4: bit walking
    // ------------------------------------------------------------

    send_byte(8'h01);
    wait_for_byte(8'h01);

    send_byte(8'h02);
    wait_for_byte(8'h02);

    send_byte(8'h04);
    wait_for_byte(8'h04);

    send_byte(8'h08);
    wait_for_byte(8'h08);

    send_byte(8'h10);
    wait_for_byte(8'h10);

    send_byte(8'h20);
    wait_for_byte(8'h20);

    send_byte(8'h40);
    wait_for_byte(8'h40);

    send_byte(8'h80);
    wait_for_byte(8'h80);

    // ------------------------------------------------------------
    // Test 5: ASCII characters
    // ------------------------------------------------------------

    send_byte("A");
    wait_for_byte("A");

    send_byte("B");
    wait_for_byte("B");

    send_byte("C");
    wait_for_byte("C");

    send_byte("H");
    wait_for_byte("H");

    send_byte("i");
    wait_for_byte("i");

    // ------------------------------------------------------------
    // Test 6: several bytes back-to-back
    // ------------------------------------------------------------

    send_byte(8'h12);
    wait_for_byte(8'h12);

    send_byte(8'h34);
    wait_for_byte(8'h34);

    send_byte(8'h56);
    wait_for_byte(8'h56);

    send_byte(8'h78);
    wait_for_byte(8'h78);

    send_byte(8'h9A);
    wait_for_byte(8'h9A);

    send_byte(8'hBC);
    wait_for_byte(8'hBC);

    send_byte(8'hDE);
    wait_for_byte(8'hDE);

    send_byte(8'hF0);
    wait_for_byte(8'hF0);

    // ------------------------------------------------------------
    // Results
    // ------------------------------------------------------------

    $display("");
    $display("========================================");
    $display(" TEST COMPLETE");
    $display(" Sent:     %0d", tests_sent);
    $display(" Received: %0d", tests_received);
    $display(" Failures: %0d", tests_failed);
    $display("========================================");

    if (tests_failed == 0) begin
      $display("RESULT: PASS");
    end else begin
      $display("RESULT: FAIL");
    end

    $display("");

    #100;
    $finish;
  end

endmodule

`default_nettype wire
