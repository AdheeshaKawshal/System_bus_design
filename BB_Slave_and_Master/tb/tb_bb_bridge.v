// ============================================================================
// tb_bb_bridge.v
// ----------------------------------------------------------------------------
// Self-checking testbench for the UART bridge cores.
//
// Topology:
//   TB stimulus -> bb_slave_core (board A) --- UART (crossed) --- bb_master_core
//                                                                  (board B)
//                                                                     |
//                                                                     v
//                                                            slave.v (board B's
//                                                            2 KB register file)
//   grant_i is tied high (single master on board B, no arbiter needed here).
//
// There are no ready signals anywhere: a slave returns rdata + rvalid only.
// The TB models a real bus master accordingly -- it holds valid_i asserted
// until rvalid comes back on a read, rather than dropping it after one cycle.
// That matters: bb_slave_core edge-detects the start of a remote access
// precisely because valid_i stays up for the whole (long) remote round trip.
//
// Tests:
//   1. Local write, high address (proves the 2 KB space, not just 16 bytes)
//   2. Local read-back
//   3. Local write/read at the top of the 2 KB space (0x7FF)
//   4. Remote write (through to slave.v on board B)
//   5. Remote read  (through to slave.v on board B, correct reply)
//   6. Remote read that times out (reply path deliberately blocked)
//   7. No spurious far-side bus traffic before any UART packet arrives
//
// This runs at the REAL hardware settings: 125 MHz clock, 100 kbaud, so
// CLKS_PER_BIT = 1250. The request is one 24-bit frame (26 bit-times, 32500
// cycles) and the reply one 8-bit frame (10 bit-times, 12500 cycles), so a
// remote read is about 45000 cycles -- roughly 360 us of simulated time. The
// whole run is a few milliseconds: slower than a scaled-down model, but it
// exercises exactly the timing that will be programmed into the part.
//
// To iterate faster, drop CLK_FREQ_HZ/BAUD_RATE to 1600/100 (CLKS_PER_BIT =
// 16); the wait budgets below rescale automatically, but RX_TIMEOUT is a
// deliberate copy of the RTL default and must be scaled by hand.
//
// Reset: active-low, matches project convention.
// ============================================================================
`timescale 1ns/1ps

module tb_bb_bridge;

    localparam CLK_FREQ_HZ = 125000000;
    localparam BAUD_RATE   = 100000;      // CLKS_PER_BIT = 1250 -> 12500 cycles/byte
    localparam RX_TIMEOUT  = 50000;       // matches the RTL default

    // Derived wait budgets, so changing the baud above rescales the tests.
    // Each UART frame is one start bit + WIDTH data bits + one stop bit.
    localparam CLKS_PER_BIT  = CLK_FREQ_HZ / BAUD_RATE;
    localparam REQ_CYCLES    = CLKS_PER_BIT * (24 + 2);    // 24-bit request frame
    localparam RPY_CYCLES    = CLKS_PER_BIT * (8  + 2);    // 8-bit reply frame
    localparam ROUND_TRIP    = REQ_CYCLES + RPY_CYCLES;

    reg clk = 0;
    reg rst = 0;                      // active-low: start in reset

    always #4 clk = ~clk;             // 125 MHz -> 8 ns period

    // ------------------------------------------------------------------
    // Board A: bb_slave_core, driven directly by TB stimulus
    // ------------------------------------------------------------------
    reg         a_cs, a_valid, a_we;
    reg  [14:0] a_addr;
    reg  [7:0]  a_wdata;
    wire [7:0]  a_rdata;
    wire        a_rvalid, a_timeout, a_overflow, a_frame_err;

    wire        uart_a_to_b;          // bb_slave_core's request TX
    wire        uart_b_to_a;          // bb_master_core's reply TX
    reg         block_reply;          // test 6: hold board A's RX idle

    wire        uart_rx_at_a = block_reply ? 1'b1 : uart_b_to_a;

    bb_slave_core #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .RX_TIMEOUT  (RX_TIMEOUT)
    ) u_slave_core (
        .clk        (clk),
        .rst        (rst),
        .cs_i       (a_cs),
        .valid_i    (a_valid),
        .we_i       (a_we),
        .addr_i     (a_addr),
        .wdata_i    (a_wdata),
        .rdata_o    (a_rdata),
        .rvalid_o   (a_rvalid),
        .uart_tx_o  (uart_a_to_b),
        .uart_rx_i  (uart_rx_at_a),
        .timeout_o  (a_timeout),
        .overflow_o (a_overflow),
        .frame_err_o(a_frame_err)
    );

    // ------------------------------------------------------------------
    // Board B: bb_master_core feeding its local 2 KB slave
    // ------------------------------------------------------------------
    wire        b_req, b_valid;
    wire [15:0] b_addr;               // {addr[14:0], we}
    wire [7:0]  b_wdata;
    wire [7:0]  b_rdata;
    wire        b_rvalid;
    wire        b_overflow, b_frame_err;

    bb_master_core #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .BAUD_RATE      (BAUD_RATE),
        .ADDR_W         (15),
        .DATA_W         (8),
        .RW             (1),
        .ACTIVE_TIMEOUT (64)
    ) u_master_core (
        .clk        (clk),
        .rst        (rst),
        .uart_rx_i  (uart_a_to_b),
        .uart_tx_o  (uart_b_to_a),
        .req_o      (b_req),
        .grant_i    (1'b1),           // single master, no arbiter needed
        .addr_o     (b_addr),
        .wdata_o    (b_wdata),
        .valid_o    (b_valid),
        .rdata_i    (b_rdata),
        .rvalid_i   (b_rvalid),
        .overflow_o (b_overflow),
        .frame_err_o(b_frame_err)
    );

    // Board B's local register file. addr_o packs {addr[14:0], we} with we in
    // bit 0, so the slave's 11 address bits are b_addr[11:1].
    slave #(
        .ADDR_W (11),
        .DATA_W (8)
    ) u_far_slave (
        .clk      (clk),
        .rst      (rst),
        .cs_i     (1'b1),
        .valid_i  (b_valid),
        .we_i     (b_addr[0]),
        .addr_i   (b_addr[11:1]),
        .wdata_i  (b_wdata),
        .rdata_o  (b_rdata),
        .rvalid_o (b_rvalid)
    );

    // ------------------------------------------------------------------
    // Bookkeeping
    // ------------------------------------------------------------------
    integer errors      = 0;
    integer far_txns    = 0;   // counts transactions board B's master drives

    always @(posedge clk)
        if (rst && b_valid) far_txns = far_txns + 1;

    task automatic reset_pulse;
        begin
            rst = 0; a_cs = 0; a_valid = 0; a_we = 0; a_addr = 0; a_wdata = 0;
            block_reply = 0;
            repeat (5) @(posedge clk);
            rst = 1;
            repeat (5) @(posedge clk);
        end
    endtask

    // A write: present the transaction, hold it a few cycles, drop it.
    // Nothing comes back on a write -- there is no ready to wait for.
    task automatic do_write(input [14:0] addr, input [7:0] wdata);
        begin
            @(posedge clk);
            a_cs <= 1; a_valid <= 1; a_we <= 1; a_addr <= addr; a_wdata <= wdata;
            @(posedge clk);
            a_cs <= 0; a_valid <= 0; a_we <= 0;
        end
    endtask

    // A read: hold the transaction up until rvalid comes back, the way a real
    // bus master does, or give up after max_wait_cycles.
    task automatic do_read(
        input  [14:0]  addr,
        output [7:0]   rdata,
        output         got_rvalid,
        input  integer max_wait_cycles
    );
        integer n;
        begin
            @(posedge clk);
            a_cs <= 1; a_valid <= 1; a_we <= 0; a_addr <= addr; a_wdata <= 8'h00;

            n          = 0;
            got_rvalid = 1'b0;
            rdata      = 8'h00;
            while (n < max_wait_cycles && !got_rvalid) begin
                @(posedge clk);
                if (a_rvalid) begin
                    got_rvalid = 1'b1;
                    rdata      = a_rdata;
                end
                n = n + 1;
            end

            a_cs <= 0; a_valid <= 0;
            @(posedge clk);
        end
    endtask

    reg [7:0] got_rdata;
    reg       got_rvalid;

    initial begin
        $display("=== bb_slave_core / bb_master_core bridge testbench ===");
        reset_pulse;

        // -------------------------------------------------------------
        // Test 7 (checked first): board B must be silent until asked.
        // The old master.v ran a hardcoded transaction table on power-up.
        // -------------------------------------------------------------
        repeat (50) @(posedge clk);
        if (far_txns != 0) begin
            $display("[%0t] ERROR: Test 7 -- board B drove %0d bus transactions with no UART packet sent",
                     $time, far_txns);
            errors = errors + 1;
        end else begin
            $display("[%0t] Test 7 (no spurious far-side traffic) PASS", $time);
        end

        // -------------------------------------------------------------
        // Test 1/2: local write + read-back at 0x123 (well past 16 bytes)
        // -------------------------------------------------------------
        do_write({1'b0, 14'h123}, 8'hA5);
        do_read({1'b0, 14'h123}, got_rdata, got_rvalid, 50);
        if (got_rdata !== 8'hA5 || got_rvalid !== 1'b1) begin
            $display("[%0t] ERROR: Test 2 (local read-back @0x123) expected A5/rvalid=1, got %h/%b",
                     $time, got_rdata, got_rvalid);
            errors = errors + 1;
        end else begin
            $display("[%0t] Test 1/2 (local write+read @0x123) PASS: rdata=%h", $time, got_rdata);
        end

        // -------------------------------------------------------------
        // Test 3: top of the 2 KB space
        // -------------------------------------------------------------
        do_write({1'b0, 14'h7FF}, 8'h3C);
        do_read({1'b0, 14'h7FF}, got_rdata, got_rvalid, 50);
        if (got_rdata !== 8'h3C || got_rvalid !== 1'b1) begin
            $display("[%0t] ERROR: Test 3 (local @0x7FF) expected 3C/rvalid=1, got %h/%b",
                     $time, got_rdata, got_rvalid);
            errors = errors + 1;
        end else begin
            $display("[%0t] Test 3 (local write+read @0x7FF) PASS: rdata=%h", $time, got_rdata);
        end

        // -------------------------------------------------------------
        // Test 4: remote write (addr[14]=1 -> forwarded to board B)
        // 3 bytes at 160 cycles/byte, plus the far-side bus access.
        // -------------------------------------------------------------
        do_write({1'b1, 14'h003}, 8'h5A);
        repeat (REQ_CYCLES + 2000) @(posedge clk);
        if (a_overflow) begin
            $display("[%0t] ERROR: Test 4 (remote write) raised overflow_o", $time);
            errors = errors + 1;
        end else if (far_txns == 0) begin
            $display("[%0t] ERROR: Test 4 (remote write) never reached board B's bus", $time);
            errors = errors + 1;
        end else begin
            $display("[%0t] Test 4 (remote write) PASS: %0d far-side transaction(s)", $time, far_txns);
        end

        // -------------------------------------------------------------
        // Test 5: remote read, correct reply expected.
        // 3 bytes out + far-side access + 1 byte back; generous headroom.
        // -------------------------------------------------------------
        do_read({1'b1, 14'h003}, got_rdata, got_rvalid, ROUND_TRIP * 2);
        if (got_rdata !== 8'h5A || got_rvalid !== 1'b1 || a_timeout) begin
            $display("[%0t] ERROR: Test 5 (remote read) expected 5A/rvalid=1/timeout=0, got %h/%b/%b",
                     $time, got_rdata, got_rvalid, a_timeout);
            errors = errors + 1;
        end else begin
            $display("[%0t] Test 5 (remote read) PASS: rdata=%h", $time, got_rdata);
        end

        // -------------------------------------------------------------
        // Test 6: remote read that times out (reply path blocked)
        // -------------------------------------------------------------
        block_reply = 1;
        do_read({1'b1, 14'h003}, got_rdata, got_rvalid, REQ_CYCLES + RX_TIMEOUT + 5000);
        if (!a_timeout) begin
            $display("[%0t] ERROR: Test 6 expected timeout_o=1, got 0", $time);
            errors = errors + 1;
        end else if (got_rvalid !== 1'b0) begin
            $display("[%0t] ERROR: Test 6 expected no rvalid pulse on timeout, got rvalid=1", $time);
            errors = errors + 1;
        end else begin
            $display("[%0t] Test 6 (remote read timeout) PASS: clean release, no rvalid", $time);
        end
        block_reply = 0;

        // -------------------------------------------------------------
        if (errors == 0) $display("=== ALL TESTS PASSED ===");
        else             $display("=== %0d TEST(S) FAILED ===", errors);
        $finish;
    end

    // Safety net in case something hangs unexpectedly. Sized well past the
    // real-baud round trips above (~400 us each); shrink it if you scale the
    // baud down for faster iteration.
    initial begin
        #50_000_000;   // 50 ms
        $display("[%0t] ERROR: global testbench timeout -- something hung", $time);
        $finish;
    end

endmodule
