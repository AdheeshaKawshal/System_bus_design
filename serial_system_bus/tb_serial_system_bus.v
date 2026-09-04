`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tb_serial_system_bus
// Description:
//   Self-checking testbench for serial_bus_top.v (the integration of
//   serial_system_bus.v with its 2 masters and 3 slaves). Both masters run
//   their built-in demo transaction tables (master.v) entirely on their own
//   once reset releases - no stimulus driving is needed beyond clk/rst.
//   This testbench just free-runs the clock, waits long enough for every
//   transaction on both masters to complete (including worst-case
//   BACKOFF/retry rounds), then reaches into each master's transaction
//   memory via hierarchical reference and checks the read-back data against
//   what was written earlier in the same table.
//////////////////////////////////////////////////////////////////////////////////

module tb_serial_system_bus;

    reg clk;
    reg rst;

    integer errors;

    // serial_2bus_top's 4-pins-per-board UART GPIOs are no longer crossed
    // internally - this testbench is "the outside world" for it now, so it
    // supplies the same bus0<->bus1 crossing that used to be internal
    // wiring: each board's bb_master_core TX goes to the OTHER board's
    // bb_slave_core RX, and vice versa.
    wire bus0_mc_tx, bus0_sc_tx;
    wire bus1_mc_tx, bus1_sc_tx;

    serial_2bus_top  dut (
        .clk (clk),
        .rst (rst),

        .bus0_mc_uart_tx_o (bus0_mc_tx),
        .bus0_mc_uart_rx_i (bus1_sc_tx),
        .bus0_sc_uart_tx_o (bus0_sc_tx),
        .bus0_sc_uart_rx_i (bus1_mc_tx),

        .bus1_mc_uart_tx_o (bus1_mc_tx),
        .bus1_mc_uart_rx_i (bus0_sc_tx),
        .bus1_sc_uart_tx_o (bus1_sc_tx),
        .bus1_sc_uart_rx_i (bus0_mc_tx)
    );

    // 100 MHz clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Active-low reset, held for a few cycles then released.
    initial begin
        rst = 1'b0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b1;
    end

    task check_byte;
        input [127:0] name;
        input [7:0]   got;
        input [7:0]   expected;
        begin
            if (got !== expected) begin
                $display("FAIL: %0s got=%h expected=%h", name, got, expected);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s == %h", name, got);
            end
        end
    endtask

    initial begin
        errors = 0;

        wait (rst == 1'b1);

        // Generous margin: each master has 8 transactions, worst case a
        // handful of ~40-cycle ACTIVE_TIMEOUT rounds plus BACKOFF_DELAY if
        // a read is ever missed, run both masters concurrently, and there
        // may be some serialization while they contend for the bus. 20000
        // cycles (200us at 100MHz) is comfortably more than needed for the
        // 8-entry tables used here.
        #200000;

        // ---------------------------------------------------------
        // Master 0's table (see master.v's own header comment, fixed in
        // this session so every entry stays internal - no entry should
        // ever route through slave_bridge):
        //   0: write slave1 0x001 <- 0x11   1: read slave1 0x001 (expect 0x11)
        //   2: write slave1 0x005 <- 0x22   3: read slave1 0x005 (expect 0x22)
        //   4: write slave1 0x001 <- 0x33 (overwrite, never read back)
        //   5: read  slave2 0x001 (never written on slave2 - not checked)
        //   6: write slave2 0x008 <- 0x44   7: read slave2 0x008 (expect 0x44)
        // ---------------------------------------------------------
        $display("---- Bus 0 / Master 0 ----");
        check_byte("Bus0 M0 tx1 (slave1 0x001)", dut.u_bus0.u_master0.rdata_mem[1], 8'h11);
        check_byte("Bus0 M0 tx3 (slave1 0x005)", dut.u_bus0.u_master0.rdata_mem[3], 8'h22);
        check_byte("Bus0 M0 tx7 (slave2 0x008)", dut.u_bus0.u_master0.rdata_mem[7], 8'h44);

        $display("---- Bus 1 / Master 0 ----");
        check_byte("Bus1 M0 tx1 (slave1 0x001)", dut.u_bus1.u_master0.rdata_mem[1], 8'h11);
        check_byte("Bus1 M0 tx3 (slave1 0x005)", dut.u_bus1.u_master0.rdata_mem[3], 8'h22);
        check_byte("Bus1 M0 tx7 (slave2 0x008)", dut.u_bus1.u_master0.rdata_mem[7], 8'h44);

        // ---------------------------------------------------------
        // dut is now serial_2bus_top, wrapping two serial_bus_top boards
        // (u_bus0/u_bus1) chained to each other over UART - see
        // serial_2bus_top.v. Each bus's M1 slot is bb_master_core.v, not a
        // local master with its own transaction table - it only relays
        // whatever the OTHER board's bb_slave_core forwards it, so there is
        // no rdata_mem to check there. Nothing on Master 0's table is
        // REMOTE-flagged (every entry is a genuine internal slave1/slave2
        // access), so the UART bridge path is never exercised by this
        // testbench as written.
        // ---------------------------------------------------------

        if (errors == 0)
            $display("\n==== ALL CHECKS PASSED ====");
        else
            $display("\n==== %0d CHECK(S) FAILED ====", errors);

        $finish;
    end

    // Safety net: if something hangs (e.g. a master stuck retrying
    // forever), don't let the simulation run away silently.
    initial begin
        #500000;
        $display("TIMEOUT: simulation did not finish in time");
        $finish;
    end

endmodule
