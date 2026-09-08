`timescale 1ns / 1ps
// tb_serial_2bus_top_random: exercises both master.v instances in
// serial_2bus_top (single bus, two local masters) against the four
// INTERNAL transaction-table entries (3,4 -> slave_sel3/u_slave2 plain
// slave; 5,6 -> slave_sel1/u_slave0 split-capable slave). Entries 1,2,7,8
// are deliberately excluded - they route external (ext_redirect/
// bb_slave_core), and a read there would hang forever with no far-side
// board attached to answer it.
//
// All scenarios run from this one file, in order:
//   1. Master0 only                      - single-master access
//   2. Master1 only                      - single-master access (other side)
//   3. M1 requests WHILE M0 is actively running (non-split contention) -
//      deterministic, every step logged
//   4. M0 SPLITS, M1 requests DURING the park window (write then read),
//      then M0 resumes - deterministic, every step logged
//   5. Mirror of 4: M1 SPLITS, M0 requests during ITS park window
//   6. Both masters, mixed random entries - unguided contention stress
//   7. Both masters hammering ONLY the split entries (5/6) back-to-back -
//      dedicated park/resume stress
//
// Every phase (3 onward) runs under its own timeout guard (fork/disable),
// so a hang in one phase is reported and the suite still moves on to the
// rest instead of dying on the single global timeout.
//
// Self-check: after any read completes, compare that master's own
// rdata_mem[idx] against the actual slave's mem[] content at that address
// (hierarchical reference) - the real, physically-completed value.
module tb_serial_2bus_top_random;

    reg clk, rst;
    reg [3:0] m0_sel, m1_sel;
    reg       m0_valid, m1_valid;
    wire [3:0] led;

    integer errors;
    integer checks;

    // Widen the split slave's park window (WAIT_CYCLES, hardcoded to 10
    // inside serial_2bus_top.v's own u_slave0 instantiation) purely from
    // here via defparam, so Phase 4/5 below have room to fit a THIRD
    // other-master transaction into the park window before resume fires -
    // no RTL file is touched for this.
    defparam dut.u_slave0.WAIT_CYCLES = 80;

    // WRITE_DELAY must cover the full 24-bit request frame (ADDR_W+RW+
    // DATA_W = 15+1+8) PLUS addr_redirect's own 6-cycle pipeline delay
    // before a slave even sees it - anything shorter releases the bus
    // (and lets the next master start driving the shared request line)
    // while the previous frame is still mid-flight, corrupting both.
    serial_2bus_top #(
        .M0_START_TXN(0), .M0_REQ_DELAY(5), .M0_WRITE_DELAY(40),
        .M1_START_TXN(0), .M1_REQ_DELAY(5), .M1_WRITE_DELAY(40)
    ) dut (
        .clk (clk),
        .rst (rst),
        .sc_uart_tx_o (),
        .sc_uart_rx_i (1'b1),
        .m0_pkt_sel_i   (m0_sel),
        .m0_pkt_valid_i (m0_valid),
        .m1_pkt_sel_i   (m1_sel),
        .m1_pkt_valid_i (m1_valid),
        .led_o (led)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst = 1'b1; m0_sel = 4'd0; m0_valid = 1'b0; m1_sel = 4'd0; m1_valid = 1'b0;
        errors = 0; checks = 0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
    end

    // localparam mirror of master.v's own state encoding
    localparam ST_WAIT = 3'd0, ST_IDLE = 3'd1, ST_REQUEST = 3'd2, ST_ACTIVE = 3'd3;

    // ------------------------------------------------------------------
    // Generic driver task - selects entry idx on the named master, waits
    // for the transaction to fully complete (back to IDLE), then reports/
    // self-checks a read against the actual slave memory.
    // ------------------------------------------------------------------
    task drive_m0(input [3:0] idx);
        begin
            @(posedge clk);
            m0_sel   = idx;
            m0_valid = 1'b1;
            @(posedge clk);
            m0_valid = 1'b0;
            wait (dut.u_master0.state == ST_IDLE);
            if (idx == 4 || idx == 6) begin
                checks = checks + 1;
                if (idx == 4) begin
                    if (dut.u_master0.rdata_mem[4] !== dut.u_slave2.mem[5]) begin
                        errors = errors + 1;
                        $display("[%0t] FAIL M0 txn4(slave3 addr5): got=%0d mem=%0d", $time, dut.u_master0.rdata_mem[4], dut.u_slave2.mem[5]);
                    end else
                        $display("[%0t] PASS M0 txn4(slave3 addr5) == %0d", $time, dut.u_master0.rdata_mem[4]);
                end else begin
                    if (dut.u_master0.rdata_mem[6] !== dut.u_slave0.mem[1]) begin
                        errors = errors + 1;
                        $display("[%0t] FAIL M0 txn6(split slave1 addr1): got=%0d mem=%0d", $time, dut.u_master0.rdata_mem[6], dut.u_slave0.mem[1]);
                    end else
                        $display("[%0t] PASS M0 txn6(split slave1 addr1) == %0d", $time, dut.u_master0.rdata_mem[6]);
                end
            end else begin
                $display("[%0t] M0 txn%0d (write) done, wdata=%0d", $time, idx, dut.u_master0.wdata_mem[idx]);
            end
        end
    endtask

    task drive_m1(input [3:0] idx);
        begin
            @(posedge clk);
            m1_sel   = idx;
            m1_valid = 1'b1;
            @(posedge clk);
            m1_valid = 1'b0;
            wait (dut.u_master1.state == ST_IDLE);
            if (idx == 4 || idx == 6) begin
                checks = checks + 1;
                if (idx == 4) begin
                    if (dut.u_master1.rdata_mem[4] !== dut.u_slave2.mem[5]) begin
                        errors = errors + 1;
                        $display("[%0t] FAIL M1 txn4(slave3 addr5): got=%0d mem=%0d", $time, dut.u_master1.rdata_mem[4], dut.u_slave2.mem[5]);
                    end else
                        $display("[%0t] PASS M1 txn4(slave3 addr5) == %0d", $time, dut.u_master1.rdata_mem[4]);
                end else begin
                    if (dut.u_master1.rdata_mem[6] !== dut.u_slave0.mem[1]) begin
                        errors = errors + 1;
                        $display("[%0t] FAIL M1 txn6(split slave1 addr1): got=%0d mem=%0d", $time, dut.u_master1.rdata_mem[6], dut.u_slave0.mem[1]);
                    end else
                        $display("[%0t] PASS M1 txn6(split slave1 addr1) == %0d", $time, dut.u_master1.rdata_mem[6]);
                end
            end else begin
                $display("[%0t] M1 txn%0d (write) done, wdata=%0d", $time, idx, dut.u_master1.wdata_mem[idx]);
            end
        end
    endtask

    // random pick: write entries {3,5} or read entries {4,6}
    function [3:0] rand_entry;
        input dummy;
        integer r;
        begin
            r = $urandom_range(0, 3);
            case (r)
                0: rand_entry = 4'd3; // write slave3
                1: rand_entry = 4'd4; // read  slave3
                2: rand_entry = 4'd5; // write split slave1
                3: rand_entry = 4'd6; // read  split slave1
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Fires BOTH masters' pkt_valid pulses on the SAME clock edge -
    // guaranteed genuine simultaneous contention every call (not just
    // "close in time"), then lets the arbiter sort it out and self-checks
    // each side independently once it completes, whichever order that
    // happens in. idx0/idx1 are drawn from the full internal pool
    // (3,4,5,6), so this exercises split-vs-split, split-vs-plain, and
    // plain-vs-plain simultaneous collisions all through the same call.
    // ------------------------------------------------------------------
    task issue_both(input [3:0] idx0, input [3:0] idx1);
        begin
            @(posedge clk);
            m0_sel = idx0; m0_valid = 1'b1;
            m1_sel = idx1; m1_valid = 1'b1;
            @(posedge clk);
            m0_valid = 1'b0;
            m1_valid = 1'b0;
            $display("[%0t] SIMULTANEOUS issue: M0->entry%0d  M1->entry%0d", $time, idx0, idx1);
            fork
                begin
                    wait (dut.u_master0.state == ST_IDLE);
                    if (idx0 == 4 || idx0 == 6) begin
                        checks = checks + 1;
                        if (idx0 == 4) begin
                            if (dut.u_master0.rdata_mem[4] !== dut.u_slave2.mem[5]) begin
                                errors = errors + 1;
                                $display("[%0t] FAIL M0 txn4(slave3): got=%0d mem=%0d", $time, dut.u_master0.rdata_mem[4], dut.u_slave2.mem[5]);
                            end else
                                $display("[%0t] PASS M0 txn4(slave3) == %0d", $time, dut.u_master0.rdata_mem[4]);
                        end else begin
                            if (dut.u_master0.rdata_mem[6] !== dut.u_slave0.mem[1]) begin
                                errors = errors + 1;
                                $display("[%0t] FAIL M0 txn6(split): got=%0d mem=%0d", $time, dut.u_master0.rdata_mem[6], dut.u_slave0.mem[1]);
                            end else
                                $display("[%0t] PASS M0 txn6(split) == %0d", $time, dut.u_master0.rdata_mem[6]);
                        end
                    end else
                        $display("[%0t] M0 txn%0d (write) done, wdata=%0d", $time, idx0, dut.u_master0.wdata_mem[idx0]);
                end
                begin
                    wait (dut.u_master1.state == ST_IDLE);
                    if (idx1 == 4 || idx1 == 6) begin
                        checks = checks + 1;
                        if (idx1 == 4) begin
                            if (dut.u_master1.rdata_mem[4] !== dut.u_slave2.mem[5]) begin
                                errors = errors + 1;
                                $display("[%0t] FAIL M1 txn4(slave3): got=%0d mem=%0d", $time, dut.u_master1.rdata_mem[4], dut.u_slave2.mem[5]);
                            end else
                                $display("[%0t] PASS M1 txn4(slave3) == %0d", $time, dut.u_master1.rdata_mem[4]);
                        end else begin
                            if (dut.u_master1.rdata_mem[6] !== dut.u_slave0.mem[1]) begin
                                errors = errors + 1;
                                $display("[%0t] FAIL M1 txn6(split): got=%0d mem=%0d", $time, dut.u_master1.rdata_mem[6], dut.u_slave0.mem[1]);
                            end else
                                $display("[%0t] PASS M1 txn6(split) == %0d", $time, dut.u_master1.rdata_mem[6]);
                        end
                    end else
                        $display("[%0t] M1 txn%0d (write) done, wdata=%0d", $time, idx1, dut.u_master1.wdata_mem[idx1]);
                end
            join
        end
    endtask

    integer i;

    initial begin
        wait (rst == 1'b0);
        #100;

        // ================= Phase 1: Master0 only =================
        $display("\n==== Phase 1: single-master (M0 only) ====");
        for (i = 0; i < 12; i = i + 1) begin
            drive_m0(rand_entry(0));
            #(($urandom_range(0, 50)) * 10);
        end

        // ================= Phase 2: Master1 only =================
        $display("\n==== Phase 2: single-master (M1 only) ====");
        for (i = 0; i < 12; i = i + 1) begin
            drive_m1(rand_entry(0));
            #(($urandom_range(0, 50)) * 10);
        end

        // ============================================================
        // Phase 3: M1 requests WHILE M0 is actively running (non-split).
        // M0 grabs the bus first (write to slave3); before it releases,
        // M1 issues its own request so the arbiter has to make M1 wait
        // for M0 to finish - plain contention, no parking involved.
        // ============================================================
        $display("\n==== Phase 3: M1 requests WHILE M0 is ACTIVE (non-split contention) ====");
        fork
            begin : ph3_body
                @(posedge clk);
                m0_sel = 4'd3; m0_valid = 1'b1;
                @(posedge clk);
                m0_valid = 1'b0;
                $display("[%0t] M0 REQUEST issued (entry3: write slave3)", $time);

                wait (dut.u_master0.state == ST_ACTIVE);
                $display("[%0t] M0 is ACTIVE (holding the grant) - issuing M1's request NOW", $time);

                @(posedge clk);
                m1_sel = 4'd5; m1_valid = 1'b1;
                @(posedge clk);
                m1_valid = 1'b0;
                $display("[%0t] M1 REQUEST issued (entry5: write split slave) while M0 still runs", $time);

                wait (dut.u_master0.state == ST_IDLE);
                $display("[%0t] M0 finished (wdata_mem[3]=%0d written) - M1 was still waiting for the bus", $time, dut.u_master0.wdata_mem[3]);

                wait (dut.u_master1.state == ST_IDLE);
                $display("[%0t] M1 finished AFTER M0 released the bus (wdata_mem[5]=%0d written)", $time, dut.u_master1.wdata_mem[5]);

                disable ph3_timeout;
            end
            begin : ph3_timeout
                #50000;
                $display("[%0t] *** Phase 3 TIMEOUT - a master hung waiting for the bus ***", $time);
                disable ph3_body;
            end
        join

        // ============================================================
        // Phase 4: M0 SPLITS (parks), M1 requests DURING the park
        // window - does a write then a read while M0 is parked - then
        // M0 resumes and completes. Every transition is logged.
        // ============================================================
        $display("\n==== Phase 4: M0 SPLITS, M1 requests DURING the park window ====");
        fork
            begin : ph4_body
                @(posedge clk);
                m0_sel = 4'd6; m0_valid = 1'b1;
                @(posedge clk);
                m0_valid = 1'b0;
                $display("[%0t] M0 REQUEST issued (entry6: read split slave)", $time);

                wait (dut.u_slave0.split_o);
                $display("[%0t] M0 PARKED (split_o pulsed) - bus is free, M1 can move in", $time);

                @(posedge clk);
                m1_sel = 4'd3; m1_valid = 1'b1;
                @(posedge clk);
                m1_valid = 1'b0;
                $display("[%0t] M1 write (entry3) issued DURING M0's park", $time);
                wait (dut.u_master1.state == ST_IDLE);
                $display("[%0t] M1 write completed during the park window (wdata_mem[3]=%0d)", $time, dut.u_master1.wdata_mem[3]);

                @(posedge clk);
                m1_sel = 4'd4; m1_valid = 1'b1;
                @(posedge clk);
                m1_valid = 1'b0;
                $display("[%0t] M1 read (entry4) issued, still around M0's park window", $time);
                wait (dut.u_master1.state == ST_IDLE);
                checks = checks + 1;
                if (dut.u_master1.rdata_mem[4] !== dut.u_slave2.mem[5]) begin
                    errors = errors + 1;
                    $display("[%0t] FAIL M1 read during park: got=%0d mem=%0d", $time, dut.u_master1.rdata_mem[4], dut.u_slave2.mem[5]);
                end else
                    $display("[%0t] PASS M1 read during park: rdata_mem[4]=%0d", $time, dut.u_master1.rdata_mem[4]);

                // A THIRD M1 transaction, still inside the same park window
                // (WAIT_CYCLES was widened via defparam above so resume_o
                // hasn't fired yet by this point) - another write to slave3.
                @(posedge clk);
                m1_sel = 4'd3; m1_valid = 1'b1;
                @(posedge clk);
                m1_valid = 1'b0;
                $display("[%0t] M1 3rd transaction (entry3: write slave3 again) issued, still during M0's park", $time);
                wait (dut.u_master1.state == ST_IDLE);
                $display("[%0t] M1 3rd transaction completed during the SAME park window (wdata_mem[3]=%0d)", $time, dut.u_master1.wdata_mem[3]);

                wait (dut.u_slave0.resume_o);
                $display("[%0t] M0 RESUME fired - bus handed back to M0", $time);

                wait (dut.u_master0.state == ST_IDLE);
                checks = checks + 1;
                if (dut.u_master0.rdata_mem[6] !== dut.u_slave0.mem[1]) begin
                    errors = errors + 1;
                    $display("[%0t] FAIL M0 split-read after resume: got=%0d mem=%0d", $time, dut.u_master0.rdata_mem[6], dut.u_slave0.mem[1]);
                end else
                    $display("[%0t] PASS M0 split-read completed after resume: rdata_mem[6]=%0d", $time, dut.u_master0.rdata_mem[6]);

                disable ph4_timeout;
            end
            begin : ph4_timeout
                #100000;
                $display("[%0t] *** Phase 4 TIMEOUT - split/resume sequence hung ***", $time);
                disable ph4_body;
            end
        join

        // ============================================================
        // Phase 5: mirror of Phase 4 - M1 SPLITS this time, M0 requests
        // during M1's park window.
        // ============================================================
        $display("\n==== Phase 5: M1 SPLITS, M0 requests DURING the park window (mirror) ====");
        fork
            begin : ph5_body
                @(posedge clk);
                m1_sel = 4'd6; m1_valid = 1'b1;
                @(posedge clk);
                m1_valid = 1'b0;
                $display("[%0t] M1 REQUEST issued (entry6: read split slave)", $time);

                wait (dut.u_slave0.split_o);
                $display("[%0t] M1 PARKED (split_o pulsed) - bus is free, M0 can move in", $time);

                @(posedge clk);
                m0_sel = 4'd3; m0_valid = 1'b1;
                @(posedge clk);
                m0_valid = 1'b0;
                $display("[%0t] M0 write (entry3) issued DURING M1's park", $time);
                wait (dut.u_master0.state == ST_IDLE);
                $display("[%0t] M0 write completed during the park window (wdata_mem[3]=%0d)", $time, dut.u_master0.wdata_mem[3]);

                @(posedge clk);
                m0_sel = 4'd4; m0_valid = 1'b1;
                @(posedge clk);
                m0_valid = 1'b0;
                $display("[%0t] M0 read (entry4) issued, still around M1's park window", $time);
                wait (dut.u_master0.state == ST_IDLE);
                checks = checks + 1;
                if (dut.u_master0.rdata_mem[4] !== dut.u_slave2.mem[5]) begin
                    errors = errors + 1;
                    $display("[%0t] FAIL M0 read during park: got=%0d mem=%0d", $time, dut.u_master0.rdata_mem[4], dut.u_slave2.mem[5]);
                end else
                    $display("[%0t] PASS M0 read during park: rdata_mem[4]=%0d", $time, dut.u_master0.rdata_mem[4]);

                // A THIRD M0 transaction, still inside the same park window.
                @(posedge clk);
                m0_sel = 4'd3; m0_valid = 1'b1;
                @(posedge clk);
                m0_valid = 1'b0;
                $display("[%0t] M0 3rd transaction (entry3: write slave3 again) issued, still during M1's park", $time);
                wait (dut.u_master0.state == ST_IDLE);
                $display("[%0t] M0 3rd transaction completed during the SAME park window (wdata_mem[3]=%0d)", $time, dut.u_master0.wdata_mem[3]);

                wait (dut.u_slave0.resume_o);
                $display("[%0t] M1 RESUME fired - bus handed back to M1", $time);

                wait (dut.u_master1.state == ST_IDLE);
                checks = checks + 1;
                if (dut.u_master1.rdata_mem[6] !== dut.u_slave0.mem[1]) begin
                    errors = errors + 1;
                    $display("[%0t] FAIL M1 split-read after resume: got=%0d mem=%0d", $time, dut.u_master1.rdata_mem[6], dut.u_slave0.mem[1]);
                end else
                    $display("[%0t] PASS M1 split-read completed after resume: rdata_mem[6]=%0d", $time, dut.u_master1.rdata_mem[6]);

                disable ph5_timeout;
            end
            begin : ph5_timeout
                #100000;
                $display("[%0t] *** Phase 5 TIMEOUT - split/resume sequence hung ***", $time);
                disable ph5_body;
            end
        join

        // ============================================================
        // Phase 6: TRUE simultaneous dual-master random contention. Each
        // round fires both masters' requests on the exact same clock edge
        // (via issue_both), so the arbiter genuinely has to arbitrate a
        // real collision every single round instead of the two streams
        // merely running close together by chance. Entries are drawn from
        // the full pool (3,4,5,6) independently for each master, so this
        // naturally covers plain-vs-plain, plain-vs-split, and
        // split-vs-split simultaneous collisions, letting the arbiter
        // handle all of it on its own - no scripted ordering here at all.
        // ============================================================
        $display("\n==== Phase 6: TRUE simultaneous dual-master random contention (incl. split) ====");
        fork
            begin : ph6_body
                integer r;
                for (r = 0; r < 25; r = r + 1) begin
                    issue_both(rand_entry(0), rand_entry(0));
                end
                disable ph6_timeout;
            end
            begin : ph6_timeout
                #600000;
                $display("[%0t] *** Phase 6 TIMEOUT - simultaneous contention hung ***", $time);
                disable ph6_body;
            end
        join

        // ============================================================
        // Phase 7: both masters hammering ONLY the split entries (5/6),
        // simultaneously issued every round (split-vs-split collision,
        // the hardest case for the park/resume path) with zero gap
        // between rounds so the next collision can start the instant the
        // arbiter frees up, instead of waiting around between rounds.
        // ============================================================
        $display("\n==== Phase 7: TRUE simultaneous dual-master split-slave (5/6) stress ====");
        fork
            begin : ph7_body
                integer r;
                for (r = 0; r < 20; r = r + 1) begin
                    issue_both(r[0] ? 4'd6 : 4'd5, r[0] ? 4'd5 : 4'd6);
                end
                disable ph7_timeout;
            end
            begin : ph7_timeout
                #600000;
                $display("[%0t] *** Phase 7 TIMEOUT - split-slave stress hung ***", $time);
                disable ph7_body;
            end
        join

        #500;
        $display("\n==== DONE: %0d checks, %0d errors ====", checks, errors);
        if (errors == 0) $display("ALL PASS");
        else $display("%0d FAILED", errors);
        $finish;
    end

    initial begin #3000000; $display("GLOBAL TIMEOUT - suite did not finish"); $finish; end

endmodule
