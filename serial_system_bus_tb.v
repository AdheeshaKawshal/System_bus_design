`timescale 1ns / 1ps
// serial_system_bus_tb: two independent instances of serial_system_bus.v
// (the module meant to be flashed onto two separate FPGAs), cross-wired
// through their tx_*/rx_* pins exactly as the two boards would be wired
// together - board A's tx_* -> board B's rx_* and vice versa. Both run
// off the same clk/rst here for simplicity (same nominal frequency,
// independent oscillators on real boards - see serial_system_bus.v's
// header), so this exercises the full master.v -> master_bridge ->
// (serial pins) -> slave_bridge -> system_busv1 -> slave.v round trip on
// both directions at once, same as bus_interconnect_tb.v but through the
// pin-level interface instead of internal wiring.
module serial_system_bus_tb;

    reg clk;
    reg rst_n;   // active-low reset

    // Board A -> Board B
    wire ab_req, ab_grant, ab_addr, ab_wdata, ab_status, ab_ready;
    // Board B -> Board A
    wire ba_req, ba_grant, ba_addr, ba_wdata, ba_status, ba_ready;

    serial_system_bus board_a (
        .clk        (clk),
        .rst_n      (rst_n),

        // Outbound (A's master reaching onto B)
        .tx_req_o   (ab_req),
        .rx_grant_i (ab_grant),
        .tx_addr_o  (ab_addr),
        .tx_wdata_o (ab_wdata),
        .rx_status_i(ab_status),
        .rx_ready_i (ab_ready),

        // Inbound (B's master reaching onto A)
        .rx_req_i   (ba_req),
        .tx_grant_o (ba_grant),
        .rx_addr_i  (ba_addr),
        .rx_wdata_i (ba_wdata),
        .tx_status_o(ba_status),
        .tx_ready_o (ba_ready)
    );

    serial_system_bus board_b (
        .clk        (clk),
        .rst_n      (rst_n),

        // Outbound (B's master reaching onto A)
        .tx_req_o   (ba_req),
        .rx_grant_i (ba_grant),
        .tx_addr_o  (ba_addr),
        .tx_wdata_o (ba_wdata),
        .rx_status_i(ba_status),
        .rx_ready_i (ba_ready),

        // Inbound (A's master reaching onto B)
        .rx_req_i   (ab_req),
        .tx_grant_o (ab_grant),
        .rx_addr_i  (ab_addr),
        .rx_wdata_i (ab_wdata),
        .tx_status_o(ab_status),
        .tx_ready_o (ab_ready)
    );

    always #5 clk = ~clk;

    initial begin
        clk   = 1'b0;
        // Same convention as bus_interconnect_tb.v: rst_n idle HIGH
        // (reset asserted), dropped LOW to release.
        rst_n = 1'b1;

        #12 rst_n = 1'b0;

        // Each board's Master 0 runs its transaction table automatically
        // once out of reset (see master.v). Any address with bit 14 set
        // decodes external on that board and crosses the serial link to
        // the other board's Master 1 port.

        #20000 $finish;
    end

    // Log every completed transfer on board A's own local slaves
    always @(posedge clk) begin
        if (!rst_n && board_a.u_bus.u_system_bus.valid_bus && board_a.u_bus.u_system_bus.ready_slave) begin
            $display("t=%0t  BOARD_A  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, board_a.u_bus.u_system_bus.we_bus, board_a.u_bus.u_system_bus.addr_bus,
                      board_a.u_bus.u_system_bus.wdata_bus, board_a.u_bus.u_system_bus.rdata_slave);
        end
    end

    // Log every completed transfer on board B (local Master 0, or Master 1
    // = board A's master arriving over the serial link)
    always @(posedge clk) begin
        if (!rst_n && board_b.u_bus.u_system_bus.valid_bus && board_b.u_bus.u_system_bus.ready_slave) begin
            $display("t=%0t  BOARD_B  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, board_b.u_bus.u_system_bus.we_bus, board_b.u_bus.u_system_bus.addr_bus,
                      board_b.u_bus.u_system_bus.wdata_bus, board_b.u_bus.u_system_bus.rdata_slave);
        end
    end

endmodule
