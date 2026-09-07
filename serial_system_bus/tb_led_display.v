// Isolated single-bus testbench: on a fixed period, advances m0_pkt_sel_i
// to the next transaction in the master's table (1..NUM_TXN, wrapping),
// regardless of whether the previous one has finished - simulates an
// external switch/selector being changed periodically rather than
// synchronized handshaking. Prints the master's own captured rdata_mem
// alongside led_o at the end of each period. Only exercises the local bus
// (M0 + slave0/slave1) - the UART cross-board link (mc_*/sc_*) is left
// idle/unused.
`timescale 1ns/1ps

module tb_led_display;

    localparam NUM_TXN        = 8;
    localparam PARK_CYCLES    = 10;   // pkt_sel held at 0 (no selection) between packets, so the master sees a fresh 0->nonzero edge each period
    localparam PERIOD_CYCLES  = 100;  // total time budget per packet, PARK_CYCLES included

    reg clk = 0;
    reg rst = 0;   // active-high reset in, inverted to rst_n inside DUT

    reg [3:0] pkt_sel;
    integer   i;

    wire mc_uart_tx_o;
    wire sc_uart_tx_o;
    wire [3:0] led_o;

    always #5 clk = ~clk;

    serial_bus_top #(
        .M0_START_TXN   (1),
        .M0_REQ_DELAY   (10),
        .M0_WRITE_DELAY (10)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .mc_uart_tx_o (mc_uart_tx_o),
        .mc_uart_rx_i (1'b1),   // UART idle - unused, single-bus test only
        .sc_uart_tx_o (sc_uart_tx_o),
        .sc_uart_rx_i (1'b1),   // UART idle - unused, single-bus test only
        .m0_pkt_sel_i (4'b0100),
        .led_o        (led_o)
    );

    initial begin
        pkt_sel = 4'd0;
        rst = 1;
        repeat (5) @(posedge clk);
        rst = 0;   // release reset

        for (i = 2; i <= NUM_TXN; i = i + 1) begin                   // park first, so this is a genuine new selection
            repeat (PARK_CYCLES) @(posedge clk);
            pkt_sel = i[3:0];
            repeat (PERIOD_CYCLES - PARK_CYCLES) @(posedge clk);

            if (dut.u_master0.we_mem[i])
                $display("[%0t] period %0d WRITE addr=%h wdata=%h",
                          $time, i, dut.u_master0.addr_mem[i], dut.u_master0.wdata_mem[i]);
            else
                $display("[%0t] period %0d READ  addr=%h -> rdata_mem=%h led_o=%b (led[3]=we, led[2:0]=data[2:0])",
                          $time, i, dut.u_master0.addr_mem[i], dut.u_master0.rdata_mem[i], led_o);
        end

        pkt_sel = 4'd0;  // back to "no selection" once the sweep is done
        $display("[%0t] periodic sweep done", $time);
        $finish;
    end

endmodule
