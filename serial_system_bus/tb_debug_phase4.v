`timescale 1ns / 1ps
module tb_debug_phase4;
    reg clk, rst;
    reg [3:0] m0_sel, m1_sel;
    reg m0_valid, m1_valid;
    wire [3:0] led;

    serial_2bus_top #(
        .M0_START_TXN(0), .M0_REQ_DELAY(5), .M0_WRITE_DELAY(10),
        .M1_START_TXN(0), .M1_REQ_DELAY(5), .M1_WRITE_DELAY(10)
    ) dut (
        .clk (clk), .rst (rst),
        .sc_uart_tx_o (), .sc_uart_rx_i (1'b1),
        .m0_pkt_sel_i (m0_sel), .m0_pkt_valid_i (m0_valid),
        .m1_pkt_sel_i (m1_sel), .m1_pkt_valid_i (m1_valid),
        .led_o (led)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst = 1'b1; m0_sel=0; m0_valid=0; m1_sel=0; m1_valid=0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
    end

    localparam ST_IDLE=1, ST_ACTIVE=3;

    always @(dut.u_master0.state)
        $display("[%0t] master0.state -> %0d req_o=%b", $time, dut.u_master0.state, dut.u_master0.req_o);
    always @(dut.u_master1.state)
        $display("[%0t] master1.state -> %0d req_o=%b", $time, dut.u_master1.state, dut.u_master1.req_o);
    always @(posedge clk)
        if (dut.u_slave0.frame_done)
            $display("[%0t] u_slave0.frame_done we=%b addr=%h", $time, dut.u_slave0.we_c, dut.u_slave0.addr_c);
    always @(dut.u_serial_system_bus.u_arbiter.state)
        $display("[%0t] arbiter.state->%0d grant_M0=%b grant_M1=%b", $time,
            dut.u_serial_system_bus.u_arbiter.state, dut.u_serial_system_bus.grant_M0, dut.u_serial_system_bus.grant_M1);

    // replicate phase3 then start of phase4
    task drive_m0(input [3:0] idx);
        begin
            @(posedge clk); m0_sel=idx; m0_valid=1'b1;
            @(posedge clk); m0_valid=1'b0;
            wait (dut.u_master0.state == ST_IDLE);
        end
    endtask
    task drive_m1(input [3:0] idx);
        begin
            @(posedge clk); m1_sel=idx; m1_valid=1'b1;
            @(posedge clk); m1_valid=1'b0;
            wait (dut.u_master1.state == ST_IDLE);
        end
    endtask

    initial begin
        wait (rst==1'b0);
        #100;
        // Phase3 replay
        @(posedge clk); m0_sel=3; m0_valid=1;
        @(posedge clk); m0_valid=0;
        wait (dut.u_master0.state == ST_ACTIVE);
        @(posedge clk); m1_sel=5; m1_valid=1;
        @(posedge clk); m1_valid=0;
        wait (dut.u_master0.state == ST_IDLE);
        wait (dut.u_master1.state == ST_IDLE);
        $display("=== entering phase4 now ===");
        @(posedge clk); m0_sel=6; m0_valid=1;
        @(posedge clk); m0_valid=0;
        $display("[%0t] entry6 pulse issued, waiting split_o", $time);
        wait (dut.u_slave0.split_o);
        $display("[%0t] split_o seen", $time);
        #2000;
        $finish;
    end
    initial begin #40000; $display("TIMEOUT"); $finish; end
endmodule
