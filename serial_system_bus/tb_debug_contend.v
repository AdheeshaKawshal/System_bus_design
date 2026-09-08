`timescale 1ns / 1ps
module tb_debug_contend;
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

    always @(dut.u_serial_system_bus.u_arbiter.state)
        $display("[%0t] arbiter.state -> %0d grant_M0=%b grant_M1=%b parked_id=%b cur_owner=%b", $time,
            dut.u_serial_system_bus.u_arbiter.state, dut.u_serial_system_bus.grant_M0, dut.u_serial_system_bus.grant_M1,
            dut.u_serial_system_bus.u_arbiter.parked_id, dut.u_serial_system_bus.u_arbiter.cur_owner);
    always @(dut.u_master0.state)
        $display("[%0t] master0.state -> %0d req_o=%b", $time, dut.u_master0.state, dut.u_master0.req_o);
    always @(dut.u_master1.state)
        $display("[%0t] master1.state -> %0d req_o=%b", $time, dut.u_master1.state, dut.u_master1.req_o);
    always @(dut.u_slave0.state)
        $display("[%0t] u_slave0.state -> %0d", $time, dut.u_slave0.state);

    always @(posedge clk) begin
        if ($time > 3900000 && $time < 4200000)
            $display("[%0t] grant_M1_wire=%b master1.grant_i=%b master1.req_o=%b master1.state=%0d req_M1_wire=%b",
                $time, dut.u_serial_system_bus.grant_M1, dut.u_master1.grant_i, dut.u_master1.req_o, dut.u_master1.state,
                dut.u_serial_system_bus.req_M1);
    end

    initial begin
        wait (rst == 1'b0);
        #100;
        @(posedge clk);
        m0_sel = 4'd5; m0_valid = 1'b1;  // M0 -> write split slave
        m1_sel = 4'd3; m1_valid = 1'b1;  // M1 -> write slave3, same time
        @(posedge clk);
        m0_valid = 1'b0; m1_valid = 1'b0;
        #3000;
        $display("--- now both read ---");
        @(posedge clk);
        m0_sel = 4'd6; m0_valid = 1'b1;  // M0 -> read split slave
        m1_sel = 4'd4; m1_valid = 1'b1;  // M1 -> read slave3
        @(posedge clk);
        m0_valid = 1'b0; m1_valid = 1'b0;
        #5000;
        $display("m0 rdata_mem[6]=%0d m1 rdata_mem[4]=%0d", dut.u_master0.rdata_mem[6], dut.u_master1.rdata_mem[4]);
        $finish;
    end
    initial begin #50000; $display("TIMEOUT"); $finish; end
endmodule
