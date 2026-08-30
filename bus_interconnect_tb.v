`timescale 1ns / 1ps
`include "bus_interconnect.v"
module bus_interconnect_tb;

    reg clk;
    reg rst;

    bus_interconnect dut (
        .clk (clk),
        .rst (rst)
    );

    // clock
    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        rst = 1'b0;

        // release reset
        #12 rst = 1'b1;

        // Bus 1's masters run their transaction tables automatically once
        // out of reset (see master.v). Master 0's first two transactions
        // target address 0x2xxx, which decodes external on bus 1 and
        // crosses over to bus 2 as bus 1's Master 1 request.

        #1200 $finish;
    end

    // Log every completed transfer on bus 1's own local slaves
    always @(posedge clk) begin
        if (rst && dut.u_bus1.u_system_bus.valid_bus && dut.u_bus1.u_system_bus.ready_slave) begin
            $display("t=%0t  BUS1  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.u_bus1.we_bus, dut.u_bus1.addr_bus,
                      dut.u_bus1.wdata_bus, dut.u_bus1.rdata_slave);
        end
    end

    // Log every completed transfer on bus 2 (Master 0 local, or Master 1
    // = bus 1 crossing over through its ext_* port)
    always @(posedge clk) begin
        if (rst && dut.u_system_bus2.valid_bus && dut.u_system_bus2.ready_slave) begin
            $display("t=%0t  BUS2  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.we_bus_b2, dut.addr_bus_b2,
                      dut.wdata_bus_b2, dut.rdata_slave_b2);
        end
    end

endmodule
