`timescale 1ns / 1ps
module bus_interconnect_tb;

    reg clk;   // shared synchronous clock for both buses
    reg rst_n; // active-low reset

    bus_interconnect_serial dut (
        .clk_a   (clk),
        .rst_n (rst_n)
    );

    always #5 clk = ~clk;

    initial begin
        clk   = 1'b0;
        // bus_interconnect inverts this internally (wire rst = !rst_n) to
        // match the board's reset button idle level, so rst_n starts
        // asserted HIGH here and drops LOW to release.
        rst_n = 1'b1;

        // release reset
        #12 rst_n = 1'b0;

        // Bus 1's masters run their transaction tables automatically once
        // out of reset (see master.v). Master 0's first two transactions
        // target address 0x2xxx, which decodes external on bus 1 and
        // crosses over to bus 2 as bus 1's Master 1 request.

        #700 $finish;
    end

    // Log every completed transfer on bus 1's own local slaves
    always @(posedge clk) begin
        if (!rst_n && dut.u_bus1.u_system_bus.valid_bus && dut.u_bus1.u_system_bus.ready_slave) begin
            $display("t=%0t  BUS1  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.u_bus1.u_system_bus.we_bus, dut.u_bus1.u_system_bus.addr_bus,
                      dut.u_bus1.u_system_bus.wdata_bus, dut.u_bus1.u_system_bus.rdata_slave);
        end
    end

    // Log every completed transfer on bus 2 (Master 0 local, or Master 1
    // = bus 1 wired straight into bus 2's exposed Master 1 slot)
    always @(posedge clk) begin
        if (!rst_n && dut.u_bus2.u_system_bus.valid_bus && dut.u_bus2.u_system_bus.ready_slave) begin
            $display("t=%0t  BUS2  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.u_bus2.u_system_bus.we_bus, dut.u_bus2.u_system_bus.addr_bus,
                      dut.u_bus2.u_system_bus.wdata_bus, dut.u_bus2.u_system_bus.rdata_slave);
        end
    end

endmodule
