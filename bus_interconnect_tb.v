`timescale 1ns / 1ps
module bus_interconnect_tb;

    reg clk_a;  // bus 1's FPGA
    reg clk_b;  // bus 2's FPGA - independent oscillator, same nominal period, offset phase
    reg rst;

    bus_interconnect_serial dut (
        .clk_a (clk_a),
        .clk_b (clk_b),
        .rst   (rst)
    );

    // Two nominally-identical clocks that are NOT phase-aligned, modeling
    // two separate FPGA oscillators - this is what actually exercises the
    // cdc_pulse_sync path on the ready line instead of accidentally
    // behaving like one shared clock. clk_b's free-run only starts after
    // a phase offset so it never lines up edge-for-edge with clk_a.
    always #5 clk_a = ~clk_a;
    initial begin
        clk_b = 1'b0;
        #3;
        forever #5 clk_b = ~clk_b;
    end

    initial begin
        clk_a = 1'b0;
        rst   = 1'b0;

        // release reset
        #12 rst = 1'b1;

        // Bus 1's masters run their transaction tables automatically once
        // out of reset (see master.v). Master 0's first two transactions
        // target address 0x2xxx, which decodes external on bus 1 and
        // crosses over to bus 2 as bus 1's Master 1 request.

        #700 $finish;
    end

    // Log every completed transfer on bus 1's own local slaves
    always @(posedge clk_a) begin
        if (rst && dut.u_bus1.u_system_bus.valid_bus && dut.u_bus1.u_system_bus.ready_slave) begin
            $display("t=%0t  BUS1  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.u_bus1.we_bus, dut.u_bus1.addr_bus,
                      dut.u_bus1.wdata_bus, dut.u_bus1.u_system_bus.rdata_slave);
        end
    end

    // Log every completed transfer on bus 2 (Master 0 local, or Master 1
    // = bus 1 wired straight into bus 2's exposed Master 1 slot)
    always @(posedge clk_b) begin
        if (rst && dut.u_bus2.u_system_bus.valid_bus && dut.u_bus2.u_system_bus.ready_slave) begin
            $display("t=%0t  BUS2  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.u_bus2.we_bus, dut.u_bus2.addr_bus,
                      dut.u_bus2.wdata_bus, dut.u_bus2.u_system_bus.rdata_slave);
        end
    end

endmodule
