`timescale 1ns / 1ps
`include "top_module.v"
module system_bus_tb;

    reg clk;
    reg rst;

    top_module top (
        .clk(clk),
        .rst(rst)
    );

    // clock
    always #5 clk = ~clk;

    initial begin
        clk        = 1'b0;
        rst        = 1'b0;

        // release reset
        #12 rst = 1'b1;

        // Master 0 runs its full transaction table (see master.v) automatically
        // once out of reset; just let it run and watch the waveform / prints below.

        #800 $finish;
    end

    // Log each completed bus transfer
    always @(posedge clk) begin
        if (rst && top.u_system_bus.valid_bus && top.u_system_bus.ready_slave) begin
            $display("t=%0t  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, top.we_bus, top.addr_bus, top.wdata_bus, top.rdata_slave);
        end
    end

endmodule
