`timescale 1ns / 1ps
module system_bus_tb;

    reg clk;
    reg rst_n;

    // No external bus modeled: top_module ties its own ext_* port off
    // internally now, so only clk/rst_n are exposed here.
    top_module top (
        .clk   (clk),
        .rst_n (rst_n)
    );

    // clock
    always #5 clk = ~clk;

    initial begin
        clk        = 1'b0;
        // top_module inverts this internally (wire rst = !rst_n) to match
        // the board's reset button idle level, so rst_n starts asserted
        // HIGH here and drops LOW to release - opposite of a normal
        // active-low reset input.
        rst_n      = 1'b1;

        // release reset
        #12 rst_n = 1'b0;

        // Master 0 runs its full transaction table (see master.v) automatically
        // once out of reset; just let it run and watch the waveform / prints below.

        #800 $finish;
    end

    // Log each completed bus transfer
    always @(posedge clk) begin
        if (!rst_n && top.u_system_bus.valid_bus && top.u_system_bus.ready_slave) begin
            $display("t=%0t  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, top.we_bus, top.addr_bus, top.wdata_bus, top.u_system_bus.rdata_slave);
        end
    end

endmodule
