`timescale 1ns / 1ps
module system_bus_tb;

    reg clk;
    reg rst;

    // No external bus modeled in this testbench: never grant it, and
    // leave its return-data lines quiet.
    wire req_ext;
    wire [15:0] addr_ext_o;
    wire [7:0]  wdata_ext_o;
    wire        ext_valid_o;

    top_module top (
        .clk         (clk),
        .rst         (rst),
        .req_ext     (req_ext),
        .grant_ext   (1'b0),
        .addr_ext_o  (addr_ext_o),
        .wdata_ext_o (wdata_ext_o),
        .ext_valid_o (ext_valid_o),
        .rdata_ext_i (8'h0),
        .ready_ext_i (1'b0),
        .rvalid_ext_i(1'b0)
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
                      $time, top.we_bus, top.addr_bus, top.wdata_bus, top.u_system_bus.rdata_slave);
        end
    end

endmodule
