`timescale 1ns / 1ps
module top_bridge_test_tb;

    reg clk;
    reg rst_n;   // active-low reset

    top_bridge_test dut (
        .clk   (clk),
        .rst_n (rst_n)
    );

    always #5 clk = ~clk;

    initial begin
        clk   = 1'b0;
        // top_bridge_test inverts this internally (wire rst = !rst_n) to
        // match the board's reset button idle level, so rst_n starts
        // asserted HIGH here and drops LOW to release - same convention as
        // bus_interconnect_tb.v.
        rst_n = 1'b1;

        #12 rst_n = 1'b0;

        // Master 0's transaction table (see master.v) fires automatically
        // once out of reset. Only the low 4 bits of the address reach the
        // simple slave responder here (see top_bridge_test.v's smem[16]),
        // so several of master.v's default transactions alias onto the
        // same entries - that's fine, it's still exercising the full
        // master -> master_bridge -> slave_bridge -> responder round trip.

        #2000 $finish;
    end

    // Log every completed transaction on the slave_bridge's internal port
    always @(posedge clk) begin
        if (!rst_n && dut.valid_s && dut.ready_s) begin
            $display("t=%0t  we=%b addr=%0h wdata=%0h rdata=%0h",
                      $time, dut.we_s, dut.idx, dut.wdata_s, dut.rdata_s);
        end
    end

endmodule
