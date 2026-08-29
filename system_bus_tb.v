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

        // TODO: drive test cases here

        #200 $finish;
    end

endmodule
