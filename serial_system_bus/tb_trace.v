`timescale 1ns / 1ps
module tb_trace;
    reg clk, rst;

    serial_bus_top #(.ADDR_W(15), .DATA_W(8), .RW(1), .NUM_SLAVES(3)) dut (.clk(clk), .rst(rst));

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst = 1'b0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b1;
    end

    initial begin
        force dut.u_master1.req_o = 1'b0;
    end

    always @(posedge clk) begin
        if (dut.u_slave0.frame_done)
            $display("[%0t] slave0(=slave1) frame_done we=%b addr=%h wdata=%h", $time, dut.u_slave0.we_c, dut.u_slave0.addr_c, dut.u_slave0.wdata_c);
        if (dut.u_slave1.frame_done)
            $display("[%0t] slave1(=slave2) frame_done we=%b addr=%h wdata=%h", $time, dut.u_slave1.we_c, dut.u_slave1.addr_c, dut.u_slave1.wdata_c);
    end

    always @(dut.u_master0.state) begin
        $display("[%0t] master0 state->%0d tx_ptr=%0d", $time, dut.u_master0.state, dut.u_master0.tx_ptr);
    end

    initial begin
        wait (rst == 1'b1);
        #3000;
        $finish;
    end
endmodule
